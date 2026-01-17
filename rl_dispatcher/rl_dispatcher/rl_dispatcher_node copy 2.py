#!/usr/bin/env python3
import sys
sys.path.append("/home/suda/.local/lib/python3.10/site-packages")
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rmf_fleet_msgs.msg import FleetState
from std_msgs.msg import String
from geometry_msgs.msg import Point
from rmf_custom_tasks_self.srv import SingleNavTask
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
import json
import time
import random
from datetime import datetime
# 关键修复：导入Gymnasium（替代原Gym）
import gymnasium as gym
from gymnasium import spaces

# 1.17 把模拟rmf的代码也混在里面了

# ===================== 第一步：完整的RL调度环境（适配真实场景） =====================
class RLDispatchingEnv(gym.Env):  # 关键修复1：继承gym.Env基类
    """
    适配Gymnasium标准的RMF调度环境
    核心接口：reset(), step(), render(), close()
    """
    metadata = {"render_modes": ["human"], "render_fps": 1}  # 标准元数据
    
    def __init__(self, node, render_mode=None):
        # 关键修复2：调用基类初始化
        super().__init__()
        # 传入ROS2节点用于日志输出
        self.node = node
        self.render_mode = render_mode
        
        # 1. 无人车配置（匹配你的RMF车队）
        self.robot_names = ["deliveryRobot_0", "deliveryRobot_1", "deliveryRobot_2"]
        self.robot_positions = {name: Point(x=0.0, y=0.0) for name in self.robot_names}
        self.robot_idle = {name: True for name in self.robot_names}  # 空闲状态
        self.robot_task_map = {}  # 记录小车当前执行的任务ID {robot_name: task_id}
        
        # 2. 任务配置
        self.pending_tasks = []  # 待执行任务队列
        self.executing_tasks = {}  # 执行中任务 {task_id: {"robot": "", "start_time": 0.0, "waypoint": ""}}
        self.completed_tasks = []  # 已完成任务
        self.failed_tasks = []  # 失败任务
        
        # 3. 训练相关配置
        self.current_time = 0.0  # 当前仿真时间（秒）
        self.episode_steps = 0  # 单轮训练步数
        self.max_episode_steps = 500  # 单轮最大步数
        self.total_reward = 0.0  # 单轮总奖励
        
        # 4. 关键修复3：定义标准的动作空间和状态空间（SB3必需）
        # 动作空间：离散空间，0/1/2对应三台小车
        self.action_space = spaces.Discrete(len(self.robot_names))
        # 状态空间：Box空间，维度和原逻辑一致，值范围归一化到[-1,1]
        self.observation_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(3*3 + 3 + 2,),  # 原observation_space_shape
            dtype=np.float32
        )
        
        # 5. 奖励系数（可调）
        self.reward_coeff = {
            "distance": -0.1,    # 距离惩罚（负数，距离越近惩罚越小）
            "completion": 100.0, # 任务完成奖励
            "idle_selection": -10.0,  # 选择忙碌小车的惩罚
            "timeout": -50.0,    # 任务超时惩罚
            "step": -0.1         # 每步基础惩罚（鼓励快速完成）
        }
        
        self.node.get_logger().info("🔧 RL环境初始化完成，小车数量：%d" % len(self.robot_names))

    def reset(self, seed=None, options=None):  # 关键修复4：适配Gymnasium的reset接口
        """重置环境（训练时每轮开始调用）"""
        # 关键：设置随机种子（SB3要求）
        super().reset(seed=seed)
        
        # 重置状态
        self.robot_idle = {name: True for name in self.robot_names}
        self.robot_task_map = {}
        self.pending_tasks = []
        self.executing_tasks = {}
        self.completed_tasks = []
        self.failed_tasks = []
        
        self.current_time = 0.0
        self.episode_steps = 0
        self.total_reward = 0.0
        
        self.node.get_logger().debug("🔄 RL环境已重置")
        
        # 关键：返回(obs, info)元组（Gymnasium标准）
        obs = self._get_observation()
        info = {"total_reward": 0.0, "completed_tasks": 0}
        return obs, info

    def _get_observation(self):
        """构建状态向量（供RL模型输入）"""
        # 1. 小车状态：x, y, 是否空闲（1/0）
        robot_obs = []
        for name in self.robot_names:
            robot_obs.extend([
                self.robot_positions[name].x / 200.0,  # 归一化（你的航点最大x约180）
                self.robot_positions[name].y / 50.0,   # 归一化（y约-50~0）
                1.0 if self.robot_idle[name] else 0.0
            ])
        
        # 2. 待执行任务状态（取第一个）
        task_obs = [0.0, 0.0, 0.0]  # x, y, wait_time
        if self.pending_tasks:
            task = self.pending_tasks[0]
            task_obs = [
                task["x"] / 200.0,
                task["y"] / 50.0,
                min(task["wait_time"] / 100.0, 1.0)  # 等待时间归一化（最大100秒）
            ]
        
        # 3. 全局状态
        global_obs = [
            self.current_time / 1000.0,  # 时间归一化
            len(self.completed_tasks) / 10.0  # 已完成任务数归一化（最大10个）
        ]
        
        # 合并为状态向量
        obs = np.array(robot_obs + task_obs + global_obs, dtype=np.float32)
        return obs

    def step(self, action):  # 关键修复5：保持逻辑不变，但返回格式适配Gymnasium
        """执行动作（核心：选车+计算奖励）"""
        self.episode_steps += 1
        self.current_time += 1.0
        reward = 0.0
        terminated = False  # 任务完成/失败导致的终止
        truncated = False   # 步数超限导致的截断
        info = {
            "selected_robot": "",
            "task_id": "",
            "distance": 0.0,
            "reward_breakdown": {},
            "total_reward": 0.0
        }
        
        # 1. 基础惩罚：每步都有（鼓励快速完成）
        reward += self.reward_coeff["step"]
        info["reward_breakdown"]["step"] = self.reward_coeff["step"]
        
        # 2. 动作映射：action→小车名
        selected_robot = self.robot_names[action]
        info["selected_robot"] = selected_robot
        
        # 3. 若无待执行任务，直接返回
        if not self.pending_tasks:
            info["reward_breakdown"]["no_task"] = 0.0
            truncated = self.episode_steps >= self.max_episode_steps  # 步数超限截断
            return self._get_observation(), reward, terminated, truncated, info
        
        # 4. 取出第一个待执行任务
        task = self.pending_tasks.pop(0)
        task_id = task["task_id"]
        info["task_id"] = task_id
        
        # 5. 检查选中的小车是否空闲
        if not self.robot_idle[selected_robot]:
            # 惩罚：选择忙碌小车
            reward += self.reward_coeff["idle_selection"]
            info["reward_breakdown"]["idle_selection"] = self.reward_coeff["idle_selection"]
            # 任务放回队列
            self.pending_tasks.insert(0, task)
            self.node.get_logger().warn(f"⚠️ 选中忙碌小车{selected_robot}，惩罚{self.reward_coeff['idle_selection']}")
        else:
            # 6. 计算小车到目标的距离惩罚
            robot_pos = self.robot_positions[selected_robot]
            distance = np.sqrt((robot_pos.x - task["x"])**2 + (robot_pos.y - task["y"])**2)
            distance_reward = self.reward_coeff["distance"] * distance
            reward += distance_reward
            info["distance"] = distance
            info["reward_breakdown"]["distance"] = distance_reward
            
            # 7. 标记小车为忙碌，任务为执行中
            self.robot_idle[selected_robot] = False
            self.robot_task_map[selected_robot] = task_id
            self.executing_tasks[task_id] = {
                "robot": selected_robot,
                "start_time": self.current_time,
                "waypoint": task["waypoint"],
                "target_x": task["x"],
                "target_y": task["y"]
            }
            self.node.get_logger().info(f"🤖 调度{selected_robot}执行任务{task_id}，距离{distance:.2f}米，奖励{distance_reward:.2f}")
        
        # 8. 检查任务超时（执行超过60秒算超时）
        timeout_tasks = []
        for tid, t_info in self.executing_tasks.items():
            if self.current_time - t_info["start_time"] > 60.0:
                # 超时惩罚
                reward += self.reward_coeff["timeout"]
                info["reward_breakdown"]["timeout"] = self.reward_coeff["timeout"]
                self.failed_tasks.append(tid)
                timeout_tasks.append(tid)
                # 释放小车
                self.robot_idle[t_info["robot"]] = True
                del self.robot_task_map[t_info["robot"]]
        
        # 移除超时任务
        for tid in timeout_tasks:
            del self.executing_tasks[tid]
        
        # 9. 检查是否达到最大步数（truncated=True）
        if self.episode_steps >= self.max_episode_steps:
            truncated = True
            self.node.get_logger().info(f"🔚 单轮训练结束（步数上限），总奖励：{self.total_reward:.2f}，完成任务数：{len(self.completed_tasks)}")
        
        # 10. 更新总奖励
        self.total_reward += reward
        info["total_reward"] = self.total_reward
        
        # 关键：返回(obs, reward, terminated, truncated, info)元组（Gymnasium标准）
        return self._get_observation(), reward, terminated, truncated, info

    def render(self):  # 关键修复6：实现标准render方法（空实现即可）
        """渲染环境（无需可视化，空实现）"""
        if self.render_mode == "human":
            self.node.get_logger().info(f"📊 渲染：当前步数{self.episode_steps}，总奖励{self.total_reward:.2f}")

    def close(self):  # 关键修复7：实现标准close方法
        """关闭环境（空实现）"""
        pass

    def add_task(self, task_id, waypoint, x, y):
        """添加新任务到待执行队列"""
        self.pending_tasks.append({
            "task_id": task_id,
            "waypoint": waypoint,
            "x": x,
            "y": y,
            "wait_time": 0.0
        })
        # 更新任务等待时间的定时器（在ROS节点中处理）
        self.node.get_logger().info(f"📥 添加新任务：{task_id} -> {waypoint} (x:{x:.2f}, y:{y:.2f})")

    def complete_task(self, task_id):
        """任务完成回调（由ROS节点调用）"""
        if task_id not in self.executing_tasks:
            self.node.get_logger().warn(f"⚠️ 任务{task_id}不在执行中，跳过完成回调")
            return
        
        # 1. 获取任务信息
        task_info = self.executing_tasks[task_id]
        robot_name = task_info["robot"]
        
        # 2. 任务完成奖励
        self.total_reward += self.reward_coeff["completion"]
        self.node.get_logger().info(f"🎯 任务{task_id}完成！奖励{self.reward_coeff['completion']}，执行小车：{robot_name}")
        
        # 3. 更新状态
        self.completed_tasks.append(task_id)
        del self.executing_tasks[task_id]
        self.robot_idle[robot_name] = True
        del self.robot_task_map[robot_name]

    def update_robot_position(self, robot_name, x, y):
        """更新小车位置（由ROS节点回调调用）"""
        if robot_name in self.robot_positions:
            self.robot_positions[robot_name].x = x
            self.robot_positions[robot_name].y = y

    def update_task_wait_time(self):
        """更新待执行任务的等待时间"""
        for task in self.pending_tasks:
            task["wait_time"] += 1.0

# ===================== 第二步：RL调度节点（训练+推理一体化） =====================
class RLDispatcherNode(Node):
    def __init__(self):
        super().__init__("rl_dispatcher_node")
        self.get_logger().info("🚀 初始化RL调度节点（训练+推理模式）...")
        
        # 1. 仿真时间配置
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        
        # 2. 核心配置：训练/推理模式（通过参数控制）
        self.declare_parameter("mode", "train")  # train / infer
        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.get_logger().info(f"🔧 当前模式：{self.mode}")
        
        # 3. 初始化RL环境（适配新的Gymnasium环境）
        self.rl_env = RLDispatchingEnv(self, render_mode="human")
        
        # 4. 初始化RL模型
        self.model = None
        self._init_rl_model()
        
        # 5. ROS订阅器
        # 5.1 订阅小车状态
        self.fleet_state_sub = self.create_subscription(
            FleetState,
            "/fleet_states",
            self.fleet_state_callback,
            10
        )
        # 5.2 订阅新任务（无人机发现目标）
        self.target_sub = self.create_subscription(
            String,
            "/task_monitor/start",
            self.target_callback,
            10
        )
        # 5.3 订阅任务完成信号（监控节点发布）
        self.completion_sub = self.create_subscription(
            String,
            "/custom_task_completion",
            self.completion_callback,
            10
        )
        
        # 6. ROS服务客户端（调用自定义任务服务）
        self.nav_client = self.create_client(SingleNavTask, "/submit_single_nav_task")
        while not self.nav_client.wait_for_service(timeout_sec=5.0):
            if not rclpy.ok():
                self.get_logger().error("❌ 等待服务时节点退出！")
                return
            self.get_logger().info("⏳ 等待/submit_single_nav_task服务可用...")
        self.get_logger().info("✅ 已连接自定义任务服务！")
        
        # 7. 航点坐标映射（复用你的监控节点配置）
        self.waypoint_coords = self._init_waypoint_coords()
        
        # 8. 定时器
        # 8.1 状态更新定时器（1秒/次）
        self.state_timer = self.create_timer(1.0, self.state_timer_callback)
        # 8.2 训练/推理定时器（训练时2秒决策一次，推理时1秒）
        self.dispatch_timer = self.create_timer(2.0 if self.mode == "train" else 1.0, self.dispatch_callback)
        
        # 9. 去重+防抖逻辑
        self.processed_target_ids = set()
        self.last_target_time = 0.0
        self.DEBOUNCE_DELAY = 1.0
        
        self.get_logger().info("✅ RL调度节点初始化完成！")

    def _init_waypoint_coords(self):
        """航点坐标映射（和你的监控节点完全一致）"""
        waypoint_coords = {}
        waypoint_coords["junction_n01"] = Point(x=1.57, y=-45.93)
        waypoint_coords["n08"] = Point(x=59.61, y=-7.42)
        waypoint_coords["n14"] = Point(x=80.84, y=-28.52)
        waypoint_coords["n13"] = Point(x=84.44, y=-4.94)
        waypoint_coords["n23"] = Point(x=182.80, y=-42.30)
        waypoint_coords["west_koi_pond"] = Point(x=34.32, y=-10.13)
        waypoint_coords["s08"] = Point(x=96.61, y=-50.50)
        waypoint_coords["s10"] = Point(x=122.10, y=-46.68)
        waypoint_coords["s11"] = Point(x=152.73, y=-43.00)
        waypoint_coords["junction_south_west"] = Point(x=84.56, y=-38.81)
        return waypoint_coords

    def _init_rl_model(self):
        """初始化RL模型（训练/推理模式）"""
        if self.mode == "train":
            # 训练模式：新建模型
            self.get_logger().info("📚 初始化PPO训练模型...")
            self.model = PPO(
                "MlpPolicy",
                self.rl_env,
                verbose=1,
                learning_rate=3e-4,
                n_steps=2048,
                batch_size=64,
                gamma=0.99,
                tensorboard_log="./rl_dispatching_tb_log/",
                device='cpu'  # 关键：强制使用CPU
            )
            # 训练检查点回调（每1000步保存一次）
            self.checkpoint_callback = CheckpointCallback(
                save_freq=1000,
                save_path="./rl_models/",
                name_prefix="dispatching_ppo"
            )
        elif self.mode == "infer":
            # 推理模式：加载预训练模型
            self.get_logger().info("🔍 加载预训练模型...")
            try:
                self.model = PPO.load(
                    "./rl_models/dispatching_ppo_final",
                    device='cpu'  # 推理时也强制用CPU
                )
                # 关键：加载模型后设置环境
                self.model.set_env(self.rl_env)
                self.get_logger().info("✅ 模型加载成功！")
            except Exception as e:
                self.get_logger().error(f"❌ 模型加载失败：{str(e)}")
                # 加载失败时退化为随机选车
                self.model = None
        else:
            self.get_logger().error(f"❌ 无效模式：{self.mode}，请选择train/infer")
            rclpy.shutdown()


    def fleet_state_callback(self, msg):
        """更新小车位置和状态"""
        if msg.name != "deliveryRobot":
            return
        for robot in msg.robots:
            robot_name = robot.name
            if robot_name in self.rl_env.robot_names:
                # 更新位置
                self.rl_env.update_robot_position(robot_name, robot.location.x, robot.location.y)
                # 更新空闲状态（task_id为空则空闲）
                self.rl_env.robot_idle[robot_name] = (robot.task_id == "")
                self.get_logger().debug(
                    f"🔍 更新小车状态：{robot_name} (x:{robot.location.x:.2f}, y:{robot.location.y:.2f}) 空闲: {self.rl_env.robot_idle[robot_name]}"
                )

    def target_callback(self, msg):
        """接收新任务（无人机发现目标）"""
        # 解析消息：task_id,target_waypoint
        data = msg.data.split(",")
        if len(data) < 2:
            self.get_logger().error("❌ 目标消息格式错误：%s", msg.data)
            return
        
        target_id = data[0].strip()
        target_waypoint = data[1].strip()
        
        # 防抖+去重
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_target_time < self.DEBOUNCE_DELAY:
            self.get_logger().debug(f"⚠️ 防抖过滤：{target_id}")
            return
        if target_id in self.processed_target_ids:
            self.get_logger().debug(f"⚠️ 去重过滤：{target_id}")
            return
        
        # 记录任务ID和时间
        self.processed_target_ids.add(target_id)
        self.last_target_time = current_time
        
        # 获取航点坐标
        if target_waypoint not in self.waypoint_coords:
            self.get_logger().error("❌ 未知航点：%s", target_waypoint)
            return
        target_coords = self.waypoint_coords[target_waypoint]
        
        # 添加到RL环境
        self.rl_env.add_task(target_id, target_waypoint, target_coords.x, target_coords.y)

    def completion_callback(self, msg):
        """接收任务完成信号（监控节点发布）"""
        # 解析消息：task_id,robot_name,waypoint,timestamp
        data = msg.data.split(",")
        if len(data) < 1:
            self.get_logger().error("❌ 完成消息格式错误：%s", msg.data)
            return
        task_id = data[0].strip()
        # 通知RL环境任务完成
        self.rl_env.complete_task(task_id)

    def state_timer_callback(self):
        """状态更新定时器（1秒/次）"""
        # 更新任务等待时间
        self.rl_env.update_task_wait_time()
        # 训练模式：更新环境时间
        if self.mode == "train":
            self.rl_env.current_time += 1.0

    def dispatch_callback(self):
        """RL决策+任务调度（定时器触发）"""
        # 1. 判空：无待执行任务则跳过
        if not self.rl_env.pending_tasks:
            return
        
        # 2. 获取当前状态（适配新的reset返回格式）
        obs = self.rl_env._get_observation()
        
        # 3. RL决策（新增完整防护逻辑）
        action = 0  # 默认动作
        # 先筛选所有空闲小车的索引
        idle_robot_indices = [i for i, name in enumerate(self.rl_env.robot_names) if self.rl_env.robot_idle[name]]
        all_robot_indices = list(range(len(self.rl_env.robot_names)))  # [0,1,2]
        
        if self.mode == "train":
            # 训练模式：探索+利用 + 空闲小车优先
            if self.model:
                # 模型预测原始动作
                raw_action, _states = self.model.predict(obs, deterministic=False)
                # 防护1：如果有空闲小车→只在空闲小车中选
                if idle_robot_indices:
                    # 方式1：如果模型选的小车空闲→用模型的选择
                    if raw_action in idle_robot_indices:
                        action = raw_action
                    # 方式2：模型选的小车忙碌→随机选一个空闲的（保证训练不阻塞）
                    else:
                        action = random.choice(idle_robot_indices)
                        self.get_logger().debug(f"📝 训练模式：模型选忙碌小车{self.rl_env.robot_names[raw_action]}，随机选空闲小车{self.rl_env.robot_names[action]}")
                # 防护2：无空闲小车→随机选一个（保证训练继续）
                else:
                    action = random.choice(all_robot_indices)
                    self.get_logger().debug(f"📝 训练模式：无空闲小车，随机选{self.rl_env.robot_names[action]}")
            else:
                # 极端情况：模型未初始化→优先选空闲，无则随机
                if idle_robot_indices:
                    action = random.choice(idle_robot_indices)
                else:
                    action = random.choice(all_robot_indices)
                self.get_logger().debug(f"📝 训练模式：模型未初始化，选小车{self.rl_env.robot_names[action]}")
        
        elif self.mode == "infer":
            # 推理模式：保留原有逻辑（确定性预测）
            if self.model:
                action, _states = self.model.predict(obs, deterministic=True)
                # 新增防护：推理时也优先选空闲
                if idle_robot_indices and action not in idle_robot_indices:
                    action = random.choice(idle_robot_indices)
                    self.get_logger().warn(f"⚠️ 推理模式：模型选忙碌小车，随机选空闲小车{self.rl_env.robot_names[action]}")
            else:
                # 降级：随机选空闲小车，无则随机
                action = random.choice(idle_robot_indices) if idle_robot_indices else random.choice(all_robot_indices)
        else:
            self.get_logger().error(f"❌ 无效模式：{self.mode}")
            return
        
        # 4. 执行动作（选车）
        selected_robot = self.rl_env.robot_names[action]
        # 日志：记录选车结果
        self.get_logger().info(f"🎯 决策结果：选中{selected_robot}（空闲状态：{self.rl_env.robot_idle[selected_robot]}）")
        
        # 5. 获取待执行任务
        task = self.rl_env.pending_tasks[0]
        task_id = task["task_id"]
        waypoint = task["waypoint"]
        
        # 6. 调用自定义任务服务（即使小车忙碌也发送，保证训练有反馈）
        self._call_nav_service(selected_robot, task_id, waypoint)


    def _call_nav_service(self, robot_name, task_id, waypoint):
        """调用自定义任务服务发送任务"""
        req = SingleNavTask.Request()
        req.target_waypoint = waypoint
        req.fleet_name = "deliveryRobot"
        req.robot_name = robot_name
        req.priority = 0
        
        # 异步调用服务
        future = self.nav_client.call_async(req)
        future.add_done_callback(lambda f: self._nav_service_response_callback(f, task_id, robot_name))
        
        self.get_logger().info(f"📤 发送任务：{robot_name} -> {waypoint} (任务ID：{task_id})")

    def _nav_service_response_callback(self, future, task_id, robot_name):
        """处理服务响应"""
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"✅ 任务{task_id}发送成功！Task ID: {res.task_id}")
                # 标记任务为执行中（RL环境）
                self.rl_env.robot_task_map[robot_name] = task_id
            else:
                self.get_logger().error(f"❌ 任务{task_id}发送失败！{res.message}")
                # 任务失败：放回队列
                target_waypoint = next(t["waypoint"] for t in self.rl_env.pending_tasks if t["task_id"] == task_id)
                target_coords = self.waypoint_coords[target_waypoint]
                self.rl_env.add_task(task_id, target_waypoint, target_coords.x, target_coords.y)
        except Exception as e:
            self.get_logger().error(f"💥 服务调用异常：{str(e)}")

    def start_training(self, total_timesteps=100000):
        """开始训练（训练模式下调用）"""
        if self.mode != "train":
            self.get_logger().error("❌ 非训练模式，无法开始训练")
            return
        
        self.get_logger().info(f"🚀 开始训练，总步数：{total_timesteps}")
        # 开始训练
        self.model.learn(
            total_timesteps=total_timesteps,
            callback=self.checkpoint_callback,
            tb_log_name=f"dispatching_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        )
        # 保存最终模型
        self.model.save("./rl_models/dispatching_ppo_final")
        self.get_logger().info("✅ 训练完成，模型已保存为dispatching_ppo_final")

# ===================== 第三步：主函数（支持命令行参数） =====================
def main(args=None):
    rclpy.init(args=args)
    
    # 创建节点
    node = RLDispatcherNode()
    
    # 训练模式：启动训练
    if node.mode == "train":
        # 先重置RL环境（适配新的reset接口）
        node.rl_env.reset()
        
        # 启动训练线程（关键：不阻塞ROS spin）
        import threading
        train_thread = threading.Thread(
            target=node.start_training,
            args=(100000,),  # 总训练步数
            daemon=True
        )
        train_thread.start()
        
        # 主线程运行ROS spin
        rclpy.spin(node)
        
        # 等待训练线程结束
        train_thread.join()
    else:
        # 推理模式：直接spin
        rclpy.spin(node)
    
    # 清理
    node.rl_env.close()  # 关闭环境
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
