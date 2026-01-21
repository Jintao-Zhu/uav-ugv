import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
# 1.21 下面注释的是昨天zjt改的，上面是我把状态改成45维的，跑出的结果还是一样，分了六个任务给小车1
import rclpy
from rclpy.node import Node
from rmf_fleet_msgs.msg import FleetState, RobotMode  # 新增：导入RobotMode枚举
from std_msgs.msg import String
from geometry_msgs.msg import Point
from rmf_custom_tasks_self.srv import SingleNavTask
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback, StopTrainingOnRewardThreshold
from stable_baselines3.common.evaluation import evaluate_policy
import time
import random
import threading
import torch as th

# ========== 彻底消除Gym警告 ==========
import warnings
warnings.filterwarnings("ignore")
import gymnasium as gym
from gymnasium import spaces

# 引入模拟环境
try:
    from mock_env import MockRMFEnv
except ImportError:
    print("❌ 错误：找不到 mock_env.py，请确保文件路径正确！")
    sys.exit(1)

# ===================== 真实 ROS 环境 =====================
class RLDispatchingEnv(gym.Env):
    """
    真实RMF环境：匹配Mock逻辑，8任务必须完成+无待命点返回
    """
    metadata = {"render_modes": ["human"], "render_fps": 1}
    
    def __init__(self, node, render_mode=None):
        super().__init__()
        self.node = node
        self.render_mode = render_mode
        
        # 1. 小车配置（无待命点返回）
        self.robot_names = ["deliveryRobot_0", "deliveryRobot_1", "deliveryRobot_2"]
        self.robot_positions = {
            "deliveryRobot_0": Point(x=96.59527587890625, y=-51.96450424194336),
            "deliveryRobot_1": Point(x=152.3477325439453, y=-44.31863021850586),
            "deliveryRobot_2": Point(x=14.776845932006836, y=-9.279278755187988)
        }
        self.robot_idle = {name: True for name in self.robot_names}
        self.robot_task_map = {}
        self.robot_remaining_time = {name: 0.0 for name in self.robot_names}
        
        # 2. 任务队列（保留所有任务，不丢弃）
        self.pending_tasks = []
        self.executing_tasks = {}
        self.completed_tasks = []
        self.task_timeout_count = {}  # 记录任务超时次数，避免重复惩罚
        
        # 3. 统计变量
        self.current_time = 0.0
        self.episode_steps = 0
        self.max_episode_steps = 1000
        self.total_reward = 0.0
        
        # 4. 状态空间（恢复45维，匹配训练好的模型）
        self.action_space = spaces.Discrete(len(self.robot_names) + 1)
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(45,), dtype=np.float32)
        
        # 5. 奖励系数（与Mock完全对齐：下调尺度后）
        self.reward_coeff = {
            "distance": -0.02,
            "completion": 30.0,
            "batch_completion": 20.0,
            "idle_selection": -1.0,
            "timeout": -5.0,
            "step": -0.001,
            "wait_short": 0.2,
            "wait_long": -0.1,
            "all_completed": 100.0,
            "valid_selection": 1.0
        }

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.robot_idle = {name: True for name in self.robot_names}
        self.robot_task_map = {}
        self.robot_remaining_time = {name: 0.0 for name in self.robot_names}
        self.pending_tasks = []
        self.executing_tasks = {}
        self.completed_tasks = []
        self.task_timeout_count = {}
        self.current_time = 0.0
        self.episode_steps = 0
        self.total_reward = 0.0
        return self._get_observation(), {"total_reward": 0.0}

    def _get_observation(self):
        """恢复45维状态向量（匹配训练模型的输入维度）"""
        # 1. 机器人状态（12维）
        robot_obs = []
        for name in self.robot_names:
            robot_obs.extend([
                self.robot_positions[name].x / 200.0,
                self.robot_positions[name].y / 100.0,
                1.0 if self.robot_idle[name] else 0.0,
                min(self.robot_remaining_time[name] / 200.0, 1.0)
            ])
        
        # 2. 任务状态（24维：8个任务）
        task_obs = []
        for _ in range(8):
            task_obs.extend([0.0, 0.0, 0.0])
        
        for i, task in enumerate(self.pending_tasks[:8]):
            task_obs[i*3] = task["x"] / 200.0
            task_obs[i*3+1] = task["y"] / 100.0
            task_obs[i*3+2] = min(task["wait_time"] / 200.0, 1.0)
        
        # 3. 全局状态（9维：恢复删除的2个特征，总维度回到45）
        idle_robot_count = sum([1 for name in self.robot_names if self.robot_idle[name]])
        robot_remaining_times = [min(self.robot_remaining_time[name]/200.0, 1.0) for name in self.robot_names]
        global_obs = [
            self.current_time / 1000.0,          # 1
            len(self.completed_tasks) / 8.0,     # 2
            len(self.pending_tasks) / 8.0,       # 3
            idle_robot_count / 3.0,              # 4
            len(self.executing_tasks) / 8.0,     # 5（恢复删除的特征）
            sum(robot_remaining_times) / 3.0,    # 6（恢复删除的特征）
            *robot_remaining_times               # 7-9（3个机器人剩余时间）
        ]
        
        # 总维度：12 + 24 + 9 = 45（匹配训练模型）
        return np.array(robot_obs + task_obs + global_obs, dtype=np.float32)

    def step(self, action):
        self.episode_steps += 1
        self.current_time += 1.0
        reward = self.reward_coeff["step"]
        
        # 奖励裁剪函数
        def clip_reward(r, min_r=-10.0, max_r=50.0):
            return max(min(r, max_r), min_r)
        
        wait_action_idx = len(self.robot_names)
        info = {
            "action_type": "wait" if action == wait_action_idx else "assign",
            "selected_action": action,
            "idle_robots": sum([1 for name in self.robot_names if self.robot_idle[name]])
        }
        
        # --- 处理等待动作 ---
        if action == wait_action_idx:
            if self.pending_tasks:
                # 更新任务等待时间
                for task in self.pending_tasks:
                    task["wait_time"] += 1.0
                
                # 等待奖励/惩罚
                first_task_wait = self.pending_tasks[0]["wait_time"]
                if first_task_wait < 10.0:
                    reward += self.reward_coeff["wait_short"]
                    reward = clip_reward(reward)
                elif first_task_wait > 30.0:
                    reward += self.reward_coeff["wait_long"]
                    reward = clip_reward(reward)
                
                # 超时处理：仅惩罚，不丢弃任务
                idle_robot_count = sum([1 for name in self.robot_names if self.robot_idle[name]])
                if idle_robot_count > 0:
                    for task in self.pending_tasks:
                        tid = task["task_id"]
                        if task["wait_time"] > 120.0 and self.task_timeout_count.get(tid, 0) == 0:
                            reward += self.reward_coeff["timeout"]
                            reward = clip_reward(reward)
                            self.task_timeout_count[tid] = 1
        
        # --- 处理选车动作 ---
        else:
            selected_robot = self.robot_names[action]
            info["selected_robot"] = selected_robot
            
            if not self.pending_tasks:
                truncated = self.episode_steps >= self.max_episode_steps
                return self._get_observation(), reward, False, truncated, info
            
            # 选距离最近的任务
            if self.robot_idle[selected_robot]:
                # 选空闲车的正向奖励
                reward += self.reward_coeff["valid_selection"]
                reward = clip_reward(reward)
                
                min_dist = float("inf")
                best_task_idx = 0
                for i, task in enumerate(self.pending_tasks):
                    rx = self.robot_positions[selected_robot].x
                    ry = self.robot_positions[selected_robot].y
                    dist = np.sqrt((rx - task["x"])**2 + (ry - task["y"])**2)
                    if dist < min_dist:
                        min_dist = dist
                        best_task_idx = i
                
                task = self.pending_tasks.pop(best_task_idx)
                task_id = task["task_id"]
                info["task_id"] = task_id
                
                # 极低的距离惩罚
                reward += self.reward_coeff["distance"] * min_dist
                reward = clip_reward(reward)
                
                # 计算剩余执行时间
                self.robot_remaining_time[selected_robot] = dist / 1.0
                
                # 更新状态
                self.robot_idle[selected_robot] = False
                self.robot_task_map[selected_robot] = task_id
                self.executing_tasks[task_id] = {
                    "robot": selected_robot,
                    "start_time": self.current_time,
                    "waypoint": task["waypoint"],
                    "distance": min_dist
                }
            else:
                # 选忙碌车：极轻惩罚
                reward += self.reward_coeff["idle_selection"]
                reward = clip_reward(reward)
        
        # --- 完成任务奖励 ---
        self.total_reward += reward
        truncated = self.episode_steps >= self.max_episode_steps
        
        return self._get_observation(), reward, False, truncated, info

    # 辅助方法：ROS 数据更新
    def update_robot_position(self, name, x, y):
        if name in self.robot_positions:
            self.robot_positions[name].x = x
            self.robot_positions[name].y = y
            
    def add_task(self, tid, wp, x, y):
        # 新增任务时初始化超时计数
        self.pending_tasks.append({
            "task_id": tid, 
            "waypoint": wp, 
            "x": x, 
            "y": y, 
            "wait_time": 0.0
        })
        self.task_timeout_count[tid] = 0
        
    def complete_task(self, tid):
        if tid in self.executing_tasks:
            robot = self.executing_tasks[tid]["robot"]
            # 任务完成核心奖励
            self.total_reward += self.reward_coeff["completion"]
            self.completed_tasks.append(tid)
            # 重置小车状态（停在任务点，不返回待命点）
            self.robot_idle[robot] = True
            self.robot_remaining_time[robot] = 0.0
            del self.executing_tasks[tid]
            self.node.get_logger().info(f"💰 任务完成奖励 +{self.reward_coeff['completion']}! (Robot: {robot})")
            
            # 完成所有8个任务：超大奖励
            if len(self.completed_tasks) == 8:
                self.total_reward += self.reward_coeff["all_completed"]
                self.node.get_logger().info(f"🎉 完成所有8个任务！额外奖励 +{self.reward_coeff['all_completed']}")

# ===================== 核心调度节点 =====================
class RLDispatcherNode(Node):
    def __init__(self):
        super().__init__("rl_dispatcher_node")
        self.get_logger().info("🚀 初始化 RL 调度节点（8任务必须完成版）...")
        
        # 1. 仿真时间配置
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])

        # 2. 模式配置
        if not self.has_parameter("mode"):
            self.declare_parameter("mode", "train")
        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.get_logger().info(f"🔵 当前模式: [{self.mode.upper()}]")
        
        # 3. 双引擎切换
        if self.mode == "train":
            self.get_logger().info("🚀 训练模式 - 挂载 Mock 虚拟引擎")
            self.rl_env = MockRMFEnv() 
        elif self.mode == "infer":
            self.get_logger().info("🤖 推理模式 - 挂载 Real 真实引擎")
            self.rl_env = RLDispatchingEnv(self, render_mode="human")
        else:
            self.get_logger().error("❌ 未知模式！请使用 mode:=train 或 mode:=infer")
            sys.exit(1)

        # 4. 初始化 RL 模型（调优超参数，解决训练震荡）
        self.model = None
        self._init_rl_model()
        
        # 5. 推理模式初始化 ROS 接口
        if self.mode == "infer":
            self._init_ros_interfaces()

    def _init_ros_interfaces(self):
        """初始化 ROS 订阅/发布/服务"""
        self.get_logger().info("🔌 连接 ROS 接口...")
        
        # 航点坐标映射
        self.waypoint_coords = {
            "n14": Point(x=80.84, y=-28.52), "n13": Point(x=84.44, y=-4.94),
            "n23": Point(x=182.80, y=-42.30), "s08": Point(x=96.61, y=-50.50),
            "s10": Point(x=122.10, y=-46.68), "west_koi_pond": Point(x=34.32, y=-10.13),
            "n08": Point(x=59.61, y=-7.42), "junction_south_west": Point(x=84.56, y=-38.81)
        }

        # 订阅器（仅保留你已验证可用的）
        self.create_subscription(FleetState, "/fleet_states", self.fleet_state_callback, 10)
        self.create_subscription(String, "/task_monitor/start", self.target_callback, 10)
        self.create_subscription(String, "/custom_task_completion", self.completion_callback, 10)
        
        # 服务客户端
        self.nav_client = self.create_client(SingleNavTask, "/submit_single_nav_task")
        
        # 决策定时器（1秒/次）
        self.create_timer(1.0, self.infer_dispatch_callback)
        
        # 去重缓存
        self.processed_ids = set()
        
        # 新增：记录正在路上的请求，防止重复提交
        self.dispatching_task_ids = set() 
        
        self.get_logger().info("✅ ROS 接口连接完成")

    def _init_rl_model(self):
        """初始化 PPO 模型（调优超参数，解决KL散度过低、奖励震荡问题）"""
        model_path = "./rl_models/ppo_800000_steps"
        log_dir = "./rl_dispatching_tb_log/"
        
        if self.mode == "train":
            # 轻量化网络（避免过拟合）
            policy_kwargs = dict(
                activation_fn=th.nn.ReLU,
                net_arch=dict(pi=[128, 128], vf=[128, 128])
            )
            self.get_logger().info("📚 初始化 PPO 模型（调优超参数版）...")
            self.model = PPO(
                "MlpPolicy", 
                self.rl_env, 
                verbose=1, 
                device='cpu', 
                tensorboard_log=log_dir,
                policy_kwargs=policy_kwargs,
                learning_rate=1e-4,        # 从3e-5→1e-4：提高学习率，增强策略更新
                n_steps=2048,              # 保持不变
                batch_size=64,             # 保持不变
                gamma=0.99,                # 保持不变
                gae_lambda=0.95,           # 保持不变
                ent_coef=0.05,             # 从0.01→0.05：提高熵系数，增强探索
                vf_coef=0.5,               # 保持不变
                max_grad_norm=0.5,         # 保持不变
                n_epochs=10,               # 保持不变
                clip_range=0.3             # 从0.2→0.3：扩大裁剪范围，解决KL散度过低
            )
            # 回调函数：监控收敛，保存最优模型
            self.checkpoint_callback = CheckpointCallback(save_freq=20000, save_path="./rl_models/", name_prefix="ppo")
            
            # 奖励阈值停止回调（作为EvalCallback的子回调）
            self.stop_callback = StopTrainingOnRewardThreshold(
                reward_threshold=20000.0,  # 设定合理奖励阈值
                verbose=1
            )
            
            # EvalCallback：将停止回调作为子回调传入
            self.eval_callback = EvalCallback(
                self.rl_env,
                eval_freq=5000,
                n_eval_episodes=5,
                best_model_save_path="./rl_models/best/",
                verbose=1,
                callback_after_eval=self.stop_callback
            )
            
        elif self.mode == "infer":
            if os.path.exists(model_path + ".zip"):
                self.get_logger().info(f"📂 加载模型: {model_path}")
                self.model = PPO.load(model_path, device='cpu')
            else:
                self.get_logger().error(f"❌ 未找到模型文件: {model_path}.zip")
                self.model = None

    # 训练逻辑
    def start_training(self, total_timesteps=1000000):
        if self.mode != "train": return
            
        self.get_logger().info(f"🔥 开始训练 {total_timesteps} 步（8任务必须完成版）...")
        start_t = time.time()
        
        self.model.learn(
            total_timesteps=total_timesteps, 
            callback=[self.checkpoint_callback, self.eval_callback]
        )
        
        # 评估最终模型
        mean_reward, std_reward = evaluate_policy(self.model, self.rl_env, n_eval_episodes=10)
        self.get_logger().info(f"📊 最终模型评估：平均奖励={mean_reward:.2f}，标准差={std_reward:.2f}")
        
        self.model.save("./rl_models/dispatching_ppo_final")
        self.get_logger().info("💾 模型已保存！")
        rclpy.shutdown()

    # 推理逻辑（核心：过滤已执行/正在提交的任务，防止重复）
    def infer_dispatch_callback(self):
        # 过滤条件：仅保留未执行、未提交的red_cube任务
        valid_tasks = [
            t for t in self.rl_env.pending_tasks 
            if t["task_id"].startswith("red_cube_") 
            and t["task_id"] not in self.rl_env.executing_tasks
            and t["task_id"] not in self.dispatching_task_ids
        ]
        if not valid_tasks:
            self.get_logger().debug("📭 无有效任务，跳过调度")
            return
        
        # 模型决策
        obs = self.rl_env._get_observation()
        action = 0
        if self.model:
            action, _ = self.model.predict(obs, deterministic=True)
        
        wait_action_idx = len(self.rl_env.robot_names)
        
        # 处理等待动作
        if action == wait_action_idx:
            self.get_logger().info("⏳ 模型决策：等待新任务（凑单后批量调度）")
            for task in self.rl_env.pending_tasks:
                task["wait_time"] += 1.0
            return
        
        # 处理选车动作
        selected_robot = self.rl_env.robot_names[action]
        
        # 安全降级：选忙碌车则切换到空闲车
        if not self.rl_env.robot_idle[selected_robot]:
            idle_indices = [i for i, name in enumerate(self.rl_env.robot_names) if self.rl_env.robot_idle[name]]
            if idle_indices:
                action = random.choice(idle_indices)
                selected_robot = self.rl_env.robot_names[action]
                self.get_logger().warn(f"🛡️ 修正：选空闲车 {selected_robot}")
        
        # 选距离最近的任务
        min_dist = float("inf")
        best_task = None
        for task in valid_tasks:
            rx = self.rl_env.robot_positions[selected_robot].x
            ry = self.rl_env.robot_positions[selected_robot].y
            dist = np.sqrt((rx - task["x"])**2 + (ry - task["y"])**2)
            if dist < min_dist:
                min_dist = dist
                best_task = task
        
        if best_task:
            tid = best_task["task_id"]
            
            # 防抖：正在提交的任务跳过
            if tid in self.dispatching_task_ids:
                self.get_logger().debug(f"⏳ 任务 {tid} 正在请求中，跳过...")
                return

            # 标记为正在提交
            self.dispatching_task_ids.add(tid)
            
            # 调用服务下发任务
            self._call_ros_service(selected_robot, tid, best_task["waypoint"])

    def _call_ros_service(self, robot, tid, wp):
        if not self.nav_client.service_is_ready():
            self.get_logger().warn("⚠️ 服务未就绪，跳过任务下发")
            # 移除标记，避免永久锁定
            self.dispatching_task_ids.discard(tid)
            return
            
        req = SingleNavTask.Request()
        req.target_waypoint = wp
        req.fleet_name = "deliveryRobot"
        req.robot_name = robot
        req.priority = 1
        
        future = self.nav_client.call_async(req)
        future.add_done_callback(lambda f: self._service_done(f, tid, robot))

    def _service_done(self, future, tid, robot):
        """修复：无论成功失败，只要有结果了，就从“发送中”名单移除，并防止字符串切割报错"""
        self.dispatching_task_ids.discard(tid)

        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"🚀 调度成功：{robot} -> {tid} @ {res.message}")
                
                # 遍历 pending_tasks 找到对应的任务对象
                target_task_idx = -1
                for i, t in enumerate(self.rl_env.pending_tasks):
                    if t["task_id"] == tid:
                        target_task_idx = i
                        break
                
                if target_task_idx != -1:
                    # 取出任务数据
                    task_data = self.rl_env.pending_tasks.pop(target_task_idx)
                    
                    # 更新状态
                    self.rl_env.robot_idle[robot] = False
                    self.rl_env.robot_task_map[robot] = tid
                    
                    # 核心修复：直接使用 task_data 中的 waypoint，不要 split ID
                    wp_name = task_data["waypoint"] 
                    
                    # 计算距离
                    if wp_name in self.waypoint_coords:
                        target_point = self.waypoint_coords[wp_name]
                        current_pos = self.rl_env.robot_positions[robot]
                        dist = np.sqrt((current_pos.x - target_point.x)**2 + (current_pos.y - target_point.y)**2)
                    else:
                        dist = 0.0 # 兜底

                    self.rl_env.executing_tasks[tid] = {
                        "robot": robot,
                        "start_time": self.rl_env.current_time,
                        "waypoint": self.waypoint_coords.get(wp_name, Point()), # 防止空指针
                        "distance": dist
                    }
            else:
                self.get_logger().error(f"❌ 调度失败：{tid} -> {res.message}")
        except Exception as e:
            self.get_logger().error(f"❌ 服务调用异常：{e}")
            import traceback
            traceback.print_exc()

    def fleet_state_callback(self, msg):
        """核心修复：基于你实际的 RobotState 字段判断空闲状态"""
        if msg.name != "deliveryRobot": return
        for r in msg.robots:
            if r.name in self.rl_env.robot_names:
                # 1. 更新小车位置（保留）
                self.rl_env.update_robot_position(r.name, r.location.x, r.location.y)
                
                # 2. 修复：基于实际字段判断空闲状态（无task_state，用mode+task_id）
                # RMF RobotMode枚举值：1=IDLE, 2=MOVING, 3=PAUSED, 4=CHARGING
                is_idle = (r.mode.mode == RobotMode.MODE_IDLE) and (not r.task_id) and (r.battery_percent > 0.0)
                self.rl_env.robot_idle[r.name] = is_idle
                
                # 3. 同步剩余执行时间（保留）
                if not is_idle and r.task_id:
                    if r.name in self.rl_env.robot_task_map:
                        tid = self.rl_env.robot_task_map[r.name]
                        if tid in self.rl_env.executing_tasks:
                            wp = self.rl_env.executing_tasks[tid]["waypoint"]
                            current_pos = Point(x=r.location.x, y=r.location.y)
                            dist = np.sqrt((current_pos.x - wp.x)**2 + (current_pos.y - wp.y)**2)
                            self.rl_env.robot_remaining_time[r.name] = dist / 1.0
                else:
                    self.rl_env.robot_remaining_time[r.name] = 0.0

    def target_callback(self, msg):
        """处理任务发布话题，新增任务到待执行队列"""
        data = msg.data.split(",")
        if len(data) < 2:
            self.get_logger().warn(f"⚠️ 无效任务格式：{msg.data}")
            return
        
        tid, wp = data[0].strip(), data[1].strip()
        # 去重：已处理的任务不再添加
        if not tid.startswith("red_cube_") or tid in self.processed_ids:
            return
        
        if wp in self.waypoint_coords:
            p = self.waypoint_coords[wp]
            self.rl_env.add_task(tid, wp, p.x, p.y)
            self.processed_ids.add(tid)
            self.get_logger().info(f"📥 新增任务：{tid} @ {wp}")
        else:
            self.get_logger().error(f"❌ 未知航点：{wp}，任务{tid}添加失败")

    def completion_callback(self, msg):
        """处理任务完成话题，重置小车状态"""
        data = msg.data.split(",")
        if len(data) < 1:
            self.get_logger().warn(f"⚠️ 无效的完成消息：{msg.data}")
            return
        tid = data[0].strip()
        
        if tid.startswith("red_cube_"):
            # 调用环境的完成任务方法
            self.rl_env.complete_task(tid)
            # 清理缓存
            if tid in self.processed_ids:
                self.processed_ids.remove(tid)
            # 强制重置对应小车状态（兜底）
            for robot in self.rl_env.robot_names:
                if self.rl_env.robot_task_map.get(robot, "") == tid:
                    self.rl_env.robot_idle[robot] = True
                    self.rl_env.robot_remaining_time[robot] = 0.0
                    del self.rl_env.robot_task_map[robot]
                    break

# 主程序
def main(args=None):
    rclpy.init(args=args)
    node = RLDispatcherNode()
    
    if node.mode == "train":
        train_thread = threading.Thread(
            target=node.start_training, 
            args=(1000000,),
            daemon=True
        )
        train_thread.start()
        try:
            rclpy.spin(node)
        except SystemExit:
            pass
        train_thread.join()
    else:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()

# #!/usr/bin/env python3
# #1.20，zjt改
# # 适配需求：8任务必须完成+无待命点返回+超时不丢任务
# # 核心优化：下调奖励系数、调优PPO超参数，解决训练震荡问题
# # 修改rl节点，确保其和rmf正常交互，保证gazebo里小车的逻辑是按照rl的决策结果来的
# import sys
# import os
# sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# import rclpy
# from rclpy.node import Node
# from rmf_fleet_msgs.msg import FleetState, RobotMode  # 新增：导入RobotMode枚举
# from std_msgs.msg import String
# from geometry_msgs.msg import Point
# from rmf_custom_tasks_self.srv import SingleNavTask
# import numpy as np
# from stable_baselines3 import PPO
# from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback, StopTrainingOnRewardThreshold
# from stable_baselines3.common.evaluation import evaluate_policy
# import time
# import random
# import threading
# import torch as th

# # ========== 彻底消除Gym警告 ==========
# import warnings
# warnings.filterwarnings("ignore")
# import gymnasium as gym
# from gymnasium import spaces

# # 引入模拟环境
# try:
#     from mock_env import MockRMFEnv
# except ImportError:
#     print("❌ 错误：找不到 mock_env.py，请确保文件路径正确！")
#     sys.exit(1)

# # ===================== 真实 ROS 环境 =====================
# class RLDispatchingEnv(gym.Env):
#     """
#     真实RMF环境：匹配Mock逻辑，8任务必须完成+无待命点返回
#     """
#     metadata = {"render_modes": ["human"], "render_fps": 1}
    
#     def __init__(self, node, render_mode=None):
#         super().__init__()
#         self.node = node
#         self.render_mode = render_mode
        
#         # 1. 小车配置（无待命点返回）
#         self.robot_names = ["deliveryRobot_0", "deliveryRobot_1", "deliveryRobot_2"]
#         self.robot_positions = {
#             "deliveryRobot_0": Point(x=96.59527587890625, y=-51.96450424194336),
#             "deliveryRobot_1": Point(x=152.3477325439453, y=-44.31863021850586),
#             "deliveryRobot_2": Point(x=14.776845932006836, y=-9.279278755187988)
#         }
#         self.robot_idle = {name: True for name in self.robot_names}
#         self.robot_task_map = {}
#         self.robot_remaining_time = {name: 0.0 for name in self.robot_names}
        
#         # 2. 任务队列（保留所有任务，不丢弃）
#         self.pending_tasks = []
#         self.executing_tasks = {}
#         self.completed_tasks = []
#         self.task_timeout_count = {}  # 记录任务超时次数，避免重复惩罚
        
#         # 3. 统计变量
#         self.current_time = 0.0
#         self.episode_steps = 0
#         self.max_episode_steps = 1000
#         self.total_reward = 0.0
        
#         # 4. 状态空间（与Mock一致：45维 -> 修复为43维适配旧模型）
#         self.action_space = spaces.Discrete(len(self.robot_names) + 1)
#         # 【修改点1】将 shape=(45,) 改回 shape=(43,)
#         self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(43,), dtype=np.float32)
        
#         # 5. 奖励系数（与Mock完全对齐：下调尺度后）
#         self.reward_coeff = {
#             "distance": -0.02,
#             "completion": 30.0,
#             "batch_completion": 20.0,
#             "idle_selection": -1.0,
#             "timeout": -5.0,
#             "step": -0.001,
#             "wait_short": 0.2,
#             "wait_long": -0.1,
#             "all_completed": 100.0,
#             "valid_selection": 1.0
#         }

#     def reset(self, seed=None, options=None):
#         super().reset(seed=seed)
#         self.robot_idle = {name: True for name in self.robot_names}
#         self.robot_task_map = {}
#         self.robot_remaining_time = {name: 0.0 for name in self.robot_names}
#         self.pending_tasks = []
#         self.executing_tasks = {}
#         self.completed_tasks = []
#         self.task_timeout_count = {}
#         self.current_time = 0.0
#         self.episode_steps = 0
#         self.total_reward = 0.0
#         return self._get_observation(), {"total_reward": 0.0}

#     def _get_observation(self):
#         """【修改点2】回退到43维状态向量，适配旧模型"""
#         # 1. 机器人状态（12维）
#         robot_obs = []
#         for name in self.robot_names:
#             robot_obs.extend([
#                 self.robot_positions[name].x / 200.0,
#                 self.robot_positions[name].y / 100.0,
#                 1.0 if self.robot_idle[name] else 0.0,
#                 min(self.robot_remaining_time[name] / 200.0, 1.0)
#             ])
        
#         # 2. 任务状态（24维：8个任务）
#         task_obs = []
#         for _ in range(8):
#             task_obs.extend([0.0, 0.0, 0.0])
        
#         for i, task in enumerate(self.pending_tasks[:8]):
#             task_obs[i*3] = task["x"] / 200.0
#             task_obs[i*3+1] = task["y"] / 100.0
#             task_obs[i*3+2] = min(task["wait_time"] / 200.0, 1.0)
        
#         # 3. 全局状态（7维，删除了新增的2个特征）
#         idle_robot_count = sum([1 for name in self.robot_names if self.robot_idle[name]])
#         robot_remaining_times = [min(self.robot_remaining_time[name]/200.0, 1.0) for name in self.robot_names]
#         global_obs = [
#             self.current_time / 1000.0,          # 1
#             len(self.completed_tasks) / 8.0,     # 2
#             len(self.pending_tasks) / 8.0,       # 3
#             idle_robot_count / 3.0,              # 4
#             # len(self.executing_tasks) / 8.0,   # <--- 已删除
#             # sum(robot_remaining_times) / 3.0,  # <--- 已删除
#             *robot_remaining_times               # 5-7（3个机器人剩余时间）
#         ]
        
#         # 总维度：12 + 24 + 7 = 43
#         return np.array(robot_obs + task_obs + global_obs, dtype=np.float32)

#     def step(self, action):
#         self.episode_steps += 1
#         self.current_time += 1.0
#         reward = self.reward_coeff["step"]
        
#         # 奖励裁剪函数
#         def clip_reward(r, min_r=-10.0, max_r=50.0):
#             return max(min(r, max_r), min_r)
        
#         wait_action_idx = len(self.robot_names)
#         info = {
#             "action_type": "wait" if action == wait_action_idx else "assign",
#             "selected_action": action,
#             "idle_robots": sum([1 for name in self.robot_names if self.robot_idle[name]])
#         }
        
#         # --- 处理等待动作 ---
#         if action == wait_action_idx:
#             if self.pending_tasks:
#                 # 更新任务等待时间
#                 for task in self.pending_tasks:
#                     task["wait_time"] += 1.0
                
#                 # 等待奖励/惩罚
#                 first_task_wait = self.pending_tasks[0]["wait_time"]
#                 if first_task_wait < 10.0:
#                     reward += self.reward_coeff["wait_short"]
#                     reward = clip_reward(reward)
#                 elif first_task_wait > 30.0:
#                     reward += self.reward_coeff["wait_long"]
#                     reward = clip_reward(reward)
                
#                 # 超时处理：仅惩罚，不丢弃任务
#                 idle_robot_count = sum([1 for name in self.robot_names if self.robot_idle[name]])
#                 if idle_robot_count > 0:
#                     for task in self.pending_tasks:
#                         tid = task["task_id"]
#                         if task["wait_time"] > 120.0 and self.task_timeout_count.get(tid, 0) == 0:
#                             reward += self.reward_coeff["timeout"]
#                             reward = clip_reward(reward)
#                             self.task_timeout_count[tid] = 1
        
#         # --- 处理选车动作 ---
#         else:
#             selected_robot = self.robot_names[action]
#             info["selected_robot"] = selected_robot
            
#             if not self.pending_tasks:
#                 truncated = self.episode_steps >= self.max_episode_steps
#                 return self._get_observation(), reward, False, truncated, info
            
#             # 选距离最近的任务
#             if self.robot_idle[selected_robot]:
#                 # 选空闲车的正向奖励
#                 reward += self.reward_coeff["valid_selection"]
#                 reward = clip_reward(reward)
                
#                 min_dist = float("inf")
#                 best_task_idx = 0
#                 for i, task in enumerate(self.pending_tasks):
#                     rx = self.robot_positions[selected_robot].x
#                     ry = self.robot_positions[selected_robot].y
#                     dist = np.sqrt((rx - task["x"])**2 + (ry - task["y"])**2)
#                     if dist < min_dist:
#                         min_dist = dist
#                         best_task_idx = i
                
#                 task = self.pending_tasks.pop(best_task_idx)
#                 task_id = task["task_id"]
#                 info["task_id"] = task_id
                
#                 # 极低的距离惩罚
#                 reward += self.reward_coeff["distance"] * min_dist
#                 reward = clip_reward(reward)
                
#                 # 计算剩余执行时间
#                 self.robot_remaining_time[selected_robot] = dist / 1.0
                
#                 # 更新状态
#                 self.robot_idle[selected_robot] = False
#                 self.robot_task_map[selected_robot] = task_id
#                 self.executing_tasks[task_id] = {
#                     "robot": selected_robot,
#                     "start_time": self.current_time,
#                     "waypoint": task["waypoint"],
#                     "distance": min_dist
#                 }
#             else:
#                 # 选忙碌车：极轻惩罚
#                 reward += self.reward_coeff["idle_selection"]
#                 reward = clip_reward(reward)
        
#         # --- 完成任务奖励 ---
#         self.total_reward += reward
#         truncated = self.episode_steps >= self.max_episode_steps
        
#         return self._get_observation(), reward, False, truncated, info

#     # 辅助方法：ROS 数据更新
#     def update_robot_position(self, name, x, y):
#         if name in self.robot_positions:
#             self.robot_positions[name].x = x
#             self.robot_positions[name].y = y
            
#     def add_task(self, tid, wp, x, y):
#         # 新增任务时初始化超时计数
#         self.pending_tasks.append({
#             "task_id": tid, 
#             "waypoint": wp, 
#             "x": x, 
#             "y": y, 
#             "wait_time": 0.0
#         })
#         self.task_timeout_count[tid] = 0
        
#     def complete_task(self, tid):
#         if tid in self.executing_tasks:
#             robot = self.executing_tasks[tid]["robot"]
#             # 任务完成核心奖励
#             self.total_reward += self.reward_coeff["completion"]
#             self.completed_tasks.append(tid)
#             # 重置小车状态（停在任务点，不返回待命点）
#             self.robot_idle[robot] = True
#             self.robot_remaining_time[robot] = 0.0
#             del self.executing_tasks[tid]
#             self.node.get_logger().info(f"💰 任务完成奖励 +{self.reward_coeff['completion']}! (Robot: {robot})")
            
#             # 完成所有8个任务：超大奖励
#             if len(self.completed_tasks) == 8:
#                 self.total_reward += self.reward_coeff["all_completed"]
#                 self.node.get_logger().info(f"🎉 完成所有8个任务！额外奖励 +{self.reward_coeff['all_completed']}")

# # ===================== 核心调度节点 =====================
# class RLDispatcherNode(Node):
#     def __init__(self):
#         super().__init__("rl_dispatcher_node")
#         self.get_logger().info("🚀 初始化 RL 调度节点（8任务必须完成版）...")
        
#         # 1. 仿真时间配置
#         if not self.has_parameter("use_sim_time"):
#             self.declare_parameter("use_sim_time", True)
#         self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])

#         # 2. 模式配置
#         if not self.has_parameter("mode"):
#             self.declare_parameter("mode", "train")
#         self.mode = self.get_parameter("mode").get_parameter_value().string_value
#         self.get_logger().info(f"🔵 当前模式: [{self.mode.upper()}]")
        
#         # 3. 双引擎切换
#         if self.mode == "train":
#             self.get_logger().info("🚀 训练模式 - 挂载 Mock 虚拟引擎")
#             self.rl_env = MockRMFEnv() 
#         elif self.mode == "infer":
#             self.get_logger().info("🤖 推理模式 - 挂载 Real 真实引擎")
#             self.rl_env = RLDispatchingEnv(self, render_mode="human")
#         else:
#             self.get_logger().error("❌ 未知模式！请使用 mode:=train 或 mode:=infer")
#             sys.exit(1)

#         # 4. 初始化 RL 模型（调优超参数，解决训练震荡）
#         self.model = None
#         self._init_rl_model()
        
#         # 5. 推理模式初始化 ROS 接口
#         if self.mode == "infer":
#             self._init_ros_interfaces()

#     def _init_ros_interfaces(self):
#         """初始化 ROS 订阅/发布/服务"""
#         self.get_logger().info("🔌 连接 ROS 接口...")
        
#         # 航点坐标映射
#         self.waypoint_coords = {
#             "n14": Point(x=80.84, y=-28.52), "n13": Point(x=84.44, y=-4.94),
#             "n23": Point(x=182.80, y=-42.30), "s08": Point(x=96.61, y=-50.50),
#             "s10": Point(x=122.10, y=-46.68), "west_koi_pond": Point(x=34.32, y=-10.13),
#             "n08": Point(x=59.61, y=-7.42), "junction_south_west": Point(x=84.56, y=-38.81)
#         }

#         # 订阅器（仅保留你已验证可用的）
#         self.create_subscription(FleetState, "/fleet_states", self.fleet_state_callback, 10)
#         self.create_subscription(String, "/task_monitor/start", self.target_callback, 10)
#         self.create_subscription(String, "/custom_task_completion", self.completion_callback, 10)
        
#         # 服务客户端
#         self.nav_client = self.create_client(SingleNavTask, "/submit_single_nav_task")
        
#         # 决策定时器（1秒/次）
#         self.create_timer(1.0, self.infer_dispatch_callback)
        
#         # 去重缓存
#         self.processed_ids = set()
        
#         # 新增：记录正在路上的请求，防止重复提交
#         self.dispatching_task_ids = set() 
        
#         self.get_logger().info("✅ ROS 接口连接完成")

#     def _init_rl_model(self):
#         """初始化 PPO 模型（调优超参数，解决KL散度过低、奖励震荡问题）"""
#         model_path = "./rl_models/ppo_800000_steps"
#         log_dir = "./rl_dispatching_tb_log/"
        
#         if self.mode == "train":
#             # 轻量化网络（避免过拟合）
#             policy_kwargs = dict(
#                 activation_fn=th.nn.ReLU,
#                 net_arch=dict(pi=[128, 128], vf=[128, 128])
#             )
#             self.get_logger().info("📚 初始化 PPO 模型（调优超参数版）...")
#             self.model = PPO(
#                 "MlpPolicy", 
#                 self.rl_env, 
#                 verbose=1, 
#                 device='cpu', 
#                 tensorboard_log=log_dir,
#                 policy_kwargs=policy_kwargs,
#                 learning_rate=1e-4,        # 从3e-5→1e-4：提高学习率，增强策略更新
#                 n_steps=2048,              # 保持不变
#                 batch_size=64,             # 保持不变
#                 gamma=0.99,                # 保持不变
#                 gae_lambda=0.95,           # 保持不变
#                 ent_coef=0.05,             # 从0.01→0.05：提高熵系数，增强探索
#                 vf_coef=0.5,               # 保持不变
#                 max_grad_norm=0.5,         # 保持不变
#                 n_epochs=10,               # 保持不变
#                 clip_range=0.3             # 从0.2→0.3：扩大裁剪范围，解决KL散度过低
#             )
#             # 回调函数：监控收敛，保存最优模型
#             self.checkpoint_callback = CheckpointCallback(save_freq=20000, save_path="./rl_models/", name_prefix="ppo")
            
#             # 奖励阈值停止回调（作为EvalCallback的子回调）
#             self.stop_callback = StopTrainingOnRewardThreshold(
#                 reward_threshold=20000.0,  # 设定合理奖励阈值
#                 verbose=1
#             )
            
#             # EvalCallback：将停止回调作为子回调传入
#             self.eval_callback = EvalCallback(
#                 self.rl_env,
#                 eval_freq=5000,
#                 n_eval_episodes=5,
#                 best_model_save_path="./rl_models/best/",
#                 verbose=1,
#                 callback_after_eval=self.stop_callback
#             )
            
#         elif self.mode == "infer":
#             if os.path.exists(model_path + ".zip"):
#                 self.get_logger().info(f"📂 加载模型: {model_path}")
#                 self.model = PPO.load(model_path, device='cpu')
#             else:
#                 self.get_logger().error(f"❌ 未找到模型文件: {model_path}.zip")
#                 self.model = None

#     # 训练逻辑
#     def start_training(self, total_timesteps=1000000):
#         if self.mode != "train": return
            
#         self.get_logger().info(f"🔥 开始训练 {total_timesteps} 步（8任务必须完成版）...")
#         start_t = time.time()
        
#         self.model.learn(
#             total_timesteps=total_timesteps, 
#             callback=[self.checkpoint_callback, self.eval_callback]
#         )
        
#         # 评估最终模型
#         mean_reward, std_reward = evaluate_policy(self.model, self.rl_env, n_eval_episodes=10)
#         self.get_logger().info(f"📊 最终模型评估：平均奖励={mean_reward:.2f}，标准差={std_reward:.2f}")
        
#         self.model.save("./rl_models/dispatching_ppo_final")
#         self.get_logger().info("💾 模型已保存！")
#         rclpy.shutdown()

#     # 推理逻辑（核心：过滤已执行/正在提交的任务，防止重复）
#     def infer_dispatch_callback(self):
#         # 过滤条件：仅保留未执行、未提交的red_cube任务
#         valid_tasks = [
#             t for t in self.rl_env.pending_tasks 
#             if t["task_id"].startswith("red_cube_") 
#             and t["task_id"] not in self.rl_env.executing_tasks
#             and t["task_id"] not in self.dispatching_task_ids
#         ]
#         if not valid_tasks:
#             self.get_logger().debug("📭 无有效任务，跳过调度")
#             return
        
#         # 模型决策
#         obs = self.rl_env._get_observation()
#         action = 0
#         if self.model:
#             action, _ = self.model.predict(obs, deterministic=True)
        
#         wait_action_idx = len(self.rl_env.robot_names)
        
#         # 处理等待动作
#         if action == wait_action_idx:
#             self.get_logger().info("⏳ 模型决策：等待新任务（凑单后批量调度）")
#             for task in self.rl_env.pending_tasks:
#                 task["wait_time"] += 1.0
#             return
        
#         # 处理选车动作
#         selected_robot = self.rl_env.robot_names[action]
        
#         # 安全降级：选忙碌车则切换到空闲车
#         if not self.rl_env.robot_idle[selected_robot]:
#             idle_indices = [i for i, name in enumerate(self.rl_env.robot_names) if self.rl_env.robot_idle[name]]
#             if idle_indices:
#                 action = random.choice(idle_indices)
#                 selected_robot = self.rl_env.robot_names[action]
#                 self.get_logger().warn(f"🛡️ 修正：选空闲车 {selected_robot}")
        
#         # 选距离最近的任务
#         min_dist = float("inf")
#         best_task = None
#         for task in valid_tasks:
#             rx = self.rl_env.robot_positions[selected_robot].x
#             ry = self.rl_env.robot_positions[selected_robot].y
#             dist = np.sqrt((rx - task["x"])**2 + (ry - task["y"])**2)
#             if dist < min_dist:
#                 min_dist = dist
#                 best_task = task
        
#         if best_task:
#             tid = best_task["task_id"]
            
#             # 防抖：正在提交的任务跳过
#             if tid in self.dispatching_task_ids:
#                 self.get_logger().debug(f"⏳ 任务 {tid} 正在请求中，跳过...")
#                 return

#             # 标记为正在提交
#             self.dispatching_task_ids.add(tid)
            
#             # 调用服务下发任务
#             self._call_ros_service(selected_robot, tid, best_task["waypoint"])

#     def _call_ros_service(self, robot, tid, wp):
#         if not self.nav_client.service_is_ready():
#             self.get_logger().warn("⚠️ 服务未就绪，跳过任务下发")
#             # 移除标记，避免永久锁定
#             self.dispatching_task_ids.discard(tid)
#             return
            
#         req = SingleNavTask.Request()
#         req.target_waypoint = wp
#         req.fleet_name = "deliveryRobot"
#         req.robot_name = robot
#         req.priority = 1
        
#         future = self.nav_client.call_async(req)
#         future.add_done_callback(lambda f: self._service_done(f, tid, robot))

#     def _service_done(self, future, tid, robot):
#         """【修改点3】修复：无论成功失败，只要有结果了，就从“发送中”名单移除，并防止字符串切割报错"""
#         self.dispatching_task_ids.discard(tid)

#         try:
#             res = future.result()
#             if res.success:
#                 self.get_logger().info(f"🚀 调度成功：{robot} -> {tid} @ {res.message}")
                
#                 # 遍历 pending_tasks 找到对应的任务对象
#                 target_task_idx = -1
#                 for i, t in enumerate(self.rl_env.pending_tasks):
#                     if t["task_id"] == tid:
#                         target_task_idx = i
#                         break
                
#                 if target_task_idx != -1:
#                     # 取出任务数据
#                     task_data = self.rl_env.pending_tasks.pop(target_task_idx)
                    
#                     # 更新状态
#                     self.rl_env.robot_idle[robot] = False
#                     self.rl_env.robot_task_map[robot] = tid
                    
#                     # 【核心修复】直接使用 task_data 中的 waypoint，不要 split ID
#                     wp_name = task_data["waypoint"] 
                    
#                     # 计算距离
#                     if wp_name in self.waypoint_coords:
#                         target_point = self.waypoint_coords[wp_name]
#                         current_pos = self.rl_env.robot_positions[robot]
#                         dist = np.sqrt((current_pos.x - target_point.x)**2 + (current_pos.y - target_point.y)**2)
#                     else:
#                         dist = 0.0 # 兜底

#                     self.rl_env.executing_tasks[tid] = {
#                         "robot": robot,
#                         "start_time": self.rl_env.current_time,
#                         "waypoint": self.waypoint_coords.get(wp_name, Point()), # 防止空指针
#                         "distance": dist
#                     }
#             else:
#                 self.get_logger().error(f"❌ 调度失败：{tid} -> {res.message}")
#         except Exception as e:
#             self.get_logger().error(f"❌ 服务调用异常：{e}")
#             import traceback
#             traceback.print_exc()

#     def fleet_state_callback(self, msg):
#         """核心修复：基于你实际的 RobotState 字段判断空闲状态"""
#         if msg.name != "deliveryRobot": return
#         for r in msg.robots:
#             if r.name in self.rl_env.robot_names:
#                 # 1. 更新小车位置（保留）
#                 self.rl_env.update_robot_position(r.name, r.location.x, r.location.y)
                
#                 # 2. 修复：基于实际字段判断空闲状态（无task_state，用mode+task_id）
#                 # RMF RobotMode枚举值：1=IDLE, 2=MOVING, 3=PAUSED, 4=CHARGING
#                 is_idle = (r.mode.mode == RobotMode.MODE_IDLE) and (not r.task_id) and (r.battery_percent > 0.0)
#                 self.rl_env.robot_idle[r.name] = is_idle
                
#                 # 3. 同步剩余执行时间（保留）
#                 if not is_idle and r.task_id:
#                     if r.name in self.rl_env.robot_task_map:
#                         tid = self.rl_env.robot_task_map[r.name]
#                         if tid in self.rl_env.executing_tasks:
#                             wp = self.rl_env.executing_tasks[tid]["waypoint"]
#                             current_pos = Point(x=r.location.x, y=r.location.y)
#                             dist = np.sqrt((current_pos.x - wp.x)**2 + (current_pos.y - wp.y)**2)
#                             self.rl_env.robot_remaining_time[r.name] = dist / 1.0
#                 else:
#                     self.rl_env.robot_remaining_time[r.name] = 0.0

#     def target_callback(self, msg):
#         """处理任务发布话题，新增任务到待执行队列"""
#         data = msg.data.split(",")
#         if len(data) < 2:
#             self.get_logger().warn(f"⚠️ 无效任务格式：{msg.data}")
#             return
        
#         tid, wp = data[0].strip(), data[1].strip()
#         # 去重：已处理的任务不再添加
#         if not tid.startswith("red_cube_") or tid in self.processed_ids:
#             return
        
#         if wp in self.waypoint_coords:
#             p = self.waypoint_coords[wp]
#             self.rl_env.add_task(tid, wp, p.x, p.y)
#             self.processed_ids.add(tid)
#             self.get_logger().info(f"📥 新增任务：{tid} @ {wp}")
#         else:
#             self.get_logger().error(f"❌ 未知航点：{wp}，任务{tid}添加失败")

#     def completion_callback(self, msg):
#         """处理任务完成话题，重置小车状态"""
#         data = msg.data.split(",")
#         if len(data) < 1:
#             self.get_logger().warn(f"⚠️ 无效的完成消息：{msg.data}")
#             return
#         tid = data[0].strip()
        
#         if tid.startswith("red_cube_"):
#             # 调用环境的完成任务方法
#             self.rl_env.complete_task(tid)
#             # 清理缓存
#             if tid in self.processed_ids:
#                 self.processed_ids.remove(tid)
#             # 强制重置对应小车状态（兜底）
#             for robot in self.rl_env.robot_names:
#                 if self.rl_env.robot_task_map.get(robot, "") == tid:
#                     self.rl_env.robot_idle[robot] = True
#                     self.rl_env.robot_remaining_time[robot] = 0.0
#                     del self.rl_env.robot_task_map[robot]
#                     break

# # 主程序
# def main(args=None):
#     rclpy.init(args=args)
#     node = RLDispatcherNode()
    
#     if node.mode == "train":
#         train_thread = threading.Thread(
#             target=node.start_training, 
#             args=(1000000,),
#             daemon=True
#         )
#         train_thread.start()
#         try:
#             rclpy.spin(node)
#         except SystemExit:
#             pass
#         train_thread.join()
#     else:
#         try:
#             rclpy.spin(node)
#         except KeyboardInterrupt:
#             pass
        
#     node.destroy_node()
#     if rclpy.ok():
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
#1.20，zjt改
# 适配需求：8任务必须完成+无待命点返回+超时不丢任务
# 核心优化：下调奖励系数、调优PPO超参数，解决训练震荡问题
