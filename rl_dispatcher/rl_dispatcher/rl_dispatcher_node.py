#!/usr/bin/env python3
#1.17用虚拟rmf环境训练
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
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
import threading
from datetime import datetime
import gymnasium as gym
from gymnasium import spaces

# ===================== 引入双引擎 =====================
# 1. 真实引擎 (Real Engine): 内嵌在当前文件中
# 2. 虚拟引擎 (Mock Engine): 从 mock_env.py 导入
try:
    from rl_dispatcher.mock_env import MockRMFEnv
except ImportError:
    # 为了防止在不同环境下路径识别错误，尝试相对导入
    try:
        from mock_env import MockRMFEnv
    except ImportError:
        print("❌ 错误：找不到 mock_env.py。请确保该文件已创建在 rl_dispatcher 目录下。")
        sys.exit(1)

# ===================== 引擎 A: 真实 ROS 环境 (Real Engine) =====================
class RLDispatchingEnv(gym.Env):
    """
    真实 ROS 环境：依赖 Gazebo 物理仿真和 RMF 状态反馈。
    """
    metadata = {"render_modes": ["human"], "render_fps": 1}
    
    def __init__(self, node, render_mode=None):
        super().__init__()
        self.node = node
        self.render_mode = render_mode
        
        # 1. 物理机器人配置
        self.robot_names = ["deliveryRobot_0", "deliveryRobot_1", "deliveryRobot_2"]
        self.robot_positions = {name: Point(x=0.0, y=0.0) for name in self.robot_names}
        self.robot_idle = {name: True for name in self.robot_names}
        self.robot_task_map = {}
        
        # 2. 任务队列
        self.pending_tasks = []
        self.executing_tasks = {}
        self.completed_tasks = []
        self.failed_tasks = []
        
        # 3. 统计变量
        self.current_time = 0.0
        self.episode_steps = 0
        self.max_episode_steps = 500
        self.total_reward = 0.0
        
        # 4. 空间定义 (必须与 MockRMFEnv 完全一致)
        self.action_space = spaces.Discrete(len(self.robot_names))
        # 14维状态: 3x3(机器人状态) + 3(任务) + 2(全局)
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(14,), dtype=np.float32)
        
        # 5. 奖励系数
        self.reward_coeff = {
            "distance": -0.1,
            "completion": 100.0,
            "idle_selection": -10.0,
            "timeout": -50.0,
            "step": -0.1
        }

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.robot_idle = {name: True for name in self.robot_names}
        self.robot_task_map = {}
        self.pending_tasks = []
        self.executing_tasks = {}
        self.completed_tasks = []
        self.current_time = 0.0
        self.episode_steps = 0
        self.total_reward = 0.0
        return self._get_observation(), {"total_reward": 0.0}

    def _get_observation(self):
        # 1. 机器人状态 (x, y, idle)
        robot_obs = []
        for name in self.robot_names:
            robot_obs.extend([
                self.robot_positions[name].x / 200.0,
                self.robot_positions[name].y / 50.0,
                1.0 if self.robot_idle[name] else 0.0
            ])
        
        # 2. 任务状态
        task_obs = [0.0, 0.0, 0.0]
        if self.pending_tasks:
            task = self.pending_tasks[0]
            task_obs = [
                task["x"] / 200.0,
                task["y"] / 50.0,
                min(task["wait_time"] / 100.0, 1.0)
            ]
            
        # 3. 全局状态
        global_obs = [
            self.current_time / 1000.0,
            len(self.completed_tasks) / 10.0
        ]
        
        return np.array(robot_obs + task_obs + global_obs, dtype=np.float32)

    def step(self, action):
        self.episode_steps += 1
        self.current_time += 1.0
        reward = self.reward_coeff["step"]
        
        info = {"selected_robot": self.robot_names[action]}
        selected_robot = self.robot_names[action]
        
        # 核心逻辑：这里只计算奖励，真正的动作执行由 Node 中的 _call_nav_service 处理
        if not self.pending_tasks:
            truncated = self.episode_steps >= self.max_episode_steps
            return self._get_observation(), reward, False, truncated, info
            
        task = self.pending_tasks.pop(0)
        task_id = task["task_id"]
        info["task_id"] = task_id
        
        # 奖励计算
        if not self.robot_idle[selected_robot]:
            reward += self.reward_coeff["idle_selection"]
            # 惩罚后放回队列
            self.pending_tasks.insert(0, task)
        else:
            rx = self.robot_positions[selected_robot].x
            ry = self.robot_positions[selected_robot].y
            dist = np.sqrt((rx - task["x"])**2 + (ry - task["y"])**2)
            
            reward += self.reward_coeff["distance"] * dist
            
            # 更新内部状态
            self.robot_idle[selected_robot] = False
            self.robot_task_map[selected_robot] = task_id
            self.executing_tasks[task_id] = {
                "robot": selected_robot,
                "start_time": self.current_time,
                "waypoint": task["waypoint"]
            }
            
        # 超时检查
        timeout_ids = []
        for tid, tinfo in self.executing_tasks.items():
            if self.current_time - tinfo["start_time"] > 60.0:
                reward += self.reward_coeff["timeout"]
                self.robot_idle[tinfo["robot"]] = True
                timeout_ids.append(tid)
        
        for tid in timeout_ids:
            del self.executing_tasks[tid]
            
        self.total_reward += reward
        truncated = self.episode_steps >= self.max_episode_steps
        
        return self._get_observation(), reward, False, truncated, info

    # 辅助方法：供 ROS 回调更新数据
    def update_robot_position(self, name, x, y):
        if name in self.robot_positions:
            self.robot_positions[name].x = x
            self.robot_positions[name].y = y
            
    def add_task(self, tid, wp, x, y):
        self.pending_tasks.append({"task_id": tid, "waypoint": wp, "x": x, "y": y, "wait_time": 0.0})
        
    def complete_task(self, tid):
        if tid in self.executing_tasks:
            robot = self.executing_tasks[tid]["robot"]
            self.total_reward += self.reward_coeff["completion"]
            self.completed_tasks.append(tid)
            self.robot_idle[robot] = True
            del self.executing_tasks[tid]
            self.node.get_logger().info(f"💰 任务完成奖励 +100! (Robot: {robot})")

# ===================== 核心节点：双模式调度器 =====================
class RLDispatcherNode(Node):
    def __init__(self):
        super().__init__("rl_dispatcher_node")
        self.get_logger().info("🚀 初始化 RL 调度节点 (Train+Infer 模式)...")
        
        # ==================== 修复：参数声明逻辑 ====================
        # 1. 仿真时间配置
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])

        # 2. 模式配置 (Train / Infer)
        if not self.has_parameter("mode"):
            self.declare_parameter("mode", "train")
        
        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.get_logger().info(f"🔵 当前模式: [{self.mode.upper()}]")
        
        # ================== 核心：双引擎切换 ==================
        if self.mode == "train":
            self.get_logger().info("🚀 模式: 训练 (TRAIN) - 挂载 Mock 虚拟引擎")
            # 训练时，使用 Mock 环境，不依赖 Gazebo/ROS 话题
            self.rl_env = MockRMFEnv() 
            
        elif self.mode == "infer":
            self.get_logger().info("🤖 模式: 推理 (INFER) - 挂载 Real 真实引擎")
            # 推理时，使用 Real 环境，依赖 ROS 话题和服务
            self.rl_env = RLDispatchingEnv(self, render_mode="human")
        else:
            self.get_logger().error("❌ 未知模式。请使用 mode:=train 或 mode:=infer")
            sys.exit(1)
        # ========================================================

        # 3. 初始化 RL 模型
        self.model = None
        self._init_rl_model()
        
        # 4. 仅在 INFER 模式下初始化 ROS 接口
        if self.mode == "infer":
            self._init_ros_interfaces()

    def _init_ros_interfaces(self):
        """初始化 ROS 订阅、发布和服务客户端 (仅推理模式)"""
        self.get_logger().info("🔌 正在连接 ROS 接口...")
        
        # 航点地图
        self.waypoint_coords = {
            "n14": Point(x=80.84, y=-28.52), "n13": Point(x=84.44, y=-4.94),
            "n23": Point(x=182.80, y=-42.30), "s08": Point(x=96.61, y=-50.50),
            "s10": Point(x=122.10, y=-46.68), "west_koi_pond": Point(x=34.32, y=-10.13),
            "s11": Point(x=152.73, y=-43.00), "junction_south_west": Point(x=84.56, y=-38.81)
        }
        
        # 订阅器
        self.create_subscription(FleetState, "/fleet_states", self.fleet_state_callback, 10)
        self.create_subscription(String, "/task_monitor/start", self.target_callback, 10)
        self.create_subscription(String, "/custom_task_completion", self.completion_callback, 10)
        
        # 客户端
        self.nav_client = self.create_client(SingleNavTask, "/submit_single_nav_task")
        
        # 决策定时器 (1秒一次)
        self.create_timer(1.0, self.infer_dispatch_callback)
        
        # 去重缓存
        self.processed_ids = set()
        
        self.get_logger().info("✅ ROS 接口连接完毕")

    def _init_rl_model(self):
        """加载或创建 PPO 模型"""
        model_path = "./rl_models/dispatching_ppo_final"
        log_dir = "./rl_dispatching_tb_log/"
        
        if self.mode == "train":
            # 创建新模型
            self.model = PPO("MlpPolicy", self.rl_env, verbose=1, device='cpu', tensorboard_log=log_dir)
            self.checkpoint_callback = CheckpointCallback(save_freq=50000, save_path="./rl_models/", name_prefix="ppo")
            
        elif self.mode == "infer":
            # 加载已有模型
            if os.path.exists(model_path + ".zip"):
                self.get_logger().info(f"📂 加载模型: {model_path}")
                self.model = PPO.load(model_path, device='cpu')
            else:
                self.get_logger().error(f"❌ 未找到模型文件: {model_path}.zip，请先运行 train 模式！")
                self.model = None

    # ===================== 训练逻辑 (极速版) =====================
    def start_training(self, total_timesteps=1000000):
        """Mock 内核驱动的极速训练"""
        if self.mode != "train": return
            
        self.get_logger().info(f"🔥 [极速训练] 开始跑 {total_timesteps} 步...")
        start_t = time.time()
        
        # 这里的 learn 会自动驱动 MockRMFEnv.step()，不依赖 ROS 回调
        self.model.learn(total_timesteps=total_timesteps, callback=self.checkpoint_callback)
        
        duration = time.time() - start_t
        self.get_logger().info(f"✅ 训练完成！耗时: {duration:.2f} 秒")
        
        self.model.save("./rl_models/dispatching_ppo_final")
        self.get_logger().info("💾 模型已保存，即将退出...")
        rclpy.shutdown()

    # ===================== 推理逻辑 (实战版) =====================
    def infer_dispatch_callback(self):
        """真实环境下的周期性调度（仅处理red_cube任务）"""
        # 过滤出仅red_cube前缀的任务
        valid_tasks = [t for t in self.rl_env.pending_tasks if t["task_id"].startswith("red_cube_")]
        if not valid_tasks:
            self.get_logger().debug("📭 无有效用户任务，跳过调度")
            return
        
        # 1. 获取真实状态
        obs = self.rl_env._get_observation()
        
        # 2. 模型决策
        action = 0
        if self.model:
            action, _ = self.model.predict(obs, deterministic=True)
            
            # 安全降级：如果模型选了忙碌的车，强制改为空闲车
            robot_name = self.rl_env.robot_names[action]
            if not self.rl_env.robot_idle[robot_name]:
                idle_indices = [i for i, name in enumerate(self.rl_env.robot_names) if self.rl_env.robot_idle[name]]
                if idle_indices:
                    action = random.choice(idle_indices)
                    self.get_logger().warn(f"🛡️ 修正：模型选了忙碌车，强制改为 {self.rl_env.robot_names[action]}")
        
        # 3. 执行调度（只处理第一个有效任务）
        selected_robot = self.rl_env.robot_names[action]
        task = valid_tasks[0]  # 只取第一个red_cube任务
        
        self._call_ros_service(selected_robot, task["task_id"], task["waypoint"])

    def _call_ros_service(self, robot, tid, wp):
        if not self.nav_client.service_is_ready():
            return
            
        req = SingleNavTask.Request()
        req.target_waypoint = wp
        req.fleet_name = "deliveryRobot"
        req.robot_name = robot
        req.priority = 1
        
        future = self.nav_client.call_async(req)
        # 使用闭包保存上下文
        future.add_done_callback(lambda f: self._service_done(f, tid, robot))

    def _service_done(self, future, tid, robot):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"🚀 [调度成功] {robot} -> 任务 {tid}")
                # 只有服务调用成功，才从等待队列移除，并在环境中标记为忙碌
                # 这一步是为了同步 ROS 状态和 RL 环境状态
                for i, t in enumerate(self.rl_env.pending_tasks):
                    if t["task_id"] == tid:
                        self.rl_env.pending_tasks.pop(i)
                        self.rl_env.robot_idle[robot] = False
                        self.rl_env.robot_task_map[robot] = tid
                        break
        except Exception as e:
            self.get_logger().error(f"服务调用失败: {e}")

    # ===================== ROS 回调函数 (仅更新数据) =====================
    def fleet_state_callback(self, msg):
        if msg.name != "deliveryRobot": return
        for r in msg.robots:
            if r.name in self.rl_env.robot_names:
                self.rl_env.update_robot_position(r.name, r.location.x, r.location.y)
                # 双重确认空闲状态
                if not r.task_id:
                    self.rl_env.robot_idle[r.name] = True

    def target_callback(self, msg):
        """仅处理无人机发布的red_cube前缀任务，过滤RMF原生的direct_任务"""
        data = msg.data.split(",")
        if len(data) < 2:
            self.get_logger().warn(f"⚠️ 无效的任务格式: {msg.data}")
            return
        
        tid, wp = data[0].strip(), data[1].strip()
        
        # 核心过滤：只处理red_cube前缀的用户任务，忽略direct_前缀的RMF原生任务
        if not tid.startswith("red_cube_"):
            self.get_logger().debug(f"🔍 忽略RMF原生任务: {tid} @ {wp}")
            return
        
        # 去重：避免重复添加同一任务
        if tid in self.processed_ids:
            self.get_logger().debug(f"⚠️ 任务已处理过: {tid}")
            return
        
        if wp in self.waypoint_coords:
            p = self.waypoint_coords[wp]
            self.rl_env.add_task(tid, wp, p.x, p.y)
            self.processed_ids.add(tid)
            self.get_logger().info(f"📥 收到新任务: {tid} @ {wp}")
        else:
            self.get_logger().error(f"❌ 未知航点: {wp}，任务{tid}添加失败")

    def completion_callback(self, msg):
        tid = msg.data.split(",")[0].strip()
        self.rl_env.complete_task(tid)
        # 清理已完成任务的去重缓存，允许相同任务再次提交
        if tid in self.processed_ids:
            self.processed_ids.remove(tid)
            self.get_logger().debug(f"🗑️  清理已完成任务缓存: {tid}")

# ===================== 主程序入口 =====================
def main(args=None):
    rclpy.init(args=args)
    node = RLDispatcherNode()
    
    if node.mode == "train":
        # 训练模式：开启独立线程跑训练
        train_thread = threading.Thread(
            target=node.start_training, 
            args=(1000000,),  # 跑100万步
            daemon=True
        )
        train_thread.start()
        # 这里的 spin 主要是为了响应 shutdown 信号，因为 Mock 不需要 ROS 回调
        try:
            rclpy.spin(node)
        except SystemExit:
            pass
        train_thread.join()
        
    else:
        # 推理模式：正常的 ROS spin
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        
    node.destroy_node()
    
    # 防止双重关机导致的 RuntimeError
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()
