#!/usr/bin/env python3
# 适配需求：8任务必须完成+无待命点返回+超时不丢任务
# 核心优化：修复任务集中分配+奖励过大+标准差高问题
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
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback, StopTrainingOnRewardThreshold, BaseCallback
from stable_baselines3.common.evaluation import evaluate_policy
import time
import random
import threading
import torch as th
from stable_baselines3.common.monitor import Monitor  # 新增：解决验证环境奖励统计错误

# ========== 修复核心：添加缺失的 gymnasium 导入 ==========
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
    真实RMF环境：完全对齐Mock逻辑，支持任务排队+无待命点返回
    优化：新增队列均衡惩罚+奖励归一化+状态特征优化
    """
    metadata = {"render_modes": ["human"], "render_fps": 1}
    
    def __init__(self, node, render_mode=None):
        super().__init__()
        self.node = node
        self.render_mode = render_mode
        
        # 1. 小车配置（支持任务排队）
        self.robot_names = ["deliveryRobot_0", "deliveryRobot_1", "deliveryRobot_2"]
        self.robot_positions = {
            "deliveryRobot_0": Point(x=96.59527587890625, y=-51.96450424194336),
            "deliveryRobot_1": Point(x=152.3477325439453, y=-44.31863021850586),
            "deliveryRobot_2": Point(x=14.776845932006836, y=-9.279278755187988)
        }
        # 升级：支持任务队列
        self.robot_states = {
            name: {
                "idle": True,
                "current_target": None,
                "task_queue": [],
                "current_task_remaining_time": 0.0
            } for name in self.robot_names
        }
        
        # 2. 任务队列（保留所有任务，不丢弃）
        self.pending_tasks = []
        self.completed_tasks = []
        self.task_timeout_count = {}  # 记录任务超时次数，避免重复惩罚
        
        # 3. 统计变量
        self.current_time = 0.0
        self.episode_steps = 0
        self.max_episode_steps = 1000
        self.total_reward = 0.0
        
        # 4. 状态空间（与Mock完全对齐：45维）
        self.action_space = spaces.Discrete(len(self.robot_names) + 1)
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(45,), dtype=np.float32)
        
        # 5. 奖励系数（核心优化：归一化+新增队列均衡惩罚）
        self.reward_coeff = {
            "distance": -0.001,         
            "completion": 1.0,          
            "batch_completion": 1.5,    
            "invalid_selection": -0.1,  
            "timeout": -0.1,            
            "step": -0.001,             
            "wait_short": 0.001,         
            "wait_long": -0.01,        
            "all_completed": 5.0,      
            "task_queue": 0.01,         
            "queue_imbalance": -0.001    
        }

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # 重置小车状态（支持任务队列）
        for name in self.robot_names:
            self.robot_states[name] = {
                "idle": True,
                "current_target": None,
                "task_queue": [],
                "current_task_remaining_time": 0.0
            }
        self.pending_tasks = []
        self.completed_tasks = []
        self.task_timeout_count = {}
        self.current_time = 0.0
        self.episode_steps = 0
        self.total_reward = 0.0
        return self._get_observation(), {"total_reward": 0.0}

    def _get_observation(self):
        """45维状态向量：优化全局特征，移除冗余，新增队列均衡特征"""
        # 1. 机器人状态（12维：x, y, idle, 队列长度）
        robot_obs = []
        for name in self.robot_names:
            r = self.robot_states[name]
            pos = self.robot_positions[name]
            queue_len = len(r["task_queue"]) if not r["idle"] else 0.0
            robot_obs.extend([
                pos.x / 200.0,
                pos.y / 100.0,
                1.0 if r["idle"] else 0.0,
                min(queue_len / 5.0, 1.0)  # 队列长度归一化
            ])
        
        # 2. 任务状态（24维：8个任务）
        task_obs = []
        for _ in range(8):
            task_obs.extend([0.0, 0.0, 0.0])
        
        for i, task in enumerate(self.pending_tasks[:8]):
            task_obs[i*3] = task["x"] / 200.0
            task_obs[i*3+1] = task["y"] / 100.0
            task_obs[i*3+2] = min(task["wait_time"] / 600.0, 1.0)  # 修正等待时间归一化
        
        # 3. 全局状态（9维：优化特征，移除冗余）
        idle_robot_count = sum([1 for name in self.robot_names if self.robot_states[name]["idle"]])
        total_queue_len = sum([len(self.robot_states[name]["task_queue"]) for name in self.robot_names])
        queue_lengths = [len(self.robot_states[name]["task_queue"]) for name in self.robot_names]
        max_queue_len = max(queue_lengths) if queue_lengths else 0
        min_queue_len = min(queue_lengths) if queue_lengths else 0
        
        global_obs = [
            self.current_time / 1000.0,          # 1. 当前时间
            len(self.completed_tasks) / 8.0,     # 2. 已完成任务进度
            len(self.pending_tasks) / 8.0,       # 3. 待处理任务进度
            idle_robot_count / 3.0,              # 4. 空闲机器人占比
            total_queue_len / 15.0,              # 5. 总队列长度占比
            (len(self.completed_tasks) + len(self.pending_tasks)) / 8.0,  # 6. 总任务进度
            # 优化：新增队列均衡特征（移除冗余的单台队列长度）
            np.var(queue_lengths) / 2.5 if queue_lengths else 0.0,        # 7. 队列方差（均衡性）
            max_queue_len / 5.0,                                          # 8. 最长队列（压力峰值）
            (max_queue_len - min_queue_len) / 5.0 if queue_lengths else 0.0,  # 9. 队列最大差（不均衡度）
        ]
        
        # 兜底：确保所有维度在[-1,1]范围内
        global_obs = [min(max(x, -1.0), 1.0) for x in global_obs]
        
        return np.array(robot_obs + task_obs + global_obs, dtype=np.float32)

    def step(self, action):
        self.episode_steps += 1
        self.current_time += 1.0
        reward_items = []  # 收集所有奖励项，最后统一计算
        reward_items.append(self.reward_coeff["step"])  # 基础时间惩罚

        # 奖励裁剪函数
        def clip_reward(r, min_r=-10.0, max_r=50.0):
            return max(min(r, max_r), min_r)
        
        wait_action_idx = len(self.robot_names)
        info = {
            "action_type": "wait" if action == wait_action_idx else "assign",
            "selected_action": action,
            "idle_robots": sum([1 for name in self.robot_names if self.robot_states[name]["idle"]]),
            "robot_task_queues": {name: len(self.robot_states[name]["task_queue"]) for name in self.robot_names}
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
                    reward_items.append(self.reward_coeff["wait_short"])
                elif first_task_wait > 30.0:
                    reward_items.append(self.reward_coeff["wait_long"])
                
                # 超时处理：仅惩罚，不丢弃任务
                for task in self.pending_tasks:
                    tid = task["task_id"]
                    if task["wait_time"] > 120.0 and self.task_timeout_count.get(tid, 0) == 0:
                        reward_items.append(self.reward_coeff["timeout"])
                        self.task_timeout_count[tid] = 1
        
        # --- 处理选车动作（支持任务排队） ---
        else:
            selected_robot = self.robot_names[action]
            info["selected_robot"] = selected_robot
            robot = self.robot_states[selected_robot]
            
            # 无任务时选车：无效操作，惩罚
            if not self.pending_tasks:
                reward_items.append(self.reward_coeff["invalid_selection"])
                reward = sum(reward_items)
                reward = clip_reward(reward)
                truncated = self.episode_steps >= self.max_episode_steps
                return self._get_observation(), reward, False, truncated, info
            
            # 选距离最近的任务
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
            
            # 距离惩罚（引导近任务优先）
            reward_items.append(self.reward_coeff["distance"] * min_dist)
            
            # 计算任务执行时间
            task_exec_time = min_dist / 1.0
            task_info = {
                "task_id": task_id,
                "waypoint": task["waypoint"],
                "x": task["x"],
                "y": task["y"],
                "exec_time": task_exec_time
            }
            
            # 核心逻辑：任务排队（无选车惩罚）
            if robot["idle"]:
                # 小车空闲：直接执行该任务
                robot["idle"] = False
                robot["current_target"] = task_info
                robot["current_task_remaining_time"] = task_exec_time
            else:
                # 小车忙碌：加入任务队列（奖励合理排队）
                robot["task_queue"].append(task_info)
                reward_items.append(self.reward_coeff["task_queue"])
        
        # --- 模拟任务执行（适配排队逻辑） ---
        task_completion_rewards = 0
        for name in self.robot_names:
            r = self.robot_states[name]
            # 小车忙碌且有当前任务
            if not r["idle"] and r["current_target"]:
                # 更新剩余执行时间
                r["current_task_remaining_time"] -= 1.0
                r["current_task_remaining_time"] = max(0.0, r["current_task_remaining_time"])
                
                # 任务完成
                if r["current_task_remaining_time"] <= 0:
                    # 任务完成奖励
                    task_completion_rewards += self.reward_coeff["completion"]
                    self.completed_tasks.append(r["current_target"]["task_id"])
                    self.node.get_logger().info(f"💰 任务完成奖励 +{self.reward_coeff['completion']}! (Robot: {name})")
                    
                    # 检查是否有排队任务
                    if r["task_queue"]:
                        # 执行下一个排队任务
                        next_task = r["task_queue"].pop(0)
                        r["current_target"] = next_task
                        r["current_task_remaining_time"] = next_task["exec_time"]
                    else:
                        # 无排队任务：小车空闲
                        r["idle"] = True
                        r["current_target"] = None
                        r["current_task_remaining_time"] = 0.0
                    
                    # 完成所有8个任务：超大奖励
                    if len(self.completed_tasks) == 8:
                        task_completion_rewards += self.reward_coeff["all_completed"]
                        self.node.get_logger().info(f"🎉 完成所有8个任务！额外奖励 +{self.reward_coeff['all_completed']}")
        
        reward_items.append(task_completion_rewards)
        
        # --- 批量完成奖励 ---
        if len(self.pending_tasks) == 0 and len(self.completed_tasks) > 0:
            reward_items.append(self.reward_coeff["batch_completion"])
        
        # --- 队列不均衡惩罚 ---
        queue_lengths = [len(self.robot_states[name]["task_queue"]) for name in self.robot_names]
        if queue_lengths:
            max_queue = max(queue_lengths)
            avg_queue = np.mean(queue_lengths)
            if max_queue > avg_queue + 2:
                reward_items.append(self.reward_coeff["queue_imbalance"] * (max_queue - avg_queue))
        
        # --- 计算总奖励并裁剪 ---
        reward = sum(reward_items)
        reward = clip_reward(reward)
        
        # --- 统计与结束条件 ---
        self.total_reward += reward
        truncated = self.episode_steps >= self.max_episode_steps
        # 结束条件：任务完成+无排队任务
        terminated = (len(self.pending_tasks) == 0 and 
                      len(self.completed_tasks) == 8 and
                      all([r["idle"] and len(r["task_queue"]) == 0 for r in self.robot_states.values()]))
        
        return self._get_observation(), reward, terminated, truncated, info

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
        """适配RMF真实回调的任务完成逻辑"""
        # 查找执行该任务的小车
        target_robot = None
        for name in self.robot_names:
            r = self.robot_states[name]
            # 检查当前执行的任务
            if not r["idle"] and r["current_target"] and r["current_target"]["task_id"] == tid:
                target_robot = name
                break
            # 检查排队任务
            for i, task in enumerate(r["task_queue"]):
                if task["task_id"] == tid:
                    # 从队列中移除（RMF中取消排队任务的场景）
                    r["task_queue"].pop(i)
                    self.node.get_logger().info(f"📤 任务 {tid} 从 {name} 的排队队列中移除")
                    return
        
        if target_robot:
            r = self.robot_states[target_robot]
            # 任务完成奖励
            self.total_reward += self.reward_coeff["completion"]
            self.completed_tasks.append(tid)
            self.node.get_logger().info(f"💰 任务完成奖励 +{self.reward_coeff['completion']}! (Robot: {target_robot})")
            
            # 执行下一个排队任务
            if r["task_queue"]:
                next_task = r["task_queue"].pop(0)
                r["current_target"] = next_task
                r["current_task_remaining_time"] = next_task["exec_time"]
            else:
                # 无排队任务：小车空闲
                r["idle"] = True
                r["current_target"] = None
                r["current_task_remaining_time"] = 0.0
            
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

        # 4. 初始化 RL 模型（调优超参数，增强探索+降低奖励震荡）
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

        # 订阅器
        self.create_subscription(FleetState, "/fleet_states", self.fleet_state_callback, 10)
        self.create_subscription(String, "/task_monitor/start", self.target_callback, 10)
        self.create_subscription(String, "/custom_task_completion", self.completion_callback, 10)
        
        # 服务客户端
        self.nav_client = self.create_client(SingleNavTask, "/submit_single_nav_task")
        
        # 决策定时器（1秒/次）
        self.create_timer(1.0, self.infer_dispatch_callback)
        
        # 去重缓存
        self.processed_ids = set()
        
        self.get_logger().info("✅ ROS 接口连接完成")

    def _init_rl_model(self):
        """初始化 PPO 模型（核心优化：增强探索+降低奖励震荡）"""
        model_path = "./rl_models/dispatching_ppo_final"
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
                learning_rate=3e-5,        # 提高学习率（从1e-5→3e-5）
                n_steps=2048,              
                batch_size=256,            
                gamma=0.98,                # 提高gamma（从0.95→0.98），增强长期奖励权重
                gae_lambda=0.95,           # 提高GAE（从0.9→0.95）
                ent_coef=0.1,              # 降低探索（从0.2→0.1），减少无意义等待
                vf_coef=0.5,               
                max_grad_norm=0.5,         
                n_epochs=5,                # 提高迭代次数（从3→5），增强学习效果
                clip_range=0.2,            # 提高裁剪范围（从0.15→0.2）
                clip_range_vf=None,        
            )


            # 回调函数：监控收敛，保存最优模型
            self.checkpoint_callback = CheckpointCallback(save_freq=20000, save_path="./rl_models/", name_prefix="ppo")
            
            # 奖励阈值停止回调
            self.stop_callback = StopTrainingOnRewardThreshold(
                reward_threshold=1000.0,   # 降低奖励阈值（适配归一化后的奖励）
                verbose=1
            )
            
            # 新建独立的验证环境
            self.eval_env = Monitor(MockRMFEnv())  # 关键：用self.eval_env，变成类属性
            self.eval_callback = EvalCallback(
                self.eval_env,  # 同步修改为self.eval_env
                eval_freq=5000,
                n_eval_episodes=5,
                best_model_save_path="./rl_models/best/",
                verbose=1,
                callback_after_eval=self.stop_callback,
                deterministic=False,
                render=False
            )

            
            class RewardNormalizationCallback(BaseCallback):
                def __init__(self, verbose=0):
                    super().__init__(verbose)
                    self.reward_sum = 0.0
                    self.reward_count = 0
                    self.reward_mean = 0.0
                    self.reward_std = 1.0

                def _on_step(self) -> bool:
                    # 收集奖励（每步都收集，而非仅episode结束）
                    if "rewards" in self.locals:
                        current_rewards = self.locals["rewards"]
                        self.reward_sum += np.sum(current_rewards)
                        self.reward_count += len(current_rewards)
                        
                        # 每500步更新一次均值和标准差（更频繁，降低波动）
                        if self.reward_count % 500 == 0:
                            self.reward_mean = self.reward_sum / self.reward_count
                            # 防止标准差为0
                            self.reward_std = max(1e-6, np.std(self.locals["rewards"]))
                            self.reward_sum = 0.0
                            self.reward_count = 0
                    
                    # 归一化奖励（使用滑动均值和标准差）
                    if self.reward_std > 0 and "rewards" in self.locals:
                        self.locals["rewards"] = (self.locals["rewards"] - self.reward_mean) / (self.reward_std + 1e-8)
                    return True


            self.reward_norm_callback = RewardNormalizationCallback()
            
        elif self.mode == "infer":
            if os.path.exists(model_path + ".zip"):
                self.get_logger().info(f"📂 加载模型: {model_path}")
                self.model = PPO.load(model_path, device='cpu')
            else:
                self.get_logger().error(f"❌ 未找到模型文件: {model_path}.zip")
                self.model = None

    # 训练逻辑（新增奖励归一化回调）
    def start_training(self, total_timesteps=1000000):
        if self.mode != "train": return
            
        self.get_logger().info(f"🔥 开始训练 {total_timesteps} 步（8任务必须完成版）...")
        start_t = time.time()  # 保留原开始时间
        
        self.model.learn(
            total_timesteps=total_timesteps, 
            callback=[self.checkpoint_callback, self.eval_callback, self.reward_norm_callback]
        )
        
        # 新增：计算并打印训练用时
        end_t = time.time()
        train_duration = end_t - start_t
        hours = int(train_duration // 3600)
        minutes = int((train_duration % 3600) // 60)
        seconds = int(train_duration % 60)
        self.get_logger().info(f"⏱️ 训练总用时：{hours}小时{minutes}分钟{seconds}秒")
        
        # 修改：用包装后的eval_env评估（而不是self.rl_env）
        mean_reward, std_reward = evaluate_policy(self.model, self.eval_env, n_eval_episodes=10)
        self.get_logger().info(f"📊 最终模型评估：平均奖励={mean_reward:.2f}，标准差={std_reward:.2f}")
        
        self.model.save("./rl_models/dispatching_ppo_final")
        self.get_logger().info("💾 模型已保存！")
        rclpy.shutdown()


    # 推理逻辑（适配任务排队，移除选忙碌车的校验）
    def infer_dispatch_callback(self):
        # 1. 过滤有效任务
        valid_tasks = [t for t in self.rl_env.pending_tasks if t["task_id"].startswith("red_cube_")]
        if not valid_tasks:
            self.get_logger().debug("📭 无有效任务，跳过调度")
            return
        
        # 2. 获取环境状态 & 模型决策
        obs = self.rl_env._get_observation()
        wait_action_idx = len(self.rl_env.robot_names)
        
        if self.model is None:
            self.get_logger().error("❌ 模型未加载，无法决策")
            return
        
        # 模型输出动作（完全信任模型的决策）
        action, _states = self.model.predict(obs, deterministic=True)
        self.get_logger().info(f"🤖 RL智能体输出动作：{action} (等待动作索引：{wait_action_idx})")

        # 3. 处理模型输出的动作
        if action == wait_action_idx:
            # 动作：等待
            self.get_logger().info("⏳ RL决策：等待新任务")
            for task in self.rl_env.pending_tasks:
                task["wait_time"] += 1.0
        else:
            # 动作：选车（直接执行，无论小车是否忙碌）
            selected_robot = self.rl_env.robot_names[action]
            
            # 选离该小车最近的任务
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
                self._call_ros_service(selected_robot, best_task["task_id"], best_task["waypoint"])

    def _call_ros_service(self, robot, tid, wp):
        """调用ROS服务下发任务"""
        if not self.nav_client.service_is_ready():
            self.get_logger().warn("⚠️ 服务未就绪，跳过任务下发")
            return
            
        req = SingleNavTask.Request()
        req.target_waypoint = wp
        req.fleet_name = "deliveryRobot"
        req.robot_name = robot
        req.priority = 1
        
        # 传递wp到回调函数
        future = self.nav_client.call_async(req)
        future.add_done_callback(lambda f, r=robot, t=tid, w=wp: self._service_done(f, t, r, w))

    def _service_done(self, future, tid, robot, wp):
        """服务调用完成回调（修复wp变量传递+waypoint_coords归属错误）"""
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"🚀 调度成功：{robot} -> {tid} @ {res.message}")
                for i, t in enumerate(self.rl_env.pending_tasks):
                    if t["task_id"] == tid:
                        self.rl_env.pending_tasks.pop(i)
                        # 标记小车为忙碌（如果当前无任务）
                        if self.rl_env.robot_states[robot]["idle"]:
                            self.rl_env.robot_states[robot]["idle"] = False
                        # 任务加入队列（与环境逻辑对齐）
                        waypoint_x = self.waypoint_coords[wp].x
                        waypoint_y = self.waypoint_coords[wp].y
                        robot_x = self.rl_env.robot_positions[robot].x
                        robot_y = self.rl_env.robot_positions[robot].y
                        
                        self.rl_env.robot_states[robot]["task_queue"].append({
                            "task_id": tid,
                            "waypoint": wp,
                            "x": waypoint_x,
                            "y": waypoint_y,
                            "exec_time": np.sqrt(
                                (robot_x - waypoint_x)**2 + (robot_y - waypoint_y)**2
                            ) / 1.0
                        })
                        break
            else:
                self.get_logger().error(f"❌ 调度失败：{tid} -> {res.message}")
        except Exception as e:
            self.get_logger().error(f"❌ 服务调用异常：{e}")

    # ROS 回调函数（简化小车状态判断）
    def fleet_state_callback(self, msg):
        if msg.name != "deliveryRobot": return
        for r in msg.robots:
            if r.name in self.rl_env.robot_names:
                # 更新位置和基础空闲状态
                self.rl_env.update_robot_position(r.name, r.location.x, r.location.y)
                self.rl_env.robot_states[r.name]["idle"] = not bool(r.task_id)
                if self.rl_env.robot_states[r.name]["idle"]:
                    self.rl_env.robot_states[r.name]["current_task_remaining_time"] = 0.0

    def target_callback(self, msg):
        data = msg.data.split(",")
        if len(data) < 2:
            self.get_logger().warn(f"⚠️ 无效任务格式：{msg.data}")
            return
        
        tid, wp = data[0].strip(), data[1].strip()
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
            # 标记小车空闲（兜底）
            for robot in self.rl_env.robot_names:
                if not self.rl_env.robot_states[robot]["idle"] and self.rl_env.robot_states[robot]["current_target"]:
                    if self.rl_env.robot_states[robot]["current_target"]["task_id"] == tid:
                        self.rl_env.robot_states[robot]["idle"] = True
                        self.rl_env.robot_states[robot]["current_target"] = None
                        self.rl_env.robot_states[robot]["current_task_remaining_time"] = 0.0
                        break
                # 检查排队任务
                for i, task in enumerate(self.rl_env.robot_states[robot]["task_queue"]):
                    if task["task_id"] == tid:
                        self.rl_env.robot_states[robot]["task_queue"].pop(i)
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
