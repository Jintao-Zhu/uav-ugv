#!/usr/bin/env python3

# 1.22【消融实验】移除锚点逻辑，统一使用小车实时位置 10min10s
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
from stable_baselines3.common.monitor import Monitor

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
    真实RMF环境：完全对齐Mock逻辑  
    【消融实验】移除锚点逻辑，统一使用小车实时位置
    """
    metadata = {"render_modes": ["human"], "render_fps": 1}
    
    def __init__(self, node, render_mode=None):
        super().__init__()
        self.node = node
        self.render_mode = render_mode
        
        self.robot_names = ["deliveryRobot_0", "deliveryRobot_1", "deliveryRobot_2"]
        self.robot_positions = {
            "deliveryRobot_0": Point(x=96.59527587890625, y=-51.96450424194336),
            "deliveryRobot_1": Point(x=152.3477325439453, y=-44.31863021850586),
            "deliveryRobot_2": Point(x=14.776845932006836, y=-9.279278755187988)
        }
        self.robot_states = {
            name: {
                "idle": True,
                "current_target": None,
                "task_queue": [],
                "current_task_remaining_time": 0.0,
                "speed": 1.0 # 假设速度，用于估算时间
            } for name in self.robot_names
        }
        
        self.pending_tasks = []
        self.completed_tasks = []
        self.task_timeout_count = {}
        
        self.current_time = 0.0
        self.episode_steps = 0
        self.max_episode_steps = 1000
        self.total_reward = 0.0
        
        # 升级：54维状态空间
        self.action_space = spaces.Discrete(len(self.robot_names) + 1)
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(54,), dtype=np.float32)
        
        # 7. 奖励系数 (⚡️ 休克疗法版 - 用于推理阶段日志显示 ⚡️)
        self.reward_coeff = {
            "valid_assign": 5.0,
            "wait_with_task": -2.0,
            "wait_no_task": 0.1,
            "distance": -0.005,
            "time_congestion": -0.01,
            "queue_congestion": -0.2,
            "step": -0.01,
            "invalid_selection": -5.0,
            "completion": 5.0,
            "all_completed": 20.0,
            "batch_completion": 2.0
        }

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        for name in self.robot_names:
            self.robot_states[name] = {
                "idle": True,
                "current_target": None,
                "task_queue": [],
                "current_task_remaining_time": 0.0,
                "speed": 1.0
            }
        self.pending_tasks = []
        self.completed_tasks = []
        self.task_timeout_count = {}
        self.current_time = 0.0
        self.episode_steps = 0
        self.total_reward = 0.0
        return self._get_observation(), {"total_reward": 0.0}

    def _get_observation(self):
        """54维状态向量：对齐MockEnv的新特征
        【消融修改1】移除末端锚点计算，统一用小车实时位置
        """
        robot_obs = []
        for name in self.robot_names:
            r = self.robot_states[name]
            pos = self.robot_positions[name]
            
            # ========== 【消融修改1】移除锚点逻辑，直接使用小车实时位置 ==========
            last_x, last_y = pos.x, pos.y
            
            # 2. 计算预计总负荷时间
            total_time = r["current_task_remaining_time"]
            for t in r["task_queue"]:
                total_time += t["exec_time"]

            queue_len = len(r["task_queue"]) if not r["idle"] else 0.0
            robot_obs.extend([
                pos.x / 200.0,
                pos.y / 100.0,
                1.0 if r["idle"] else 0.0,
                min(queue_len / 5.0, 1.0),
                last_x / 200.0,                 # New
                last_y / 100.0,                 # New
                min(total_time / 200.0, 1.0)    # New
            ])
        
        task_obs = []
        for _ in range(8):
            task_obs.extend([0.0, 0.0, 0.0])
        for i, task in enumerate(self.pending_tasks[:8]):
            task_obs[i*3] = task["x"] / 200.0
            task_obs[i*3+1] = task["y"] / 100.0
            task_obs[i*3+2] = min(task["wait_time"] / 600.0, 1.0)
        
        total_queue_len = sum([len(self.robot_states[name]["task_queue"]) for name in self.robot_names])
        idle_robot_count = sum([1 for name in self.robot_names if self.robot_states[name]["idle"]])
        queue_lengths = [len(self.robot_states[name]["task_queue"]) for name in self.robot_names]
        max_queue_len = max(queue_lengths) if queue_lengths else 0
        min_queue_len = min(queue_lengths) if queue_lengths else 0
        
        global_obs = [
            self.current_time / 1000.0,
            len(self.completed_tasks) / 8.0,
            len(self.pending_tasks) / 8.0,
            idle_robot_count / 3.0,
            total_queue_len / 15.0,
            (len(self.completed_tasks) + len(self.pending_tasks)) / 8.0,
            np.var(queue_lengths) / 2.5 if queue_lengths else 0.0,
            max_queue_len / 5.0,
            (max_queue_len - min_queue_len) / 5.0 if queue_lengths else 0.0,
        ]
        global_obs = [min(max(x, -1.0), 1.0) for x in global_obs]
        
        return np.array(robot_obs + task_obs + global_obs, dtype=np.float32)

    def step(self, action):
        self.episode_steps += 1
        self.current_time += 1.0
        # 在推理模式下，Step的奖励计算仅用于日志记录
        reward = self.reward_coeff["step"]

        def clip_reward(r, min_r=-20.0, max_r=20.0):
            return max(min(r, max_r), min_r)
        
        wait_action_idx = len(self.robot_names)
        info = {
            "action_type": "wait" if action == wait_action_idx else "assign",
            "selected_action": action,
            "robot_task_queues": {name: len(self.robot_states[name]["task_queue"]) for name in self.robot_names}
        }
        
        if action == wait_action_idx:
            if self.pending_tasks:
                reward += self.reward_coeff["wait_with_task"]
                for task in self.pending_tasks:
                    task["wait_time"] += 1.0
            else:
                reward += self.reward_coeff["wait_no_task"]
        else:
            selected_robot = self.robot_names[action]
            info["selected_robot"] = selected_robot
            robot = self.robot_states[selected_robot]
            
            if not self.pending_tasks:
                reward += self.reward_coeff["invalid_selection"]
                return self._get_observation(), reward, False, self.episode_steps >= self.max_episode_steps, info
            
            # 有效派单
            reward += self.reward_coeff["valid_assign"]

            # ========== 【消融修改2】移除锚点逻辑，直接使用小车实时位置 ==========
            anchor_x = self.robot_positions[selected_robot].x
            anchor_y = self.robot_positions[selected_robot].y

            # 寻找最近任务
            min_dist = float("inf")
            best_task_idx = 0
            for i, task in enumerate(self.pending_tasks):
                dist = np.sqrt((anchor_x - task["x"])**2 + (anchor_y - task["y"])**2)
                if dist < min_dist:
                    min_dist = dist
                    best_task_idx = i
            
            task = self.pending_tasks.pop(best_task_idx)
            task_id = task["task_id"]
            info["task_id"] = task_id
            
            # --- 奖励计算 ---
            reward += self.reward_coeff["distance"] * min_dist
            
            wait_time_for_new_task = robot["current_task_remaining_time"] + \
                                     sum([t["exec_time"] for t in robot["task_queue"]])
            reward += self.reward_coeff["time_congestion"] * wait_time_for_new_task
            
            new_queue_len = len(robot["task_queue"]) + (1 if not robot["idle"] else 0)
            reward += self.reward_coeff["queue_congestion"] * (new_queue_len ** 2)
            
            task_exec_time = min_dist / 1.0
            task_info = {
                "task_id": task_id,
                "waypoint": task["waypoint"],
                "x": task["x"],
                "y": task["y"],
                "exec_time": task_exec_time
            }
            
            if robot["idle"]:
                robot["idle"] = False
                robot["current_target"] = task_info
                robot["current_task_remaining_time"] = task_exec_time
            else:
                robot["task_queue"].append(task_info)
                # ========== 【消融修改3】队列重排序 - 移除锚点，用小车实时位置 ==========
                anchor_x, anchor_y = self.robot_positions[selected_robot].x, self.robot_positions[selected_robot].y
                
                def sort_by_distance(task):
                    return np.sqrt((anchor_x - task["x"])**2 + (anchor_y - task["y"])**2)
                
                robot["task_queue"].sort(key=sort_by_distance)
        
        # ROS模拟执行更新 (部分逻辑在回调中处理)
        task_completion_rewards = 0
        for name in self.robot_names:
            r = self.robot_states[name]
            if not r["idle"] and r["current_target"]:
                r["current_task_remaining_time"] -= 1.0
                r["current_task_remaining_time"] = max(0.0, r["current_task_remaining_time"])
                # ROS环境中任务完成主要由回调触发

        reward += task_completion_rewards
        reward = clip_reward(reward)
        self.total_reward += reward
        truncated = self.episode_steps >= self.max_episode_steps
        terminated = (len(self.pending_tasks) == 0 and 
                      len(self.completed_tasks) == 8 and
                      all([r["idle"] and len(r["task_queue"]) == 0 for r in self.robot_states.values()]))
        
        return self._get_observation(), reward, terminated, truncated, info

    def update_robot_position(self, name, x, y):
        if name in self.robot_positions:
            self.robot_positions[name].x = x
            self.robot_positions[name].y = y
            
    def add_task(self, tid, wp, x, y):
        self.pending_tasks.append({
            "task_id": tid, 
            "waypoint": wp, 
            "x": x, 
            "y": y, 
            "wait_time": 0.0
        })
        self.task_timeout_count[tid] = 0
        
    def complete_task(self, tid):
        target_robot = None
        for name in self.robot_names:
            r = self.robot_states[name]
            if not r["idle"] and r["current_target"] and r["current_target"]["task_id"] == tid:
                target_robot = name
                break
            for i, task in enumerate(r["task_queue"]):
                if task["task_id"] == tid:
                    r["task_queue"].pop(i)
                    return # 排队任务取消
        
        if target_robot:
            r = self.robot_states[target_robot]
            self.total_reward += self.reward_coeff["completion"]
            self.completed_tasks.append(tid)
            self.node.get_logger().info(f"💰 任务完成奖励 +{self.reward_coeff['completion']}! ({target_robot})")
            
            if r["task_queue"]:
                next_task = r["task_queue"].pop(0)
                r["current_target"] = next_task
                r["current_task_remaining_time"] = next_task["exec_time"]
            else:
                r["idle"] = True
                r["current_target"] = None
                r["current_task_remaining_time"] = 0.0
            
            if len(self.completed_tasks) == 8:
                self.total_reward += self.reward_coeff["all_completed"]

# ===================== 核心调度节点 =====================
class RLDispatcherNode(Node):
    def __init__(self):
        super().__init__("rl_dispatcher_node")
        self.get_logger().info("🚀 初始化 RL 调度节点（优化版：链式派单+拥堵感知）...")
        self.get_logger().warning("⚠️ 【消融实验】已移除锚点逻辑，统一使用小车实时位置！")
        
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])

        if not self.has_parameter("mode"):
            self.declare_parameter("mode", "train")
        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.get_logger().info(f"🔵 当前模式: [{self.mode.upper()}]")
        
        if self.mode == "train":
            self.get_logger().info("🚀 训练模式 - 挂载 Mock 虚拟引擎")
            self.rl_env = MockRMFEnv() 
        elif self.mode == "infer":
            self.get_logger().info("🤖 推理模式 - 挂载 Real 真实引擎")
            self.rl_env = RLDispatchingEnv(self, render_mode="human")
        else:
            self.get_logger().error("❌ 未知模式！请使用 mode:=train 或 mode:=infer")
            sys.exit(1)

        self.model = None
        self._init_rl_model()
        
        if self.mode == "infer":
            self._init_ros_interfaces()

    def _init_ros_interfaces(self):
        self.get_logger().info("🔌 连接 ROS 接口...")
        
        self.waypoint_coords = {
            "n14": Point(x=80.84, y=-28.52), "n13": Point(x=84.44, y=-4.94),
            "n23": Point(x=182.80, y=-42.30), "s08": Point(x=96.61, y=-50.50),
            "s10": Point(x=122.10, y=-46.68), "west_koi_pond": Point(x=34.32, y=-10.13),
            "n08": Point(x=59.61, y=-7.42), "junction_south_west": Point(x=84.56, y=-38.81)
        }

        self.create_subscription(FleetState, "/fleet_states", self.fleet_state_callback, 10)
        self.create_subscription(String, "/task_monitor/start", self.target_callback, 10)
        self.create_subscription(String, "/custom_task_completion", self.completion_callback, 10)
        self.nav_client = self.create_client(SingleNavTask, "/submit_single_nav_task")
        self.create_timer(1.0, self.infer_dispatch_callback)
        self.processed_ids = set()
        
        self.get_logger().info("✅ ROS 接口连接完成")

    def _init_rl_model(self):
        model_path = "./rl_models/dispatching_ppo_final_no_anchor"  # 【消融修改】修改模型保存路径
        log_dir = "./rl_dispatching_tb_log_no_anchor/"              # 【消融修改】修改日志路径
        
        if self.mode == "train":
            policy_kwargs = dict(
                activation_fn=th.nn.ReLU,
                net_arch=dict(pi=[128, 128], vf=[128, 128])
            )
            self.get_logger().info("📚 初始化 PPO 模型...")
            
            # ⬇️⬇️⬇️ 关键修改：降低熵系数 (ent_coef)，移除归一化
            self.model = PPO(
                "MlpPolicy", 
                self.rl_env, 
                verbose=1, 
                device='cpu', 
                tensorboard_log=log_dir,
                policy_kwargs=policy_kwargs,
                learning_rate=3e-4, # 提高学习率
                n_steps=2048,
                batch_size=64, # 减小batch让更新更频繁
                gamma=0.95, # 降低远期折扣
                gae_lambda=0.95,
                ent_coef=0.01, # 降低随机性 (之前是0.1)
                vf_coef=0.5,
                max_grad_norm=0.5,
                n_epochs=10, # 增加更新轮数
                clip_range=0.2
            )

            self.checkpoint_callback = CheckpointCallback(save_freq=10000, save_path="./rl_models/no_anchor/", name_prefix="ppo")  # 【消融修改】
            self.stop_callback = StopTrainingOnRewardThreshold(reward_threshold=1000.0, verbose=1)
            
            self.eval_env = Monitor(MockRMFEnv())
            self.eval_callback = EvalCallback(
                self.eval_env,
                eval_freq=5000,
                n_eval_episodes=5,
                best_model_save_path="./rl_models/best/no_anchor/",  # 【消融修改】
                verbose=1,
                callback_after_eval=self.stop_callback,
                deterministic=False,
                render=False
            )
            # ⚡️ 已移除 RewardNormalizationCallback

        elif self.mode == "infer":
            if os.path.exists(model_path + ".zip"):
                self.get_logger().info(f"📂 加载模型: {model_path}")
                self.model = PPO.load(model_path, device='cpu')
            else:
                self.get_logger().error(f"❌ 未找到模型文件: {model_path}.zip")
                self.model = None

    def start_training(self, total_timesteps=1000000):
        if self.mode != "train": return
            
        self.get_logger().info(f"🔥 开始休克疗法训练 {total_timesteps} 步...")
        self.get_logger().warning("⚠️ 【消融实验】训练无锚点逻辑的模型！")
        start_t = time.time()
        
        self.model.learn(
            total_timesteps=total_timesteps, 
            callback=[self.checkpoint_callback, self.eval_callback] # 移除归一化回调
        )
        
        end_t = time.time()
        train_duration = end_t - start_t
        hours = int(train_duration // 3600)
        minutes = int((train_duration % 3600) // 60)
        seconds = int(train_duration % 60)
        self.get_logger().info(f"⏱️ 训练总用时：{hours}小时{minutes}分钟{seconds}秒")
        
        mean_reward, std_reward = evaluate_policy(self.model, self.eval_env, n_eval_episodes=10)
        self.get_logger().info(f"📊 最终模型评估：平均奖励={mean_reward:.2f}，标准差={std_reward:.2f}")
        
        self.model.save("./rl_models/dispatching_ppo_final_no_anchor")  # 【消融修改】
        self.get_logger().info("💾 模型已保存！")
        rclpy.shutdown()

    def infer_dispatch_callback(self):
        valid_tasks = [t for t in self.rl_env.pending_tasks if t["task_id"].startswith("red_cube_")]
        if not valid_tasks:
            self.get_logger().debug("📭 无有效任务，跳过调度")
            return
        
        obs = self.rl_env._get_observation()
        wait_action_idx = len(self.rl_env.robot_names)
        
        if self.model is None:
            self.get_logger().error("❌ 模型未加载，无法决策")
            return
        
        action, _states = self.model.predict(obs, deterministic=True)
        self.get_logger().info(f"🤖 RL智能体输出动作：{action} (等待动作索引：{wait_action_idx})")

        if action == wait_action_idx:
            # 增加对偷懒行为的检测日志
            if valid_tasks:
                 self.get_logger().warn(f"😡 模型想偷懒(Wait)，但当前有 {len(valid_tasks)} 个任务！")
            else:
                 self.get_logger().info("⏳ RL决策：等待新任务")
            
            for task in self.rl_env.pending_tasks:
                task["wait_time"] += 1.0
        else:
            selected_robot = self.rl_env.robot_names[action]
            robot = self.rl_env.robot_states[selected_robot]
            
            # ========== 【消融修改4】移除锚点逻辑，直接使用小车实时位置 ==========
            anchor_x = self.rl_env.robot_positions[selected_robot].x
            anchor_y = self.rl_env.robot_positions[selected_robot].y

            min_dist = float("inf")
            best_task = None
            for task in valid_tasks:
                dist = np.sqrt((anchor_x - task["x"])**2 + (anchor_y - task["y"])**2)
                if dist < min_dist:
                    min_dist = dist
                    best_task = task
            
            if best_task:
                self._call_ros_service(selected_robot, best_task["task_id"], best_task["waypoint"])

    def _call_ros_service(self, robot, tid, wp):
        if not self.nav_client.service_is_ready():
            self.get_logger().warn("⚠️ 服务未就绪，跳过任务下发")
            return
            
        req = SingleNavTask.Request()
        req.target_waypoint = wp
        req.fleet_name = "deliveryRobot"
        req.robot_name = robot
        req.priority = 1
        
        future = self.nav_client.call_async(req)
        future.add_done_callback(lambda f, r=robot, t=tid, w=wp: self._service_done(f, t, r, w))

    def _service_done(self, future, tid, robot, wp):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"🚀 调度成功：{robot} -> {tid} @ {res.message}")
                for i, t in enumerate(self.rl_env.pending_tasks):
                    if t["task_id"] == tid:
                        self.rl_env.pending_tasks.pop(i)
                        
                        if self.rl_env.robot_states[robot]["idle"]:
                            self.rl_env.robot_states[robot]["idle"] = False
                        
                        waypoint_x = self.waypoint_coords[wp].x
                        waypoint_y = self.waypoint_coords[wp].y
                        
                        r_state = self.rl_env.robot_states[robot]
                        if r_state["task_queue"]:
                            prev = r_state["task_queue"][-1]
                            dist = np.sqrt((prev["x"] - waypoint_x)**2 + (prev["y"] - waypoint_y)**2)
                        elif r_state["current_target"]:
                            prev = r_state["current_target"]
                            dist = np.sqrt((prev["x"] - waypoint_x)**2 + (prev["y"] - waypoint_y)**2)
                        else:
                            r_pos = self.rl_env.robot_positions[robot]
                            dist = np.sqrt((r_pos.x - waypoint_x)**2 + (r_pos.y - waypoint_y)**2)

                        new_task = {
                            "task_id": tid,
                            "waypoint": wp,
                            "x": waypoint_x,
                            "y": waypoint_y,
                            "exec_time": dist / 1.0
                        }
                        r_state["task_queue"].append(new_task)
                        
                        # ========== 【消融修改5】队列重排序 - 移除锚点，用小车实时位置 ==========
                        anchor_x, anchor_y = self.rl_env.robot_positions[robot].x, self.rl_env.robot_positions[robot].y
                        
                        def sort_by_distance(task):
                            return np.sqrt((anchor_x - task["x"])**2 + (anchor_y - task["y"])**2)
                        
                        r_state["task_queue"].sort(key=sort_by_distance)
                        # =========================================
                        break
            else:
                self.get_logger().error(f"❌ 调度失败：{tid} -> {res.message}")
        except Exception as e:
            self.get_logger().error(f"❌ 服务调用异常：{e}")


    def fleet_state_callback(self, msg):
        if msg.name != "deliveryRobot": return
        for r in msg.robots:
            if r.name in self.rl_env.robot_names:
                self.rl_env.update_robot_position(r.name, r.location.x, r.location.y)
                if not r.task_id:
                     if not self.rl_env.robot_states[r.name]["task_queue"]:
                         self.rl_env.robot_states[r.name]["idle"] = True
                         self.rl_env.robot_states[r.name]["current_task_remaining_time"] = 0.0
                else:
                     self.rl_env.robot_states[r.name]["idle"] = False

    def target_callback(self, msg):
        data = msg.data.split(",")
        if len(data) < 2: return
        tid, wp = data[0].strip(), data[1].strip()
        if not tid.startswith("red_cube_") or tid in self.processed_ids: return
        if wp in self.waypoint_coords:
            p = self.waypoint_coords[wp]
            self.rl_env.add_task(tid, wp, p.x, p.y)
            self.processed_ids.add(tid)
            self.get_logger().info(f"📥 新增任务：{tid} @ {wp}")

    def completion_callback(self, msg):
        data = msg.data.split(",")
        if len(data) < 1: return
        tid = data[0].strip()
        if tid.startswith("red_cube_"):
            self.rl_env.complete_task(tid)
            if tid in self.processed_ids:
                self.processed_ids.remove(tid)
            # 状态清理兜底
            for robot in self.rl_env.robot_names:
                r = self.rl_env.robot_states[robot]
                if not r["idle"] and r["current_target"] and r["current_target"]["task_id"] == tid:
                    r["idle"] = True
                    r["current_target"] = None
                    r["current_task_remaining_time"] = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = RLDispatcherNode()
    if node.mode == "train":
        train_thread = threading.Thread(target=node.start_training, args=(1000000,), daemon=True)
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
