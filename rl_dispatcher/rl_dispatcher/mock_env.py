import gymnasium as gym
from gymnasium import spaces
import numpy as np
import math
import random

class MockRMFEnv(gym.Env):
    """
    纯数学模拟的 RMF 环境（极速训练专用）
    状态空间与动作空间必须与真实的 RLDispatchingEnv 保持 100% 一致
    核心调整：固定8个任务按15秒间隔发布、修正超时逻辑、延长最大步数
    """
    def __init__(self):
        super().__init__()
        
        # 1. 定义虚拟小车 (模拟真实环境中的坐标)
        self.robot_names = ["deliveryRobot_0", "deliveryRobot_1", "deliveryRobot_2"]
        self.robot_states = {
            "deliveryRobot_0": {"x": 96.59527587890625, "y": -51.96450424194336, "idle": True, "target": None, "speed": 1.0},
            "deliveryRobot_1": {"x": 152.3477325439453, "y": -44.31863021850586, "idle": True, "target": None, "speed": 1.0},
            "deliveryRobot_2": {"x": 14.776845932006836, "y": -9.279278755187988, "idle": True, "target": None, "speed": 1.0}
        }
        
        # 2. 固定8个任务列表（按指定顺序）
        self.fixed_tasks = [
            {"task_id": "red_cube_n14", "waypoint": "n14", "x": 80.84, "y": -28.52},
            {"task_id": "red_cube_n13", "waypoint": "n13", "x": 84.44, "y": -4.94},
            {"task_id": "red_cube_n23", "waypoint": "n23", "x": 182.80, "y": -42.30},
            {"task_id": "red_cube_s08", "waypoint": "s08", "x": 96.61, "y": -50.50},
            {"task_id": "red_cube_s10", "waypoint": "s10", "x": 122.10, "y": -46.68},
            {"task_id": "red_cube_west_koi_pond", "waypoint": "west_koi_pond", "x": 34.32, "y": -10.13},
            {"task_id": "red_cube_n08", "waypoint": "n08", "x": 59.61, "y": -7.42},
            {"task_id": "red_cube_junction_south_west", "waypoint": "junction_south_west", "x": 84.56, "y": -38.81}
        ]
        
        # 3. 任务发布参数（固定）
        self.task_release_interval = 15.0  # 固定15秒发一个
        self.current_task_idx = 0          # 当前要发布的任务索引
        self.next_release_time = self.task_release_interval  # 第一个任务15秒发布
        self.task_release_done = False     # 是否发完8个任务
        
        # 4. 超时参数（修正版：仅有空闲车时计时）
        self.task_timeout = 60.0  # 超时时间60秒
        self.pending_tasks = []   # 待处理任务队列
        
        # 5. 动作空间 & 状态空间 (与真实环境完全一致)
        self.action_space = spaces.Discrete(len(self.robot_names))
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(14,), dtype=np.float32)

        # 6. 仿真参数（延长最大步数）
        self.dt = 1.0  # 虚拟时间步长
        self.current_time = 0.0
        self.max_steps = 1000  # 延长到1000步，保证8个任务能处理完
        self.step_count = 0
        self.completed_tasks = 0

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.current_time = 0.0
        self.step_count = 0
        self.completed_tasks = 0
        self.pending_tasks = []
        
        # 重置小车状态
        self.robot_states = {
            "deliveryRobot_0": {"x": 96.59527587890625, "y": -51.96450424194336, "idle": True, "target": None, "speed": 1.0},
            "deliveryRobot_1": {"x": 152.3477325439453, "y": -44.31863021850586, "idle": True, "target": None, "speed": 1.0},
            "deliveryRobot_2": {"x": 14.776845932006836, "y": -9.279278755187988, "idle": True, "target": None, "speed": 1.0}
        }
        
        # 重置任务发布参数
        self.current_task_idx = 0
        self.next_release_time = self.task_release_interval
        self.task_release_done = False
        
        return self._get_obs(), {}

    def step(self, action):
        self.step_count += 1
        self.current_time += self.dt
        reward = -0.01  # 降低基础时间惩罚，避免模型恐慌
        
        # --- 1. 固定任务发布逻辑（15秒一个，发完8个停止）---
        if not self.task_release_done and self.current_time >= self.next_release_time:
            if self.current_task_idx < len(self.fixed_tasks):
                # 发布当前索引的任务
                task = self.fixed_tasks[self.current_task_idx]
                self.pending_tasks.append({
                    "x": task["x"],
                    "y": task["y"],
                    "wait_time": 0.0,
                    "task_id": task["task_id"],
                    "waypoint": task["waypoint"]
                })
                # 更新索引和下一次发布时间
                self.current_task_idx += 1
                self.next_release_time = self.current_time + self.task_release_interval
            else:
                # 8个任务发完，标记停止
                self.task_release_done = True
        
        # --- 2. 修正版超时逻辑：只有有空闲车时，任务才计时 ---
        if self.pending_tasks:
            # 统计当前空闲车数量
            idle_robot_count = sum([1 for r in self.robot_states.values() if r["idle"]])
            first_task = self.pending_tasks[0]
            
            if idle_robot_count > 0:
                # 有空闲车：等待时间+1
                first_task["wait_time"] += self.dt
                # 超时判断：有空闲车+等待超60秒 → 惩罚
                if first_task["wait_time"] > self.task_timeout:
                    reward += -50.0  # 超时惩罚（仅罚决策失误）
            # 无空闲车：等待时间不增加，不惩罚
        
        # --- 3. 调度逻辑模拟 ---
        selected_robot_name = self.robot_names[action]
        robot = self.robot_states[selected_robot_name]
        
        if self.pending_tasks:
            task = self.pending_tasks[0]
            
            if not robot["idle"]:
                # 选了忙碌的车：轻惩罚
                reward += -10.0
            else:
                # 选了空闲车：距离惩罚 + 正向奖励
                task_x, task_y = task["x"], task["y"]
                dist = math.hypot(robot["x"] - task_x, robot["y"] - task_y)
                reward += -0.1 * dist  # 距离越远罚越多
                reward += 5.0          # 选对空闲车的正向奖励
                
                # 更新状态
                robot["idle"] = False
                robot["target"] = (task_x, task_y)
                self.pending_tasks.pop(0)

        # --- 4. 物理运动模拟 ---
        for name, r in self.robot_states.items():
            if not r["idle"] and r["target"]:
                tx, ty = r["target"]
                dx, dy = tx - r["x"], ty - r["y"]
                dist = math.hypot(dx, dy)
                move_dist = r["speed"] * self.dt
                
                if dist <= move_dist:
                    # 到达：完成任务奖励
                    r["x"], r["y"] = tx, ty
                    r["idle"] = True
                    r["target"] = None
                    reward += 80.0  # 与真实环境对齐
                    self.completed_tasks += 1
                else:
                    # 移动
                    angle = math.atan2(dy, dx)
                    r["x"] += math.cos(angle) * move_dist
                    r["y"] += math.sin(angle) * move_dist

        # --- 5. 结束条件 ---
        terminated = False
        # 提前结束条件：任务发完 + 无待处理任务 + 所有车空闲
        if self.task_release_done and len(self.pending_tasks) == 0 and all([r["idle"] for r in self.robot_states.values()]):
            terminated = True
        # 最大步数结束
        truncated = self.step_count >= self.max_steps
        
        # 返回额外信息，方便训练监控
        info = {
            "completed_tasks": self.completed_tasks,
            "pending_tasks_count": len(self.pending_tasks),
            "task_release_done": self.task_release_done
        }
        
        return self._get_obs(), reward, terminated, truncated, info

    def _get_obs(self):
        """构建与真实环境一模一样的状态向量"""
        # 1. 小车状态 (x, y, idle)
        robot_obs = []
        for name in self.robot_names:
            r = self.robot_states[name]
            robot_obs.extend([
                r["x"] / 200.0, 
                r["y"] / 50.0, 
                1.0 if r["idle"] else 0.0
            ])
        
        # 2. 任务状态
        task_obs = [0.0, 0.0, 0.0]
        if self.pending_tasks:
            t = self.pending_tasks[0]
            task_obs = [
                t["x"] / 200.0,
                t["y"] / 50.0,
                min(t["wait_time"] / 100.0, 1.0)  # 暂时保留原有归一化
            ]
            
        # 3. 全局状态
        global_obs = [
            self.current_time / 1000.0,
            self.completed_tasks / 8.0  # 按8个任务归一化，最大1.0
        ]
        
        return np.array(robot_obs + task_obs + global_obs, dtype=np.float32)
