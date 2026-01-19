import gymnasium as gym
from gymnasium import spaces
import numpy as np
import math
import random
# 1.18 可以跑出初步结果，进行第一次修改，每隔一段时间发布目标点
class MockRMFEnv(gym.Env):
    """
    纯数学模拟的 RMF 环境（极速训练专用）
    状态空间与动作空间必须与真实的 RLDispatchingEnv 保持 100% 一致
    """
    def __init__(self):
        super().__init__()
        
        # 1. 定义虚拟小车 (模拟真实环境中的坐标)
        self.robot_names = ["deliveryRobot_0", "deliveryRobot_1", "deliveryRobot_2"]
        # 初始都在 (0,0)

        # 替换为真实坐标（对应你的fleet_states输出）
        self.robot_states = {
            "deliveryRobot_0": {"x": 96.59527587890625, "y": -51.96450424194336, "idle": True, "target": None, "speed": 1.0},
            "deliveryRobot_1": {"x": 152.3477325439453, "y": -44.31863021850586, "idle": True, "target": None, "speed": 1.0},
            "deliveryRobot_2": {"x": 14.776845932006836, "y": -9.279278755187988, "idle": True, "target": None, "speed": 1.0}
        }
        
        # 2. 定义航点 (复用真实地图坐标，确保模型学到的距离感是真实的)
        self.waypoints = {
            "n14": (80.84, -28.52), "n13": (84.44, -4.94), "n23": (182.80, -42.30),
            "s08": (96.61, -50.50), "s10": (122.10, -46.68), "west_koi_pond": (34.32, -10.13),
            "s11": (152.73, -43.00), "junction_south_west": (84.56, -38.81)
        }
        self.waypoint_keys = list(self.waypoints.keys())

        # 3. 动作空间 & 状态空间 (必须与真实环境完全一致！)
        self.action_space = spaces.Discrete(len(self.robot_names))
        # 状态维度：3台车(x,y,idle) + 1个任务(x,y,wait) + 全局(time, done_count) = 14维
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(14,), dtype=np.float32)

        # 仿真参数
        self.dt = 1.0  # 虚拟时间步长
        self.current_time = 0.0
        self.max_steps = 500
        self.step_count = 0
        self.pending_tasks = [] 
        self.completed_tasks = 0

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.current_time = 0.0
        self.step_count = 0
        self.completed_tasks = 0
        self.pending_tasks = []
        
        # 替换为重置回真实初始坐标
        self.robot_states = {
            "deliveryRobot_0": {"x": 96.59527587890625, "y": -51.96450424194336, "idle": True, "target": None, "speed": 1.0},
            "deliveryRobot_1": {"x": 152.3477325439453, "y": -44.31863021850586, "idle": True, "target": None, "speed": 1.0},
            "deliveryRobot_2": {"x": 14.776845932006836, "y": -9.279278755187988, "idle": True, "target": None, "speed": 1.0}
        }
        
        # 生成初始任务
        self._generate_random_task()
        return self._get_obs(), {}

    def step(self, action):
        self.step_count += 1
        self.current_time += self.dt
        reward = -0.1 # 基础时间惩罚
        
        # --- 1. 调度逻辑模拟 ---
        selected_robot_name = self.robot_names[action]
        robot = self.robot_states[selected_robot_name]
        
        if self.pending_tasks:
            task = self.pending_tasks[0]
            
            if not robot["idle"]:
                # 惩罚：选了忙碌的车
                reward += -10.0
            else:
                # 成功指派
                task_x, task_y = task["x"], task["y"]
                dist = math.hypot(robot["x"] - task_x, robot["y"] - task_y)
                
                # 奖励：距离越近奖励越高 (参数要和真实环境对齐)
                reward += -0.1 * dist 
                
                # 更新状态
                robot["idle"] = False
                robot["target"] = (task_x, task_y)
                self.pending_tasks.pop(0)

        # --- 2. 物理运动模拟 (瞬移或匀速运动) ---
        for name, r in self.robot_states.items():
            if not r["idle"] and r["target"]:
                tx, ty = r["target"]
                dx, dy = tx - r["x"], ty - r["y"]
                dist = math.hypot(dx, dy)
                move_dist = r["speed"] * self.dt
                
                if dist <= move_dist:
                    # 到达
                    r["x"], r["y"] = tx, ty
                    r["idle"] = True
                    r["target"] = None
                    reward += 100.0 # 完成任务大奖
                    self.completed_tasks += 1
                    self._generate_random_task()
                else:
                    # 移动
                    angle = math.atan2(dy, dx)
                    r["x"] += math.cos(angle) * move_dist
                    r["y"] += math.sin(angle) * move_dist

        # 3. 待执行任务等待时间增加
        if self.pending_tasks:
            self.pending_tasks[0]["wait_time"] += self.dt
            # 超时模拟 (可选)
            if self.pending_tasks[0]["wait_time"] > 60.0:
                reward += -50.0

        # 4. 结束条件
        terminated = False
        truncated = self.step_count >= self.max_steps
        
        return self._get_obs(), reward, terminated, truncated, {}

    def _generate_random_task(self):
        if len(self.pending_tasks) == 0:
            wp_name = random.choice(self.waypoint_keys)
            coords = self.waypoints[wp_name]
            self.pending_tasks.append({
                "x": coords[0], "y": coords[1], "wait_time": 0.0
            })

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
                min(t["wait_time"] / 100.0, 1.0)
            ]
            
        # 3. 全局状态
        global_obs = [
            self.current_time / 1000.0,
            self.completed_tasks / 10.0
        ]
        
        return np.array(robot_obs + task_obs + global_obs, dtype=np.float32)                                              