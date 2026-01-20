#1.19 zjt修改
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import math
import random

class MockRMFEnv(gym.Env):
    """
    1.19 纯数学模拟的 RMF 环境（极速训练专用）
    优化版：增加了随机初始位置、随机任务顺序、修正了归一化参数，防止过拟合。
    """
    def __init__(self):
        super().__init__()
        
        # 1. 定义虚拟小车
        self.robot_names = ["deliveryRobot_0", "deliveryRobot_1", "deliveryRobot_2"]
        # 初始化时先给个默认值，真正的随机化在 reset() 里做
        self.robot_states = {
            name: {"x": 0.0, "y": 0.0, "idle": True, "target": None, "speed": 1.0}
            for name in self.robot_names
        }
        
        # 2. 定义任务池 (不再是固定顺序列表，而是作为随机抽取的池子)
        self.all_possible_tasks = [
            {"task_id": "red_cube_n14", "waypoint": "n14", "x": 80.84, "y": -28.52},
            {"task_id": "red_cube_n13", "waypoint": "n13", "x": 84.44, "y": -4.94},
            {"task_id": "red_cube_n23", "waypoint": "n23", "x": 182.80, "y": -42.30},
            {"task_id": "red_cube_s08", "waypoint": "s08", "x": 96.61, "y": -50.50},
            {"task_id": "red_cube_s10", "waypoint": "s10", "x": 122.10, "y": -46.68},
            {"task_id": "red_cube_west_koi_pond", "waypoint": "west_koi_pond", "x": 34.32, "y": -10.13},
            {"task_id": "red_cube_n08", "waypoint": "n08", "x": 59.61, "y": -7.42},
            {"task_id": "red_cube_junction_south_west", "waypoint": "junction_south_west", "x": 84.56, "y": -38.81}
        ]
        self.current_episode_tasks = [] # 当前回合要执行的任务列表
        
        # 3. 任务发布参数
        self.task_release_interval = 15.0  
        self.current_task_idx = 0          
        self.next_release_time = self.task_release_interval
        self.task_release_done = False     
        
        # 4. 超时参数
        self.task_timeout = 60.0
        self.pending_tasks = []   
        
        # 5. 动作空间 & 状态空间
        self.action_space = spaces.Discrete(len(self.robot_names))
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(14,), dtype=np.float32)

        # 6. 仿真参数
        self.dt = 1.0  
        self.current_time = 0.0
        self.max_steps = 1000 
        self.step_count = 0
        self.completed_tasks = 0

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.current_time = 0.0
        self.step_count = 0
        self.completed_tasks = 0
        self.pending_tasks = []
        
        # === 优化点 1: 随机初始化小车位置 ===
        # 防止模型记住 "开局 Robot0 就在右边" 这种固定模式
        # 地图大致范围: X[0, 190], Y[-60, 0]
        for name in self.robot_names:
            self.robot_states[name] = {
                "x": random.uniform(0.0, 190.0), # 随机生成 X 坐标
                "y": random.uniform(-60.0, 0.0), # 随机生成 Y 坐标
                "idle": True,
                "target": None,
                "speed": 1.0
            }
            
        # === 优化点 2: 随机打乱任务顺序 ===
        # 防止模型背诵任务序列 (例如: "第3个任务一定是去n23")
        # 每次 reset 都重新复制一份并打乱
        self.current_episode_tasks = self.all_possible_tasks.copy()
        random.shuffle(self.current_episode_tasks)
        
        # 重置任务发布参数
        self.current_task_idx = 0
        self.next_release_time = self.task_release_interval
        self.task_release_done = False
        
        return self._get_obs(), {}

    def step(self, action):
        self.step_count += 1
        self.current_time += self.dt
        reward = -0.01 
        
        # --- 1. 任务发布逻辑 (使用打乱后的 current_episode_tasks) ---
        if not self.task_release_done and self.current_time >= self.next_release_time:
            if self.current_task_idx < len(self.current_episode_tasks):
                task = self.current_episode_tasks[self.current_task_idx]
                self.pending_tasks.append({
                    "x": task["x"],
                    "y": task["y"],
                    "wait_time": 0.0,
                    "task_id": task["task_id"],
                    "waypoint": task["waypoint"]
                })
                self.current_task_idx += 1
                self.next_release_time = self.current_time + self.task_release_interval
            else:
                self.task_release_done = True
        
        # --- 2. 超时逻辑 ---
        if self.pending_tasks:
            idle_robot_count = sum([1 for r in self.robot_states.values() if r["idle"]])
            first_task = self.pending_tasks[0]
            
            if idle_robot_count > 0:
                first_task["wait_time"] += self.dt
                if first_task["wait_time"] > self.task_timeout:
                    reward += -50.0 
        
        # --- 3. 调度逻辑 ---
        selected_robot_name = self.robot_names[action]
        robot = self.robot_states[selected_robot_name]
        
        if self.pending_tasks:
            task = self.pending_tasks[0]
            
            if not robot["idle"]:
                reward += -10.0 # 惩罚选择忙碌车
            else:
                task_x, task_y = task["x"], task["y"]
                dist = math.hypot(robot["x"] - task_x, robot["y"] - task_y)
                
                # 关键：距离惩罚系数 (让模型学会选近的)
                reward += -0.1 * dist 
                reward += 5.0 # 选对空闲车的正向奖励
                
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
                    r["x"], r["y"] = tx, ty
                    r["idle"] = True
                    r["target"] = None
                    reward += 80.0 
                    self.completed_tasks += 1
                else:
                    angle = math.atan2(dy, dx)
                    r["x"] += math.cos(angle) * move_dist
                    r["y"] += math.sin(angle) * move_dist

        # --- 5. 结束条件 ---
        terminated = False
        if self.task_release_done and len(self.pending_tasks) == 0 and all([r["idle"] for r in self.robot_states.values()]):
            terminated = True
        
        truncated = self.step_count >= self.max_steps
        
        info = {
            "completed_tasks": self.completed_tasks,
            "pending_tasks_count": len(self.pending_tasks)
        }
        
        return self._get_obs(), reward, terminated, truncated, info

    def _get_obs(self):
        """
        构建状态向量
        优化点 3: 修正归一化参数，防止数值溢出 [-1, 1]
        """
        # 1. 小车状态 (x, y, idle)
        robot_obs = []
        for name in self.robot_names:
            r = self.robot_states[name]
            robot_obs.extend([
                # 你的地图最大X约182，最大Y约-52
                # 使用更大的分母确保归一化在 [-1, 1] 之间且留有余地
                r["x"] / 200.0, 
                r["y"] / 100.0,  # 之前是 60.0，现在改大一点更安全
                1.0 if r["idle"] else 0.0
            ])
        
        # 2. 任务状态
        task_obs = [0.0, 0.0, 0.0]
        if self.pending_tasks:
            t = self.pending_tasks[0]
            task_obs = [
                t["x"] / 200.0,  # 同样修正分母
                t["y"] / 100.0,  # 同样修正分母
                min(t["wait_time"] / 100.0, 1.0)
            ]
            
        # 3. 全局状态
        global_obs = [
            self.current_time / 1000.0,
            self.completed_tasks / 8.0
        ]
        
        return np.array(robot_obs + task_obs + global_obs, dtype=np.float32)

