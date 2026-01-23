# 核心修改：保留锚点+排序逻辑，但将奖励系数减半 (验证休克疗法的作用)
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import math
import random

class MockRMFEnv(gym.Env):
    """
    消融实验：RL - 温和奖励
    配置：锚点派单 + 队列重排序 + 奖惩系数减半
    目的：验证“休克疗法”对加速收敛的作用
    """
    def __init__(self):
        super().__init__()
        # 保持 reward_scale = 1.0，只修改 coeff 数值
        self.reward_scale = 1.0
        self.episode_reward = 0.0
        
        # 1. 小车配置
        self.robot_names = ["deliveryRobot_0", "deliveryRobot_1", "deliveryRobot_2"]
        self.fixed_robot_init = {
            "deliveryRobot_0": {"x": 96.59527587890625, "y": -51.96450424194336},
            "deliveryRobot_1": {"x": 152.3477325439453, "y": -44.31863021850586},
            "deliveryRobot_2": {"x": 14.776845932006836, "y": -9.279278755187988}
        }
        self.robot_states = {}
        
        # 2. 任务池
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
        self.current_episode_tasks = []
        
        # 3. 任务发布参数
        self.task_release_interval = 20.0  
        self.current_task_idx = 0          
        self.next_release_time = self.task_release_interval
        self.task_release_done = False     
        
        # 4. 超时参数
        self.task_timeout = 120.0   
        self.pending_tasks = []   
        
        # 5. 动作空间 & 状态空间 (54维)
        self.action_space = spaces.Discrete(len(self.robot_names) + 1)
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(54,), dtype=np.float32)

        # 6. 仿真参数
        self.dt = 1.0  
        self.current_time = 0.0
        self.max_steps = 1000 
        self.step_count = 0
        self.completed_tasks = 0
        
# 7. 奖励系数 (⚡️ 1/10 稀疏奖励版：验证信号强度对收敛的影响 ⚡️)
        self.reward_coeff = {
            "valid_assign": 0.5,         # 原 5.0 -> 0.5 (奖励变得很不明显)
            "wait_with_task": -0.2,      # 原 -2.0 -> -0.2 (惩罚变得不痛不痒)
            "wait_no_task": 0.01,        # 原 0.1 -> 0.01
            
            "distance": -0.0005,         # 原 -0.005 -> -0.0005
            "time_congestion": -0.001,   # 原 -0.01 -> -0.001
            "queue_congestion": -0.02,   # 原 -0.2 -> -0.02
            
            "completion": 0.5,           # 原 5.0 -> 0.5
            "all_completed": 2.0,        # 原 20.0 -> 2.0
            
            "invalid_selection": -0.5,   # 原 -5.0 -> -0.5
            "batch_completion": 0.2,     # 原 2.0 -> 0.2
            "step": -0.001               # 原 -0.01 -> -0.001
        }

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.current_time = 0.0
        self.step_count = 0
        self.completed_tasks = 0
        self.pending_tasks = []
        self.episode_reward = 0.0
        
        for name in self.robot_names:
            init_pos = self.fixed_robot_init[name]
            self.robot_states[name] = {
                "x": init_pos["x"],
                "y": init_pos["y"],
                "idle": True,
                "current_target": None,
                "task_queue": [],
                "speed": 1.0,
                "current_task_remaining_time": 0.0
            }
        
        self.current_episode_tasks = self.all_possible_tasks.copy()
        random.shuffle(self.current_episode_tasks)
        
        self.current_task_idx = 0
        self.next_release_time = self.task_release_interval
        self.task_release_done = False
        
        return self._get_obs(), {}

    def step(self, action):
        self.step_count += 1
        self.current_time += self.dt
        reward = self.reward_coeff["step"] 

        def clip_reward(r, min_r=-2.0, max_r=2.0): # 裁剪范围也相应减半
            return max(min(r, max_r), min_r)
        
        wait_action_idx = len(self.robot_names)
        info = {
            "action_type": "wait" if action == wait_action_idx else "assign",
            "completed_tasks": self.completed_tasks,
            "robot_task_queues": {name: len(r["task_queue"]) for name, r in self.robot_states.items()}
        }
        
        # --- 1. 任务发布逻辑 ---
        if not self.task_release_done and self.current_time >= self.next_release_time:
            if self.current_task_idx < len(self.current_episode_tasks):
                task = self.current_episode_tasks[self.current_task_idx]
                self.pending_tasks.append({
                    "x": task["x"],
                    "y": task["y"],
                    "wait_time": 0.0,
                    "task_id": task["task_id"],
                    "waypoint": task["waypoint"],
                    "timeout_count": 0,
                    "exec_time": 0.0 
                })
                self.current_task_idx += 1
                self.next_release_time = self.current_time + self.task_release_interval
            else:
                self.task_release_done = True
        
        # --- 2. 动作处理 ---
        if action == wait_action_idx:
            if self.pending_tasks:
                reward += self.reward_coeff["wait_with_task"]
                for task in self.pending_tasks:
                    task["wait_time"] += self.dt
            else:
                reward += self.reward_coeff["wait_no_task"]
        else:
            if not self.pending_tasks:
                reward += self.reward_coeff["invalid_selection"]
            else:
                reward += self.reward_coeff["valid_assign"]
                
                selected_robot_name = self.robot_names[action]
                robot = self.robot_states[selected_robot_name]
                info["selected_robot"] = selected_robot_name

                # 锚点逻辑 (保留)
                if robot["task_queue"]:
                    anchor_wp = robot["task_queue"][-1]
                    anchor_x, anchor_y = anchor_wp["x"], anchor_wp["y"]
                elif robot["current_target"]:
                    anchor_x, anchor_y = robot["current_target"]["x"], robot["current_target"]["y"]
                else:
                    anchor_x, anchor_y = robot["x"], robot["y"]

                # 寻找最近任务 (保留)
                min_dist = float("inf")
                best_task_idx = 0
                for i, task in enumerate(self.pending_tasks):
                    dist = math.hypot(anchor_x - task["x"], anchor_y - task["y"])
                    if dist < min_dist:
                        min_dist = dist
                        best_task_idx = i
                
                task = self.pending_tasks.pop(best_task_idx)
                
                # 计算奖励
                reward += self.reward_coeff["distance"] * min_dist
                
                wait_time_for_new_task = robot["current_task_remaining_time"] + \
                                         sum([t["exec_time"] for t in robot["task_queue"]])
                reward += self.reward_coeff["time_congestion"] * wait_time_for_new_task
                
                new_queue_len = len(robot["task_queue"]) + (1 if not robot["idle"] else 0)
                reward += self.reward_coeff["queue_congestion"] * (new_queue_len ** 2)

                # 构建任务信息
                task_exec_time = min_dist / robot["speed"]
                task_info = {
                    "x": task["x"],
                    "y": task["y"],
                    "task_id": task["task_id"],
                    "waypoint": task["waypoint"],
                    "exec_time": task_exec_time
                }
                
                if robot["idle"]:
                    robot["idle"] = False
                    robot["current_target"] = task_info
                    robot["current_task_remaining_time"] = task_exec_time
                else:
                    robot["task_queue"].append(task_info)
                    
                    # 队列重排序逻辑 (保留)
                    if robot["current_target"]:
                        curr_x, curr_y = robot["current_target"]["x"], robot["current_target"]["y"]
                    else:
                        curr_x, curr_y = robot["x"], robot["y"]
                    
                    def sort_by_distance(t):
                        return math.hypot(curr_x - t["x"], curr_y - t["y"])
                    
                    robot["task_queue"].sort(key=sort_by_distance)
        
        # --- 3. 物理运动模拟 ---
        task_completion_rewards = 0
        for name, r in self.robot_states.items():
            if not r["idle"] and r["current_target"]:
                r["current_task_remaining_time"] -= self.dt
                
                tx, ty = r["current_target"]["x"], r["current_target"]["y"]
                dx, dy = tx - r["x"], ty - r["y"]
                dist = math.hypot(dx, dy)
                if dist > 0:
                    move = min(dist, r["speed"] * self.dt)
                    r["x"] += (dx/dist) * move
                    r["y"] += (dy/dist) * move
                
                if r["current_task_remaining_time"] <= 0:
                    r["x"], r["y"] = tx, ty
                    task_completion_rewards += self.reward_coeff["completion"]
                    self.completed_tasks += 1
                    
                    if r["task_queue"]:
                        next_task = r["task_queue"].pop(0)
                        r["current_target"] = next_task
                        r["current_task_remaining_time"] = next_task["exec_time"]
                    else:
                        r["idle"] = True
                        r["current_target"] = None
                        r["current_task_remaining_time"] = 0.0
                    
                    if self.completed_tasks == len(self.all_possible_tasks):
                        task_completion_rewards += self.reward_coeff["all_completed"]
        
        reward += task_completion_rewards
        
        if len(self.pending_tasks) == 0 and self.completed_tasks > 0 and self.task_release_done:
            reward += self.reward_coeff["batch_completion"]
        
        # --- 4. 结算与返回 ---
        reward = clip_reward(reward) * self.reward_scale 
        self.episode_reward += reward
        
        terminated = False
        all_robots_idle = all([r["idle"] and len(r["task_queue"]) == 0 for r in self.robot_states.values()])
        if (self.task_release_done and len(self.pending_tasks) == 0 and 
            all_robots_idle and self.completed_tasks == len(self.all_possible_tasks)):
            terminated = True
            
        truncated = self.step_count >= self.max_steps
        
        return self._get_obs(), reward, terminated, truncated, info

    def _get_obs(self):
        robot_obs = []
        for name in self.robot_names:
            r = self.robot_states[name]
            
            if r["task_queue"]:
                last_wp = r["task_queue"][-1]
                last_x, last_y = last_wp["x"], last_wp["y"]
            elif r["current_target"]:
                last_x, last_y = r["current_target"]["x"], r["current_target"]["y"]
            else:
                last_x, last_y = r["x"], r["y"]
            
            total_time = r["current_task_remaining_time"]
            for t in r["task_queue"]:
                total_time += t["exec_time"]

            robot_obs.extend([
                r["x"] / 200.0,
                r["y"] / 100.0,
                1.0 if r["idle"] else 0.0,
                min(len(r["task_queue"]) / 5.0, 1.0),
                last_x / 200.0,                 
                last_y / 100.0,                 
                min(total_time / 200.0, 1.0)    
            ])
        
        task_obs = []
        for _ in range(8):
            task_obs.extend([0.0, 0.0, 0.0])
        for i, task in enumerate(self.pending_tasks[:8]):
            task_obs[i*3] = task["x"] / 200.0
            task_obs[i*3+1] = task["y"] / 100.0
            task_obs[i*3+2] = min(task["wait_time"] / 600.0, 1.0)
        
        total_queue_len = sum([len(r["task_queue"]) for r in self.robot_states.values()])
        idle_robot_count = sum([1 for r in self.robot_states.values() if r["idle"]])
        queue_lengths = [len(r["task_queue"]) for r in self.robot_states.values()]
        max_queue_len = max(queue_lengths) if queue_lengths else 0
        min_queue_len = min(queue_lengths) if queue_lengths else 0
        
        global_obs = [
            self.current_time / 1000.0,
            self.completed_tasks / 8.0,
            len(self.pending_tasks) / 8.0,
            idle_robot_count / 3.0,
            total_queue_len / 15.0,
            (self.completed_tasks + len(self.pending_tasks)) / 8.0,
            np.var(queue_lengths) / 2.5 if queue_lengths else 0.0,
            max_queue_len / 5.0,
            (max_queue_len - min_queue_len) / 5.0 if queue_lengths else 0.0,
        ]
        global_obs = [min(max(x, -1.0), 1.0) for x in global_obs]
        
        return np.array(robot_obs + task_obs + global_obs, dtype=np.float32)