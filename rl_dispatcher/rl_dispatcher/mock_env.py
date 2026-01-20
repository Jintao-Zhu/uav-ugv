import gymnasium as gym
from gymnasium import spaces
import numpy as np
import math
import random

class MockRMFEnv(gym.Env):
    """
    适配需求：8任务必须完成+无待命点返回+超时不丢任务
    核心改进：
    1. 超时仅惩罚不丢弃任务（确保8个任务都能被处理）
    2. 移除所有“返回待命点”逻辑（匹配小车完成任务后原地待命）
    3. 任务发布间隔改为20秒（与你实际一致）
    4. 奖励聚焦“完成任务”，弱化无意义惩罚
    5. 下调奖励尺度，增加奖励裁剪，解决训练震荡问题
    """
    def __init__(self):
        super().__init__()
        
        # 1. 小车配置（无待命点，完成任务后停在任务点）
        self.robot_names = ["deliveryRobot_0", "deliveryRobot_1", "deliveryRobot_2"]
        self.fixed_robot_init = {
            "deliveryRobot_0": {"x": 96.59527587890625, "y": -51.96450424194336},
            "deliveryRobot_1": {"x": 152.3477325439453, "y": -44.31863021850586},
            "deliveryRobot_2": {"x": 14.776845932006836, "y": -9.279278755187988}
        }
        self.robot_states = {}
        
        # 2. 任务池（8个固定任务，必须全部完成）
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
        
        # 3. 任务发布参数（与你实际一致：20秒发布一个）
        self.task_release_interval = 20.0  
        self.current_task_idx = 0          
        self.next_release_time = self.task_release_interval
        self.task_release_done = False     
        
        # 4. 超时参数（仅惩罚，不丢弃）
        self.task_timeout = 120.0  # 超时阈值放宽到120秒
        self.pending_tasks = []   
        
        # 5. 动作空间 & 状态空间（扩展到45维，感知所有8个任务）
        self.action_space = spaces.Discrete(len(self.robot_names) + 1)  # 4维：3选车+1等待
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(45,), dtype=np.float32)

        # 6. 仿真参数
        self.dt = 1.0  
        self.current_time = 0.0
        self.max_steps = 1000 
        self.step_count = 0
        self.completed_tasks = 0
        
        # 7. 奖励系数（核心调整：下调奖励尺度，平衡正负反馈）
        self.reward_coeff = {
            "distance": -0.02,         # 保持不变：仅引导近任务优先
            "completion": 30.0,        # 从150→30：大幅下调单任务完成奖励
            "batch_completion": 20.0,  # 从100→20：下调批量完成奖励
            "idle_selection": -1.0,    # 保持不变：极轻的选忙碌车惩罚
            "timeout": -5.0,           # 保持不变：轻超时惩罚
            "step": -0.001,            # 从-0.0001→-0.001：轻微增强时间惩罚（避免无限等待）
            "wait_short": 0.2,         # 从1.0→0.2：下调短等待奖励
            "wait_long": -0.1,         # 保持不变：极轻长等待惩罚
            "all_completed": 100.0,    # 从500→100：下调全任务完成奖励
            "valid_selection": 1.0     # 从5.0→1.0：下调选空闲车奖励
        }

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.current_time = 0.0
        self.step_count = 0
        self.completed_tasks = 0
        self.pending_tasks = []
        
        # 小车初始位置固定，无待命点返回逻辑
        for name in self.robot_names:
            init_pos = self.fixed_robot_init[name]
            self.robot_states[name] = {
                "x": init_pos["x"],
                "y": init_pos["y"],
                "idle": True,
                "target": None,
                "speed": 1.0,
                "task_remaining_time": 0.0
            }
        
        # 随机打乱任务顺序（保持任务多样性）
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
        reward = self.reward_coeff["step"]  # 基础时间惩罚
        
        # 新增：奖励裁剪函数，限制单步奖励波动
        def clip_reward(r, min_r=-10.0, max_r=50.0):
            return max(min(r, max_r), min_r)
        
        wait_action_idx = len(self.robot_names)
        info = {
            "action_type": "wait" if action == wait_action_idx else "assign",
            "completed_tasks": self.completed_tasks,
            "pending_tasks_count": len(self.pending_tasks),
            "task_release_done": self.task_release_done,
            "idle_robots": sum([1 for r in self.robot_states.values() if r["idle"]])
        }
        
        # --- 1. 任务发布逻辑（20秒发布一个） ---
        if not self.task_release_done and self.current_time >= self.next_release_time:
            if self.current_task_idx < len(self.current_episode_tasks):
                task = self.current_episode_tasks[self.current_task_idx]
                self.pending_tasks.append({
                    "x": task["x"],
                    "y": task["y"],
                    "wait_time": 0.0,
                    "task_id": task["task_id"],
                    "waypoint": task["waypoint"],
                    "timeout_count": 0  # 记录超时次数，仅惩罚不丢弃
                })
                self.current_task_idx += 1
                self.next_release_time = self.current_time + self.task_release_interval
            else:
                self.task_release_done = True
        
        # --- 2. 处理等待动作 ---
        if action == wait_action_idx:
            if self.pending_tasks:
                # 更新所有待处理任务的等待时间
                for task in self.pending_tasks:
                    task["wait_time"] += self.dt
                
                # 等待奖励/惩罚（鼓励短等待凑单）
                first_task_wait = self.pending_tasks[0]["wait_time"]
                if first_task_wait < 10.0:
                    reward += self.reward_coeff["wait_short"]
                    reward = clip_reward(reward)  # 奖励裁剪
                elif first_task_wait > 30.0:
                    reward += self.reward_coeff["wait_long"]
                    reward = clip_reward(reward)  # 奖励裁剪
                
                # 超时处理：仅惩罚，不丢弃任务（核心修改）
                idle_robot_count = sum([1 for r in self.robot_states.values() if r["idle"]])
                if idle_robot_count > 0:
                    # 遍历所有任务，超时则加惩罚（但保留任务）
                    for task in self.pending_tasks:
                        if task["wait_time"] > self.task_timeout and task["timeout_count"] == 0:
                            reward += self.reward_coeff["timeout"]
                            reward = clip_reward(reward)  # 奖励裁剪
                            task["timeout_count"] = 1  # 标记已惩罚，避免重复惩罚
        
        # --- 3. 处理选车动作 ---
        else:
            selected_robot_name = self.robot_names[action]
            robot = self.robot_states[selected_robot_name]
            info["selected_robot"] = selected_robot_name
            
            if self.pending_tasks:
                if not robot["idle"]:
                    # 选忙碌车：极轻惩罚（常态事件）
                    reward += self.reward_coeff["idle_selection"]
                    reward = clip_reward(reward)  # 奖励裁剪
                else:
                    # 选空闲车：正向奖励（强化正确行为）
                    reward += self.reward_coeff["valid_selection"]
                    reward = clip_reward(reward)  # 奖励裁剪
                    
                    # 选离该小车最近的任务（核心逻辑）
                    min_dist = float("inf")
                    best_task_idx = 0
                    for i, task in enumerate(self.pending_tasks):
                        dist = math.hypot(robot["x"] - task["x"], robot["y"] - task["y"])
                        if dist < min_dist:
                            min_dist = dist
                            best_task_idx = i
                    
                    # 移除选中的任务（分配给小车，不是丢弃）
                    task = self.pending_tasks.pop(best_task_idx)
                    info["assigned_task_id"] = task["task_id"]
                    
                    # 极低的距离惩罚（仅引导近任务优先）
                    reward += self.reward_coeff["distance"] * min_dist
                    reward = clip_reward(reward)  # 奖励裁剪
                    
                    # 计算任务预计执行时间
                    task_exec_time = min_dist / robot["speed"]
                    
                    # 更新小车状态（无返回待命点逻辑）
                    robot["idle"] = False
                    robot["target"] = (task["x"], task["y"])
                    robot["task_remaining_time"] = task_exec_time
                    robot["current_task_id"] = task["task_id"]
        
        # --- 4. 物理运动模拟（完成任务后停在任务点） ---
        for name, r in self.robot_states.items():
            if not r["idle"] and r["target"]:
                # 更新剩余执行时间
                r["task_remaining_time"] -= self.dt
                r["task_remaining_time"] = max(0.0, r["task_remaining_time"])
                
                tx, ty = r["target"]
                dx, dy = tx - r["x"], ty - r["y"]
                dist = math.hypot(dx, dy)
                move_dist = r["speed"] * self.dt
                
                if dist <= move_dist:
                    # 到达目标：完成任务（核心正向奖励）
                    r["x"], r["y"] = tx, ty  # 停在任务点，不返回待命点
                    r["idle"] = True
                    r["target"] = None
                    r["task_remaining_time"] = 0.0
                    r.pop("current_task_id", None)
                    reward += self.reward_coeff["completion"]
                    reward = clip_reward(reward)  # 奖励裁剪
                    self.completed_tasks += 1
                    
                    # 完成所有8个任务：超大奖励（核心目标）
                    if self.completed_tasks == len(self.all_possible_tasks):
                        reward += self.reward_coeff["all_completed"]
                        reward = clip_reward(reward)  # 奖励裁剪
                else:
                    # 移动到目标
                    angle = math.atan2(dy, dx)
                    r["x"] += math.cos(angle) * move_dist
                    r["y"] += math.sin(angle) * move_dist
        
        # --- 5. 批量完成奖励（所有任务处理完） ---
        if len(self.pending_tasks) == 0 and self.completed_tasks > 0 and self.task_release_done:
            reward += self.reward_coeff["batch_completion"]
            reward = clip_reward(reward)  # 奖励裁剪
        
        # --- 6. 结束条件（必须完成所有8个任务才结束） ---
        terminated = False
        # 结束条件：任务发完 + 无待处理任务 + 所有车空闲 + 完成8个任务
        if (self.task_release_done and len(self.pending_tasks) == 0 and 
            all([r["idle"] for r in self.robot_states.values()]) and 
            self.completed_tasks == len(self.all_possible_tasks)):
            terminated = True
        truncated = self.step_count >= self.max_steps
        
        return self._get_obs(), reward, terminated, truncated, info

    def _get_obs(self):
        """45维状态向量：感知所有8个任务+小车执行状态"""
        # 1. 机器人状态 (x, y, idle, 剩余执行时间) - 3×4=12维
        robot_obs = []
        for name in self.robot_names:
            r = self.robot_states[name]
            robot_obs.extend([
                r["x"] / 200.0, 
                r["y"] / 100.0,  
                1.0 if r["idle"] else 0.0,
                min(r["task_remaining_time"] / 200.0, 1.0)
            ])
        
        # 2. 任务状态 - 8个任务×3维度=24维（无任务填0）
        task_obs = []
        for _ in range(8):
            task_obs.extend([0.0, 0.0, 0.0])
        
        for i, task in enumerate(self.pending_tasks[:8]):
            task_obs[i*3] = task["x"] / 200.0
            task_obs[i*3+1] = task["y"] / 100.0
            task_obs[i*3+2] = min(task["wait_time"] / 200.0, 1.0)
        
        # 3. 全局状态 - 9维（时间、完成数、待处理数、空闲车数等）
        idle_robot_count = sum([1 for r in self.robot_states.values() if r["idle"]])
        robot_remaining_times = [min(r["task_remaining_time"]/200.0, 1.0) for r in self.robot_states.values()]
        global_obs = [
            self.current_time / 1000.0,          # 1
            self.completed_tasks / 8.0,          # 2
            len(self.pending_tasks) / 8.0,       # 3
            idle_robot_count / 3.0,              # 4
            len([r for r in self.robot_states.values() if not r["idle"]]) / 3.0,  # 5（新增：忙碌车数）
            sum(robot_remaining_times) / 3.0,    # 6（新增：剩余时间均值）
            *robot_remaining_times               # 7-9（3个机器人剩余时间）
        ]
        
        return np.array(robot_obs + task_obs + global_obs, dtype=np.float32)
