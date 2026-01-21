import gymnasium as gym
from gymnasium import spaces
import numpy as np
import math
import random

class MockRMFEnv(gym.Env):
    """
    适配RMF真实逻辑：
    1. 支持给忙碌小车分配任务（任务排队执行）
    2. 核心优化：新增队列均衡惩罚+奖励归一化+状态特征优化
    3. 8任务必须完成+无待命点返回+超时不丢任务
    """
    def __init__(self):
        super().__init__()
        # 奖励归一化参数
        self.reward_scale = 0.1  # 全局奖励缩放系数
        self.episode_reward = 0.0  # 累计奖励
        # 1. 小车配置（支持任务排队）
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
        
        # 3. 任务发布参数（与实际一致：20秒发布一个）
        self.task_release_interval = 20.0  
        self.current_task_idx = 0          
        self.next_release_time = self.task_release_interval
        self.task_release_done = False     
        
        # 4. 超时参数（仅惩罚，不丢弃）
        self.task_timeout = 120.0  
        self.pending_tasks = []   
        
        # 5. 动作空间 & 状态空间（保持45维）
        self.action_space = spaces.Discrete(len(self.robot_names) + 1)  # 4维：3选车+1等待
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(45,), dtype=np.float32)

        # 6. 仿真参数
        self.dt = 1.0  
        self.current_time = 0.0
        self.max_steps = 1000 
        self.step_count = 0
        self.completed_tasks = 0
        
        # 7. 奖励系数（核心优化：归一化+新增队列均衡惩罚）
        self.reward_coeff = {
            "distance": -0.001,         # 保持低距离惩罚（引导近任务优先）
            "completion": 1.0,           # 单任务完成奖励（从10→1，缩小10倍）
            "batch_completion": 1.5,     # 批量完成奖励（从15→1.5，缩小10倍）
            "invalid_selection": -0.1,   # 无效选车惩罚（从-1→-0.1，缩小10倍）
            "timeout": -0.1,             # 超时惩罚（从-1→-0.1，缩小10倍）
            "step": -0.001,              # 保持基础时间惩罚（避免agent躺平）
            "wait_short": 0.001,         # 短等待奖励（从0.01→0.001，缩小10倍）
            "wait_long": -0.01,          # 长等待惩罚（从-0.1→-0.01，缩小10倍）
            "all_completed": 5.0,        # 全完成奖励（从50→5，缩小10倍）
            "task_queue": 0.01,          # 排队奖励（从0.1→0.01，缩小10倍）
            "queue_imbalance": -0.001    # 队列不均衡惩罚（从-0.01→-0.001，缩小10倍）
        }



    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.current_time = 0.0
        self.step_count = 0
        self.completed_tasks = 0
        self.pending_tasks = []
        
        # 小车初始状态：支持任务队列
        for name in self.robot_names:
            init_pos = self.fixed_robot_init[name]
            self.robot_states[name] = {
                "x": init_pos["x"],
                "y": init_pos["y"],
                "idle": True,
                "current_target": None,  # 当前执行的任务
                "task_queue": [],        # 排队的任务列表
                "speed": 1.0,
                "current_task_remaining_time": 0.0
            }
        
        # 随机打乱任务顺序
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
        reward_items = []  # 收集所有奖励项，最后统一计算
        reward_items.append(self.reward_coeff["step"])  # 基础时间惩罚

        # 奖励裁剪函数
        # 替换原有的clip_reward函数
        def clip_reward(r, min_r=-5.0, max_r=5.0):
            """严格裁剪单步奖励，避免单步奖励过大"""
            return max(min(r, max_r), min_r)

        
        wait_action_idx = len(self.robot_names)
        info = {
            "action_type": "wait" if action == wait_action_idx else "assign",
            "completed_tasks": self.completed_tasks,
            "pending_tasks_count": len(self.pending_tasks),
            "task_release_done": self.task_release_done,
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
                    "timeout_count": 0
                })
                self.current_task_idx += 1
                self.next_release_time = self.current_time + self.task_release_interval
            else:
                self.task_release_done = True
        
        # --- 2. 处理等待动作 ---
        if action == wait_action_idx:
            if self.pending_tasks:
                # 更新任务等待时间
                for task in self.pending_tasks:
                    task["wait_time"] += self.dt
                
                # 等待奖励/惩罚
                first_task_wait = self.pending_tasks[0]["wait_time"]
                if first_task_wait < 10.0:
                    reward_items.append(self.reward_coeff["wait_short"])
                elif first_task_wait > 30.0:
                    reward_items.append(self.reward_coeff["wait_long"])
                
                # 超时处理：仅惩罚，不丢弃
                for task in self.pending_tasks:
                    if task["wait_time"] > self.task_timeout and task["timeout_count"] == 0:
                        reward_items.append(self.reward_coeff["timeout"])
                        task["timeout_count"] = 1
        
        # --- 3. 处理选车动作（核心改动：支持任务排队） ---
        else:
            selected_robot_name = self.robot_names[action]
            robot = self.robot_states[selected_robot_name]
            info["selected_robot"] = selected_robot_name
            
            # 无任务时选车：无效操作，惩罚
            if not self.pending_tasks:
                reward_items.append(self.reward_coeff["invalid_selection"])
            else:
                # 选离该小车最近的任务
                min_dist = float("inf")
                best_task_idx = 0
                for i, task in enumerate(self.pending_tasks):
                    dist = math.hypot(robot["x"] - task["x"], robot["y"] - task["y"])
                    if dist < min_dist:
                        min_dist = dist
                        best_task_idx = i
                
                task = self.pending_tasks.pop(best_task_idx)
                info["assigned_task_id"] = task["task_id"]
                
                # 距离惩罚（引导近任务优先）
                reward_items.append(self.reward_coeff["distance"] * min_dist)
                
                # 计算任务执行时间
                task_exec_time = min_dist / robot["speed"]
                task_info = {
                    "x": task["x"],
                    "y": task["y"],
                    "task_id": task["task_id"],
                    "waypoint": task["waypoint"],
                    "exec_time": task_exec_time
                }
                
                # 核心逻辑：任务排队
                if robot["idle"]:
                    # 小车空闲：直接执行该任务
                    robot["idle"] = False
                    robot["current_target"] = task_info
                    robot["current_task_remaining_time"] = task_exec_time
                else:
                    # 小车忙碌：加入任务队列（奖励合理排队）
                    robot["task_queue"].append(task_info)
                    reward_items.append(self.reward_coeff["task_queue"])
        
        # --- 4. 物理运动模拟（支持任务队列执行） ---
        task_completion_rewards = 0
        for name, r in self.robot_states.items():
            # 小车忙碌且有当前任务
            if not r["idle"] and r["current_target"]:
                # 更新剩余执行时间
                r["current_task_remaining_time"] -= self.dt
                r["current_task_remaining_time"] = max(0.0, r["current_task_remaining_time"])
                
                tx, ty = r["current_target"]["x"], r["current_target"]["y"]
                dx, dy = tx - r["x"], ty - r["y"]
                dist = math.hypot(dx, dy)
                move_dist = r["speed"] * self.dt
                
                if dist <= move_dist or r["current_task_remaining_time"] <= 0:
                    # 完成当前任务
                    r["x"], r["y"] = tx, ty  # 停在任务点，不返回待命点
                    task_completion_rewards += self.reward_coeff["completion"]
                    self.completed_tasks += 1
                    info["completed_task_id"] = r["current_target"]["task_id"]
                    
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
                    if self.completed_tasks == len(self.all_possible_tasks):
                        task_completion_rewards += self.reward_coeff["all_completed"]
                else:
                    # 移动到目标
                    angle = math.atan2(dy, dx)
                    r["x"] += math.cos(angle) * move_dist
                    r["y"] += math.sin(angle) * move_dist
        
        reward_items.append(task_completion_rewards)
        
        # --- 5. 批量完成奖励 ---
        if len(self.pending_tasks) == 0 and self.completed_tasks > 0 and self.task_release_done:
            reward_items.append(self.reward_coeff["batch_completion"])
        
        # --- 6. 队列不均衡惩罚 ---
        queue_lengths = [len(r["task_queue"]) for r in self.robot_states.values()]
        if queue_lengths:
            max_queue = max(queue_lengths)
            avg_queue = np.mean(queue_lengths)
            if max_queue > avg_queue + 2:
                reward_items.append(self.reward_coeff["queue_imbalance"] * (max_queue - avg_queue))
        
        # --- 7. 计算总奖励并裁剪 ---
        reward = sum(reward_items)
        reward = clip_reward(reward)
        # 新增：全局奖励缩放（降低绝对值）
        reward = reward * self.reward_scale
        self.episode_reward += reward
        # --- 8. 结束条件 ---
        terminated = self.completed_tasks >= len(self.all_possible_tasks)
        truncated = self.step_count >= self.max_steps
        if truncated:
            # 未完成任务惩罚（同步缩小10倍）
            reward -= (len(self.all_possible_tasks) - self.completed_tasks) * 0.1
            reward = clip_reward(reward)
            reward = reward * self.reward_scale  # 新增：惩罚也缩放

        # 补充结束条件：所有任务发布+无待处理+所有小车空闲
        all_robots_idle = all([
            r["idle"] and len(r["task_queue"]) == 0 
            for r in self.robot_states.values()
        ])
        if (self.task_release_done and len(self.pending_tasks) == 0 and 
            all_robots_idle and self.completed_tasks == len(self.all_possible_tasks)):
            terminated = True
            
        return self._get_obs(), reward, terminated, truncated, info

    def _get_obs(self):
        """45维状态向量：优化全局特征，移除冗余，新增队列均衡特征"""
        # 1. 机器人状态 (x, y, idle, 任务队列长度) - 3×4=12维
        robot_obs = []
        for name in self.robot_names:
            r = self.robot_states[name]
            queue_len = len(r["task_queue"]) if not r["idle"] else 0.0
            robot_obs.extend([
                r["x"] / 200.0, 
                r["y"] / 100.0,  
                1.0 if r["idle"] else 0.0,
                min(queue_len / 5.0, 1.0)  # 队列长度归一化（最大5个任务）
            ])
        
        # 2. 任务状态 - 8×3=24维
        task_obs = []
        for _ in range(8):
            task_obs.extend([0.0, 0.0, 0.0])
        
        for i, task in enumerate(self.pending_tasks[:8]):
            task_obs[i*3] = task["x"] / 200.0
            task_obs[i*3+1] = task["y"] / 100.0
            task_obs[i*3+2] = min(task["wait_time"] / 600.0, 1.0)  # 修正等待时间归一化
        
        # 3. 全局状态 - 9维（优化：移除冗余，新增队列均衡特征）
        total_queue_len = sum([len(r["task_queue"]) for r in self.robot_states.values()])
        idle_robot_count = sum([1 for r in self.robot_states.values() if r["idle"]])
        queue_lengths = [len(r["task_queue"]) for r in self.robot_states.values()]
        max_queue_len = max(queue_lengths) if queue_lengths else 0
        min_queue_len = min(queue_lengths) if queue_lengths else 0
        
        global_obs = [
            self.current_time / 1000.0,          # 1. 当前时间
            self.completed_tasks / 8.0,          # 2. 已完成任务进度
            len(self.pending_tasks) / 8.0,       # 3. 待处理任务进度
            idle_robot_count / 3.0,              # 4. 空闲机器人占比
            total_queue_len / 15.0,              # 5. 总队列长度占比
            (self.completed_tasks + len(self.pending_tasks)) / 8.0,  # 6. 总任务进度
            # 优化：新增队列均衡特征
            np.var(queue_lengths) / 2.5 if queue_lengths else 0.0,        # 7. 队列方差（均衡性）
            max_queue_len / 5.0,                                          # 8. 最长队列（压力峰值）
            (max_queue_len - min_queue_len) / 5.0 if queue_lengths else 0.0,  # 9. 队列最大差（不均衡度）
        ]
        
        # 兜底：确保所有维度在[-1,1]范围内
        global_obs = [min(max(x, -1.0), 1.0) for x in global_obs]
        
        return np.array(robot_obs + task_obs + global_obs, dtype=np.float32)
