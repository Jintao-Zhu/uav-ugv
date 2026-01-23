#!/usr/bin/env python3

# 【对比实验】轮询调度对比节点 - 核心逻辑：按固定顺序分配任务（不考虑距离/忙碌状态）-20min47s
import rclpy
from rclpy.node import Node
from rmf_fleet_msgs.msg import FleetState
from std_msgs.msg import String
from geometry_msgs.msg import Point
from rmf_custom_tasks_self.srv import SingleNavTask
import math
import numpy as np

class RoundRobinCompareNode(Node):
    def __init__(self):
        super().__init__("round_robin_compare_node")
        self.get_logger().info("🚀 初始化轮询调度对比节点（固定顺序分配）...")
        
        # ========== 1. 基础配置（和贪心/RL节点完全对齐） ==========
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        
        # 机器人初始位置（和贪心/RL节点一致）
        self.robot_positions = {
            "deliveryRobot_0": Point(x=96.59527587890625, y=-51.96450424194336),
            "deliveryRobot_1": Point(x=152.3477325439453, y=-44.31863021850586),
            "deliveryRobot_2": Point(x=14.776845932006836, y=-9.279278755187988)
        }
        self.robot_names = list(self.robot_positions.keys())
        
        # 任务坐标（和贪心/RL节点完全一致）
        self.waypoint_coords = {
            "n14": Point(x=80.84, y=-28.52), "n13": Point(x=84.44, y=-4.94),
            "n23": Point(x=182.80, y=-42.30), "s08": Point(x=96.61, y=-50.50),
            "s10": Point(x=122.10, y=-46.68), "west_koi_pond": Point(x=34.32, y=-10.13),
            "n08": Point(x=59.61, y=-7.42), "junction_south_west": Point(x=84.56, y=-38.81)
        }
        
        # ========== 2. 轮询核心配置 ==========
        self.next_robot_idx = 0  # 轮询索引：下一个要分配的机器人下标（核心）
        
        # ========== 3. 状态缓存 ==========
        self.pending_tasks = []  # 待处理任务队列
        self.processed_task_ids = set()  # 已处理任务ID（避免重复）
        self.robot_task_queues = {name: [] for name in self.robot_names}  # 机器人任务队列（仅记录）
        
        # ========== 4. 实验数据记录（和贪心节点完全对齐） ==========
        self.experiment_metrics = {
            "total_tasks": 0,          # 总任务数
            "completed_tasks": 0,      # 完成任务数
            "task_wait_time": {},      # 每个任务的等待时间 {task_id: time}
            "robot_task_count": {name: 0 for name in self.robot_names},  # 每个机器人的任务数
            "total_movement_distance": 0.0,  # 总移动距离
            "task_receive_time": {},   # 任务接收时间
            "start_time": self.get_clock().now().nanoseconds / 1e9  # 实验开始时间
        }
        
        # ========== 5. ROS接口（和贪心/RL节点完全对齐） ==========
        # 订阅机器人状态（更新位置）
        self.create_subscription(FleetState, "/fleet_states", self.fleet_state_callback, 10)
        # 订阅任务发布话题（和贪心/RL节点同一话题）
        self.create_subscription(String, "/task_monitor/start", self.task_callback, 10)
        # 订阅任务完成话题（统计完成情况）
        self.create_subscription(String, "/custom_task_completion", self.completion_callback, 10)
        # 任务下发服务（和贪心/RL节点同一服务）
        self.nav_client = self.create_client(SingleNavTask, "/submit_single_nav_task")
        
        # ========== 6. 调度定时器（1Hz，和贪心/RL节点决策频率一致） ==========
        self.dispatch_timer = self.create_timer(1.0, self.round_robin_dispatch)
        
        self.get_logger().info("✅ 轮询调度对比节点初始化完成！")

    def fleet_state_callback(self, msg):
        """更新机器人位置（和贪心节点逻辑完全一致）"""
        if msg.name != "deliveryRobot":
            return
        for robot in msg.robots:
            if robot.name in self.robot_positions:
                self.robot_positions[robot.name].x = robot.location.x
                self.robot_positions[robot.name].y = robot.location.y

    def task_callback(self, msg):
        """接收任务（和贪心节点逻辑完全一致）"""
        data = msg.data.split(",")
        if len(data) < 2:
            self.get_logger().warn(f"⚠️ 无效任务格式：{msg.data}")
            return
        task_id, waypoint = data[0].strip(), data[1].strip()
        
        # 过滤重复任务、非目标任务
        if task_id in self.processed_task_ids or not waypoint in self.waypoint_coords:
            return
        if not task_id.startswith("red_cube_"):
            return
        
        # 记录任务信息（和贪心节点对齐）
        task_x = self.waypoint_coords[waypoint].x
        task_y = self.waypoint_coords[waypoint].y
        self.pending_tasks.append({
            "task_id": task_id,
            "waypoint": waypoint,
            "x": task_x,
            "y": task_y
        })
        self.processed_task_ids.add(task_id)
        self.experiment_metrics["total_tasks"] += 1
        self.experiment_metrics["task_receive_time"][task_id] = self.get_clock().now().nanoseconds / 1e9
        self.experiment_metrics["task_wait_time"][task_id] = 0.0
        self.get_logger().info(f"📥 接收任务：{task_id} @ {waypoint}，当前待处理队列：{len(self.pending_tasks)}")

    def round_robin_dispatch(self):
        """核心：轮询调度逻辑（替换贪心的距离计算逻辑）"""
        if not self.pending_tasks or not self.nav_client.service_is_ready():
            return
        
        # 遍历待处理任务（按接收顺序）
        for task_idx in range(len(self.pending_tasks)):
            task = self.pending_tasks[task_idx]
            task_id = task["task_id"]
            task_x, task_y = task["x"], task["y"]
            
            # ========== 核心：轮询分配逻辑 ==========
            # 1. 获取当前轮询的机器人（固定顺序：0→1→2→0...）
            selected_robot = self.robot_names[self.next_robot_idx]
            # 2. 更新轮询索引（循环）
            self.next_robot_idx = (self.next_robot_idx + 1) % len(self.robot_names)
            
            # ========== 指标统计（和贪心节点对齐） ==========
            # 计算移动距离（用于对比，轮询不优化但仍统计）
            r_pos = self.robot_positions[selected_robot]
            dist = math.hypot(r_pos.x - task_x, r_pos.y - task_y)
            self.experiment_metrics["total_movement_distance"] += dist
            self.experiment_metrics["robot_task_count"][selected_robot] += 1
            
            # ========== 下发任务（和贪心节点逻辑一致） ==========
            self._send_task(selected_robot, task_id, task["waypoint"])
            
            # ========== 移除任务（和贪心节点一致） ==========
            self.pending_tasks.pop(task_idx)
            self.get_logger().info(f"🔄 轮询调度：{selected_robot} 执行 {task_id}（轮询索引：{self.next_robot_idx}）- 不考虑距离/忙碌状态")
            break  # 每次调度只处理1个任务（和贪心/RL节点一致）

    def _send_task(self, robot_name, task_id, waypoint):
        """下发任务（和贪心节点逻辑完全一致）"""
        req = SingleNavTask.Request()
        req.target_waypoint = waypoint
        req.fleet_name = "deliveryRobot"
        req.robot_name = robot_name
        req.priority = 1
        
        future = self.nav_client.call_async(req)
        future.add_done_callback(lambda f, tid=task_id, r=robot_name: self._task_send_callback(f, tid, r))

    def _task_send_callback(self, future, task_id, robot_name):
        """任务下发结果回调（和贪心节点逻辑完全一致）"""
        try:
            res = future.result()
            if not res.success:
                self.get_logger().error(f"❌ 任务 {task_id} 下发失败：{res.message}")
                # 失败则重新加入队列
                self.pending_tasks.append(next(t for t in self.pending_tasks if t["task_id"] == task_id))
        except Exception as e:
            self.get_logger().error(f"❌ 任务下发异常：{e}")

    def completion_callback(self, msg):
        """任务完成回调（和贪心节点逻辑完全一致）"""
        task_id = msg.data.strip()
        if task_id not in self.experiment_metrics["task_receive_time"]:
            return
        
        # 计算任务等待时间
        complete_time = self.get_clock().now().nanoseconds / 1e9
        receive_time = self.experiment_metrics["task_receive_time"][task_id]
        self.experiment_metrics["task_wait_time"][task_id] = complete_time - receive_time
        
        # 更新完成数
        self.experiment_metrics["completed_tasks"] += 1
        self.get_logger().info(
            f"✅ 任务 {task_id} 完成！等待时间：{self.experiment_metrics['task_wait_time'][task_id]:.2f}秒 | "
            f"已完成：{self.experiment_metrics['completed_tasks']}/{self.experiment_metrics['total_tasks']}"
        )
        
        # 所有任务完成后打印实验结果
        if self.experiment_metrics["completed_tasks"] == 8:
            self._print_experiment_result()

    def _print_experiment_result(self):
        """打印实验结果（和贪心节点完全一致，保证对比公平）"""
        self.get_logger().info("\n==================== 轮询调度实验结果 ====================")
        # 1. 任务完成率
        completion_rate = (self.experiment_metrics["completed_tasks"] / self.experiment_metrics["total_tasks"]) * 100
        self.get_logger().info(f"1. 任务完成率：{completion_rate:.2f}%")
        
        # 2. 平均任务等待时间
        avg_wait_time = sum(self.experiment_metrics["task_wait_time"].values()) / len(self.experiment_metrics["task_wait_time"])
        self.get_logger().info(f"2. 平均任务等待时间：{avg_wait_time:.2f}秒")
        
        # 3. 机器人负载分布（计算标准差，体现均衡性）
        self.get_logger().info(f"3. 机器人任务数分布：{self.experiment_metrics['robot_task_count']}")
        task_counts = list(self.experiment_metrics["robot_task_count"].values())
        avg_task_count = sum(task_counts) / 3
        load_variance = sum([(x - avg_task_count)**2 for x in task_counts]) / 3
        self.get_logger().info(f"   负载标准差：{math.sqrt(load_variance):.2f}")
        
        # 4. 总移动距离
        self.get_logger().info(f"4. 总移动距离：{self.experiment_metrics['total_movement_distance']:.2f}米")
        
        # 5. 总耗时
        total_time = self.get_clock().now().nanoseconds / 1e9 - self.experiment_metrics["start_time"]
        self.get_logger().info(f"5. 总耗时：{total_time:.2f}秒")
        self.get_logger().info("===========================================================\n")

    def destroy_node(self):
        """节点销毁时打印最终结果（和贪心节点一致）"""
        self._print_experiment_result()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RoundRobinCompareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
