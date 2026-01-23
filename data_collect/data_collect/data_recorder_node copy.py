import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from rmf_fleet_msgs.msg import FleetState, RobotState
from rmf_task_msgs.msg import TaskSummary, ApiRequest
from std_msgs.msg import String
import json
import csv
import os
from datetime import datetime
from collections import defaultdict

class RMFDataRecorder(Node):
    def __init__(self):
        super().__init__('rmf_data_recorder')

        # 使用 reliable QoS 以兼容 /clock（通常来自 Gazebo/Ignition）
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # 订阅话题
        self.create_subscription(Clock, '/clock', self.clock_callback, qos_reliable)
        self.create_subscription(FleetState, '/fleet_states', self.fleet_state_callback, qos_reliable)
        self.create_subscription(TaskSummary, '/task_summaries', self.task_summary_callback, qos_reliable)
        self.create_subscription(ApiRequest, '/task_api_requests', self.api_request_callback, qos_reliable)
        self.create_subscription(String, '/custom_task_completion', self.custom_completion_callback, qos_reliable)
        # === 新增：订阅 task_monitor 的 start 信号 ===
        self.create_subscription(String, '/task_monitor/start', self.task_start_callback, qos_reliable)

        # 缓存
        self.current_sim_time = 0.0
        self.robot_pos_cache = {}  # robot_name -> (x, y)
        self.task_info_cache = {}  # task_id -> dict
        self.distance_ok_cache = defaultdict(float)  # task_id -> accumulated time (sec) within 2m

        # 航点坐标（必须与 task_monitor.cpp 一致！）
        self.waypoint_coords = {
            "n08": (59.61, -7.42),
            "s11": (152.73, -43.00),
            "s07": (139.88, -30.15),
            "s05": (127.03, -17.30),
            "s03": (114.18, -4.45),
            "n01": (101.33, 8.40),
            "n03": (88.48, 21.25),
            "n05": (75.63, 34.10),
        }

        # CSV 设置
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = f"/home/suda/drone_ugv_ws/src/data_collect/rmf_task_data_{timestamp}.csv"
        self.last_write_time = -10.0  # 确保首次写入
        self.write_interval = 2.0  # 每2秒写一次

        # 写入表头
        with open(self.csv_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "sim_time", "task_id", "target_waypoint", "target_x", "target_y",
                "robot_name", "robot_x", "robot_y", "distance_to_target", "distance_ok_duration",
                "task_status", "task_start_time", "task_complete_time", "cube_delete_result"
            ])

        self.get_logger().info(f"RMF Data Recorder started. Saving to {self.csv_path}")

    def clock_callback(self, msg):
        self.current_sim_time = msg.clock.sec + msg.clock.nanosec / 1e9

    def fleet_state_callback(self, msg):
        if msg.name != "deliveryRobot":
            return  # 只关心 deliveryRobot 车队
        for robot in msg.robots:
            if robot.name:
                x = robot.location.x
                y = robot.location.y
                self.robot_pos_cache[robot.name] = (x, y)

                # 如果该机器人正在执行 red_cube_ 任务，更新关联
                if robot.task_id and robot.task_id.startswith("red_cube_"):
                    if robot.task_id not in self.task_info_cache:
                        self.task_info_cache[robot.task_id] = {
                            "robot_name": robot.name,
                            "target_waypoint": None,
                            "target_x": None,
                            "target_y": None,
                            "task_start_time": None,  # 将由 /task_monitor/start 填充
                            "task_complete_time": None,
                            "status": "ASSIGNED",
                            "cube_delete_result": ""
                        }
                    else:
                        self.task_info_cache[robot.task_id]["robot_name"] = robot.name

    def api_request_callback(self, msg):
        try:
            payload = json.loads(msg.json_msg)
            task_id = payload.get("task_id")
            if not task_id or not task_id.startswith("red_cube_"):
                return  # 忽略非自定义任务

            # 提取目标航点（假设在 request 描述中）
            # 注意：你的 task_monitor 是从 /task_monitor/start 获取 waypoint，
            # 所以此处仅作备用，不强制设置 target
            # 实际 target 由 /task_monitor/start 设置
        except Exception as e:
            self.get_logger().warn(f"Failed to parse ApiRequest: {e}")

    def task_start_callback(self, msg):
        """新增：接收 /task_monitor/start 消息，格式: 'red_cube_n08,n08'"""
        parts = msg.data.strip().split(',')
        if len(parts) < 2:
            self.get_logger().warn(f"Invalid /task_monitor/start format: {msg.data}")
            return
        task_id = parts[0]
        waypoint = parts[1]

        if not task_id.startswith("red_cube_"):
            return

        if waypoint not in self.waypoint_coords:
            self.get_logger().warn(f"Unknown waypoint: {waypoint}")
            return

        x, y = self.waypoint_coords[waypoint]
        if task_id not in self.task_info_cache:
            self.task_info_cache[task_id] = {
                "robot_name": None,
                "target_waypoint": waypoint,
                "target_x": x,
                "target_y": y,
                "task_start_time": self.current_sim_time,
                "task_complete_time": None,
                "status": "MONITORING",
                "cube_delete_result": ""
            }
        else:
            # 更新或确认
            self.task_info_cache[task_id].update({
                "target_waypoint": waypoint,
                "target_x": x,
                "target_y": y,
                "task_start_time": self.current_sim_time,
                "status": "MONITORING"
            })

    def custom_completion_callback(self, msg):
        """处理 /custom_task_completion，格式: 'task_id,robot_name,waypoint,timestamp'"""
        parts = msg.data.strip().split(',')
        if len(parts) < 3:
            return
        task_id = parts[0]
        if task_id in self.task_info_cache:
            self.task_info_cache[task_id]["task_complete_time"] = self.current_sim_time
            self.task_info_cache[task_id]["cube_delete_result"] = "成功"
            self.task_info_cache[task_id]["status"] = "COMPLETED"

    def task_summary_callback(self, msg):
        # 可选：用于对比，但不作为主要完成依据
        pass

    def timer_callback(self):
        if self.current_sim_time - self.last_write_time < self.write_interval:
            return

        self.last_write_time = self.current_sim_time

        rows_to_write = []
        for task_id, info in self.task_info_cache.items():
            # 只记录有目标且已分配机器人的任务
            if info["target_x"] is None or info["robot_name"] is None:
                continue

            robot_name = info["robot_name"]
            if robot_name not in self.robot_pos_cache:
                continue

            rx, ry = self.robot_pos_cache[robot_name]
            tx, ty = info["target_x"], info["target_y"]
            distance = ((rx - tx)**2 + (ry - ty)**2)**0.5

            # 更新 distance_ok_duration（每2秒累加2秒，若距离<2）
            if distance < 2.0:
                self.distance_ok_cache[task_id] += self.write_interval

            rows_to_write.append([
                round(self.current_sim_time, 3),
                task_id,
                info["target_waypoint"],
                round(tx, 3),
                round(ty, 3),
                robot_name,
                round(rx, 3),
                round(ry, 3),
                round(distance, 3),
                round(self.distance_ok_cache[task_id], 3),
                info["status"],
                round(info["task_start_time"], 3) if info["task_start_time"] else "",
                round(info["task_complete_time"], 3) if info["task_complete_time"] else "",
                info["cube_delete_result"]
            ])

        if rows_to_write:
            with open(self.csv_path, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(rows_to_write)

def main(args=None):
    rclpy.init(args=args)
    node = RMFDataRecorder()
    # 使用 timer 来定期写入（避免在回调中频繁写文件）
    node.timer = node.create_timer(0.1, node.timer_callback)  # 高频检查，但受 write_interval 控制
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()