#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import csv
import os
from datetime import datetime
import json

# 导入消息类型
from rmf_fleet_msgs.msg import FleetState, RobotState
from rmf_task_msgs.msg import TaskSummary, ApiRequest
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

class RMFDataRecorder(Node):
    def __init__(self):
        super().__init__('rmf_data_recorder')
        # 1. 配置CSV路径
        self.csv_dir = "/home/suda/drone_ugv_ws/src/data_collect"
        os.makedirs(self.csv_dir, exist_ok=True)
        self.csv_filename = f"{self.csv_dir}/rmf_task_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        
        # 2. CSV表头
        self.csv_header = [
            'sim_time', 'task_id', 'target_waypoint', 'target_x', 'target_y',
            'robot_name', 'robot_x', 'robot_y', 'distance_to_target',
            'distance_ok_duration', 'task_status', 'task_start_time',
            'task_complete_time', 'cube_delete_result'
        ]
        
        # 3. 初始化CSV（只写表头，避免空行）
        with open(self.csv_filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=self.csv_header)
            writer.writeheader()
        self.get_logger().info(f"✅ 数据记录节点启动，CSV文件：{self.csv_filename}")

        # 4. 缓存变量
        self.current_sim_time = 0.0
        self.task_info_cache = {}    # {task_id: 任务数据}
        self.robot_pos_cache = {}    # {robot_name: (x,y)}
        self.distance_ok_cache = {}  # {task_id: 满足<2米的持续时间}
        self.last_write_time = 0.0   # 上次写入CSV的时间（去重用）

        # 5. 航点坐标字典
        self.waypoint_coords = {
            "junction_n01": (1.57, -45.93),
            "n08": (59.61, -7.42),
            "n14": (80.84, -28.52),
            "n13": (84.44, -4.94),
            "n23": (182.80, -42.30),
            "west_koi_pond": (34.32, -10.13),
            "s08": (96.61, -50.50),
            "s10": (122.10, -46.68),
            "s11": (152.73, -43.00),
            "junction_south_west": (84.56, -38.81)
        }

        # 6. 配置QoS（解决/clock话题不兼容）
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 7. 订阅话题（全部使用匹配的QoS）
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, qos_profile)
        self.fleet_state_sub = self.create_subscription(FleetState, '/fleet_states', self.fleet_state_callback, qos_profile)
        self.task_summary_sub = self.create_subscription(TaskSummary, '/task_summaries', self.task_summary_callback, qos_profile)
        self.task_complete_sub = self.create_subscription(String, '/custom_task_completion', self.task_complete_callback, qos_profile)
        self.task_api_sub = self.create_subscription(ApiRequest, '/task_api_requests', self.task_api_callback, qos_profile)

        # 8. 定时写入CSV（改为2秒一次，减少冗余）
        self.write_timer = self.create_timer(2.0, self.write_to_csv_periodically)

    # -------------------------- 修复后的回调函数 --------------------------
    def clock_callback(self, msg):
        """修复：正确解析仿真时间"""
        self.current_sim_time = msg.clock.sec + msg.clock.nanosec / 1e9

    def fleet_state_callback(self, msg):
        """修复：正确更新小车位置+任务关联"""
        # 只处理deliveryRobot车队
        if msg.name != "deliveryRobot":
            return
        for robot in msg.robots:
            # 1. 更新小车位置缓存
            self.robot_pos_cache[robot.name] = (robot.location.x, robot.location.y)
            # 2. 任务关联：小车绑定任务ID
            if robot.task_id and robot.task_id in self.task_info_cache:
                task_data = self.task_info_cache[robot.task_id]
                task_data['robot_name'] = robot.name
                task_data['robot_x'] = round(robot.location.x, 2)
                task_data['robot_y'] = round(robot.location.y, 2)
                # 3. 计算到目标的距离
                target_x, target_y = task_data['target_x'], task_data['target_y']
                if target_x != 0 and target_y != 0:
                    distance = ((robot.location.x - target_x)**2 + (robot.location.y - target_y)**2)**0.5
                    task_data['distance_to_target'] = round(distance, 2)
                    # 4. 统计距离<2米的持续时间
                    if distance < 2.0:
                        self.distance_ok_cache[robot.task_id] = self.distance_ok_cache.get(robot.task_id, 0.0) + 2.0
                        task_data['distance_ok_duration'] = round(self.distance_ok_cache[robot.task_id], 2)

    def task_api_callback(self, msg):
        """修复：正确解析任务请求，初始化任务数据"""
        try:
            task_json = json.loads(msg.json_msg)
            if task_json.get("type") == "dispatch_task_request":
                task_id = msg.request_id
                target_waypoint = task_json['request']['description']['places'][0]
                # 初始化任务数据（带真实启动时间）
                self.task_info_cache[task_id] = {
                    'task_id': task_id,
                    'target_waypoint': target_waypoint,
                    'target_x': self.waypoint_coords.get(target_waypoint, (0.0, 0.0))[0],
                    'target_y': self.waypoint_coords.get(target_waypoint, (0.0, 0.0))[1],
                    'robot_name': '',
                    'robot_x': 0.0,
                    'robot_y': 0.0,
                    'distance_to_target': 0.0,
                    'distance_ok_duration': 0.0,
                    'task_status': 'EXECUTING',
                    'task_start_time': round(self.current_sim_time, 2),  # 修复：用真实仿真时间
                    'task_complete_time': 0.0,
                    'cube_delete_result': ''
                }
                self.get_logger().debug(f"📌 初始化任务缓存：{task_id} -> {target_waypoint} (启动时间：{self.current_sim_time})")
        except Exception as e:
            self.get_logger().warn(f"⚠️ 解析任务API失败：{e}")

    def task_summary_callback(self, msg):
        """修复：正确更新任务状态"""
        task_id = msg.task_id
        if task_id in self.task_info_cache:
            state_map = {1: 'EXECUTING', 2: 'PAUSED', 3: 'COMPLETED', 4: 'FAILED', 5: 'CANCELED'}
            self.task_info_cache[task_id]['task_status'] = state_map.get(msg.state, 'UNKNOWN')
            # 任务完成时记录完成时间
            if msg.state == 3:  # COMPLETED
                self.task_info_cache[task_id]['task_complete_time'] = round(self.current_sim_time, 2)

    def task_complete_callback(self, msg):
        """修复：解析自定义任务完成信号"""
        try:
            data = msg.data.split(',')
            if len(data) >= 4:
                task_id = data[0]
                complete_time = float(data[3])
                if task_id in self.task_info_cache:
                    self.task_info_cache[task_id]['task_complete_time'] = round(complete_time, 2)
                    self.task_info_cache[task_id]['cube_delete_result'] = '成功'
        except Exception as e:
            self.get_logger().warn(f"⚠️ 解析任务完成信号失败：{e}")

    def write_to_csv_periodically(self):
        """优化：只写入有变化的有效数据，避免重复"""
        # 跳过无仿真时间的情况
        if self.current_sim_time == 0.0:
            return
        # 跳过重复时间（避免每秒写相同数据）
        if abs(self.current_sim_time - self.last_write_time) < 1.9:
            return
        
        # 只写入有小车关联的任务数据（过滤空数据）
        for task_id, task_data in self.task_info_cache.items():
            if task_data['robot_name'] != '' and task_data['target_waypoint'] != '':
                # 补充当前仿真时间
                task_data['sim_time'] = round(self.current_sim_time, 2)
                # 写入CSV
                with open(self.csv_filename, 'a', newline='', encoding='utf-8') as f:
                    writer = csv.DictWriter(f, fieldnames=self.csv_header)
                    writer.writerow(task_data)
        
        self.last_write_time = self.current_sim_time  # 更新上次写入时间

def main(args=None):
    rclpy.init(args=args)
    node = RMFDataRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
