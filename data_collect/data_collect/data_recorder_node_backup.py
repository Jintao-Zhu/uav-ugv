#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import csv
import os
from datetime import datetime
import math
from collections import defaultdict
# 1.23 除时间以外，收集到的数据都是正确的

# 导入消息类型
from rmf_fleet_msgs.msg import FleetState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

class RMFExperimentDataRecorder(Node):
    def __init__(self):
        super().__init__('rmf_experiment_data_recorder')
        self.get_logger().info("🚀 初始化RMF实验数据收集节点（适配时间戳采集）...")
        
        # 1. 设置use_sim_time
        try:
            sim_time_param = rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
            self.set_parameters([sim_time_param])
            self.get_logger().info(f"✅ use_sim_time已设置为：{self.get_parameter('use_sim_time').value}")
        except Exception as e:
            self.get_logger().warn(f"⚠️ 设置use_sim_time警告：{e}")
        
        # 2. 算法类型参数
        self.declare_parameter("algorithm_type", "tan_xin")
        self.algorithm_type = self.get_parameter("algorithm_type").value
        self.get_logger().info(f"🔧 当前算法类型：{self.algorithm_type}")
        
        # 3. CSV配置
        self.csv_dir = "/home/suda/drone_ugv_ws/src/data_collect/experiment_results"
        os.makedirs(self.csv_dir, exist_ok=True)
        self.csv_filename = f"{self.csv_dir}/{self.algorithm_type}_experiment_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        # 新增时间戳相关字段
        self.csv_header = [
            "time", "algorithm_type", "total_tasks", "completed_tasks", 
            "avg_wait_time", "max_wait_time", "avg_completion_time",
            "total_movement_distance",
            "robot_0_task_count", "robot_1_task_count", "robot_2_task_count",
            "robot_0_distance", "robot_1_distance", "robot_2_distance",
            "robot_0_idle_ratio", "robot_1_idle_ratio", "robot_2_idle_ratio",
            "active_tasks"
        ]
        self._init_csv()
        
        # 4. 航点坐标（和task_monitor_node完全一致）
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
        
        # 5. 核心数据缓存（新增时间戳字段）
        self.current_sim_time = 0.0
        self.experiment_start_time = self.current_sim_time  # 实验开始时间
        self.task_data = {}  # 存储所有任务的完整信息：task_id -> {publish_time, complete_time, robot, waypoint, ...}
        self.robot_last_pos = {}  # 机器人上一时刻位置
        self.robot_total_dist = defaultdict(float)  # 每个机器人累计移动距离
        self.robot_task_count = defaultdict(int)  # 每个机器人执行的任务数
        self.robot_busy_time = defaultdict(float)  # 每个机器人忙碌时间
        self.robot_last_status = defaultdict(bool)  # 机器人上一时刻状态：True=忙碌，False=空闲
        # ========== 关键修改1：把static变量改成类成员变量 ==========
        self.last_callback_time = 0.0  # 用于计算回调时间间隔
        
        # 6. QoS配置（适配仿真的BEST_EFFORT）
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1000
        )
        
        # 7. 订阅核心话题
        self.create_subscription(Clock, '/clock', self.clock_callback, self.qos_profile)
        self.create_subscription(FleetState, '/fleet_states', self.fleet_state_callback, self.qos_profile)
        self.create_subscription(String, '/task_monitor/start', self.task_publish_callback, self.qos_profile)
        self.create_subscription(String, '/custom_task_completion', self.task_complete_callback, self.qos_profile)
        
        # 8. 定时写入CSV（5秒一次）
        self.write_timer = self.create_timer(5.0, self.write_experiment_data)
        
        self.get_logger().info(f"✅ 数采节点初始化完成！CSV文件：{self.csv_filename}")

    def _init_csv(self):
        """初始化CSV表头"""
        with open(self.csv_filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=self.csv_header)
            writer.writeheader()

    def clock_callback(self, msg):
        """更新仿真时间"""
        self.current_sim_time = msg.clock.sec + msg.clock.nanosec / 1e9
        # 记录实验开始时间（仅第一次）
        if self.experiment_start_time == 0.0:
            self.experiment_start_time = self.current_sim_time

    def task_publish_callback(self, msg):
        """接收任务发布消息，记录发布时间戳"""
        data = msg.data.split(",")
        if len(data) < 2:
            self.get_logger().warn(f"⚠️ 无效的任务发布格式：{msg.data}")
            return
        
        task_id = data[0].strip()
        waypoint = data[1].strip()
        
        # 仅处理red_cube前缀的任务
        if not task_id.startswith("red_cube_"):
            return
        
        # 记录任务发布信息
        self.task_data[task_id] = {
            "publish_time": self.current_sim_time,  # 发布时间戳
            "complete_time": 0.0,                  # 完成时间戳（初始0）
            "waypoint": waypoint,
            "robot": "",                           # 执行任务的机器人
            "is_completed": False,                 # 是否完成
            "wait_time": 0.0,                      # 等待时间（发布到完成）
            "completion_time": 0.0                 # 完成耗时（暂未启用）
        }
        self.get_logger().info(f"📥 任务发布：{task_id} | 航点：{waypoint} | 发布时间：{self.current_sim_time:.2f}秒")

    def task_complete_callback(self, msg):
        """接收任务完成消息，解析完成时间戳"""
        data = msg.data.split(",")
        if len(data) < 4:
            self.get_logger().warn(f"⚠️ 无效的任务完成格式：{msg.data}")
            return
        
        task_id = data[0].strip()
        robot_name = data[1].strip()
        waypoint = data[2].strip()
        complete_time_sec = float(data[3].strip())  # 从消息中提取完成时间戳
        
        # 更新任务完成信息
        if task_id in self.task_data:
            self.task_data[task_id]["complete_time"] = complete_time_sec
            self.task_data[task_id]["robot"] = robot_name
            self.task_data[task_id]["is_completed"] = True
            # 计算等待时间（完成时间 - 发布时间）
            self.task_data[task_id]["wait_time"] = complete_time_sec - self.task_data[task_id]["publish_time"]
            # 统计机器人任务数
            self.robot_task_count[robot_name] += 1
            self.get_logger().info(f"✅ 任务完成：{task_id} | 执行机器人：{robot_name} | 等待时间：{self.task_data[task_id]['wait_time']:.2f}秒")

    def fleet_state_callback(self, msg):
        """更新机器人位置、移动距离、忙碌状态"""
        if msg.name != "deliveryRobot":
            return
        
        # ========== 关键修改2：使用类成员变量替代static变量 ==========
        # 计算本次回调的时间间隔（用于统计忙碌时间）
        if self.last_callback_time == 0.0:
            self.last_callback_time = self.current_sim_time
        time_delta = self.current_sim_time - self.last_callback_time
        self.last_callback_time = self.current_sim_time
        
        for robot in msg.robots:
            robot_name = robot.name
            current_pos = (robot.location.x, robot.location.y)
            
            # 1. 计算机器人累计移动距离
            if robot_name in self.robot_last_pos:
                last_x, last_y = self.robot_last_pos[robot_name]
                delta_dist = math.hypot(current_pos[0]-last_x, current_pos[1]-last_y)
                self.robot_total_dist[robot_name] += delta_dist
            self.robot_last_pos[robot_name] = current_pos
            
            # 2. 判断机器人状态（有task_id=忙碌，无=空闲）
            current_busy = (robot.task_id != "")
            # 统计忙碌时间（如果当前忙碌，累加时间间隔）
            if current_busy:
                self.robot_busy_time[robot_name] += time_delta
            # 更新机器人最后状态
            self.robot_last_status[robot_name] = current_busy

    def write_experiment_data(self):
        """计算并写入所有新增指标"""
        # 1. 基础统计
        total_tasks = len(self.task_data)
        completed_tasks = sum(1 for t in self.task_data.values() if t["is_completed"])
        completion_rate = (completed_tasks / total_tasks) * 100 if total_tasks > 0 else 0.0
        
        # 2. 等待时间统计（均值/最大值）
        wait_times = [t["wait_time"] for t in self.task_data.values() if t["is_completed"] and t["wait_time"] > 0]
        avg_wait_time = sum(wait_times) / len(wait_times) if wait_times else 0.0
        max_wait_time = max(wait_times) if wait_times else 0.0
        
        # 3. 平均任务完成时间（暂用等待时间替代，可后续优化）
        avg_completion_time = avg_wait_time
        
        # 4. 总移动距离
        total_movement_dist = sum(self.robot_total_dist.values())
        
        # 5. 机器人空闲时间占比（空闲时间 / 总实验时间）
        total_experiment_time = self.current_sim_time - self.experiment_start_time
        robot_idle_ratio = {}
        for robot in ["deliveryRobot_0", "deliveryRobot_1", "deliveryRobot_2"]:
            if total_experiment_time <= 0:
                robot_idle_ratio[robot] = 0.0
            else:
                # 空闲占比 = (总时间 - 忙碌时间) / 总时间
                busy_time = self.robot_busy_time.get(robot, 0.0)
                robot_idle_ratio[robot] = (total_experiment_time - busy_time) / total_experiment_time
        
        # 6. 组装数据
        data = {
            "time": datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            "algorithm_type": self.algorithm_type,
            "total_tasks": total_tasks,
            "completed_tasks": completed_tasks,
            "avg_wait_time": round(avg_wait_time, 2),
            "max_wait_time": round(max_wait_time, 2),
            "avg_completion_time": round(avg_completion_time, 2),
            "total_movement_distance": round(total_movement_dist, 2),
            "robot_0_task_count": self.robot_task_count.get("deliveryRobot_0", 0),
            "robot_1_task_count": self.robot_task_count.get("deliveryRobot_1", 0),
            "robot_2_task_count": self.robot_task_count.get("deliveryRobot_2", 0),
            "robot_0_distance": round(self.robot_total_dist.get("deliveryRobot_0", 0), 2),
            "robot_1_distance": round(self.robot_total_dist.get("deliveryRobot_1", 0), 2),
            "robot_2_distance": round(self.robot_total_dist.get("deliveryRobot_2", 0), 2),
            "robot_0_idle_ratio": round(robot_idle_ratio.get("deliveryRobot_0", 0.0) * 100, 2),
            "robot_1_idle_ratio": round(robot_idle_ratio.get("deliveryRobot_1", 0.0) * 100, 2),
            "robot_2_idle_ratio": round(robot_idle_ratio.get("deliveryRobot_2", 0.0) * 100, 2),
            "active_tasks": total_tasks - completed_tasks
        }
        
        # 写入CSV
        with open(self.csv_filename, 'a', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=self.csv_header)
            writer.writerow(data)
        
        # 打印监控日志
        self.get_logger().info(f"\n📊 实验数据快照（{self.algorithm_type}）：")
        self.get_logger().info(f"   总任务数：{total_tasks} | 完成数：{completed_tasks} | 完成率：{completion_rate:.2f}%")
        self.get_logger().info(f"   等待时间：均值{avg_wait_time:.2f}秒 | 最大值{max_wait_time:.2f}秒")
        self.get_logger().info(f"   平均完成时间：{avg_completion_time:.2f}秒 | 总移动距离：{total_movement_dist:.2f}米")
        self.get_logger().info(f"   机器人任务数：0:{self.robot_task_count.get('deliveryRobot_0',0)} 1:{self.robot_task_count.get('deliveryRobot_1',0)} 2:{self.robot_task_count.get('deliveryRobot_2',0)}")
        self.get_logger().info(f"   机器人移动距离：0:{self.robot_total_dist.get('deliveryRobot_0',0):.2f} 1:{self.robot_total_dist.get('deliveryRobot_1',0):.2f} 2:{self.robot_total_dist.get('deliveryRobot_2',0):.2f}米")
        self.get_logger().info(f"   机器人空闲占比：0:{robot_idle_ratio.get('deliveryRobot_0',0.0)*100:.2f}% 1:{robot_idle_ratio.get('deliveryRobot_1',0.0)*100:.2f}% 2:{robot_idle_ratio.get('deliveryRobot_2',0.0)*100:.2f}%")

def main(args=None):
    rclpy.init(args=args)
    node = RMFExperimentDataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.write_experiment_data()
        node.get_logger().info("🛑 数采节点终止，已保存最后数据")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
