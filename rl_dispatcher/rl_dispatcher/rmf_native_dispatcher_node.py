#!/usr/bin/env python3
# 1.17 RMF原生贪心调度对比节点（和RL调度节点对齐输入）
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rmf_custom_tasks_self.srv import SingleNavTask
import time

class RMFNativeDispatcherNode(Node):
    def __init__(self):
        super().__init__("rmf_native_dispatcher_node")
        self.get_logger().info("🚀 初始化 RMF 原生贪心调度对比节点...")
        
        # 1. 强制开启仿真时间（和RL节点对齐）
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])

        # 2. 航点坐标映射（和RL节点完全一致）
        self.waypoint_coords = {
            "n14": (80.84, -28.52), "n13": (84.44, -4.94),
            "n23": (182.80, -42.30), "s08": (96.61, -50.50),
            "s10": (122.10, -46.68), "west_koi_pond": (34.32, -10.13),
            "n08": (59.61, -7.42), "junction_south_west": (84.56, -38.81)
        }

        # 3. ROS接口初始化（和RL节点完全对齐）
        # 订阅auto_send_waypoints发布的/task_monitor/start话题
        self.task_sub = self.create_subscription(
            String,
            "/task_monitor/start",
            self.target_callback,
            10
        )
        
        # 创建RMF服务客户端（和RL节点调用同一个服务）
        self.nav_client = self.create_client(SingleNavTask, "/submit_single_nav_task")
        
        # 去重缓存（和RL节点对齐，避免重复处理任务）
        self.processed_ids = set()
        
        self.get_logger().info("✅ RMF原生调度节点初始化完成！")

    def target_callback(self, msg):
        """
        回调函数：接收auto_send_waypoints的任务，直接转发给RMF（不指定小车）
        逻辑和RL节点的target_callback完全对齐，仅移除RL决策
        """
        data = msg.data.split(",")
        if len(data) < 2:
            self.get_logger().warn(f"⚠️ 无效的任务格式: {msg.data}")
            return
        
        tid, wp = data[0].strip(), data[1].strip()
        
        # 核心过滤：只处理red_cube前缀的任务（和RL节点完全一致）
        if not tid.startswith("red_cube_"):
            self.get_logger().debug(f"🔍 忽略非red_cube任务: {tid} @ {wp}")
            return
        
        # 去重：避免重复处理同一任务（和RL节点对齐）
        if tid in self.processed_ids:
            self.get_logger().debug(f"⚠️ 任务已处理过: {tid}")
            return
        
        # 校验航点有效性（和RL节点对齐）
        if wp not in self.waypoint_coords:
            self.get_logger().error(f"❌ 未知航点: {wp}，任务{tid}跳过")
            return
        
        # 直接调用RMF服务（核心：不指定robot_name，触发原生贪心调度）
        self._call_rmf_service(tid, wp)
        
        # 标记任务已处理（和RL节点对齐）
        self.processed_ids.add(tid)
        self.get_logger().info(f"📥 转发任务到RMF原生调度: {tid} @ {wp} (不指定小车)")

    def _call_rmf_service(self, tid, wp):
        """调用RMF服务，不指定小车，触发原生贪心调度"""
        # 等待服务可用（和RL节点对齐）
        if not self.nav_client.service_is_ready():
            self.get_logger().warn(f"⚠️ RMF服务未就绪，等待重试...")
            self.nav_client.wait_for_service(timeout_sec=5.0)
            if not self.nav_client.service_is_ready():
                self.get_logger().error(f"❌ RMF服务超时，任务{tid}发送失败")
                return
        
        # 构造请求（核心：不设置robot_name，其他参数和RL节点对齐）
        req = SingleNavTask.Request()
        req.target_waypoint = wp
        req.fleet_name = "deliveryRobot"  # 和RL节点一致
        # 关键：不指定robot_name → RMF自动选车
        req.priority = 1  # 和RL节点一致
        
        # 异步调用服务（和RL节点对齐）
        future = self.nav_client.call_async(req)
        future.add_done_callback(lambda f: self._service_done(f, tid, wp))

    def _service_done(self, future, tid, wp):
        """服务调用结果回调（和RL节点对齐）"""
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"✅ [RMF原生调度成功] 任务 {tid} @ {wp} (RMF自动选车)")
            else:
                self.get_logger().error(f"❌ [RMF原生调度失败] 任务 {tid} @ {wp}: {res.message}")
        except Exception as e:
            self.get_logger().error(f"❌ 服务调用异常: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RMFNativeDispatcherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 收到终止信号，退出节点...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
