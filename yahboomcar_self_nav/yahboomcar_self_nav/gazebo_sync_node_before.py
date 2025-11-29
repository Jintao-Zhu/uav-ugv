#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
import time

class GazeboSyncNode(Node):
    def __init__(self):
        super().__init__('gazebo_sync_node')

        # 初始化变量
        self.last_update_time = 0.0
        self.update_interval = 0.1  # 每 0.1s 更新一次
        self.model_name='yahboomcar_X3'

        # 订阅现实小车的 /odom
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 创建 Gazebo 服务客户端
        self.cli = self.create_client(SetEntityState, '/set_entity_state')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('等待 /set_entity_state 服务中...')

        self.req = SetEntityState.Request()
        self.get_logger().info("Gazebo 同步节点初始化完成 ✅")


    def odom_callback(self, msg):
        now = time.time()
        if now - self.last_update_time < self.update_interval:
            return  # 控制频率
        self.last_update_time = now

        if hasattr(self, 'last_pose'):
            dx = abs(msg.pose.pose.position.x - self.last_pose.position.x)
            dy = abs(msg.pose.pose.position.y - self.last_pose.position.y)
            if dx < 0.002 and dy < 0.002:
                return
        self.last_pose = msg.pose.pose


        # 构造状态对象
        state = EntityState()
        state.name = self.model_name
        state.pose = msg.pose.pose

        # 不使用 twist 避免物理干扰
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0


        self.req.state = state
        self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
