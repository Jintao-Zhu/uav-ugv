#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

import tf2_ros
from tf2_ros import TransformException

import time


class GazeboSyncNode(Node):
    def __init__(self):
        super().__init__('gazebo_sync_node')

        # 更新控制
        self.last_update_time = 0.0
        self.update_interval = 0.1     # 每 0.1s 更新一次

        # Gazebo 中模拟小车的名字
        self.model_name = 'yahboomcar_X3'

        # ---- TF 监听器，核心关键点 ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Gazebo 服务 ----
        self.cli = self.create_client(SetEntityState, '/set_entity_state')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('等待 /set_entity_state 服务中...')

        self.req = SetEntityState.Request()

        # 使用计时器定时同步
        self.create_timer(self.update_interval, self.timer_callback)

        self.get_logger().info("Gazebo TF 同步节点初始化完成 ✅")

    def timer_callback(self):
        """定时从 TF 获取 map → base_link，然后同步到 Gazebo"""

        now = time.time()
        if now - self.last_update_time < self.update_interval:
            return
        self.last_update_time = now

        # ---- 获取 TF：map → base_link ----
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',           # 目标坐标系（全局）
                'base_link',     # 真实小车的主体
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f"等待 TF 中: {ex}")
            return

        # ---- 构造 EntityState 发送到 Gazebo ----
        state = EntityState()
        state.name = self.model_name

        # 位姿（包含 xyz + 四元数）
        state.pose.position.x = trans.transform.translation.x
        state.pose.position.y = trans.transform.translation.y
        state.pose.position.z = 0.0   # 小车本来就在地面上

        state.pose.orientation = trans.transform.rotation

        # 禁用 twist，避免与物理引擎冲突
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0

        # 发送更新
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
