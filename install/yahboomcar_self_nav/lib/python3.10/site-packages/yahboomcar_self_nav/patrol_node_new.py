import sys
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        # 导航器
        self.navigator = BasicNavigator()

        # 声明参数
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.declare_parameter(
            'target_points',
            [0.0, 0.0, 0.0, 0.9, 0.3, 1.57, 0.3, 0.9, 0.0, 0.9, 0.3, -1.57]
        )

        self.initial_point = self.get_parameter('initial_point').value
        self.target_points = self.get_parameter('target_points').value

        # 初始位姿
        self.initial_pose = self._pose_from_xyyaw(
            self.initial_point[0],
            self.initial_point[1],
            self.initial_point[2]
        )

        # 控制状态
        self._returning = False
        self._shutdown = False

        # 创建周期性检查定时器
        self.timer = self.create_timer(0.5, self._monitor)

    # --------- 内部方法 ---------

    def _pose_from_xyyaw(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    def _parse_target_points(self):
        raw = self.get_parameter('target_points').value
        if len(raw) % 3 != 0:
            self.get_logger().warn("target_points 长度不是 3 的倍数，忽略多余元素")
        pts = []
        for i in range(0, len(raw) - len(raw) % 3, 3):
            pts.append((raw[i], raw[i + 1], raw[i + 2]))
        return pts

    def _monitor(self):
        """
        定期检查任务状态或退出标志。
        """
        if self._shutdown:
            self.get_logger().info("关闭节点中...")
            self.navigator.cancelTask()
            rclpy.shutdown()
            return

    # --------- 业务逻辑 ---------

    def init_robot_pose(self):
        self.navigator.setInitialPose(self.initial_pose)
        self.get_logger().info(f"设置初始位姿: {self.initial_point}")
        active = self.navigator.waitUntilNav2Active(
            timeout=Duration(seconds=60.0)
        )
        if not active:
            self.get_logger().error("Nav2 未能激活，准备退出")
            self._shutdown = True

    def nav_to_pose(self, target_pose):
        if self._shutdown:
            return False

        self.get_logger().info(
            f"导航到 ({target_pose.pose.position.x:.2f}, {target_pose.pose.position.y:.2f})"
        )
        self.navigator.goToPose(target_pose)

        while (not self.navigator.isTaskComplete()) and not self._shutdown:
            if self._returning:
                self.get_logger().info("取消当前导航，返回初始点")
                self.navigator.cancelTask()
                return False
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f"剩余距离: {feedback.distance_remaining:.2f} m"
                )
            rclpy.spin_once(self, timeout_sec=0.5)

        if self._shutdown:
            return False

        result = self.navigator.getResult()
        return result == TaskResult.SUCCEEDED

    def patrol_loop(self):
        while rclpy.ok() and not self._shutdown:
            for (x, y, yaw) in self._parse_target_points():
                if self._returning or self._shutdown:
                    break
                pose = self._pose_from_xyyaw(x, y, yaw)
                ok = self.nav_to_pose(pose)
                if not ok:
                    break

            if self._returning:
                self.get_logger().info("返回初始点中...")
                self.nav_to_pose(self.initial_pose)
                self.get_logger().info("已返回初始点")
                self._shutdown = True

            rclpy.spin_once(self, timeout_sec=0.5)

    # --------- 外部接口 ---------

    def request_shutdown(self, returning=False):
        """
        设置退出标志，returning=True 表示先返回初始点
        """
        if returning:
            self._returning = True
        else:
            self._shutdown = True


def main():
    rclpy.init()
    node = PatrolNode()

    try:
        node.init_robot_pose()
        node.patrol_loop()
    except KeyboardInterrupt:
        node.get_logger().info("捕获 Ctrl+C，准备退出")
        node.request_shutdown()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
