import rclpy
import signal
import sys
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.duration import Duration


class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)
        # 导航相关定义
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points', [0.0,0.0,0.0, 0.9,0.3,1.57, 0.3,0.9,0.0, 0.9,0.3,-1.57]) #可用配置文件启动
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value

        # 保存初始位姿
        self.initial_pose = self.get_pose_by_xyyaw(
            self.initial_point_[0], self.initial_point_[1], self.initial_point_[2])
        
        # 实时位置获取 TF 相关定义
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)

        # 信号处理
        self.returning_to_initial = False
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, sig, frame):
        """处理Ctrl+C，返回初始点后退出"""
        if self.returning_to_initial:
            self.get_logger().info("已在返回初始点中，再次Ctrl+C强制退出")
            rclpy.shutdown()
            sys.exit(0)

        self.get_logger().info("收到终止信号，正在返回初始点...")
        self.returning_to_initial = True

        # 返回初始点
        self.nav_to_pose(self.initial_pose)
        self.get_logger().info("已返回初始点，程序退出")

        rclpy.shutdown()
        sys.exit(0)
   

    def get_pose_by_xyyaw(self, x, y, yaw):
        """
        通过 x,y,yaw 合成 PoseStamped
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        rotation_quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = rotation_quat[0]
        pose.pose.orientation.y = rotation_quat[1]
        pose.pose.orientation.z = rotation_quat[2]
        pose.pose.orientation.w = rotation_quat[3]
        return pose

    def init_robot_pose(self):
        """
        初始化机器人位姿
        """
        # 从参数获取初始化点
        self.initial_point_ = self.get_parameter('initial_point').value
        # 合成位姿并进行初始化
        self.setInitialPose(self.get_pose_by_xyyaw(
            self.initial_point_[0], self.initial_point_[1], self.initial_point_[2]))
        # 等待直到导航激活
        self.waitUntilNav2Active()

    def get_target_points(self):
        """
        通过参数值获取目标点集合        
        """
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_)/3)):
            x = self.target_points_[index*3]
            y = self.target_points_[index*3+1]
            yaw = self.target_points_[index*3+2]
            points.append([x, y, yaw])
            self.get_logger().info(f'获取到目标点: {index}->({x},{y},{yaw})')
        return points

    def nav_to_pose(self, target_pose):
        """
        导航到指定位姿
        """
        self.waitUntilNav2Active()
        result = self.goToPose(target_pose)

        while not self.isTaskComplete():
            pass
        
        # 最终结果判断
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航结果：成功')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('导航结果：被取消')
        elif result == TaskResult.FAILED:
            self.get_logger().error('导航结果：失败')
        else:
            self.get_logger().error('导航结果：返回状态无效')

    def get_current_pose(self):
        """
        通过TF获取当前位姿
        """
        while rclpy.ok():
            try:
                tf = self.buffer_.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time(), rclpy.time.Duration(seconds=1))
                transform = tf.transform
                rotation_euler = euler_from_quaternion([
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w
                ])
                self.get_logger().info(
                    f'平移:{transform.translation},旋转四元数:{transform.rotation}:旋转欧拉角:{rotation_euler}')
                return transform
            except Exception as e:
                self.get_logger().warn(f'不能够获取坐标变换，原因: {str(e)}')

    def main_loop(self):
        """主循环，执行巡逻任务"""
        while rclpy.ok() and not self.returning_to_initial:
            for point in self.get_target_points():
                if self.returning_to_initial:
                    break
                x, y, yaw = point[0], point[1], point[2]
                target_pose = self.get_pose_by_xyyaw(x, y, yaw)
                self.nav_to_pose(target_pose)
    
def main():
    rclpy.init()
    patrol = PatrolNode()
    patrol.init_robot_pose()

    patrol.main_loop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()