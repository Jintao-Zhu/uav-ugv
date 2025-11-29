#include "rclcpp/rclcpp.hpp"   //11.11这是原来的代码，发布的无人机姿态严重错误
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

class PX4OdometryBridge : public rclcpp::Node
{
public:
    PX4OdometryBridge()
        : Node("px4_odometry_bridge")
    {
        // PX4 ������ BestEffort QoS
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.best_effort();

        subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            qos_profile,
            std::bind(&PX4OdometryBridge::odom_callback, this, _1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/drone/odom", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/drone/pose", 10);

        RCLCPP_INFO(this->get_logger(), "PX4 Odometry Bridge started (NED -> ENU)");
    }

private:
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // --------------------------
        // NED -> ENU ����ת��
        // --------------------------
        double x_enu = msg->position[1];  // y_NED �� x_ENU
        double y_enu = msg->position[0];  // x_NED �� y_ENU
        double z_enu = -msg->position[2]; // -z_NED �� z_ENU (Up)

        // PX4 quaternion (w, x, y, z) in NED
        tf2::Quaternion q_ned(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);

        // ��ת����NED �� ENU
        // �൱���� X ����ת 180�㣬���� Z ����ת 90��
        tf2::Quaternion q_rot;
        q_rot.setRPY(M_PI, 0.0, M_PI_2); // (roll=180��, pitch=0, yaw=90��)

        // Ӧ�ñ任
        tf2::Quaternion q_enu = q_rot * q_ned;
        q_enu.normalize();

        // --------------------------
        // ���� nav_msgs/Odometry
        // --------------------------
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->get_clock()->now();
        odom.header.frame_id = "map";
        odom.child_frame_id = "drone_base_link";

        // λ��
        odom.pose.pose.position.x = x_enu;
        odom.pose.pose.position.y = y_enu;
        odom.pose.pose.position.z = z_enu;

        // ��̬
        odom.pose.pose.orientation = tf2::toMsg(q_enu);

        // �ٶ� (NED �� ENU)
        odom.twist.twist.linear.x = msg->velocity[1];  // vy_NED �� x_ENU
        odom.twist.twist.linear.y = msg->velocity[0];  // vx_NED �� y_ENU
        odom.twist.twist.linear.z = -msg->velocity[2]; // -vz_NED �� z_ENU

        odom.twist.twist.angular.x = msg->angular_velocity[1];  // wy_NED �� x_ENU
        odom.twist.twist.angular.y = msg->angular_velocity[0];  // wx_NED �� y_ENU
        odom.twist.twist.angular.z = -msg->angular_velocity[2]; // -wz_NED �� z_ENU

        // ���� odom
        odom_pub_->publish(odom);

        // --------------------------
        // ���� geometry_msgs/PoseStamped
        // --------------------------
        geometry_msgs::msg::PoseStamped pose;
        pose.header = odom.header;
        pose.pose = odom.pose.pose;

        pose_pub_->publish(pose);

        // �������
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "ENU Pose: (%.2f, %.2f, %.2f)", x_enu, y_enu, z_enu);
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4OdometryBridge>());
    rclcpp::shutdown();
    return 0;
}
