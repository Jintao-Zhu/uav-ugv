#include "rclcpp/rclcpp.hpp" //11.30 增加全局偏移参数，在航站楼中起点是40，-40，0.2
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
        // --------------------------
        // 新增：声明并获取全局偏移参数（从启动命令传入，默认适配新场景）
        // --------------------------
        this->declare_parameter<double>("offset_x", 40.0);  // 新场景x偏移（对应spawn的-x 40.0）
        this->declare_parameter<double>("offset_y", -40.0); // 新场景y偏移（对应spawn的-y -40.0）
        this->declare_parameter<double>("offset_z", 0.2);   // 新场景z偏移（对应spawn的-z 0.2）

        // 获取参数（支持启动时动态修改）
        offset_x_ = this->get_parameter("offset_x").as_double();
        offset_y_ = this->get_parameter("offset_y").as_double();
        offset_z_ = this->get_parameter("offset_z").as_double();

        // PX4 发布是 BestEffort QoS
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.best_effort();

        subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            qos_profile,
            std::bind(&PX4OdometryBridge::odom_callback, this, _1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/drone/odom", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/drone/pose", 10);

        RCLCPP_INFO(this->get_logger(), "PX4 Odometry Bridge started (NED -> ENU + RMF 全局偏移)");
        RCLCPP_INFO(this->get_logger(), "当前全局偏移：(%.2f, %.2f, %.2f)", offset_x_, offset_y_, offset_z_);
    }

private:
    // 存储全局偏移（适配不同场景）
    double offset_x_;
    double offset_y_;
    double offset_z_;

    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // --------------------------
        // 1. NED -> ENU 局部坐标转换（原有逻辑不变）
        // --------------------------
        double x_enu_local = msg->position[1];  // y_NED → x_ENU（局部）
        double y_enu_local = msg->position[0];  // x_NED → y_ENU（局部）
        double z_enu_local = -msg->position[2]; // -z_NED → z_ENU（局部，Up）

        // PX4 quaternion (w, x, y, z) in NED → ENU 姿态转换（原有逻辑不变）
        tf2::Quaternion q_ned(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
        tf2::Quaternion q_enu(
            q_ned.y(),  // NED的y分量 → ENU的x分量
            q_ned.x(),  // NED的x分量 → ENU的y分量
            -q_ned.z(), // NED的z分量取反 → ENU的z分量
            q_ned.w()   // 四元数实部（w）不变
        );
        q_enu.normalize(); // 确保是单位四元数

        // --------------------------
        // 2. 新增：叠加 RMF map 系全局偏移（关键修复）
        // --------------------------
        double x_enu_global = x_enu_local + offset_x_;  // 局部x + 全局x偏移 → 最终map系x
        double y_enu_global = y_enu_local + offset_y_;  // 局部y + 全局y偏移 → 最终map系y
        double z_enu_global = z_enu_local + offset_z_;  // 局部z + 全局z偏移 → 最终map系z

        // --------------------------
        // 3. 构造并发布 nav_msgs/Odometry（使用全局坐标）
        // --------------------------
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->get_clock()->now();
        odom.header.frame_id = "map";  // 明确是RMF的map坐标系
        odom.child_frame_id = "drone_base_link";

        // 位置：使用叠加偏移后的全局坐标
        odom.pose.pose.position.x = x_enu_global;
        odom.pose.pose.position.y = y_enu_global;
        odom.pose.pose.position.z = z_enu_global;

        // 姿态：保持不变（姿态是相对的，与位置偏移无关）
        odom.pose.pose.orientation = tf2::toMsg(q_enu);

        // 速度：仍使用局部转换后的速度（速度是相对量，无需叠加位置偏移）
        odom.twist.twist.linear.x = msg->velocity[1];  // vy_NED → x_ENU
        odom.twist.twist.linear.y = msg->velocity[0];  // vx_NED → y_ENU
        odom.twist.twist.linear.z = -msg->velocity[2]; // -vz_NED → z_ENU

        odom.twist.twist.angular.x = msg->angular_velocity[1];  // wy_NED → x_ENU
        odom.twist.twist.angular.y = msg->angular_velocity[0];  // wx_NED → y_ENU
        odom.twist.twist.angular.z = -msg->angular_velocity[2]; // -wz_NED → z_ENU

        odom_pub_->publish(odom);

        // --------------------------
        // 4. 构造并发布 geometry_msgs/PoseStamped（使用全局坐标）
        // --------------------------
        geometry_msgs::msg::PoseStamped pose;
        pose.header = odom.header;
        pose.pose = odom.pose.pose;

        pose_pub_->publish(pose);

        // 调试输出：显示全局坐标（方便验证）
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "RMF map 系全局坐标: (%.2f, %.2f, %.2f)", 
                             x_enu_global, y_enu_global, z_enu_global);
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
