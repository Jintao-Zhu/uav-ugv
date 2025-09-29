#include <rclcpp/rclcpp.hpp> //配合自定义的避障节点obstacle_avoidance_node使用的跟随节点，先启动这个节点，再启动避障节点
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

using namespace std::chrono_literals;

class YahboomcarCoordinationNode : public rclcpp::Node
{
public:
    YahboomcarCoordinationNode() : Node("yahboomcar_coordination_node")
    {
        // 声明参数并设置适合yahboomcar的默认值
        this->declare_parameter("kp_linear", 1.2);                // 线速度比例增益
        this->declare_parameter("kp_angular", 2.8);               // 角速度比例增益
        this->declare_parameter("max_linear_vel", 2.5);           // 最大线速度
        this->declare_parameter("max_angular_vel", 2.0);          // 最大角速度
        this->declare_parameter("follow_height", 0.5);            // 跟随高度阈值
        this->declare_parameter("land_threshold", 0.3);           // 降落阈值
        this->declare_parameter("timeout_ms", 1000);              // 超时时间
        this->declare_parameter("desired_distance", 2.8);         // 期望跟随距离
        this->declare_parameter("min_safe_distance", 1.5);        // 最小安全距离
        this->declare_parameter("max_follow_distance", 6.0);      // 最大跟随距离
        this->declare_parameter("angular_tolerance", 0.15);       // 角度容差
        this->declare_parameter("distance_tolerance", 0.3);       // 距离容差

        // Yahboomcar特定参数
        this->declare_parameter("wheel_separation", 0.169);         // 轮距
        this->declare_parameter("wheel_diameter", 0.082);           // 轮径
        this->declare_parameter("vehicle_namespace", "yahboomcar"); // 车辆命名空间

        // 获取车辆命名空间
        std::string vehicle_ns = this->get_parameter("vehicle_namespace").as_string();

        // QoS设置
        auto drone_qos = rclcpp::QoS(10).best_effort().durability_volatile();
        auto ugv_qos = rclcpp::QoS(10).reliable().durability_volatile();

        // 订阅无人机位置
        drone_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position",
            drone_qos,
            std::bind(&YahboomcarCoordinationNode::drone_position_callback, this, std::placeholders::_1));

        // 订阅yahboomcar里程计 (使用命名空间)
        std::string odom_topic = "/" + vehicle_ns + "/odom";
        ugv_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic,
            ugv_qos,
            std::bind(&YahboomcarCoordinationNode::ugv_odom_callback, this, std::placeholders::_1));

        // 订阅避障节点的障碍物状态
        std::string obstacle_state_topic = "/obstacle_avoidance/obstacle_detected";
        obstacle_state_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            obstacle_state_topic,
            10,
            std::bind(&YahboomcarCoordinationNode::obstacle_state_callback, this, std::placeholders::_1)
        );

        // 发布yahboomcar跟随指令 (使用命名空间)
        std::string follow_cmd_topic = "/" + vehicle_ns + "/follow_cmd_vel";
        follow_cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            follow_cmd_topic, 
            rclcpp::QoS(10).reliable().durability_volatile()
        );

        // 定时器
        timer_ = this->create_wall_timer(
            50ms, // 20Hz控制频率
            std::bind(&YahboomcarCoordinationNode::timer_callback, this));

        // 初始化状态
        drone_position_received_ = false;
        ugv_odom_received_ = false;
        follow_enabled_ = false;
        obstacle_detected_ = false;
        last_drone_time_ = this->get_clock()->now();
        last_ugv_time_ = this->get_clock()->now();
        debug_counter_ = 0;

        // 参数检查：确保跟随距离大于避障安全距离
        double follow_desired = this->get_parameter("desired_distance").as_double();
        double obstacle_safety = 1.2; // 对应避障节点的safety_distance默认值
        if (follow_desired < obstacle_safety)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ 跟随期望距离(%.2fm) < 避障安全距离(%.2fm)，可能触发频繁避障",
                        follow_desired, obstacle_safety);
        }

        RCLCPP_INFO(this->get_logger(), "=== Yahboomcar四驱无人车协同跟随节点初始化完成 ===");
        RCLCPP_INFO(this->get_logger(), "车辆配置:");
        RCLCPP_INFO(this->get_logger(), "  vehicle_namespace: %s", vehicle_ns.c_str());
        RCLCPP_INFO(this->get_logger(), "  wheel_separation: %.3f m", this->get_parameter("wheel_separation").as_double());
        RCLCPP_INFO(this->get_logger(), "  wheel_diameter: %.3f m", this->get_parameter("wheel_diameter").as_double());
        RCLCPP_INFO(this->get_logger(), "话题配置:");
        RCLCPP_INFO(this->get_logger(), "  里程计订阅: %s", odom_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  障碍物状态订阅: %s", obstacle_state_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  跟随指令发布: %s", follow_cmd_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "安全参数设置:");
        RCLCPP_INFO(this->get_logger(), "  desired_distance: %.2f m", this->get_parameter("desired_distance").as_double());
        RCLCPP_INFO(this->get_logger(), "  min_safe_distance: %.2f m", this->get_parameter("min_safe_distance").as_double());
        RCLCPP_INFO(this->get_logger(), "控制参数:");
        RCLCPP_INFO(this->get_logger(), "  kp_linear: %.2f", this->get_parameter("kp_linear").as_double());
        RCLCPP_INFO(this->get_logger(), "  kp_angular: %.2f", this->get_parameter("kp_angular").as_double());
        RCLCPP_INFO(this->get_logger(), "  max_linear_vel: %.2f m/s", this->get_parameter("max_linear_vel").as_double());
        RCLCPP_INFO(this->get_logger(), "  max_angular_vel: %.2f rad/s", this->get_parameter("max_angular_vel").as_double());
        RCLCPP_INFO(this->get_logger(), "🚗 等待Yahboomcar和无人机数据...");
    }

private:
    // 订阅器和发布器
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr drone_position_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ugv_odom_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_state_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr follow_cmd_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 状态变量
    px4_msgs::msg::VehicleLocalPosition latest_drone_pos_;
    nav_msgs::msg::Odometry latest_ugv_odom_;
    bool drone_position_received_;
    bool ugv_odom_received_;
    bool follow_enabled_;
    bool obstacle_detected_;
    rclcpp::Time last_drone_time_;
    rclcpp::Time last_ugv_time_;
    int debug_counter_;

    // 工具函数：将四元数转换为yaw角度
    double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(tf_q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    // 工具函数：角度归一化到[-π, π]
    double normalize_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    // 无人机位置回调
    void drone_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        // NED到ENU坐标系转换
        double ned_x = msg->x;
        double ned_y = msg->y;
        double ned_z = msg->z;

        latest_drone_pos_.x = ned_y;  // ENU的X（东） = NED的Y（东）
        latest_drone_pos_.y = ned_x;  // ENU的Y（北） = NED的X（北）
        latest_drone_pos_.z = -ned_z; // ENU的Z（天） = -NED的Z（地）

        drone_position_received_ = true;
        last_drone_time_ = this->get_clock()->now();

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "🚁 收到无人机位置(ENU): x=%.2f, y=%.2f, z=%.2f",
                             latest_drone_pos_.x, latest_drone_pos_.y, latest_drone_pos_.z);

        // 跟随模式切换逻辑
        double follow_height = this->get_parameter("follow_height").as_double();
        if (latest_drone_pos_.z > follow_height && !follow_enabled_)
        {
            follow_enabled_ = true;
            RCLCPP_INFO(this->get_logger(), "🟢 Yahboomcar开始跟随模式（无人机高度: %.2fm > 阈值: %.2fm）",
                        latest_drone_pos_.z, follow_height);
        }
        else if (latest_drone_pos_.z <= follow_height && follow_enabled_)
        {
            follow_enabled_ = false;
            stop_ugv();
            RCLCPP_INFO(this->get_logger(), "🔴 Yahboomcar停止跟随（无人机高度: %.2fm <= 阈值: %.2fm）",
                        latest_drone_pos_.z, follow_height);
        }
    }

    // Yahboomcar里程计回调
    void ugv_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_ugv_odom_ = *msg;
        ugv_odom_received_ = true;
        last_ugv_time_ = this->get_clock()->now();

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "🚗 收到Yahboomcar位置: x=%.2f, y=%.2f, yaw=%.2f°",
                             latest_ugv_odom_.pose.pose.position.x,
                             latest_ugv_odom_.pose.pose.position.y,
                             quaternion_to_yaw(latest_ugv_odom_.pose.pose.orientation) * 180.0 / M_PI);
    }

    // 障碍物状态回调
    void obstacle_state_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        obstacle_detected_ = msg->data;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "🚨 避障节点检测到障碍物: %s", obstacle_detected_ ? "是" : "否");
    }

    // 定时器回调（核心跟随控制逻辑）
    void timer_callback()
    {
        debug_counter_++;

        // 数据有效性检查
        if (!data_validity_check())
            return;

        // 跟随模式检查
        if (!follow_mode_check())
            return;

        // 降落检查
        if (landing_check())
            return;

        // 执行车辆跟随控制
        vehicle_following_control();
    }

    // 数据有效性检查
    bool data_validity_check()
    {
        if (!drone_position_received_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "⚠️ 未收到无人机位置数据!");
            stop_ugv();  // 发送停止指令
            return false;
        }

        if (!ugv_odom_received_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "⚠️ 未收到Yahboomcar里程计数据!");
            stop_ugv();  // 发送停止指令
            return false;
        }

        // 超时检查
        int timeout_ms = this->get_parameter("timeout_ms").as_int();
        auto now = this->get_clock()->now();

        if ((now - last_drone_time_).nanoseconds() / 1000000 > timeout_ms)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ 无人机位置数据超时，Yahboomcar停止跟随");
            stop_ugv();  // 发送停止指令
            return false;
        }

        if ((now - last_ugv_time_).nanoseconds() / 1000000 > timeout_ms)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ Yahboomcar位置数据超时，停止跟随");
            stop_ugv();  // 发送停止指令
            return false;
        }

        return true;
    }

    // 跟随模式检查
    bool follow_mode_check()
    {
        if (!follow_enabled_)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "⏸️ Yahboomcar跟随模式未启用 (无人机高度: %.2fm, 需要 > %.2fm)",
                                 latest_drone_pos_.z, this->get_parameter("follow_height").as_double());
            return false;
        }
        return true;
    }

    // 降落检查
    bool landing_check()
    {
        double land_threshold = this->get_parameter("land_threshold").as_double();
        if (latest_drone_pos_.z < land_threshold)
        {
            stop_ugv();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "🛬 无人机降落中 (高度: %.2fm < %.2fm)，Yahboomcar已停止",
                                 latest_drone_pos_.z, land_threshold);
            return true;
        }
        return false;
    }

    // Yahboomcar跟随控制核心逻辑
    void vehicle_following_control()
    {
        // 获取当前位置和朝向
        double ugv_x = latest_ugv_odom_.pose.pose.position.x;
        double ugv_y = latest_ugv_odom_.pose.pose.position.y;
        double ugv_yaw = quaternion_to_yaw(latest_ugv_odom_.pose.pose.orientation);

        double drone_x = latest_drone_pos_.x;
        double drone_y = latest_drone_pos_.y;
        double drone_z = latest_drone_pos_.z;

        // 计算当前距离
        double error_x = drone_x - ugv_x;
        double error_y = drone_y - ugv_y;
        double current_distance = sqrt(error_x * error_x + error_y * error_y);

        // 获取安全参数
        double desired_distance = this->get_parameter("desired_distance").as_double();
        double min_safe_distance = this->get_parameter("min_safe_distance").as_double();
        double max_follow_distance = this->get_parameter("max_follow_distance").as_double();
        double distance_tolerance = this->get_parameter("distance_tolerance").as_double();
        double land_threshold = this->get_parameter("land_threshold").as_double();

        // 无人机降落时的安全距离控制
        bool drone_is_landing = drone_z < (land_threshold + 0.8);
        double target_distance = desired_distance;

        if (drone_is_landing)
        {
            // 增加安全距离
            target_distance = std::max(desired_distance * 1.6, min_safe_distance * 2.0);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "🛬 无人机降落中，Yahboomcar增加安全距离至 %.2fm", target_distance);
        }

        // 距离过远检查
        if (current_distance > max_follow_distance)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "📏 距离过远 (%.2fm > %.2fm)，Yahboomcar加速追赶",
                                 current_distance, max_follow_distance);
        }

        // 计算距离误差（正值表示需要靠近，负值表示需要远离）
        double distance_error = current_distance - target_distance;

        // 如果距离在容差范围内，精细调整
        if (abs(distance_error) < distance_tolerance)
        {
            fine_position_control(ugv_x, ugv_y, ugv_yaw, drone_x, drone_y, current_distance, target_distance);
            return;
        }

        // 计算目标位置（在无人机与无人车连线上，距离无人机target_distance的位置）
        double target_x, target_y;
        if (current_distance > 0.1) // 避免除零
        {
            double ratio = target_distance / current_distance;
            target_x = drone_x - error_x * ratio;
            target_y = drone_y - error_y * ratio;
        }
        else
        {
            // 如果距离太近，直接后退
            target_x = ugv_x - cos(ugv_yaw) * target_distance;
            target_y = ugv_y - sin(ugv_yaw) * target_distance;
        }

        // 计算到目标位置的误差
        double target_error_x = target_x - ugv_x;
        double target_error_y = target_y - ugv_y;
        double target_distance_error = sqrt(target_error_x * target_error_x + target_error_y * target_error_y);

        // 计算目标角度
        double target_yaw = atan2(target_error_y, target_error_x);
        double yaw_error = normalize_angle(target_yaw - ugv_yaw);

        // 控制参数
        double kp_linear = this->get_parameter("kp_linear").as_double();
        double kp_angular = this->get_parameter("kp_angular").as_double();
        double max_linear_vel = this->get_parameter("max_linear_vel").as_double();
        double max_angular_vel = this->get_parameter("max_angular_vel").as_double();
        double angular_tolerance = this->get_parameter("angular_tolerance").as_double();

        // 计算控制指令
        geometry_msgs::msg::Twist twist;

        // 角速度控制（转向）
        twist.angular.z = kp_angular * yaw_error;
        twist.angular.z = std::clamp(twist.angular.z, -max_angular_vel, max_angular_vel);

        // 线速度控制
        double speed_factor = 1.0;

        // 避障时降低期望速度
        if (obstacle_detected_)
        {
            speed_factor *= 0.5; // 障碍物存在时，期望速度减半
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "⚠️ 检测到障碍物，跟随期望速度系数调整为: %.1f", speed_factor);
        }

        // 根据当前距离调整速度因子
        if (current_distance < min_safe_distance * 1.2)
        {
            speed_factor = 0.4; // 接近最小安全距离时降速
        }
        else if (current_distance < desired_distance * 0.8)
        {
            speed_factor = 0.7; // 接近期望距离时适度降速
        }
        else if (distance_error > 2.0)
        {
            speed_factor = 1.3; // 可以更快追赶
        }

        // 降落模式下的速度调整
        if (drone_is_landing)
        {
            speed_factor *= 0.6; // 降落时适度降速
        }

        // 计算线速度
        if (abs(yaw_error) < angular_tolerance)
        {
            // 朝向基本正确，前进/后退
            twist.linear.x = kp_linear * target_distance_error * speed_factor;

            // 如果需要后退（距离太近）
            if (distance_error < 0)
            {
                twist.linear.x = -abs(twist.linear.x);
            }
        }
        else
        {
            // 朝向不正确，边转向边前进
            twist.linear.x = kp_linear * target_distance_error * 0.5 * speed_factor;

            if (distance_error < 0)
            {
                twist.linear.x = -abs(twist.linear.x);
            }
        }

        // 速度限制
        twist.linear.x = std::clamp(twist.linear.x, -max_linear_vel, max_linear_vel);
        twist.linear.y = 0.0; // 差速驱动不能侧移

        // 发布控制指令到避障节点
        follow_cmd_publisher_->publish(twist);

        // 详细状态打印
        std::string mode = drone_is_landing ? "🛬降落模式" : "🎯跟随模式";
        std::string direction = distance_error > 0 ? "靠近" : "远离";
        std::string safety_status = current_distance < min_safe_distance ? "⚠️接近安全边界" : "✅安全距离";
        std::string obstacle_status = obstacle_detected_ ? "⚠️检测到障碍物" : "🚫无障碍物";
        std::string vehicle_status = "🚗Yahboomcar四驱";

        RCLCPP_INFO(this->get_logger(),
                    "%s %s: 当前距离=%.2fm | 目标距离=%.2fm | %s | %s | %s | yaw误差=%.1f°",
                    vehicle_status.c_str(), mode.c_str(), current_distance, target_distance,
                    direction.c_str(), safety_status.c_str(), obstacle_status.c_str(),
                    yaw_error * 180.0 / M_PI);

        RCLCPP_INFO(this->get_logger(),
                    "   🎮 控制指令: 线速度=%.2fm/s | 角速度=%.2frad/s | 安全因子=%.1f",
                    twist.linear.x, twist.angular.z, speed_factor);
    }

    // 精细位置控制（距离在目标范围内时）
    void fine_position_control(double ugv_x, double ugv_y, double ugv_yaw,
                               double drone_x, double drone_y, double current_distance, double target_distance)
    {
        // 在目标距离附近时的精细控制
        double distance_error = current_distance - target_distance;

        // 微调控制，保持位置
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0.25 * distance_error; // 精细调整

        // 避障时降低微调速度
        if (obstacle_detected_)
        {
            twist.linear.x *= 0.5;
        }

        // 限制微调速度
        twist.linear.x = std::clamp(twist.linear.x, -0.3, 0.3);
        twist.linear.y = 0.0;
        twist.angular.z = 0.0; // 保持当前朝向

        follow_cmd_publisher_->publish(twist);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "🎯 Yahboomcar精细控制: 当前距离=%.2fm | 目标距离=%.2fm | 距离误差=%.2fm | 微调速度=%.2fm/s",
                             current_distance, target_distance, distance_error, twist.linear.x);
    }

    // 停止Yahboomcar（向避障节点发送停止指令）
    void stop_ugv()
    {
        geometry_msgs::msg::Twist stop_twist;
        stop_twist.linear.x = 0.0;
        stop_twist.linear.y = 0.0;
        stop_twist.angular.z = 0.0;
        follow_cmd_publisher_->publish(stop_twist);
        RCLCPP_DEBUG(this->get_logger(), "🛑 Yahboomcar发送停止指令");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YahboomcarCoordinationNode>();

    RCLCPP_INFO(rclcpp::get_logger("main"), "🚀 启动Yahboomcar四驱无人车协同跟随控制节点");

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "节点运行异常: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
