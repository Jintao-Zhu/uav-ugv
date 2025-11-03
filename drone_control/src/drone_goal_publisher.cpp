#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

/**
 * 高性能无人机位置发布节点 - 针对实时跟随优化
 * 1. 解决 “坐标系不兼容” 问题，统一完成 NED→ENU 转换
 * 2. 提供 “标准化消息格式”，将数据封装为 ROS 标准消息
 * 3. 性能优化与状态监控
 */
class DronePositionPublisher : public rclcpp::Node
{
public:
    DronePositionPublisher() : Node("drone_position_publisher")
    {
        // 声明参数 - 优化默认值
        this->declare_parameter("base_frame_id", "map");  // 从odom改为map，与MAVROS统一
        this->declare_parameter("drone_frame_id", "iris::base_link");  // 与SDF保持一致
        this->declare_parameter("drone_height_threshold", -1.0);
        this->declare_parameter("connection_timeout", 5.0); // 稍微放宽超时
        this->declare_parameter("simulation_mode", false);
        this->declare_parameter("publish_rate", 20.0);           // 大幅提升到20Hz
        this->declare_parameter("simulation_speed_factor", 1.5); // 新增：仿真速度因子
        this->declare_parameter("smooth_trajectory", true);      // 新增：平滑轨迹选项

        // 获取参数
        base_frame_id_ = this->get_parameter("base_frame_id").as_string();
        drone_frame_id_ = this->get_parameter("drone_frame_id").as_string();
        height_threshold_ = this->get_parameter("drone_height_threshold").as_double();
        connection_timeout_ = this->get_parameter("connection_timeout").as_double();
        simulation_mode_ = this->get_parameter("simulation_mode").as_bool();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        simulation_speed_factor_ = this->get_parameter("simulation_speed_factor").as_double();
        smooth_trajectory_ = this->get_parameter("smooth_trajectory").as_bool();

        // 初始化状态
        drone_connected_ = simulation_mode_;
        last_drone_pos_ = {0.0, 0.0, 0.0};
        last_drone_msg_time_ = this->get_clock()->now();
        sim_time_ = 0.0;

        // 优化的QoS设置 - 为实时性优化
        auto px4_qos = rclcpp::QoS(5).best_effort().durability_volatile();
        auto pub_qos = rclcpp::QoS(5).reliable(); // 发布使用可靠传输确保不丢失

        // 订阅无人机位置
        if (!simulation_mode_)
        {
            drone_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position", px4_qos,
                std::bind(&DronePositionPublisher::drone_pos_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "📡 订阅PX4无人机位置话题");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "🎮 仿真模式：生成高性能测试轨迹");
            // 仿真模式下设置更好的初始位置
            last_drone_pos_ = {6.0, 0.0, 2.5};
        }

        // 发布无人机位置信息
        drone_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/drone/pose", pub_qos);

        drone_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/drone/odom", pub_qos);

        // 高频率位置发布定时器
        auto timer_period = std::chrono::microseconds(
            static_cast<int64_t>(1000000.0 / publish_rate_));
        position_pub_timer_ = this->create_wall_timer(
            timer_period, std::bind(&DronePositionPublisher::publish_position, this));

        // 连接监控定时器 - 仅在非仿真模式下启用
        if (!simulation_mode_)
        {
            connection_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000),
                std::bind(&DronePositionPublisher::check_connection, this));
        }

        // 状态报告定时器 - 降低频率减少日志干扰
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(10), // 延长到10秒
            std::bind(&DronePositionPublisher::report_status, this));

        RCLCPP_INFO(this->get_logger(),
                    "\n========================================\n"
                    "🚁 无人机位置发布节点 - 高性能版本\n"
                    "发布频率: %.1f Hz (高性能优化)\n"
                    "仿真模式: %s\n"
                    "坐标系: %s\n"
                    "平滑轨迹: %s\n"
                    "========================================",
                    publish_rate_,
                    simulation_mode_ ? "启用" : "禁用",
                    base_frame_id_.c_str(),
                    smooth_trajectory_ ? "启用" : "禁用");
    }

private:
    // 订阅器和发布器
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr drone_pos_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr drone_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr drone_odom_pub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr position_pub_timer_;
    rclcpp::TimerBase::SharedPtr connection_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // 参数
    std::string base_frame_id_, drone_frame_id_;
    double height_threshold_, connection_timeout_;
    bool simulation_mode_, smooth_trajectory_;
    double publish_rate_, simulation_speed_factor_;

    // 状态变量
    bool drone_connected_;
    std::array<double, 3> last_drone_pos_;
    std::array<double, 3> last_velocity_; // 新增：速度记录
    rclcpp::Time last_drone_msg_time_;
    double sim_time_; // 仿真时间

    /**
     * 生成高性能测试轨迹 - 优化版本
     */
    void update_simulation_position()
    {
        sim_time_ += 1.0 / publish_rate_;

        if (smooth_trajectory_)
        {
            // 生成更复杂但平滑的轨迹，测试跟随系统性能
            // 结合多个正弦波形成复杂轨迹
            double base_freq = 0.1 * simulation_speed_factor_;
            double fast_freq = 0.3 * simulation_speed_factor_;

            // X方向：大范围移动 + 小幅震荡
            last_drone_pos_[0] = 8.0 + 6.0 * std::cos(base_freq * sim_time_) +
                                 1.0 * std::sin(fast_freq * sim_time_);

            // Y方向：椭圆运动 + 扰动
            last_drone_pos_[1] = 4.0 * std::sin(base_freq * sim_time_) +
                                 0.8 * std::cos(fast_freq * sim_time_ * 1.7);

            // Z方向：稳定飞行高度 + 轻微上下浮动
            last_drone_pos_[2] = 2.5 + 0.5 * std::sin(base_freq * sim_time_ * 0.7);

            // 计算速度（用于里程计）
            static double prev_time = sim_time_;
            double dt = sim_time_ - prev_time;
            if (dt > 0.001)
            {
                static std::array<double, 3> prev_pos = last_drone_pos_;
                last_velocity_[0] = (last_drone_pos_[0] - prev_pos[0]) / dt;
                last_velocity_[1] = (last_drone_pos_[1] - prev_pos[1]) / dt;
                last_velocity_[2] = (last_drone_pos_[2] - prev_pos[2]) / dt;
                prev_pos = last_drone_pos_;
            }
            prev_time = sim_time_;
        }
        else
        {
            // 简单轨迹模式
            last_drone_pos_[0] = 5.0 + 3.0 * std::cos(sim_time_ * 0.2);
            last_drone_pos_[1] = 3.0 * std::sin(sim_time_ * 0.2);
            last_drone_pos_[2] = 2.0;

            // 简单速度计算
            last_velocity_[0] = -3.0 * 0.2 * std::sin(sim_time_ * 0.2);
            last_velocity_[1] = 3.0 * 0.2 * std::cos(sim_time_ * 0.2);
            last_velocity_[2] = 0.0;
        }
    }

    /**
     * 状态报告 - 减少频繁输出
     */
    void report_status()
    {
        std::string status_msg = drone_connected_ ? "✅连接" : "❌断开";
        double speed = std::sqrt(last_velocity_[0] * last_velocity_[0] +
                                 last_velocity_[1] * last_velocity_[1]);

        RCLCPP_INFO(this->get_logger(),
                    "📊 状态: %s | 位置(%.2f, %.2f, %.2f) | 速度: %.2fm/s | 频率: %.1fHz",
                    status_msg.c_str(),
                    last_drone_pos_[0], last_drone_pos_[1], last_drone_pos_[2],
                    speed, publish_rate_);
    }

    /**
     * 无人机位置回调 - 优化版本
     */
    void drone_pos_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        drone_connected_ = true;
        last_drone_msg_time_ = this->get_clock()->now();

        // 高度检查 - 减少警告频率
        if (msg->z > height_threshold_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, // 10秒节流
                                 "⏸️ 无人机高度过低: %.2f m", -msg->z);
        }

        // NED → ENU 坐标转换
        last_drone_pos_[0] = msg->y;  // North -> East (x)
        last_drone_pos_[1] = msg->x;  // East -> North (y)
        last_drone_pos_[2] = -msg->z; // Down -> Up (z)

        // 速度转换 (NED -> ENU)
        last_velocity_[0] = msg->vy;  // North -> East (vx)
        last_velocity_[1] = msg->vx;  // East -> North (vy)
        last_velocity_[2] = -msg->vz; // Down -> Up (vz)

        // 使用DEBUG级别减少日志输出
        RCLCPP_DEBUG(this->get_logger(),
                     "📌 无人机ENU: 位置(%.2f, %.2f, %.2f) 速度(%.2f, %.2f, %.2f)",
                     last_drone_pos_[0], last_drone_pos_[1], last_drone_pos_[2],
                     last_velocity_[0], last_velocity_[1], last_velocity_[2]);
    }

    /**
     * 高性能位置发布 - 优化版本
     */
    void publish_position()
    {
        if (!drone_connected_)
        {
            return;
        }

        auto now = this->get_clock()->now();

        // 仿真模式下更新轨迹
        if (simulation_mode_)
        {
            update_simulation_position();
        }

        // 预构造通用数据，减少重复计算
        geometry_msgs::msg::Point position;
        position.x = last_drone_pos_[0];
        position.y = last_drone_pos_[1];
        position.z = last_drone_pos_[2];

        geometry_msgs::msg::Quaternion orientation;
        orientation.x = 0.0;
        orientation.y = 0.0;
        orientation.z = 0.0;
        orientation.w = 1.0;

        // 发布PoseStamped消息
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = now;
        pose_msg.header.frame_id = base_frame_id_;
        pose_msg.pose.position = position;
        pose_msg.pose.orientation = orientation;

        drone_pose_pub_->publish(pose_msg);

        // 发布Odometry消息 - 包含速度信息
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = base_frame_id_;
        odom_msg.child_frame_id = drone_frame_id_;

        // 位置信息
        odom_msg.pose.pose.position = position;
        odom_msg.pose.pose.orientation = orientation;

        // 优化的位置协方差 - 较高置信度
        std::fill(odom_msg.pose.covariance.begin(), odom_msg.pose.covariance.end(), 0.0);
        odom_msg.pose.covariance[0] = 0.01;  // x
        odom_msg.pose.covariance[7] = 0.01;  // y
        odom_msg.pose.covariance[14] = 0.01; // z
        odom_msg.pose.covariance[21] = 0.01; // roll
        odom_msg.pose.covariance[28] = 0.01; // pitch
        odom_msg.pose.covariance[35] = 0.01; // yaw

        // 速度信息 - 真实速度数据
        odom_msg.twist.twist.linear.x = simulation_mode_ ? last_velocity_[0] : last_velocity_[0];
        odom_msg.twist.twist.linear.y = simulation_mode_ ? last_velocity_[1] : last_velocity_[1];
        odom_msg.twist.twist.linear.z = simulation_mode_ ? last_velocity_[2] : last_velocity_[2];
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;

        // 速度协方差
        std::fill(odom_msg.twist.covariance.begin(), odom_msg.twist.covariance.end(), 0.0);
        odom_msg.twist.covariance[0] = 0.01;  // vx
        odom_msg.twist.covariance[7] = 0.01;  // vy
        odom_msg.twist.covariance[14] = 0.01; // vz
        odom_msg.twist.covariance[21] = 0.01; // wx
        odom_msg.twist.covariance[28] = 0.01; // wy
        odom_msg.twist.covariance[35] = 0.01; // wz

        drone_odom_pub_->publish(odom_msg);

        // 减少DEBUG输出频率
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "📤 发布@%.1fHz: (%.2f, %.2f, %.2f)",
                              publish_rate_,
                              last_drone_pos_[0], last_drone_pos_[1], last_drone_pos_[2]);
    }

    /**
     * 连接状态检查 - 优化版本
     */
    void check_connection()
    {
        if (simulation_mode_)
        {
            return;
        }

        auto now = this->get_clock()->now();
        auto time_since_last_msg = (now - last_drone_msg_time_).seconds();

        if (time_since_last_msg > connection_timeout_)
        {
            if (drone_connected_)
            {
                RCLCPP_ERROR(this->get_logger(),
                             "❌ 无人机连接超时 (%.1fs)", time_since_last_msg);
                drone_connected_ = false;
            }
        }
        else if (!drone_connected_ && time_since_last_msg <= connection_timeout_)
        {
            RCLCPP_INFO(this->get_logger(), "✅ 无人机连接已恢复");
            drone_connected_ = true;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<DronePositionPublisher>();

        // 使用多线程执行器提高性能
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
        executor.add_node(node);
        executor.spin();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("drone_position_publisher"),
                     "节点异常退出: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}