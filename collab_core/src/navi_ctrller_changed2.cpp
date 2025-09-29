#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cmath>
#include <deque>
#include <vector>
#include <mutex>
#include <atomic>

//zjt 9.2晚写
/**
 * 协同控制节点 - 无人机跟随功能
 *
 * 修复内容：
 * 1. 修复频繁目标取消导致的停止问题
 * 2. 改进状态管理，防止死锁
 * 3. 优化目标发送逻辑，避免Nav2过载
 * 4. 改进重试机制，防止突然停止
 * 5. 添加更好的错误恢复机制
 */
class NavigationController : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavigationController() : Node("navigation_controller"),
                             tf_buffer_(this->get_clock()),
                             tf_listener_(tf_buffer_)
    {
        // 声明参数 - 优化默认值
        this->declare_parameter("base_frame_id", "map");
        this->declare_parameter("robot_frame_id", "base_link");
        this->declare_parameter("robot_odom_topic", "/yahboomcar/odom");
        this->declare_parameter("drone_odom_topic", "/drone/odom");
        this->declare_parameter("cmd_vel_topic", "/yahboomcar/cmd_vel");

        // 跟随模式参数 - 调整为更稳定的值
        this->declare_parameter("follow_mode", true);
        this->declare_parameter("follow_distance", 2.0);
        this->declare_parameter("follow_angle", 3.14159);
        this->declare_parameter("waypoint_tolerance", 2.0);  // 增加容忍度，减少目标更新
        this->declare_parameter("min_follow_distance", 0.8); // 增加最小距离
        this->declare_parameter("max_follow_distance", 10.0);

        // Nav2参数 - 更稳定的配置
        this->declare_parameter("goal_tolerance", 1.0);
        this->declare_parameter("min_time_between_goals", 2.0); // 增加到2秒，防止过于频繁

        // 安全参数
        this->declare_parameter("connection_timeout", 3.0);
        this->declare_parameter("enable_emergency_stop", true);
        this->declare_parameter("max_linear_vel", 2.0);
        this->declare_parameter("max_angular_vel", 1.8);
        this->declare_parameter("drone_height_min", 0.3);
        this->declare_parameter("drone_height_max", 15.0);

        // 修改后的跟随参数 - 更稳定
        this->declare_parameter("adaptive_follow", true);
        this->declare_parameter("urgent_follow_distance", 5.0); // 增加紧急距离阈值
        this->declare_parameter("urgent_goal_interval", 1.5);   // 增加到1.5秒，更稳定
        this->declare_parameter("goal_timeout", 15.0);          // 增加超时时间
        this->declare_parameter("max_retries", 3);
        this->declare_parameter("position_prediction", false); // 默认关闭预测，更稳定
        this->declare_parameter("prediction_time", 0.3);
        this->declare_parameter("goal_update_threshold", 1.5); // 新增：目标更新阈值
        this->declare_parameter("stuck_detection_time", 5.0);  // 新增：卡住检测时间
        this->declare_parameter("recovery_wait_time", 2.0);    // 新增：恢复等待时间

        // 获取参数
        base_frame_id_ = this->get_parameter("base_frame_id").as_string();
        robot_frame_id_ = this->get_parameter("robot_frame_id").as_string();
        follow_mode_ = this->get_parameter("follow_mode").as_bool();
        follow_distance_ = this->get_parameter("follow_distance").as_double();
        follow_angle_ = this->get_parameter("follow_angle").as_double();
        waypoint_tolerance_ = this->get_parameter("waypoint_tolerance").as_double();
        min_follow_distance_ = this->get_parameter("min_follow_distance").as_double();
        max_follow_distance_ = this->get_parameter("max_follow_distance").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        min_time_between_goals_ = this->get_parameter("min_time_between_goals").as_double();
        connection_timeout_ = this->get_parameter("connection_timeout").as_double();
        enable_emergency_stop_ = this->get_parameter("enable_emergency_stop").as_bool();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        drone_height_min_ = this->get_parameter("drone_height_min").as_double();
        drone_height_max_ = this->get_parameter("drone_height_max").as_double();
        adaptive_follow_ = this->get_parameter("adaptive_follow").as_bool();
        urgent_follow_distance_ = this->get_parameter("urgent_follow_distance").as_double();
        urgent_goal_interval_ = this->get_parameter("urgent_goal_interval").as_double();
        goal_timeout_ = this->get_parameter("goal_timeout").as_double();
        max_retries_ = this->get_parameter("max_retries").as_int();
        position_prediction_ = this->get_parameter("position_prediction").as_bool();
        prediction_time_ = this->get_parameter("prediction_time").as_double();
        goal_update_threshold_ = this->get_parameter("goal_update_threshold").as_double();
        stuck_detection_time_ = this->get_parameter("stuck_detection_time").as_double();
        recovery_wait_time_ = this->get_parameter("recovery_wait_time").as_double();

        // 初始化状态 - 使用atomic保证线程安全
        drone_connected_.store(false);
        nav2_ready_.store(false);
        goal_in_progress_.store(false);
        in_recovery_mode_.store(false);
        goal_retry_count_ = 0;
        consecutive_failures_ = 0;
        progressive_approach_depth_ = 0;  // 初始化递归深度
        last_goal_time_ = this->get_clock()->now();
        last_drone_msg_time_ = this->get_clock()->now();
        goal_sent_time_ = this->get_clock()->now();
        last_robot_movement_time_ = this->get_clock()->now();
        recovery_start_time_ = this->get_clock()->now();
        last_progressive_approach_time_ = this->get_clock()->now();

        // 初始化位置和速度
        robot_position_ = geometry_msgs::msg::Point();
        drone_position_ = geometry_msgs::msg::Point();
        last_goal_position_ = geometry_msgs::msg::Point();
        last_robot_position_ = geometry_msgs::msg::Point();
        drone_velocity_ = geometry_msgs::msg::Vector3();

        // 位置历史记录
        drone_position_history_.clear();

        // 创建订阅器
        robot_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("robot_odom_topic").as_string(),
            rclcpp::QoS(10).best_effort(),
            std::bind(&NavigationController::robot_odom_callback, this, std::placeholders::_1));

        drone_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("drone_odom_topic").as_string(),
            rclcpp::QoS(10).best_effort(),
            std::bind(&NavigationController::drone_odom_callback, this, std::placeholders::_1));

        // 创建发布器
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            this->get_parameter("cmd_vel_topic").as_string(), 10);

        // 创建Nav2动作客户端
        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        // 创建定时器
        nav2_check_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&NavigationController::check_nav2_server, this));

        // 跟随模式的目标更新定时器 - 使用更稳定的频率
        goal_update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), // 保持500ms，更稳定
            std::bind(&NavigationController::update_follow_navigation, this));

        // 无人机连接检查定时器
        connection_check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&NavigationController::check_drone_connection, this));

        // 目标超时和卡住检查定时器
        goal_timeout_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&NavigationController::check_goal_status, this));

        // 状态报告定时器
        status_report_timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&NavigationController::report_status, this));

        RCLCPP_INFO(this->get_logger(),
                    "\n========================================\n"
                    "协同控制节点已启动 - 稳定修复版本\n"
                    "跟随距离: %.2f m\n"
                    "跟随角度: %.2f rad (%.1f°)\n"
                    "最小跟随距离: %.2f m\n"
                    "最大跟随距离: %.2f m\n"
                    "目标更新间隔: %.3f s\n"
                    "紧急跟随距离: %.2f m\n"
                    "目标超时时间: %.2f s\n"
                    "位置预测: %s\n"
                    "========================================",
                    follow_distance_,
                    follow_angle_, follow_angle_ * 180.0 / 3.14159,
                    min_follow_distance_, max_follow_distance_,
                    min_time_between_goals_,
                    urgent_follow_distance_,
                    goal_timeout_,
                    position_prediction_ ? "启用" : "禁用");
    }

private:
    // 订阅器和发布器
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;

    // TF2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 定时器
    rclcpp::TimerBase::SharedPtr nav2_check_timer_;
    rclcpp::TimerBase::SharedPtr goal_update_timer_;
    rclcpp::TimerBase::SharedPtr connection_check_timer_;
    rclcpp::TimerBase::SharedPtr goal_timeout_timer_;
    rclcpp::TimerBase::SharedPtr status_report_timer_;

    // 参数
    std::string base_frame_id_, robot_frame_id_;
    bool follow_mode_, adaptive_follow_, position_prediction_;
    double follow_distance_, follow_angle_;
    double waypoint_tolerance_, min_follow_distance_, max_follow_distance_;
    double goal_tolerance_, min_time_between_goals_;
    double connection_timeout_;
    bool enable_emergency_stop_;
    double max_linear_vel_, max_angular_vel_;
    double drone_height_min_, drone_height_max_;
    double urgent_follow_distance_, urgent_goal_interval_;
    double goal_timeout_, prediction_time_;
    double goal_update_threshold_, stuck_detection_time_, recovery_wait_time_;
    int max_retries_;

    // 状态变量 - 使用atomic保证线程安全
    std::atomic<bool> drone_connected_, nav2_ready_, goal_in_progress_, in_recovery_mode_;
    geometry_msgs::msg::Point robot_position_, drone_position_, last_goal_position_;
    geometry_msgs::msg::Point last_robot_position_;
    geometry_msgs::msg::Vector3 drone_velocity_;
    rclcpp::Time last_goal_time_, last_drone_msg_time_, goal_sent_time_;
    rclcpp::Time last_robot_movement_time_, recovery_start_time_;
    std::shared_ptr<GoalHandleNavigateToPose> current_goal_handle_;
    int goal_retry_count_;
    int consecutive_failures_;
    std::mutex goal_mutex_;
    
    // 新增：渐进式靠近控制变量
    int progressive_approach_depth_;
    rclcpp::Time last_progressive_approach_time_;
    static const int MAX_PROGRESSIVE_DEPTH = 3;  // 最大递归深度
    static constexpr double MIN_PROGRESSIVE_INTERVAL = 1.0;  // 最小间隔1秒
    
    // 新增：恢复模式相关状态
    enum class RecoveryReason {
        CONSECUTIVE_FAILURES,
        GOAL_TIMEOUT,
        ROBOT_STUCK,
        DRONE_DISCONNECTED
    };
    RecoveryReason recovery_reason_;

    // 位置历史记录
    struct PositionRecord
    {
        rclcpp::Time timestamp;
        geometry_msgs::msg::Point position;
    };
    std::deque<PositionRecord> drone_position_history_;

    /**
     * 计算无人机速度
     */
    void calculate_drone_velocity()
    {
        if (drone_position_history_.size() < 2)
        {
            drone_velocity_.x = 0.0;
            drone_velocity_.y = 0.0;
            drone_velocity_.z = 0.0;
            return;
        }

        auto &latest = drone_position_history_.back();
        auto &previous = drone_position_history_[drone_position_history_.size() - 2];

        double dt = (latest.timestamp - previous.timestamp).seconds();
        if (dt > 0.001)
        {
            drone_velocity_.x = (latest.position.x - previous.position.x) / dt;
            drone_velocity_.y = (latest.position.y - previous.position.y) / dt;
            drone_velocity_.z = (latest.position.z - previous.position.z) / dt;
        }
    }

    /**
     * 预测无人机位置
     */
    geometry_msgs::msg::Point predict_drone_position(double prediction_time)
    {
        geometry_msgs::msg::Point predicted_pos = drone_position_;

        if (position_prediction_)
        {
            predicted_pos.x += drone_velocity_.x * prediction_time;
            predicted_pos.y += drone_velocity_.y * prediction_time;
            predicted_pos.z += drone_velocity_.z * prediction_time;
        }

        return predicted_pos;
    }

    /**
     * 无人机里程计回调
     */
    void drone_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        drone_connected_.store(true);
        auto now = this->get_clock()->now();
        last_drone_msg_time_ = now;
        drone_position_ = msg->pose.pose.position;

        // 记录位置历史
        PositionRecord record;
        record.timestamp = now;
        record.position = drone_position_;
        drone_position_history_.push_back(record);

        while (drone_position_history_.size() > 10)
        {
            drone_position_history_.pop_front();
        }

        calculate_drone_velocity();

        // 高度检查 - 改进版本
        if (drone_position_.z < drone_height_min_)
        {
            static int height_warning_count = 0;
            height_warning_count++;
            
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "⚠️ 无人机高度过低: %.2f m (最小: %.2f m) - 警告计数: %d",
                                 drone_position_.z, drone_height_min_, height_warning_count);
            
            // 如果高度异常持续，暂停跟随以确保安全
            if (height_warning_count > 5 && follow_mode_)
            {
                RCLCPP_ERROR(this->get_logger(), "🚨 无人机高度异常超过阈值，暂停跟随确保安全");
                
                // 发送停止命令
                geometry_msgs::msg::Twist stop_cmd;
                cmd_vel_pub_->publish(stop_cmd);
                
                // 取消当前目标
                if (current_goal_handle_)
                {
                    nav_action_client_->async_cancel_goal(current_goal_handle_);
                    current_goal_handle_.reset();
                }
                goal_in_progress_.store(false);
                
                // 重置计数器
                height_warning_count = 0;
            }
        }
        else
        {
            // 高度正常，重置警告计数
            static int height_warning_count = 0;
            height_warning_count = 0;
        }
    }

    /**
     * 机器人里程计回调 - 添加运动检测
     */
    void robot_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_position_ = msg->pose.pose.position;

        // 检测机器人是否在移动
        double movement = calculate_distance(robot_position_, last_robot_position_);
        if (movement > 0.05) // 移动超过5cm
        {
            last_robot_movement_time_ = this->get_clock()->now();
            last_robot_position_ = robot_position_;
        }

        RCLCPP_DEBUG(this->get_logger(),
                     "🤖 机器人位置: (%.2f, %.2f, %.2f)",
                     robot_position_.x, robot_position_.y, robot_position_.z);
    }

    /**
     * 计算两点距离
     */
    double calculate_distance(const geometry_msgs::msg::Point &p1,
                              const geometry_msgs::msg::Point &p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) +
                         std::pow(p1.y - p2.y, 2));
    }

    /**
     * 计算跟随位置
     */
    geometry_msgs::msg::Point calculate_follow_position()
    {
        auto target_drone_pos = position_prediction_ ? predict_drone_position(prediction_time_) : drone_position_;

        geometry_msgs::msg::Point follow_pos;
        follow_pos.x = target_drone_pos.x + follow_distance_ * std::cos(follow_angle_);
        follow_pos.y = target_drone_pos.y + follow_distance_ * std::sin(follow_angle_);
        follow_pos.z = 0.0;
        return follow_pos;
    }

    /**
     * 检查目标状态 - 合并超时和卡住检测
     */
    void check_goal_status()
    {
        if (!goal_in_progress_.load())
        {
            return;
        }

        auto now = this->get_clock()->now();

        // 检查目标超时
        double time_since_goal = (now - goal_sent_time_).seconds();
        if (time_since_goal > goal_timeout_)
        {
            RCLCPP_WARN(this->get_logger(),
                        "⏰ 目标超时 (%.1f秒)，进入恢复模式", time_since_goal);
            enter_recovery_mode(RecoveryReason::GOAL_TIMEOUT);
            return;
        }

        // 检查机器人是否卡住（长时间没有移动）
        double time_since_movement = (now - last_robot_movement_time_).seconds();
        if (time_since_movement > stuck_detection_time_)
        {
            RCLCPP_WARN(this->get_logger(),
                        "🚫 检测到机器人卡住 (%.1f秒未移动)，进入恢复模式",
                        time_since_movement);
            enter_recovery_mode(RecoveryReason::ROBOT_STUCK);
        }
    }

    /**
     * 进入恢复模式 - 改进版本
     */
    void enter_recovery_mode(RecoveryReason reason = RecoveryReason::CONSECUTIVE_FAILURES)
    {
        if (in_recovery_mode_.load())
        {
            return; // 已经在恢复模式中
        }

        recovery_reason_ = reason;
        std::string reason_str;
        
        switch (reason) {
            case RecoveryReason::CONSECUTIVE_FAILURES:
                reason_str = "连续失败";
                break;
            case RecoveryReason::GOAL_TIMEOUT:
                reason_str = "目标超时";
                break;
            case RecoveryReason::ROBOT_STUCK:
                reason_str = "机器人卡住";
                break;
            case RecoveryReason::DRONE_DISCONNECTED:
                reason_str = "无人机断连";
                break;
        }

        RCLCPP_INFO(this->get_logger(), "🔧 进入恢复模式... 原因: %s", reason_str.c_str());

        in_recovery_mode_.store(true);
        recovery_start_time_ = this->get_clock()->now();

        // 取消当前目标
        if (current_goal_handle_)
        {
            nav_action_client_->async_cancel_goal(current_goal_handle_);
            current_goal_handle_.reset();
        }

        // 重置状态
        goal_in_progress_.store(false);
        goal_retry_count_ = 0;
        progressive_approach_depth_ = 0;  // 重置渐进式靠近深度

        // 如果是因为卡住进入恢复模式，执行后退策略
        if (reason == RecoveryReason::ROBOT_STUCK || reason == RecoveryReason::CONSECUTIVE_FAILURES)
        {
            execute_backup_strategy();
        }
        else
        {
            // 其他情况只发送停止命令
            geometry_msgs::msg::Twist stop_cmd;
            cmd_vel_pub_->publish(stop_cmd);
        }
    }

    /**
     * 退出恢复模式 - 改进版本
     */
    void exit_recovery_mode()
    {
        if (!in_recovery_mode_.load())
        {
            return;
        }

        auto now = this->get_clock()->now();
        double recovery_time = (now - recovery_start_time_).seconds();
        
        // 根据恢复原因确定不同的等待时间
        double required_wait_time;
        switch (recovery_reason_) {
            case RecoveryReason::CONSECUTIVE_FAILURES:
                required_wait_time = recovery_wait_time_ * 2.0;  // 失败需要更长时间
                break;
            case RecoveryReason::GOAL_TIMEOUT:
                required_wait_time = recovery_wait_time_ * 1.5;  // 超时需要中等时间
                break;
            case RecoveryReason::ROBOT_STUCK:
                required_wait_time = recovery_wait_time_ * 3.0;  // 卡住需要最长时间
                break;
            case RecoveryReason::DRONE_DISCONNECTED:
                required_wait_time = recovery_wait_time_ * 0.5;  // 断连可以快速恢复
                break;
            default:
                required_wait_time = recovery_wait_time_;
        }

        if (recovery_time >= required_wait_time)
        {
            RCLCPP_INFO(this->get_logger(), "✅ 退出恢复模式，恢复正常跟随 (等待%.1fs)", recovery_time);
            in_recovery_mode_.store(false);
            consecutive_failures_ = 0;
            last_robot_movement_time_ = now;
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "🔧 恢复中... 还需等待%.1fs", required_wait_time - recovery_time);
        }
    }

    /**
     * 执行后退策略 - 当机器人卡住时
     */
    void execute_backup_strategy()
    {
        RCLCPP_INFO(this->get_logger(), "🔄 执行后退策略...");
        
        // 计算后退方向（与无人机方向相反）
        double direction_x = robot_position_.x - drone_position_.x;
        double direction_y = robot_position_.y - drone_position_.y;
        double direction_magnitude = std::sqrt(direction_x * direction_x + direction_y * direction_y);
        
        if (direction_magnitude > 0.1)
        {
            // 标准化方向向量
            direction_x /= direction_magnitude;
            direction_y /= direction_magnitude;
            
            // 后退1.5米
            geometry_msgs::msg::Point backup_target;
            backup_target.x = robot_position_.x + direction_x * 1.5;
            backup_target.y = robot_position_.y + direction_y * 1.5;
            backup_target.z = 0.0;
            
            RCLCPP_INFO(this->get_logger(),
                        "📍 后退目标: (%.2f, %.2f) | 后退距离: 1.5m",
                        backup_target.x, backup_target.y);
            
            // 发送后退目标（禁用渐进式靠近）
            send_follow_goal(backup_target, false, "后退策略", false);
        }
        else
        {
            // 如果无法计算方向，直接发送停止命令
            geometry_msgs::msg::Twist stop_cmd;
            cmd_vel_pub_->publish(stop_cmd);
            RCLCPP_WARN(this->get_logger(), "⚠️ 无法计算后退方向，停止机器人");
        }
    }

    /**
     * 跟随模式的导航更新 - 改进版本
     */
    void update_follow_navigation()
    {
        // 恢复模式检查
        if (in_recovery_mode_.load())
        {
            exit_recovery_mode();
            return;
        }

        // 基本检查
        if (!nav2_ready_.load() || !drone_connected_.load() || !follow_mode_)
        {
            return;
        }

        // 使用try_lock避免死锁
        std::unique_lock<std::mutex> lock(goal_mutex_, std::try_to_lock);
        if (!lock.owns_lock())
        {
            RCLCPP_DEBUG(this->get_logger(), "目标更新被跳过（mutex被占用）");
            return;
        }

        // 计算当前状态
        geometry_msgs::msg::Point target_pos = calculate_follow_position();
        double distance_to_target = calculate_distance(robot_position_, target_pos);
        double distance_to_drone = calculate_distance(robot_position_, drone_position_);

        // 无人机速度计算（用于稳定性判断）
        double drone_speed = std::sqrt(drone_velocity_.x * drone_velocity_.x + 
                                       drone_velocity_.y * drone_velocity_.y);

        // 判断是否需要发送新目标 - 改进版本
        bool need_new_goal = false;
        bool urgent = false;
        std::string reason = "";

        // 检查是否需要更新目标
        if (distance_to_drone > urgent_follow_distance_)
        {
            // 紧急情况：距离过远
            need_new_goal = true;
            urgent = true;
            reason = "距离过远(" + std::to_string(distance_to_drone) + "m)";
        }
        else if (!goal_in_progress_.load() && distance_to_target > waypoint_tolerance_)
        {
            // 没有目标且距离较远 - 添加无人机速度检查
            if (drone_speed < 0.5 || distance_to_target > waypoint_tolerance_ * 1.5)
            {
                // 只有当无人机速度较慢或距离足够远时才发送新目标
                need_new_goal = true;
                reason = "无目标，距离目标" + std::to_string(distance_to_target) + "m";
            }
        }
        else if (goal_in_progress_.load())
        {
            // 检查目标是否需要更新 - 增加无人机速度权重
            double goal_position_change = calculate_distance(target_pos, last_goal_position_);
            double dynamic_threshold = goal_update_threshold_;
            
            // 如果无人机移动很快，增加更新阈值，减少频繁更新
            if (drone_speed > 1.0)
            {
                dynamic_threshold *= 1.5;  // 无人机快速移动时，更保守地更新目标
            }
            
            if (goal_position_change > dynamic_threshold)
            {
                need_new_goal = true;
                reason = "目标位置变化" + std::to_string(goal_position_change) + "m";
            }
        }

        // 时间间隔检查 - 改进版本，自适应间隔
        auto now = this->get_clock()->now();
        double time_since_last_goal = (now - last_goal_time_).seconds();
        
        // 自适应时间间隔：距离越远，间隔越长，给Nav2更多时间处理
        double base_interval = urgent ? urgent_goal_interval_ : min_time_between_goals_;
        double distance_factor = 1.0;
        if (urgent && distance_to_drone > urgent_follow_distance_ * 1.5) {
            distance_factor = 1.5;  // 极远距离时，增加50%间隔
        }
        double effective_min_time = base_interval * distance_factor;

        if (need_new_goal && time_since_last_goal >= effective_min_time)
        {
            // 不再频繁取消目标，只在必要时取消
            if (goal_in_progress_.load() && urgent)
            {
                // 只在紧急情况下取消当前目标
                if (current_goal_handle_)
                {
                    nav_action_client_->async_cancel_goal(current_goal_handle_);
                    current_goal_handle_.reset();
                }
                goal_in_progress_.store(false);
            }

            // 如果没有目标在执行，发送新目标
            if (!goal_in_progress_.load())
            {
                send_follow_goal(target_pos, urgent, reason);
            }
        }
    }

    /**
     * 渐进式靠近策略 - 改进版本，防止无限递归
     */
    void send_progressive_approach_goal(const geometry_msgs::msg::Point &final_target)
    {
        auto now = this->get_clock()->now();
        
        // 检查时间间隔，防止过于频繁调用
        double time_since_last = (now - last_progressive_approach_time_).seconds();
        if (time_since_last < MIN_PROGRESSIVE_INTERVAL)
        {
            RCLCPP_DEBUG(this->get_logger(),
                        "⏱️ 渐进式靠近间隔过短(%.2fs)，跳过", time_since_last);
            return;
        }
        
        // 检查递归深度，防止无限递归
        if (progressive_approach_depth_ >= MAX_PROGRESSIVE_DEPTH)
        {
            RCLCPP_WARN(this->get_logger(),
                        "🚫 渐进式靠近达到最大深度(%d)，进入恢复模式",
                        MAX_PROGRESSIVE_DEPTH);
            progressive_approach_depth_ = 0;  // 重置深度
            enter_recovery_mode(RecoveryReason::CONSECUTIVE_FAILURES);
            return;
        }
        
        // 计算当前距离
        double distance_to_target = calculate_distance(robot_position_, final_target);
        
        // 如果距离已经足够近，直接发送目标
        if (distance_to_target <= waypoint_tolerance_ * 1.5)
        {
            RCLCPP_INFO(this->get_logger(),
                        "✅ 距离已足够近(%.2fm)，直接发送目标", distance_to_target);
            progressive_approach_depth_ = 0;  // 重置深度
            send_follow_goal(final_target, false, "渐进式靠近完成", false);  // 禁用渐进式靠近
            return;
        }
        
        // 计算中间目标点
        geometry_msgs::msg::Point intermediate_target;
        double direction_x = final_target.x - robot_position_.x;
        double direction_y = final_target.y - robot_position_.y;
        double direction_magnitude = std::sqrt(direction_x * direction_x + direction_y * direction_y);
        
        if (direction_magnitude > 0.1)
        {
            // 标准化方向向量
            direction_x /= direction_magnitude;
            direction_y /= direction_magnitude;
            
            // 设置中间目标，距离根据递归深度动态调整
            double base_step = std::min(4.0, distance_to_target * 0.6);
            double depth_factor = 1.0 / (1.0 + progressive_approach_depth_ * 0.3);  // 深度越深，步长越小
            double step_distance = base_step * depth_factor;
            
            intermediate_target.x = robot_position_.x + direction_x * step_distance;
            intermediate_target.y = robot_position_.y + direction_y * step_distance;
            intermediate_target.z = 0.0;
            
            // 更新状态
            progressive_approach_depth_++;
            last_progressive_approach_time_ = now;
            
            RCLCPP_INFO(this->get_logger(),
                        "🎯 渐进式靠近[深度%d]: 中间目标(%.2f, %.2f) | 步长: %.2fm | 最终距离: %.2fm",
                        progressive_approach_depth_, intermediate_target.x, intermediate_target.y, 
                        step_distance, distance_to_target);
            
            send_follow_goal(intermediate_target, false, "渐进式靠近", false);  // 禁用渐进式靠近
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ 目标距离过近，无法计算方向");
            progressive_approach_depth_ = 0;  // 重置深度
        }
    }

    /**
     * 发送跟随目标 - 改进版本
     */
    void send_follow_goal(const geometry_msgs::msg::Point &target_pos, bool urgent = false,
                          const std::string &reason = "", bool allow_progressive = true)
    {
        if (!nav_action_client_)
        {
            RCLCPP_ERROR(this->get_logger(), "Nav2客户端未初始化");
            return;
        }

        // 防止在恢复模式下发送目标
        if (in_recovery_mode_.load())
        {
            RCLCPP_DEBUG(this->get_logger(), "恢复模式中，跳过目标发送");
            return;
        }

        // 检查连续失败次数 - 改进版本
        if (consecutive_failures_ >= max_retries_)
        {
            // 检查是否是由于距离过远导致的失败
            double distance_to_drone = calculate_distance(robot_position_, drone_position_);
            
            if (distance_to_drone > urgent_follow_distance_ * 1.5 && allow_progressive)
            {
                // 距离过远，采用渐进式靠近策略而非恢复模式
                RCLCPP_WARN(this->get_logger(),
                            "⚠️ 距离过远(%.2fm)，采用渐进式靠近策略", distance_to_drone);
                send_progressive_approach_goal(target_pos);
                consecutive_failures_ = std::max(0, consecutive_failures_ - 1); // 稍微减少失败计数
                return;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "⚠️ 连续失败%d次，进入恢复模式", consecutive_failures_);
                enter_recovery_mode();
                return;
            }
        }

        // 构造目标消息
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.stamp = this->get_clock()->now();
        goal_msg.pose.header.frame_id = base_frame_id_;
        goal_msg.pose.pose.position = target_pos;

        // 计算朝向
        double yaw = std::atan2(
            drone_position_.y - target_pos.y,
            drone_position_.x - target_pos.x);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        goal_msg.pose.pose.orientation = tf2::toMsg(q);

        // 设置发送选项
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        // 目标响应回调
        send_goal_options.goal_response_callback =
            [this, urgent](auto goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_WARN(this->get_logger(), "❌ 目标被Nav2拒绝");
                this->goal_in_progress_.store(false);
                this->consecutive_failures_++;
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "✅ %s目标被Nav2接受",
                             urgent ? "紧急" : "常规");
                this->current_goal_handle_ = goal_handle;
                this->goal_in_progress_.store(true);
                this->goal_sent_time_ = this->get_clock()->now();
                // 成功发送不重置consecutive_failures_，等结果回调确认
            }
        };

        // 反馈回调
        send_goal_options.feedback_callback =
            [this](auto, auto)
        {
            // 收到反馈说明导航正在进行
            this->last_robot_movement_time_ = this->get_clock()->now();
        };

        // 结果回调
        send_goal_options.result_callback =
            [this](const GoalHandleNavigateToPose::WrappedResult &result)
        {
            this->goal_in_progress_.store(false);
            this->current_goal_handle_.reset();

            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_DEBUG(this->get_logger(), "✅ 到达跟随位置");
                this->consecutive_failures_ = 0; // 成功，重置失败计数
                this->progressive_approach_depth_ = 0;  // 成功时重置渐进式靠近深度
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_DEBUG(this->get_logger(), "⚠️ 导航中止");
                this->consecutive_failures_++;
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_DEBUG(this->get_logger(), "🔄 导航取消");
                // 取消不计入失败
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "❌ 导航未知结果");
                this->consecutive_failures_++;
            }
        };

        // 发送目标
        try
        {
            auto future = nav_action_client_->async_send_goal(goal_msg, send_goal_options);

            // 更新状态
            last_goal_position_ = target_pos;
            last_goal_time_ = this->get_clock()->now();

            double distance_from_robot = calculate_distance(target_pos, robot_position_);
            double distance_to_drone = calculate_distance(robot_position_, drone_position_);

            RCLCPP_INFO(this->get_logger(),
                        "🎯 发送%s目标: (%.2f, %.2f) | 原因: %s | 距机器人: %.2fm | 距无人机: %.2fm",
                        urgent ? "紧急" : "常规",
                        target_pos.x, target_pos.y,
                        reason.c_str(),
                        distance_from_robot, distance_to_drone);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ 发送目标失败: %s", e.what());
            goal_in_progress_.store(false);
            consecutive_failures_++;
        }
    }

    /**
     * 检查Nav2服务器
     */
    void check_nav2_server()
    {
        bool was_ready = nav2_ready_.load();
        bool is_ready = nav_action_client_->wait_for_action_server(std::chrono::milliseconds(100));
        nav2_ready_.store(is_ready);

        if (is_ready != was_ready)
        {
            if (is_ready)
            {
                RCLCPP_INFO(this->get_logger(), "✅ Nav2服务器连接成功");
                // 重置状态
                goal_in_progress_.store(false);
                consecutive_failures_ = 0;
                in_recovery_mode_.store(false);
                if (current_goal_handle_)
                {
                    current_goal_handle_.reset();
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "❌ Nav2服务器连接丢失");
                goal_in_progress_.store(false);
                if (current_goal_handle_)
                {
                    current_goal_handle_.reset();
                }
            }
        }
    }

    /**
     * 检查无人机连接
     */
    void check_drone_connection()
    {
        if (!drone_connected_.load())
        {
            return;
        }

        auto now = this->get_clock()->now();
        double time_since_msg = (now - last_drone_msg_time_).seconds();

        if (time_since_msg > connection_timeout_)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "❌ 无人机连接超时 (%.1f秒无消息)", time_since_msg);
            drone_connected_.store(false);

            if (enable_emergency_stop_)
            {
                emergency_stop();
            }
        }
    }

    /**
     * 紧急停止
     */
    void emergency_stop()
    {
        // 发送停止命令
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);

        // 取消当前导航目标
        if (current_goal_handle_)
        {
            nav_action_client_->async_cancel_goal(current_goal_handle_);
            current_goal_handle_.reset();
        }

        goal_in_progress_.store(false);
        consecutive_failures_ = 0;
        RCLCPP_ERROR(this->get_logger(), "🚨 执行紧急停止！");
    }

    /**
     * 状态报告
     */
    void report_status()
    {
        if (!drone_connected_.load())
        {
            RCLCPP_INFO(this->get_logger(), "⏸️ 状态: 等待无人机连接...");
            return;
        }

        double distance_to_drone = calculate_distance(robot_position_, drone_position_);
        geometry_msgs::msg::Point target_pos = calculate_follow_position();
        double dist_to_target = calculate_distance(robot_position_, target_pos);

        std::string mode_status;
        if (in_recovery_mode_.load())
        {
            mode_status = "🔧恢复中";
        }
        else if (goal_in_progress_.load())
        {
            mode_status = "🏃跟随中";
        }
        else
        {
            mode_status = "⏸️待机中";
        }

        std::string urgent_status = (distance_to_drone > urgent_follow_distance_) ? "🚨紧急" : "🟢正常";

        double drone_speed = std::sqrt(drone_velocity_.x * drone_velocity_.x +
                                       drone_velocity_.y * drone_velocity_.y);

        RCLCPP_INFO(this->get_logger(),
                    "📊 %s %s | 距无人机: %.2fm | 距目标: %.2fm | "
                    "Nav2: %s | 失败: %d/%d | 无人机速度: %.2fm/s",
                    mode_status.c_str(), urgent_status.c_str(),
                    distance_to_drone, dist_to_target,
                    nav2_ready_.load() ? "✅" : "❌",
                    consecutive_failures_, max_retries_,
                    drone_speed);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<NavigationController>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "异常: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}