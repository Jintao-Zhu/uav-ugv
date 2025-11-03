/*
改为发送话题而不是Action  11.1在另一个代码里加入卡住强制更新目标点逻辑
 */

// ==== ROS2核心库 ====
#include <rclcpp/rclcpp.hpp>              // ROS2核心功能

// ==== 消息类型定义 ====
#include <nav_msgs/msg/odometry.hpp>      // 里程计消息类型
#include <geometry_msgs/msg/twist.hpp>    // 速度控制消息类型
#include <geometry_msgs/msg/pose_stamped.hpp> // 位姿消息类型

// ==== 坐标变换库 ====
#include <tf2/LinearMath/Quaternion.h>    // 四元数数学运算
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // TF2与geometry_msgs转换
#include <tf2_ros/transform_listener.h>   // TF2坐标变换监听器
#include <tf2_ros/buffer.h>               // TF2变换缓冲区

// ==== 标准库 ====
#include <cmath>     // 数学计算函数
#include <deque>     // 双端队列容器，用于位置历史记录
#include <vector>    // 动态数组容器
#include <mutex>     // 互斥锁，保证线程安全
#include <atomic>    // 原子操作，用于线程安全的状态变量

/**
 * @class NavigationFollowGoal
 * @brief 无人机协同控制跟随目标发布节点类
 * 
 * @details 
 * NavigationFollowGoal是一个ROS2节点类，专门设计用于计算并发布地面机器人
 * 跟随无人机的目标位置。该类继承自rclcpp::Node，具备以下核心功能：
 * 
 * ## 主要功能模块：
 * 
 * ### 1. 数据采集与处理
 * - 实时接收无人机和地面机器人的里程计数据
 * - 计算无人机的运动速度和预测位置
 * - 维护位置历史记录用于趋势分析
 * 
 * ### 2. 智能跟随算法
 * - 根据无人机位置动态计算最优跟随位置
 * - 支持自定义跟随距离和角度
 * - 实现自适应跟随策略，根据距离调整跟随行为
 * 
 * ### 3. 目标发布系统
 * - 发布跟随目标到位姿话题
 * - 智能目标管理，避免频繁的目标更新
 * - 支持紧急跟随模式，应对快速移动场景
 * 
 * ### 4. 安全保护机制
 * - 连接状态监控，检测无人机通信中断
 * - 目标超时检测，防止异常状态
 * - 紧急停止功能，确保系统安全
 * 
 * ## 设计特点：
 * - **线程安全**：使用atomic变量和mutex确保多线程安全
 * - **事件驱动**：基于ROS2回调机制的异步事件处理
 * - **模块化设计**：清晰的功能模块划分，便于维护和扩展
 * - **参数化配置**：丰富的参数配置，适应不同应用场景
 * - **状态机管理**：明确的状态转换逻辑，保证系统可靠性
 */
class NavigationFollowGoal : public rclcpp::Node
{
public:
    NavigationFollowGoal() : Node("navigation_follow_goal"),
                             tf_buffer_(this->get_clock()),
                             tf_listener_(tf_buffer_)
    {


        // 声明参数 - 优化默认值
        this->declare_parameter("base_frame_id", "map");
        this->declare_parameter("robot_frame_id", "base_link");
        this->declare_parameter("robot_odom_topic", "/yahboomcar/odom");
        this->declare_parameter("drone_odom_topic", "/drone/odom");
        this->declare_parameter("cmd_vel_topic", "/yahboomcar/cmd_vel");
        this->declare_parameter("follow_goal_topic", "/navigation/follow_goal");

        // 跟随模式参数 - 调整为更稳定的值
        this->declare_parameter("follow_mode", true);
        this->declare_parameter("follow_distance", 2.0);
        this->declare_parameter("follow_angle", 3.14159);
        this->declare_parameter("waypoint_tolerance", 2.0);  // 增加容忍度，减少目标更新
        this->declare_parameter("min_follow_distance", 0.8); // 增加最小距离
        this->declare_parameter("max_follow_distance", 10.0);

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
        this->declare_parameter("urgent_goal_interval", 1.5);   // 提高到1.5秒，降低更新频率
        // 紧急模式滞回阈值（避免在边缘来回抖动）
        this->declare_parameter("urgent_on_distance", 20.0);  //10.31注意：由5.5改成20.0
        this->declare_parameter("urgent_off_distance", 15.0); //10.31注意：由4.5改成15.0
        this->declare_parameter("goal_timeout", 15.0);          // 增加超时时间
        this->declare_parameter("max_retries", 3);
        this->declare_parameter("position_prediction", false); // 默认关闭预测，更稳定
        this->declare_parameter("prediction_time", 0.3);
        this->declare_parameter("goal_update_threshold", 1.5); // 目标更新阈值
        this->declare_parameter("stuck_detection_time", 5.0);  // 卡住检测时间

        // 获取参数
        base_frame_id_ = this->get_parameter("base_frame_id").as_string();
        robot_frame_id_ = this->get_parameter("robot_frame_id").as_string();
        follow_mode_ = this->get_parameter("follow_mode").as_bool();
        follow_distance_ = this->get_parameter("follow_distance").as_double();
        follow_angle_ = this->get_parameter("follow_angle").as_double();
        waypoint_tolerance_ = this->get_parameter("waypoint_tolerance").as_double();
        min_follow_distance_ = this->get_parameter("min_follow_distance").as_double();
        max_follow_distance_ = this->get_parameter("max_follow_distance").as_double();
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

        // 读取滞回阈值并做基本校验
        urgent_on_distance_ = this->get_parameter("urgent_on_distance").as_double();
        urgent_off_distance_ = this->get_parameter("urgent_off_distance").as_double();
        // 若用户未配置合理数值，则基于 urgent_follow_distance_ 设置默认滞回
        if (!(urgent_off_distance_ < urgent_follow_distance_ && urgent_follow_distance_ < urgent_on_distance_))
        {
            urgent_on_distance_ = urgent_follow_distance_ + 0.5;
            urgent_off_distance_ = std::max(min_follow_distance_, urgent_follow_distance_ - 0.5);
        }

        // 初始化状态 - 使用atomic保证线程安全
        drone_connected_.store(false);
        goal_in_progress_.store(false);
        in_recovery_mode_.store(false);
        goal_retry_count_ = 0;
        consecutive_failures_ = 0;
        last_goal_time_ = this->get_clock()->now();
        last_drone_msg_time_ = this->get_clock()->now();
        goal_sent_time_ = this->get_clock()->now();
        last_robot_movement_time_ = this->get_clock()->now();
        recovery_start_time_ = this->get_clock()->now();

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
            std::bind(&NavigationFollowGoal::robot_odom_callback, this, std::placeholders::_1));

        drone_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("drone_odom_topic").as_string(),
            rclcpp::QoS(10).best_effort(),
            std::bind(&NavigationFollowGoal::drone_odom_callback, this, std::placeholders::_1));

        // 创建发布器
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            this->get_parameter("cmd_vel_topic").as_string(), 10);

        // 创建跟随目标发布器（核心修改：只发布话题，不发送Action）
        follow_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            this->get_parameter("follow_goal_topic").as_string(),
            rclcpp::QoS(10).reliable());

        // 创建定时器
        goal_update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), // 保持500ms，更稳定
            std::bind(&NavigationFollowGoal::update_follow_navigation, this));

        // 无人机连接检查定时器
        connection_check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&NavigationFollowGoal::check_drone_connection, this));

        // 目标超时和卡住检查定时器
        goal_timeout_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&NavigationFollowGoal::check_goal_status, this));

        // 状态报告定时器
        status_report_timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&NavigationFollowGoal::report_status, this));

        RCLCPP_INFO(this->get_logger(),
                    "\n========================================\n"
                    "跟随目标发布节点已启动\n"
                    "发布话题: %s\n"
                    "跟随距离: %.2f m\n"
                    "跟随角度: %.2f rad (%.1f°)\n"
                    "最小跟随距离: %.2f m\n"
                    "最大跟随距离: %.2f m\n"
                    "目标更新间隔: %.3f s\n"
                    "紧急跟随距离: %.2f m\n"
                    "位置预测: %s\n"
                    "========================================",
                    this->get_parameter("follow_goal_topic").as_string().c_str(),
                    follow_distance_,
                    follow_angle_, follow_angle_ * 180.0 / M_PI,
                    min_follow_distance_, max_follow_distance_,
                    urgent_goal_interval_,
                    urgent_follow_distance_,
                    position_prediction_ ? "启用" : "禁用");
    }

private:
    // ============================================================================
    // ==== ROS2通信组件 ====
    // ============================================================================
    
    /** @brief 机器人里程计数据订阅器 - 接收地面机器人的位置和姿态信息 */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_odom_sub_;
    
    /** @brief 无人机里程计数据订阅器 - 接收无人机的位置、姿态和速度信息 */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_odom_sub_;
    
    /** @brief 速度控制命令发布器 - 向机器人发送运动控制指令 */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    /** @brief 跟随目标发布器 - 发布计算出的跟随目标位置 */
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr follow_goal_pub_;

    // ============================================================================
    // ==== 坐标变换组件 ====
    // ============================================================================
    
    /** @brief TF2变换缓冲区 - 存储和管理坐标系之间的变换关系 */
    tf2_ros::Buffer tf_buffer_;
    
    /** @brief TF2变换监听器 - 监听并缓存坐标变换数据 */
    tf2_ros::TransformListener tf_listener_;

    // ============================================================================
    // ==== 定时器组件 ====
    // ============================================================================
    
    /** @brief 跟随目标更新定时器 - 定期计算和更新跟随导航目标 */
    rclcpp::TimerBase::SharedPtr goal_update_timer_;
    
    /** @brief 无人机连接检查定时器 - 监控无人机通信状态 */
    rclcpp::TimerBase::SharedPtr connection_check_timer_;
    
    /** @brief 目标超时和状态检查定时器 - 检测导航目标超时和机器人卡住状态 */
    rclcpp::TimerBase::SharedPtr goal_timeout_timer_;
    
    /** @brief 系统状态报告定时器 - 定期输出系统运行状态信息 */
    rclcpp::TimerBase::SharedPtr status_report_timer_;

    // ============================================================================
    // ==== 系统配置参数 ====
    // ============================================================================
    
    /** @brief 基础坐标系ID - 全局坐标系名称，通常为"map" */
    std::string base_frame_id_;
    
    /** @brief 机器人坐标系ID - 机器人本体坐标系名称，通常为"base_link" */
    std::string robot_frame_id_;
    
    /** @brief 跟随模式开关 - 控制是否启用自动跟随功能 */
    bool follow_mode_;
    
    /** @brief 自适应跟随开关 - 启用基于距离的动态跟随策略 */
    bool adaptive_follow_;
    
    /** @brief 位置预测开关 - 启用无人机位置预测功能 */
    bool position_prediction_;
    
    /** @brief 跟随距离 - 机器人与无人机之间的理想跟随距离(米) */
    double follow_distance_;
    
    /** @brief 跟随角度 - 机器人相对于无人机的跟随角度(弧度) */
    double follow_angle_;
    
    /** @brief 航点容忍度 - 到达目标点的距离容忍度(米) */
    double waypoint_tolerance_;
    
    /** @brief 最小跟随距离 - 机器人与无人机的最小安全距离(米) */
    double min_follow_distance_;
    
    /** @brief 最大跟随距离 - 超过此距离将触发紧急跟随模式(米) */
    double max_follow_distance_;
    
    /** @brief 连接超时时间 - 无人机通信超时阈值(秒) */
    double connection_timeout_;
    
    /** @brief 紧急停止开关 - 是否启用自动紧急停止功能 */
    bool enable_emergency_stop_;
    
    /** @brief 最大线速度 - 机器人运动的最大线性速度限制(m/s) */
    double max_linear_vel_;
    
    /** @brief 最大角速度 - 机器人运动的最大角速度限制(rad/s) */
    double max_angular_vel_;
    
    /** @brief 无人机最小飞行高度 - 安全飞行的最低高度(米) */
    double drone_height_min_;
    
    /** @brief 无人机最大飞行高度 - 安全飞行的最高高度(米) */
    double drone_height_max_;
    
    /** @brief 紧急跟随距离阈值 - 超过此距离启动紧急跟随模式(米) */
    double urgent_follow_distance_;
    
    /** @brief 紧急模式目标发送间隔 - 紧急情况下的目标更新频率(秒) */
    double urgent_goal_interval_;
    
    /** @brief 导航目标超时时间 - 导航任务的最大执行时间(秒) */
    double goal_timeout_;
    
    /** @brief 位置预测时间 - 预测无人机未来位置的时间跨度(秒) */
    double prediction_time_;
    
    /** @brief 目标更新阈值 - 触发目标更新的位置变化阈值(米) */
    double goal_update_threshold_;
    
    /** @brief 卡住检测时间 - 检测机器人卡住的时间阈值(秒) */
    double stuck_detection_time_;
    
    /** @brief 最大重试次数 - 导航失败的最大重试次数 */
    int max_retries_;

    // ============================================================================
    // ==== 系统状态变量 (线程安全) ====
    // ============================================================================
    
    /** @brief 无人机连接状态 - 原子变量，指示无人机通信是否正常 */
    std::atomic<bool> drone_connected_;
    
    /** @brief 导航目标执行状态 - 原子变量，指示是否有导航目标正在执行 */
    std::atomic<bool> goal_in_progress_;
    
    /** @brief 恢复模式状态 - 原子变量，指示系统是否处于故障恢复模式 */
    std::atomic<bool> in_recovery_mode_;

    // ============================================================================
    // ==== 位置和运动状态 ====
    // ============================================================================
    
    /** @brief 机器人当前位置 - 地面机器人在全局坐标系中的实时位置 */
    geometry_msgs::msg::Point robot_position_;
    
    /** @brief 无人机当前位置 - 无人机在全局坐标系中的实时位置 */
    geometry_msgs::msg::Point drone_position_;
    
    /** @brief 上次发送的目标位置 - 用于检测目标位置变化 */
    geometry_msgs::msg::Point last_goal_position_;
    
    /** @brief 机器人上次记录位置 - 用于检测机器人是否在移动 */
    geometry_msgs::msg::Point last_robot_position_;
    
    /** @brief 无人机速度向量 - 无人机的三维速度分量 */
    geometry_msgs::msg::Vector3 drone_velocity_;

    // ============================================================================
    // ==== 时间戳管理 ====
    // ============================================================================
    
    /** @brief 上次目标发送时间 - 用于控制目标发送频率 */
    rclcpp::Time last_goal_time_;
    
    /** @brief 上次无人机消息时间 - 用于检测无人机通信超时 */
    rclcpp::Time last_drone_msg_time_;
    
    /** @brief 目标发送时间戳 - 记录当前目标的发送时刻 */
    rclcpp::Time goal_sent_time_;
    
    /** @brief 机器人上次移动时间 - 用于检测机器人是否卡住 */
    rclcpp::Time last_robot_movement_time_;
    
    /** @brief 恢复模式开始时间 - 记录进入恢复模式的时刻 */
    rclcpp::Time recovery_start_time_;

    /** @brief 最近一次我们主动取消目标的时间，用于过滤随后的异常 */
    rclcpp::Time last_self_cancel_time_;

    /** @brief 紧急模式状态（带滞回，避免阈值抖动） */
    bool in_urgent_mode_ = false;

    /** @brief 紧急模式滞回阈值（进入/退出） */
    double urgent_on_distance_ = 20.0;
    double urgent_off_distance_ = 15.0;

    // ============================================================================
    // ==== 导航控制 ====
    // ============================================================================
    
    /** @brief 目标重试计数器 - 当前目标的重试次数 */
    int goal_retry_count_;
    
    /** @brief 连续失败计数器 - 连续导航失败的次数 */
    int consecutive_failures_;
    
    /** @brief 目标操作互斥锁 - 保护导航目标相关操作的线程安全 */
    std::mutex goal_mutex_;

    // ============================================================================
    // ==== 数据结构定义 ====
    // ============================================================================
    
    /**
     * @struct PositionRecord
     * @brief 位置记录结构体
     * @details 用于存储带时间戳的位置信息，支持速度计算和运动趋势分析
     */
    struct PositionRecord
    {
        rclcpp::Time timestamp;                ///< 位置记录的时间戳
        geometry_msgs::msg::Point position;    ///< 三维位置坐标
    };
    
    /** @brief 无人机位置历史记录 - 存储最近的无人机位置数据，用于速度计算 */
    std::deque<PositionRecord> drone_position_history_;

    /**
     * @brief 计算无人机速度
     * @details 根据无人机位置历史记录计算当前速度向量
     */
    void calculate_drone_velocity()
    {
        // 检查是否有足够的历史数据点
        if (drone_position_history_.size() < 2)
        {
            // 数据不足，将速度设置为零
            drone_velocity_.x = 0.0;
            drone_velocity_.y = 0.0;
            drone_velocity_.z = 0.0;
            return;
        }

        // 获取最新和次新的位置记录
        auto &latest = drone_position_history_.back();      // 最新位置
        auto &previous = drone_position_history_[drone_position_history_.size() - 2]; // 次新位置

        // 计算时间差（秒）
        double dt = (latest.timestamp - previous.timestamp).seconds();
        
        // 检查时间间隔是否有效（防止除零和过小时间间隔造成的噪声）
        if (dt > 0.001)
        {
            // 数值微分计算速度：v = Δs / Δt
            drone_velocity_.x = (latest.position.x - previous.position.x) / dt;
            drone_velocity_.y = (latest.position.y - previous.position.y) / dt;
            drone_velocity_.z = (latest.position.z - previous.position.z) / dt;
        }
    }

    /**
     * @brief 预测无人机位置
     * @param prediction_time 预测的时间跨度（秒）
     * @return geometry_msgs::msg::Point 预测的无人机位置
     */
    geometry_msgs::msg::Point predict_drone_position(double prediction_time)
    {
        // 从当前位置开始预测
        geometry_msgs::msg::Point predicted_pos = drone_position_;

        // 如果启用了位置预测功能
        if (position_prediction_)
        {
            // 线性外推：新位置 = 当前位置 + 速度 × 时间
            predicted_pos.x += drone_velocity_.x * prediction_time;
            predicted_pos.y += drone_velocity_.y * prediction_time;
            predicted_pos.z += drone_velocity_.z * prediction_time;
        }
        // 如果未启用预测，直接返回当前位置

        return predicted_pos;
    }

    /**
     * @brief 无人机里程计数据回调函数
     * @param msg 接收到的无人机里程计消息
     */
    void drone_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 更新无人机连接状态（原子操作，线程安全）
        drone_connected_.store(true);
        
        // 记录当前时间戳，用于连接超时检测
        auto now = this->get_clock()->now();
        last_drone_msg_time_ = now;
        
        // 提取无人机位置信息
        drone_position_ = msg->pose.pose.position;

        // 创建新的位置记录并添加到历史队列
        PositionRecord record;
        record.timestamp = now;
        record.position = drone_position_;
        drone_position_history_.push_back(record);

        // 维护位置历史队列大小，保留最近10个记录
        while (drone_position_history_.size() > 10)
        {
            drone_position_history_.pop_front();
        }

        // 根据位置历史计算当前速度
        calculate_drone_velocity();

        // 安全检查：监控无人机飞行高度
        if (drone_position_.z < drone_height_min_)
        {
            // 使用节流日志避免日志洪水，每5秒最多输出一次警告
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "⚠️ 无人机高度过低: %.2f m (最小: %.2f m)",
                                 drone_position_.z, drone_height_min_);
        }
    }

    /**
     * @brief 机器人里程计数据回调函数
     * @param msg 接收到的机器人里程计消息
     */
    void robot_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 更新机器人当前位置
        robot_position_ = msg->pose.pose.position;

        // 运动检测：计算与上次记录位置的距离变化
        double movement = calculate_distance(robot_position_, last_robot_position_);
        
        // 如果移动距离超过阈值（5cm），更新运动状态
        if (movement > 0.05) // 5cm的移动阈值
        {
            // 更新最后移动时间，用于卡住检测
            last_robot_movement_time_ = this->get_clock()->now();
            // 保存当前位置作为下次比较的基准
            last_robot_position_ = robot_position_;
        }

        // 调试级别的位置信息输出
        RCLCPP_DEBUG(this->get_logger(),
                     "🤖 机器人位置: (%.2f, %.2f, %.2f)",
                     robot_position_.x, robot_position_.y, robot_position_.z);
    }

    /**
     * @brief 计算两点间的欧几里得距离
     * @param p1 第一个点的位置
     * @param p2 第二个点的位置
     * @return double 两点间的二维平面距离（米）
     */
    double calculate_distance(const geometry_msgs::msg::Point &p1,
                              const geometry_msgs::msg::Point &p2)
    {
        // 使用欧几里得距离公式计算二维平面距离
        return std::sqrt(std::pow(p1.x - p2.x, 2) +
                         std::pow(p1.y - p2.y, 2));
    }

    /**
     * @brief 计算理想的跟随位置
     * @return geometry_msgs::msg::Point 计算得出的跟随目标位置
     */
    geometry_msgs::msg::Point calculate_follow_position()
    {
        // 确定无人机目标位置：使用当前位置或预测位置
        auto target_drone_pos = position_prediction_ ? 
            predict_drone_position(prediction_time_) : drone_position_;

        // 初始化跟随位置
        geometry_msgs::msg::Point follow_pos;
        
        // 根据极坐标公式计算跟随位置
        // 在无人机位置基础上，按照指定距离和角度计算偏移
        follow_pos.x = target_drone_pos.x + follow_distance_ * std::cos(follow_angle_);
        follow_pos.y = target_drone_pos.y + follow_distance_ * std::sin(follow_angle_);
        follow_pos.z = 0.0; // 地面机器人高度设为0
        
        return follow_pos;
    }

    /**
     * @brief 检查导航目标状态和系统健康状况
     */
    void check_goal_status()
    {
        // 只有在导航目标执行中才进行状态检查
        if (!goal_in_progress_.load())
        {
            return;
        }

        auto now = this->get_clock()->now();

        // ==== 导航目标超时检测 ====
        double time_since_goal = (now - goal_sent_time_).seconds();
        if (time_since_goal > goal_timeout_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "⏰ 导航目标超时 (%.1f秒)", time_since_goal);
            consecutive_failures_++;
        }

        // ==== 机器人卡住检测 ====
        double time_since_movement = (now - last_robot_movement_time_).seconds();
        if (time_since_movement > stuck_detection_time_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "🚫 检测到机器人可能卡住 (%.1f秒未移动)", time_since_movement);
        }
    }

    /**
     * @brief 进入系统恢复模式
     */
    void enter_recovery_mode()
    {
        // 防止重复进入恢复模式
        if (in_recovery_mode_.load())
        {
            return; // 已经在恢复模式中，直接返回
        }

        RCLCPP_INFO(this->get_logger(), "🔧 进入恢复模式，开始系统恢复...");

        // 原子操作：设置恢复模式标志
        in_recovery_mode_.store(true);
        recovery_start_time_ = this->get_clock()->now();

        // ==== 清理导航状态 ====
        goal_in_progress_.store(false);  // 清除目标执行标志
        goal_retry_count_ = 0;           // 重置重试计数器

        // ==== 安全停止机器人 ====
        geometry_msgs::msg::Twist stop_cmd; // 默认构造函数创建零速度命令
        cmd_vel_pub_->publish(stop_cmd);     // 发送停止命令

        RCLCPP_INFO(this->get_logger(), "✅ 恢复模式初始化完成，等待系统稳定...");
    }

    /**
     * @brief 退出系统恢复模式
     */
    void exit_recovery_mode()
    {
        // 检查是否确实处于恢复模式
        if (!in_recovery_mode_.load())
        {
            return; // 不在恢复模式中，无需退出
        }

        auto now = this->get_clock()->now();
        double recovery_time = (now - recovery_start_time_).seconds();

        // 检查是否已经过足够的恢复等待时间
        if (recovery_time >= 2.0) // 使用2秒作为固定恢复时间
        {
            RCLCPP_INFO(this->get_logger(), 
                       "✅ 退出恢复模式，恢复正常跟随功能 (恢复时间: %.1f秒)", 
                       recovery_time);
            
            // ==== 状态重置 ====
            in_recovery_mode_.store(false);    // 原子操作：退出恢复模式
            consecutive_failures_ = 0;         // 重置连续失败计数
            last_robot_movement_time_ = now;   // 更新移动时间戳，防止立即卡住检测
            
            RCLCPP_INFO(this->get_logger(), "🚀 系统恢复完成，准备继续跟随任务");
        }
        else
        {
            // 还需要继续等待
            RCLCPP_DEBUG(this->get_logger(), 
                        "⏳ 恢复模式等待中... (%.1f/2.0秒)", 
                        recovery_time);
        }
    }

    /**
     * 跟随模式的导航更新 - 发布跟随目标话题
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
        if (!drone_connected_.load() || !follow_mode_)
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
        double goal_position_change = calculate_distance(target_pos, last_goal_position_);

        // 判断是否需要发送新目标
        bool need_new_goal = false;
        bool urgent = false;
        std::string reason = "";

        // 紧急模式滞回判断：
        if (distance_to_drone >= urgent_on_distance_)
        {
            in_urgent_mode_ = true;
        }
        else if (distance_to_drone <= urgent_off_distance_)
        {
            in_urgent_mode_ = false;
        }

        // 检查是否需要更新目标
        if (in_urgent_mode_)
        {
            // 紧急情况：距离过远
            need_new_goal = true;
            urgent = true;
            reason = "距离过远(" + std::to_string(distance_to_drone) + "m)";
        }
        else if (!goal_in_progress_.load() && distance_to_target > waypoint_tolerance_)
        {
            // 没有目标且距离较远
            need_new_goal = true;
            reason = "无目标，距离目标" + std::to_string(distance_to_target) + "m";
        }
        else if (goal_in_progress_.load())
        {
            // 检查目标是否需要更新
            if (goal_position_change > goal_update_threshold_)
            {
                need_new_goal = true;
                reason = "目标位置变化" + std::to_string(goal_position_change) + "m";
            }
        }

        // 时间间隔检查
        auto now = this->get_clock()->now();
        double time_since_last_goal = (now - last_goal_time_).seconds();
        double effective_min_time = urgent ? urgent_goal_interval_ : 1.0; // 常规模式1秒间隔

        if (need_new_goal && time_since_last_goal >= effective_min_time)
        {
            // 发布新的跟随目标
            publish_follow_goal(target_pos, urgent, reason);
            goal_in_progress_.store(true);
        }
    }

    /**
     * 发布跟随目标到话题
     */
    void publish_follow_goal(const geometry_msgs::msg::Point &target_pos, bool urgent = false,
                          const std::string &reason = "")
    {
        // 防止在恢复模式下发布目标
        if (in_recovery_mode_.load())
        {
            RCLCPP_DEBUG(this->get_logger(), "恢复模式中，跳过目标发布");
            return;
        }

        // 构造目标消息
        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.stamp = this->get_clock()->now();
        goal_msg.header.frame_id = base_frame_id_;
        goal_msg.pose.position = target_pos;

        // 计算朝向（面向无人机）
        double yaw = std::atan2(
            drone_position_.y - target_pos.y,
            drone_position_.x - target_pos.x);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        goal_msg.pose.orientation = tf2::toMsg(q);

        // 发布目标
        try
        {
            follow_goal_pub_->publish(goal_msg);
            double dist_from_last = calculate_distance(target_pos, last_goal_position_); // 计算与上一目标的距离
            // 更新状态
            last_goal_position_ = target_pos;
            last_goal_time_ = this->get_clock()->now();
            goal_sent_time_ = last_goal_time_;

            double distance_from_robot = calculate_distance(target_pos, robot_position_);
            double distance_to_drone = calculate_distance(robot_position_, drone_position_);

            RCLCPP_INFO(this->get_logger(),
                        "🎯 发布%s跟随目标: (%.2f, %.2f) | 原因: %s | 距机器人: %.2fm | 距无人机: %.2fm",
                        urgent ? "紧急" : "常规",
                        target_pos.x, target_pos.y,
                        reason.c_str(),
                        distance_from_robot, distance_to_drone);
            // 新增：打印新目标与旧目标的距离（验证目标是否更新）
            
            RCLCPP_INFO(this->get_logger(), "📌 新目标与旧目标距离: %.2f 米", dist_from_last);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ 发布目标失败: %s", e.what());
            goal_in_progress_.store(false);
            consecutive_failures_++;
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
                    "失败: %d/%d | 无人机速度: %.2fm/s",
                    mode_status.c_str(), urgent_status.c_str(),
                    distance_to_drone, dist_to_target,
                    consecutive_failures_, max_retries_,
                    drone_speed);
    }
};

/**
 * @brief 程序主入口函数
 */
int main(int argc, char **argv)
{
    // ==== ROS2系统初始化 ====
    rclcpp::init(argc, argv);

    try
    {
        // ==== 创建导航控制节点 ====
        auto node = std::make_shared<NavigationFollowGoal>();
        
        // ==== 启动ROS2事件循环 ====
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        // ==== 异常处理 ====
        RCLCPP_ERROR(rclcpp::get_logger("main"), 
                     "❌ 程序异常退出: %s", e.what());
    }

    // ==== 资源清理 ====
    rclcpp::shutdown();
    
    return 0; // 正常退出
}