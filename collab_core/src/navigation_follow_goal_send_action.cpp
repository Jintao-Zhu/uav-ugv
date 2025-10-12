 /**       这是原来的发送Action的代码
 * @file navi_controller_version1.cpp
 * @brief ROS2无人机协同控制节点 - 地面机器人跟随无人机导航系统
 * 
 * @details 
 * 该文件实现了一个高级的无人机-地面机器人协同控制系统，主要功能包括：
 * - 地面机器人实时跟随无人机飞行
 * - 智能路径规划与导航控制
 * - 动态避障与安全保护机制
 * - 自适应跟随距离调整
 * - 故障检测与自动恢复
 * 
 * 系统特点：
 * - 采用Nav2导航框架进行路径规划
 * - 使用TF2进行坐标变换管理
 * - 实现线程安全的状态管理
 * - 支持位置预测与自适应控制
 * - 具备完善的错误处理机制
 * 
 * @author zjt
 * @version 1.0
 * @date 2024年9月2日
 * @copyright Copyright (c) 2024
 * 
 * @note 修复版本说明：
 * 1. 修复频繁目标取消导致的停止问题
 * 2. 改进状态管理，防止死锁
 * 3. 优化目标发送逻辑，避免Nav2过载
 * 4. 改进重试机制，防止突然停止
 * 5. 添加更好的错误恢复机制
 */

// ==== ROS2核心库 ====
#include <rclcpp/rclcpp.hpp>              // ROS2核心功能
#include <rclcpp_action/rclcpp_action.hpp> // ROS2动作客户端/服务器支持

// ==== 消息类型定义 ====
#include <nav_msgs/msg/odometry.hpp>      // 里程计消息类型
#include <geometry_msgs/msg/twist.hpp>    // 速度控制消息类型
#include <nav2_msgs/action/navigate_to_pose.hpp> // Nav2导航动作消息

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
 * @class NavigationController
 * @brief 无人机协同控制导航节点类
 * 
 * @details 
 * NavigationController是一个高级的ROS2节点类，专门设计用于实现地面机器人跟随无人机的
 * 协同导航控制系统。该类继承自rclcpp::Node，具备以下核心功能：
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
 * ### 3. 导航控制系统
 * - 与Nav2导航系统深度集成
 * - 智能目标管理，避免频繁的目标更新
 * - 支持紧急跟随模式，应对快速移动场景
 * 
 * ### 4. 安全保护机制
 * - 连接状态监控，检测无人机通信中断
 * - 目标超时检测，防止导航卡死
 * - 机器人运动监控，检测卡住状态
 * - 紧急停止功能，确保系统安全
 * 
 * ### 5. 故障恢复系统
 * - 自动检测各种异常状态
 * - 智能恢复策略，最小化服务中断
 * - 连续失败计数，防止无限重试
 * - 恢复模式管理，确保系统稳定性
 * 
 * ## 设计特点：
 * - **线程安全**：使用atomic变量和mutex确保多线程安全
 * - **事件驱动**：基于ROS2回调机制的异步事件处理
 * - **模块化设计**：清晰的功能模块划分，便于维护和扩展
 * - **参数化配置**：丰富的参数配置，适应不同应用场景
 * - **状态机管理**：明确的状态转换逻辑，保证系统可靠性
 * 
 * ## 应用场景：
 * - 搜救任务中的地面-空中协同作业
 * - 环境监测中的多平台协同巡检
 * - 物流配送中的跟随护送任务
 * - 科研实验中的多机器人协作
 * 
 * @note 本类专为高可靠性应用设计，具备完善的错误处理和恢复机制
 * @warning 使用前请确保Nav2导航系统正确配置，并建立合适的地图
 */
class NavigationController : public rclcpp::Node
{
public:
    // ==== 类型别名定义 ====
    // 为复杂的模板类型定义简洁的别名，提高代码可读性
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
    this->declare_parameter("urgent_goal_interval", 1.5);   // 提高到1.5秒，降低抢占频率
    // 紧急模式滞回阈值（避免在边缘来回抖动），默认在 urgent_follow_distance 上下各 0.5m
    this->declare_parameter("urgent_on_distance", 5.5);
    this->declare_parameter("urgent_off_distance", 4.5);
        this->declare_parameter("goal_timeout", 15.0);          // 增加超时时间
        this->declare_parameter("max_retries", 3);
        this->declare_parameter("position_prediction", false); // 默认关闭预测，更稳定
        this->declare_parameter("prediction_time", 0.3);
    this->declare_parameter("goal_update_threshold", 1.5); // 新增：目标更新阈值
    this->declare_parameter("stuck_detection_time", 5.0);  // 新增：卡住检测时间
    this->declare_parameter("recovery_wait_time", 2.0);    // 新增：恢复等待时间
    // 新增：是否启用节点级恢复（默认关闭，低层恢复交给Nav2行为树）
    this->declare_parameter("controller_recovery_enabled", false);

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
    controller_recovery_enabled_ = this->get_parameter("controller_recovery_enabled").as_bool();

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
        nav2_ready_.store(false);
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

    // 记录 Nav2 就绪时间（用于预热）
    nav2_ready_since_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

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
    // ============================================================================
    // ==== ROS2通信组件 ====
    // ============================================================================
    
    /** @brief 机器人里程计数据订阅器 - 接收地面机器人的位置和姿态信息 */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_odom_sub_;
    
    /** @brief 无人机里程计数据订阅器 - 接收无人机的位置、姿态和速度信息 */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_odom_sub_;
    
    /** @brief 速度控制命令发布器 - 向机器人发送运动控制指令 */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    /** @brief Nav2导航动作客户端 - 与Nav2导航系统通信，发送导航目标 */
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;

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
    
    /** @brief Nav2服务器连接检查定时器 - 定期检查Nav2导航服务器的可用性 */
    rclcpp::TimerBase::SharedPtr nav2_check_timer_;
    
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
    
    /** @brief 目标容忍度 - Nav2导航的目标到达容忍度(米) */
    double goal_tolerance_;
    
    /** @brief 最小目标发送间隔 - 防止过于频繁发送导航目标(秒) */
    double min_time_between_goals_;
    
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
    
    /** @brief 恢复等待时间 - 故障恢复模式的等待时间(秒) */
    double recovery_wait_time_;
    
    /** @brief 是否启用节点级恢复（默认关闭，优先由Nav2恢复） */
    bool controller_recovery_enabled_;
    
    /** @brief 最大重试次数 - 导航失败的最大重试次数 */
    int max_retries_;

    // ============================================================================
    // ==== 系统状态变量 (线程安全) ====
    // ============================================================================
    
    /** @brief 无人机连接状态 - 原子变量，指示无人机通信是否正常 */
    std::atomic<bool> drone_connected_;
    
    /** @brief Nav2就绪状态 - 原子变量，指示Nav2导航服务器是否可用 */
    std::atomic<bool> nav2_ready_;
    
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

    /** @brief Nav2 就绪的起始时间，用于预热延迟 */
    rclcpp::Time nav2_ready_since_;

    /** @brief 最近一次我们主动取消目标的时间，用于过滤随后的 ABORTED 结果 */
    rclcpp::Time last_self_cancel_time_;

    /** @brief 紧急模式状态（带滞回，避免阈值抖动） */
    bool in_urgent_mode_ = false;

    /** @brief 紧急模式滞回阈值（进入/退出） */
    double urgent_on_distance_ = 5.5;
    double urgent_off_distance_ = 4.5;

    // ============================================================================
    // ==== 导航控制 ====
    // ============================================================================
    
    /** @brief 当前导航目标句柄 - 指向正在执行的Nav2导航目标 */
    std::shared_ptr<GoalHandleNavigateToPose> current_goal_handle_;
    
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
     * @details 
     * 根据无人机位置历史记录计算当前速度向量。该函数使用数值微分方法，
     * 通过最近两个位置记录之间的位移和时间差来计算速度。
     * 
     * 算法步骤：
     * 1. 检查位置历史记录是否有足够的数据点(至少2个)
     * 2. 获取最新和次新的位置记录
     * 3. 计算时间差，确保时间间隔有效(>0.001秒)
     * 4. 通过位移差除以时间差计算各方向速度分量
     * 
     * @note 
     * - 速度计算结果存储在drone_velocity_成员变量中
     * - 如果数据不足或时间间隔过短，速度将被设置为零
     * - 该函数在每次接收到无人机位置数据时被调用
     * 
     * @warning 
     * - 需要确保position_history_有足够的数据点
     * - 时间差检查防止除零错误和噪声影响
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
     * 
     * @details 
     * 基于当前位置和速度向量，使用线性外推法预测无人机在指定时间后的位置。
     * 这种预测方法假设无人机在短时间内保持恒定速度运动。
     * 
     * 预测算法：
     * - 如果启用位置预测：predicted_pos = current_pos + velocity * prediction_time
     * - 如果禁用位置预测：返回当前位置
     * 
     * 应用场景：
     * - 计算跟随目标位置时考虑无人机的运动趋势
     * - 提前规划路径，减少跟随滞后
     * - 在快速运动场景中提高跟随精度
     * 
     * @note 
     * - 预测精度取决于预测时间跨度和无人机运动的规律性
     * - 适用于短时间预测，长时间预测可能存在较大误差
     * - 可通过position_prediction_参数控制是否启用此功能
     * 
     * @warning 
     * - 预测时间不宜过长，建议在0.1-1.0秒范围内
     * - 在无人机急转弯或变速时预测误差可能较大
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
     * 
     * @details 
     * 这是系统的核心数据输入函数，负责处理来自无人机的实时位置和姿态信息。
     * 该函数在每次接收到无人机里程计数据时被ROS2框架自动调用。
     * 
     * 主要处理流程：
     * 1. 更新无人机连接状态和消息时间戳
     * 2. 提取并存储无人机当前位置信息
     * 3. 维护位置历史记录队列（保留最近10个数据点）
     * 4. 调用速度计算函数更新运动状态
     * 5. 执行安全检查（高度限制等）
     * 
     * 数据流转：
     * 无人机传感器 → 里程计消息 → 本回调函数 → 位置存储 → 速度计算 → 跟随控制
     * 
     * @note 
     * - 位置历史记录用于速度计算和运动趋势分析
     * - 队列大小限制为10个记录，自动清理过旧数据
     * - 高度检查确保无人机在安全飞行范围内
     * 
     * @warning 
     * - 此函数频繁调用，避免执行耗时操作
     * - 线程安全：使用atomic变量更新连接状态
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
     * 
     * @details 
     * 处理地面机器人的位置和姿态信息，主要用于跟随控制和运动状态监控。
     * 该函数负责跟踪机器人的实时位置，并检测机器人的运动状态。
     * 
     * 功能职责：
     * 1. 更新机器人当前位置
     * 2. 检测机器人是否在移动（运动检测）
     * 3. 更新最后移动时间（用于卡住检测）
     * 4. 提供调试信息输出
     * 
     * 运动检测逻辑：
     * - 计算当前位置与上次记录位置的距离
     * - 如果移动距离超过5cm，认为机器人在运动
     * - 更新最后移动时间戳，用于后续的卡住检测
     * 
     * @note 
     * - 运动检测阈值为5cm，平衡了精度和噪声抗干扰性
     * - 调试信息仅在DEBUG级别输出，避免日志过多
     * 
     * @see check_goal_status() 使用最后移动时间进行卡住检测
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
     * 
     * @details 
     * 计算两个三维点在XY平面上的欧几里得距离，忽略Z轴（高度）分量。
     * 这在地面机器人导航中很有用，因为地面机器人主要关心水平面的距离。
     * 
     * 数学公式：
     * distance = √[(x₁-x₂)² + (y₁-y₂)²]
     * 
     * 应用场景：
     * - 计算机器人与无人机的跟随距离
     * - 检测机器人的移动距离
     * - 判断是否到达目标位置
     * - 确定是否需要更新导航目标
     * 
     * @note 
     * - 此函数仅计算二维平面距离，不考虑高度差
     * - 适用于地面机器人的导航距离计算
     * - 计算效率高，适合频繁调用
     * 
     * @example
     * ```cpp
     * geometry_msgs::msg::Point pos1, pos2;
     * pos1.x = 0.0; pos1.y = 0.0;
     * pos2.x = 3.0; pos2.y = 4.0;
     * double dist = calculate_distance(pos1, pos2); // 结果: 5.0米
     * ```
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
     * 
     * @details 
     * 根据无人机的当前位置（或预测位置）和配置的跟随参数，计算地面机器人
     * 应该到达的理想跟随位置。该函数是跟随算法的核心计算部分。
     * 
     * 计算流程：
     * 1. 获取无人机目标位置（当前位置或预测位置）
     * 2. 根据跟随距离和角度计算相对偏移
     * 3. 将相对偏移应用到无人机位置上
     * 4. 设置地面高度（Z=0）
     * 
     * 数学模型：
     * ```
     * follow_x = drone_x + follow_distance * cos(follow_angle)
     * follow_y = drone_y + follow_distance * sin(follow_angle)
     * follow_z = 0.0  // 地面机器人高度
     * ```
     * 
     * 跟随角度说明：
     * - 0°     : 机器人在无人机正前方
     * - 90°    : 机器人在无人机左侧
     * - 180°   : 机器人在无人机正后方（常用配置）
     * - 270°   : 机器人在无人机右侧
     * 
     * @note 
     * - 跟随距离和角度可通过参数配置
     * - 支持位置预测功能，提高跟随精度
     * - 计算结果的Z坐标始终为0（地面高度）
     * 
     * @see predict_drone_position() 位置预测功能
     * @see follow_distance_ 跟随距离配置
     * @see follow_angle_ 跟随角度配置
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
     * @details 
     * 这是系统的核心监控函数，负责检测各种异常状况并触发相应的恢复机制。
     * 该函数通过定时器定期调用，确保系统的可靠性和稳定性。
     * 
     * 监控范围：
     * 1. **导航目标超时检测**
     *    - 监控导航任务的执行时间
     *    - 防止导航系统卡死或无响应
     *    - 超时阈值可通过goal_timeout_参数配置
     * 
     * 2. **机器人卡住检测**
     *    - 监控机器人的运动状态
     *    - 检测长时间无移动的异常情况
     *    - 基于最后移动时间戳进行判断
     * 
     * 异常处理策略：
     * - 检测到异常时立即进入恢复模式
     * - 记录详细的异常信息用于调试
     * - 避免重复处理同一异常状况
     * 
     * @note 
     * - 仅在有目标执行时进行检查，避免误报
     * - 使用节流日志防止日志洪水
     * - 异常检测阈值可通过参数调整
     * 
     * @see enter_recovery_mode() 恢复模式处理
     * @see goal_timeout_ 目标超时配置
     * @see stuck_detection_time_ 卡住检测配置
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
            if (controller_recovery_enabled_)
            {
                RCLCPP_WARN(this->get_logger(),
                            "⏰ 导航目标超时 (%.1f秒)，进入控制器恢复模式", time_since_goal);
                enter_recovery_mode();
                return;
            }
            else
            {
                // 交由Nav2行为树处理低层恢复，这里仅提示
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "⏰ 导航目标超时 (%.1f秒)，已交由Nav2恢复处理", time_since_goal);
            }
        }

        // ==== 机器人卡住检测 ====
        double time_since_movement = (now - last_robot_movement_time_).seconds();
        if (time_since_movement > stuck_detection_time_)
        {
            if (controller_recovery_enabled_)
            {
                RCLCPP_WARN(this->get_logger(),
                            "🚫 检测到机器人卡住 (%.1f秒未移动)，进入控制器恢复模式",
                            time_since_movement);
                enter_recovery_mode();
            }
            else
            {
                // 交由Nav2行为树处理（例如BackUp/Spin/ClearCostmap）
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "🚫 检测到机器人可能卡住 (%.1f秒未移动)，交由Nav2恢复", time_since_movement);
            }
        }
    }

    /**
     * @brief 进入系统恢复模式
     * @details 
     * 当检测到系统异常时，启动恢复模式来恢复系统正常运行。
     * 恢复模式采用保守策略，优先保证系统安全，然后逐步恢复功能。
     * 
     * 恢复步骤：
     * 1. **状态保护**：防止重复进入恢复模式
     * 2. **导航清理**：取消当前导航目标和任务
     * 3. **状态重置**：重置相关状态变量和计数器
     * 4. **安全停止**：发送停止命令确保机器人安全
     * 5. **时间记录**：记录恢复开始时间用于后续退出判断
     * 
     * 设计原则：
     * - **安全第一**：优先确保机器人和环境安全
     * - **快速响应**：立即停止可能有问题的操作
     * - **状态清理**：彻底清理可能不一致的状态
     * - **可追踪性**：记录恢复过程用于调试分析
     * 
     * @note 
     * - 使用原子操作确保线程安全
     * - 恢复模式有时间限制，避免长期停留
     * - 重置重试计数器防止累积错误
     * 
     * @warning 
     * - 恢复过程中系统功能受限
     * - 频繁进入恢复模式可能表示配置问题
     * 
     * @see exit_recovery_mode() 退出恢复模式
     * @see recovery_wait_time_ 恢复等待时间配置
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

        // ==== 清理导航状态（高层）====
        if (current_goal_handle_)
        {
            // 取消当前导航目标
            nav_action_client_->async_cancel_goal(current_goal_handle_);
            current_goal_handle_.reset(); // 释放目标句柄
        }

        // ==== 重置系统状态 ====
        goal_in_progress_.store(false);  // 清除目标执行标志
        goal_retry_count_ = 0;           // 重置重试计数器

    // ==== 安全停止机器人（仅发布零速，避免发低层恢复动作）====
        geometry_msgs::msg::Twist stop_cmd; // 默认构造函数创建零速度命令
        cmd_vel_pub_->publish(stop_cmd);     // 发送停止命令

        RCLCPP_INFO(this->get_logger(), "✅ 恢复模式初始化完成，等待系统稳定...");
    }

    /**
     * @brief 退出系统恢复模式
     * @details 
     * 在系统稳定后退出恢复模式，恢复正常的跟随功能。
     * 该函数实现渐进式恢复策略，确保系统平稳过渡到正常状态。
     * 
     * 退出条件检查：
     * 1. **模式验证**：确认当前处于恢复模式
     * 2. **时间验证**：确保已经过足够的恢复等待时间
     * 3. **状态清理**：重置错误计数和状态标志
     * 
     * 恢复流程：
     * 1. 检查恢复模式状态和时间条件
     * 2. 重置所有错误计数器和失败标志
     * 3. 更新运动时间戳防止立即卡住检测
     * 4. 原子操作退出恢复模式
     * 5. 记录恢复完成信息
     * 
     * 时间管理：
     * - 恢复等待时间通过recovery_wait_time_配置
     * - 防止过早退出导致问题重现
     * - 确保系统有足够时间稳定
     * 
     * @note 
     * - 退出恢复模式后立即恢复正常跟随功能
     * - 重置所有错误状态避免历史错误影响
     * - 更新时间戳防止误触发异常检测
     * 
     * @see enter_recovery_mode() 进入恢复模式
     * @see recovery_wait_time_ 恢复等待时间配置
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
        if (recovery_time >= recovery_wait_time_)
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
                        "⏳ 恢复模式等待中... (%.1f/%.1f秒)", 
                        recovery_time, recovery_wait_time_);
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

        // Nav2 就绪后的预热时间，避免刚激活时目标被拒绝（典型成本图未就绪）
        {
            const double warmup_sec = 1.5; // 轻量预热
            auto now = this->get_clock()->now();
            if (nav2_ready_since_.nanoseconds() > 0)
            {
                double ready_elapsed = (now - nav2_ready_since_).seconds();
                if (ready_elapsed < warmup_sec)
                {
                    RCLCPP_DEBUG(this->get_logger(), "Nav2预热中，等待%.1fs (已%.1fs)", warmup_sec, ready_elapsed);
                    return;
                }
            }
        }

    // 使用try_lock避免死锁；此处仅进行高层目标管理（节流、目标更新），
    // 低层恢复（清图/后退/旋转）交由Nav2行为树处理，避免相互打断
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
            // 仅在无目标运行，或目标位置变化足够大时才考虑更新
            if (!goal_in_progress_.load() || goal_position_change > goal_update_threshold_)
            {
                need_new_goal = true;
            }
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
            double goal_position_change = calculate_distance(target_pos, last_goal_position_);
            if (goal_position_change > goal_update_threshold_)
            {
                need_new_goal = true;
                reason = "目标位置变化" + std::to_string(goal_position_change) + "m";
            }
        }

        // 时间间隔检查
        auto now = this->get_clock()->now();
        double time_since_last_goal = (now - last_goal_time_).seconds();
        double effective_min_time = urgent ? urgent_goal_interval_ : min_time_between_goals_;

        if (need_new_goal && time_since_last_goal >= effective_min_time)
        {
            // 不再频繁取消目标，只在必要时取消
            if (goal_in_progress_.load() && urgent)
            {
                // 仅当目标位置变化显著时才取消以减少抢占抖动
                if (goal_position_change > goal_update_threshold_)
                {
                    if (current_goal_handle_)
                    {
                        nav_action_client_->async_cancel_goal(current_goal_handle_);
                        last_self_cancel_time_ = this->get_clock()->now();
                        current_goal_handle_.reset();
                    }
                    goal_in_progress_.store(false);
                }
            }

            // 如果没有目标在执行，发送新目标
            if (!goal_in_progress_.load())
            {
                send_follow_goal(target_pos, urgent, reason);
            }
        }
    }

    /**
     * 发送跟随目标 - 改进版本
     */
    void send_follow_goal(const geometry_msgs::msg::Point &target_pos, bool urgent = false,
                          const std::string &reason = "")
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

        // 检查连续失败次数（仅在启用控制器恢复时介入）
        if (consecutive_failures_ >= max_retries_)
        {
            if (controller_recovery_enabled_)
            {
                RCLCPP_WARN(this->get_logger(),
                            "⚠️ 连续失败%d次，进入恢复模式", consecutive_failures_);
                enter_recovery_mode();
                return;
            }
            else
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                     "ℹ️ 连续失败累计 %d 次（已交由Nav2恢复），如机器人在前进可忽略此提示",
                                     consecutive_failures_);
                // 继续尝试发送目标，由Nav2行为树/恢复行为处理
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
                // 接受即视作系统状态在好转，衰减一次失败计数
                if (this->consecutive_failures_ > 0) {
                    this->consecutive_failures_--;
                }
                // 成功发送不重置consecutive_failures_，等结果回调确认
            }
        };

        // 反馈回调
        send_goal_options.feedback_callback =
            [this](auto, auto)
        {
            // 收到反馈说明导航正在进行
            this->last_robot_movement_time_ = this->get_clock()->now();
            // 有反馈也衰减一次失败计数，避免累计过快
            if (this->consecutive_failures_ > 0) {
                this->consecutive_failures_--;
            }
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
                break;
            case rclcpp_action::ResultCode::ABORTED:
                {
                    RCLCPP_DEBUG(this->get_logger(), "⚠️ 导航中止");
                    auto now = this->get_clock()->now();
                    double since_cancel = (now - last_self_cancel_time_).seconds();
                    double since_moved = (now - last_robot_movement_time_).seconds();
                    // 若刚由我们主动取消，或最近有移动反馈（说明在推进），则不计失败
                    if ((since_cancel > 0.0 && since_cancel < 1.2) || (since_moved >= 0.0 && since_moved < 1.5))
                    {
                        RCLCPP_DEBUG(this->get_logger(), "⏱️ 近期取消/运动中止(%.2fs/%.2fs)，忽略失败计数", since_cancel, since_moved);
                    }
                    else
                    {
                        this->consecutive_failures_++;
                    }
                }
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
                nav2_ready_since_ = this->get_clock()->now(); // 记录就绪开始，用于预热
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

/**
 * @brief 程序主入口函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出代码，0表示正常退出
 * 
 * @details 
 * 这是ROS2节点程序的标准入口点，负责初始化ROS2系统、创建节点实例、
 * 启动事件循环以及处理程序退出。
 * 
 * 程序执行流程：
 * 1. **ROS2初始化**：调用rclcpp::init()初始化ROS2运行时环境
 * 2. **节点创建**：实例化NavigationController节点
 * 3. **事件循环**：通过rclcpp::spin()启动ROS2事件处理循环
 * 4. **异常处理**：捕获并记录运行时异常
 * 5. **资源清理**：调用rclcpp::shutdown()清理ROS2资源
 * 
 * 错误处理：
 * - 使用try-catch块捕获标准异常
 * - 记录详细的错误信息用于调试
 * - 确保即使发生异常也能正确清理资源
 * 
 * 生命周期管理：
 * - 程序启动时自动初始化所有系统组件
 * - 运行过程中响应ROS2信号和回调
 * - 接收到终止信号时优雅退出
 * 
 * @note 
 * - 使用智能指针管理节点实例，自动内存管理
 * - spin()函数会阻塞直到节点被要求停止
 * - 支持Ctrl+C等标准终止信号
 * 
 * @example
 * 启动程序的命令行示例：
 * ```bash
 * ros2 run package_name navi_controller_version1
 * ```
 * 
 * @see NavigationController 主要的节点类实现
 */
int main(int argc, char **argv)
{
    // ==== ROS2系统初始化 ====
    // 初始化ROS2运行时环境，包括通信中间件、日志系统等
    rclcpp::init(argc, argv);

    try
    {
        // ==== 创建导航控制节点 ====
        // 使用智能指针创建节点实例，确保自动内存管理
        auto node = std::make_shared<NavigationController>();
        
        // ==== 启动ROS2事件循环 ====
        // 开始处理订阅回调、定时器、服务请求等事件
        // 此函数会阻塞直到节点被要求停止（如收到Ctrl+C信号）
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        // ==== 异常处理 ====
        // 捕获并记录运行时异常，包括初始化失败、通信错误等
        RCLCPP_ERROR(rclcpp::get_logger("main"), 
                     "❌ 程序异常退出: %s", e.what());
    }

    // ==== 资源清理 ====
    // 关闭ROS2系统，清理所有资源（通信连接、内存等）
    rclcpp::shutdown();
    
    return 0; // 正常退出
}