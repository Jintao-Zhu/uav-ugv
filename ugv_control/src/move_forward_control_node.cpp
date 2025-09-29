#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>

#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

using namespace std::chrono_literals;

class SimpleMoveForward : public rclcpp::Node
{
public:
    SimpleMoveForward(const rclcpp::NodeOptions& options) : Node("simple_move_forward", options),
                         is_moving_(false), 
                         completed_(false),
                         is_interrupted_(false)
    {
        // 修复：删除手动 declare_parameter，直接获取参数（避免重复声明）
        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
        if (!use_sim_time) {
            RCLCPP_FATAL(this->get_logger(), "❌ 必须启用仿真时间！请检查节点参数");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "✅ 小车控制节点启动 | 仿真时间: 已启用");

        // 发布到小车控制话题（/yahboomcar/cmd_vel）
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/yahboomcar/cmd_vel", 100);  // 队列长度100，避免指令丢失

        // 定时器（20ms周期 = 50Hz，适配Gazebo控制器）
        timer_ = this->create_wall_timer(20ms, std::bind(&SimpleMoveForward::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "⏱️  控制定时器启动 | 发布频率: 50Hz");

        // 控制参数
        linear_speed_ = 0.1;                             // 前进速度（m/s，适配小车性能）
        target_distance_ = 5.0;                          // 目标距离（米）
        travel_time_ = target_distance_ / linear_speed_; // 理论行驶时间（秒）

        RCLCPP_INFO(this->get_logger(), "⚙️  控制参数 | 速度: %.2f m/s | 目标: %.1f 米 | 预计时间: %.1f 秒",
                    linear_speed_, target_distance_, travel_time_);

        // 等待仿真时间初始化（避免启动时时间为0）
        wait_for_valid_time();
        // 开始移动（用仿真时间记录启动时刻）
        start_moving();
    }

    // 中断处理接口（响应 Ctrl+C）
    void set_interrupted()
    {
        is_interrupted_.store(true);
        RCLCPP_WARN(this->get_logger(), "⚠️  收到中断信号，准备停止小车");
    }

private:
    // 等待有效的仿真时间（避免启动时时间未同步）
    void wait_for_valid_time()
    {
        RCLCPP_INFO(this->get_logger(), "⌛ 等待仿真时间就绪...");
        int wait_count = 0;
        // 等待时间大于0，且最多等3秒（避免无限阻塞）
        while (rclcpp::ok() && this->now().seconds() < 0.1 && wait_count < 30) {
            std::this_thread::sleep_for(100ms);
            wait_count++;
        }

        if (this->now().seconds() < 0.1) {
            RCLCPP_FATAL(this->get_logger(), "❌ 超时未收到仿真时间！请确认Gazebo已启动");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "✅ 仿真时间就绪 | 当前时间: %.2f 秒", this->now().seconds());
    }

    // 启动移动（记录仿真时间下的启动时刻）
    void start_moving()
    {
        start_time_ = this->now();  // 关键：用节点的now()（已绑定仿真时间）
        is_moving_ = true;
        RCLCPP_INFO(this->get_logger(), "🚗 开始前进 | 启动时间: %.2f 秒", start_time_.seconds());
    }

    // 统一停止逻辑（确保可靠停止）
    void stop_robot()
    {
        if (is_moving_ || !completed_)
        {
            geometry_msgs::msg::Twist stop_msg;
            // 全零速度指令（停止）
            stop_msg.linear.x = 0.0; stop_msg.linear.y = 0.0; stop_msg.linear.z = 0.0;
            stop_msg.angular.x = 0.0; stop_msg.angular.y = 0.0; stop_msg.angular.z = 0.0;

            // 连续发布5次停止指令（应对Gazebo通信延迟）
            for (int i = 0; i < 5; ++i) {
                cmd_vel_publisher_->publish(stop_msg);
                std::this_thread::sleep_for(20ms);
            }

            is_moving_ = false;
            completed_ = true;
            RCLCPP_INFO(this->get_logger(), "🛑 小车已停止 | 当前仿真时间: %.2f 秒", this->now().seconds());
        }
    }

    // 定时器回调（核心控制逻辑）
    void timer_callback()
    {
        // 优先处理中断（最高优先级）
        if (is_interrupted_.load()) {
            stop_robot();
            timer_->cancel();       // 停止定时器，避免重复回调
            rclcpp::shutdown();     // 关闭节点
            return;
        }

        // 任务完成则退出
        if (completed_) {
            return;
        }

        if (is_moving_) {
            // 用仿真时间计算已行驶时间（核心修复：now()返回仿真时间）
            rclcpp::Time current_time = this->now();
            rclcpp::Duration elapsed_duration = current_time - start_time_;
            double elapsed_seconds = elapsed_duration.seconds();
            double distance_traveled = elapsed_seconds * linear_speed_;

            // 检查是否达到目标（基于仿真时间判断）
            if (elapsed_seconds >= travel_time_ || distance_traveled >= target_distance_) {
                stop_robot();
                RCLCPP_INFO(this->get_logger(), "🎉 任务完成！");
                RCLCPP_INFO(this->get_logger(), "📊 统计 | 实际行驶: %.2f 米 | 仿真耗时: %.1f 秒",
                            distance_traveled, elapsed_seconds);

                // 5秒后自动关闭节点
                shutdown_timer_ = this->create_wall_timer(5s, [this]() {
                    RCLCPP_INFO(this->get_logger(), "🔚 节点关闭");
                    rclcpp::shutdown(); 
                });
            } else {
                // 发布前进指令
                geometry_msgs::msg::Twist twist_msg;
                twist_msg.linear.x = linear_speed_;  // 仅需要x方向速度（前进）
                twist_msg.linear.y = 0.0;
                twist_msg.linear.z = 0.0;
                twist_msg.angular.x = 0.0;
                twist_msg.angular.y = 0.0;
                twist_msg.angular.z = 0.0;
                cmd_vel_publisher_->publish(twist_msg);

                // 每隔1秒打印进度（避免日志刷屏）
                static double last_print_time = 0.0;
                if (elapsed_seconds - last_print_time >= 1.0) {
                    RCLCPP_INFO(this->get_logger(), "📈 进度 | 已行驶: %.2f 米 | 已耗时: %.1f 秒",
                                distance_traveled, elapsed_seconds);
                    last_print_time = elapsed_seconds;
                }
            }
        }
    }

    // ROS 核心组件
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;  // 速度指令发布者
    rclcpp::TimerBase::SharedPtr timer_;                                          // 控制定时器
    rclcpp::TimerBase::SharedPtr shutdown_timer_;                                 // 退出延迟定时器

    // 状态变量
    bool is_moving_;                  // 是否正在运动
    bool completed_;                  // 是否完成任务
    std::atomic<bool> is_interrupted_;// 中断标记（线程安全）

    // 时间与参数
    rclcpp::Time start_time_;         // 启动时间（仿真时间）
    double linear_speed_;             // 前进速度（m/s）
    double target_distance_;          // 目标距离（米）
    double travel_time_;              // 理论行驶时间（秒）
};

// 全局节点指针（供信号处理器访问）
std::shared_ptr<SimpleMoveForward> g_node;

// 信号处理器（捕获 Ctrl+C）
void signal_handler(int signum)
{
    if (signum == SIGINT && g_node) {
        g_node->set_interrupted();
    }
}

int main(int argc, char **argv)
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    // 注册中断信号处理器
    signal(SIGINT, signal_handler);

    try {
        // 强制启用仿真时间（覆盖所有外部配置）
        rclcpp::NodeOptions node_options;
        node_options.append_parameter_override("use_sim_time", true)
                    .automatically_declare_parameters_from_overrides(true);
        
        // 创建节点（传入带仿真时间的参数）
        g_node = std::make_shared<SimpleMoveForward>(node_options);
        // 运行节点（阻塞）
        rclcpp::spin(g_node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "❌ 程序异常终止: %s", e.what());
    }

    // 资源清理
    rclcpp::shutdown();
    g_node.reset();
    return 0;
}
