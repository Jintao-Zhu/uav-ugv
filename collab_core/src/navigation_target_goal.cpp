#include <rclcpp/rclcpp.hpp>  //11.2此代码优化follow链，现优化cube，可以运行
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/utils.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <queue>
#include <mutex>
#include <chrono>
#include <cmath>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class NavigationTargetGoalNode : public rclcpp::Node
{
public:
    NavigationTargetGoalNode()
        : Node("navigation_target_goal_node"),
          current_mode_(Mode::FOLLOWING),
          goal_in_progress_(false),
          last_follow_send_time_(this->get_clock()->now()),
          last_valid_distance_(-1.0)
    {
        RCLCPP_INFO(this->get_logger(), "🚀 启动 navigation_target_goal 节点（双模式仲裁 + 持续导航）");

        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        RCLCPP_INFO(this->get_logger(), "✅ 已启用仿真时钟（use_sim_time=true）");

        this->declare_parameter<std::string>("cube_goal_topic", "/navigation/cube_goal");
        this->declare_parameter<std::string>("follow_goal_topic", "/navigation/follow_goal");
        this->declare_parameter<std::string>("nav2_action_name", "navigate_to_pose");
        this->declare_parameter<double>("cube_timeout", 5.0);
        this->declare_parameter<double>("follow_timeout", 2.0);
        this->declare_parameter<double>("follow_debounce", 0.4);
        this->declare_parameter<int>("queue_max_size", 10);
        this->declare_parameter<double>("check_interval", 0.5);

        cube_goal_topic_ = this->get_parameter("cube_goal_topic").as_string();
        follow_goal_topic_ = this->get_parameter("follow_goal_topic").as_string();
        nav2_action_name_ = this->get_parameter("nav2_action_name").as_string();
        cube_timeout_ = this->get_parameter("cube_timeout").as_double();
        follow_timeout_ = this->get_parameter("follow_timeout").as_double();
        follow_debounce_ = this->get_parameter("follow_debounce").as_double();
        queue_max_size_ = this->get_parameter("queue_max_size").as_int();
        check_interval_ = this->get_parameter("check_interval").as_double();

        cube_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            cube_goal_topic_, rclcpp::QoS(10),
            std::bind(&NavigationTargetGoalNode::cube_goal_callback, this, std::placeholders::_1));

        follow_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            follow_goal_topic_, rclcpp::QoS(10),
            std::bind(&NavigationTargetGoalNode::follow_goal_callback, this, std::placeholders::_1));

        nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav2_action_name_);
        if (!nav2_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_FATAL(this->get_logger(),
                         "❌ 无法连接 Nav2 服务 '%s'，请确认导航栈已启动。", nav2_action_name_.c_str());
            rclcpp::shutdown();
        }

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(check_interval_),
            std::bind(&NavigationTargetGoalNode::arbitrate, this));

        // 初始化新增成员
        last_follow_goal_receive_time_ = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(),
                    "✅ 初始化完成 | Cube超时: %.1fs | Follow超时: %.1fs | 队列上限: %d",
                    cube_timeout_, follow_timeout_, queue_max_size_);
    }

private:
    enum class Mode
    {
        FOLLOWING,
        NAVIGATING,
        IDLE
    };

    Mode current_mode_;
    bool goal_in_progress_;
    std::string cube_goal_topic_, follow_goal_topic_, nav2_action_name_;
    double cube_timeout_, follow_timeout_, follow_debounce_, check_interval_;
    int queue_max_size_;

    geometry_msgs::msg::PoseStamped latest_follow_goal_, last_follow_goal_, last_sent_goal_;
    rclcpp::Time cube_goal_time_, follow_goal_time_;
    std::queue<geometry_msgs::msg::PoseStamped> cube_queue_;
    std::mutex mtx_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cube_goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr follow_goal_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<GoalHandleNavigateToPose> current_goal_;

    rclcpp::Time last_follow_send_time_;
    const double follow_send_interval_ = 3.0;

    double last_valid_distance_; // 新增：记录上次有效距离
    // 新增：卡住目标检测
    rclcpp::Time last_follow_goal_receive_time_; // 上次接收跟随目标的时间
    double follow_goal_timeout_ = 2.0;           // 跟随目标超时阈值（2秒未更新视为异常）

    // ---------------- 工具函数 ----------------
    double distance_xy(const geometry_msgs::msg::Pose &a, const geometry_msgs::msg::Pose &b)
    {
        return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
    }

    bool cube_available()
    {
        return !cube_queue_.empty();
    }

    bool follow_available()
    {
        if (follow_goal_time_.nanoseconds() == 0)
            return false;
        return (this->get_clock()->now() - follow_goal_time_).seconds() <= follow_timeout_;
    }

    // ---------------- 回调 ----------------
    void cube_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        cube_goal_time_ = this->get_clock()->now();

        if (cube_queue_.size() >= (size_t)queue_max_size_)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ Cube队列已满，丢弃最旧目标");
            cube_queue_.pop();
        }
        cube_queue_.push(*msg);
        RCLCPP_INFO(this->get_logger(),
                    "📥 收到Cube目标 (%.2f, %.2f) | 队列长度: %ld",
                    msg->pose.position.x, msg->pose.position.y, cube_queue_.size());
    }

    void follow_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        latest_follow_goal_ = *msg;
        follow_goal_time_ = this->get_clock()->now();
        last_follow_goal_receive_time_ = this->get_clock()->now(); // 新增：记录接收时间
    }

    // ---------------- 仲裁 ----------------
    void arbitrate()
    {
        std::lock_guard<std::mutex> lock(mtx_);

        bool cube_valid = cube_available();
        bool follow_valid = follow_available();

        // 1. 最高优先级：Cube队列非空时，强制执行，直至队列清空
        if (cube_valid)
        {
            // 切换到导航模式（若当前不是）
            if (current_mode_ != Mode::NAVIGATING)
            {
                RCLCPP_WARN(this->get_logger(), "🔄 检测到Cube目标，切换 FOLLOW → NAVIGATE 模式");
                cancel_current_goal(); // 取消当前跟随目标，优先执行Cube
                current_mode_ = Mode::NAVIGATING;
            }

            // 若当前无导航任务，立即执行下一个Cube目标
            if (!goal_in_progress_)
            {
                process_next_cube_goal();
            }
            return; // Cube优先，本次仲裁结束，不处理后续逻辑
        }

        // 2. 第二优先级：FOLLOW 目标
        if (follow_valid)
        {
            if (current_mode_ != Mode::FOLLOWING)
            {
                RCLCPP_INFO(this->get_logger(), "🔄 Cube任务完成，切换 NAVIGATE → FOLLOW 模式");
                current_mode_ = Mode::FOLLOWING;
                last_follow_goal_ = latest_follow_goal_;
                last_sent_goal_ = latest_follow_goal_;
                last_follow_send_time_ = this->get_clock()->now();
                send_goal(latest_follow_goal_, "FOLLOW");
                return;
            }

            // [新逻辑] 计算跟随目标的变化
            double time_since_last_send = (this->get_clock()->now() - last_follow_send_time_).seconds();
            // 检查新收到的目标和上一个已发送的目标
            double goal_change_m = distance_xy(latest_follow_goal_.pose, last_sent_goal_.pose);

            // A. NAV2 空闲：立即发送新目标（如果目标有变化）
            if (!goal_in_progress_)
            {
                // 使用一个小阈值(10cm)防止重复发送相同目标
                if (goal_change_m > 0.1)
                {
                    RCLCPP_INFO(this->get_logger(), "NAV: 空闲，发送新跟随目标 (变化: %.2fm)", goal_change_m);
                    send_goal(latest_follow_goal_, "FOLLOW");
                    last_follow_send_time_ = this->get_clock()->now();
                }
            }
            // B. NAV2 忙碌：检查是否达到3秒更新周期
            else if (time_since_last_send >= follow_send_interval_)
            {
                // 仅当目标有显著变化时才打断
                if (goal_change_m > 0.1)
                {
                    RCLCPP_WARN(this->get_logger(), "NAV: 3s更新周期到达，取消并准备发送新目标 (变化: %.2fm)", goal_change_m);
                    // 异步取消当前目标
                    cancel_current_goal();
                    // 在下一次 arbitrate 周期（当 goal_in_progress_ 变为 false 时），
                    // 上面的 (A) 逻辑会自动发送最新的 goal。
                }
            }
            return; // Follow 逻辑结束
        }

        // 在“无任何目标”分支前新增：正在执行Cube导航时，不处理无目标逻辑
        if (current_mode_ == Mode::NAVIGATING && goal_in_progress_)
        {
            return; // 跳过后续无目标判定，确保Cube导航持续执行
        }
        // 3. 无任何目标时，进入IDLE模式（仅当无正在执行的任务时）
        if (current_mode_ != Mode::IDLE)
        {
            // 正在执行Cube导航时，不取消，等待任务完成
            if (!goal_in_progress_)
            {
                RCLCPP_WARN(this->get_logger(), "⚠️ 无有效目标，进入 IDLE 模式");
                current_mode_ = Mode::IDLE;
            }
            else
            {
                // 保留日志，确认任务在执行中
                RCLCPP_INFO(this->get_logger(), "⚙️ 正在执行Cube导航，完成后进入 IDLE 模式");
            }
        }

    }

    // ---------------- 发送目标 ----------------
    void send_goal(const geometry_msgs::msg::PoseStamped &pose, const std::string &type)
    {
        goal_in_progress_ = true;
        last_valid_distance_ = -1.0; // reset

        NavigateToPose::Goal goal;
        goal.pose = pose;
        goal.pose.header.stamp = this->get_clock()->now();

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        options.goal_response_callback = [this, type](auto handle)
        {
            if (!handle)
            {
                RCLCPP_ERROR(this->get_logger(), "❌ Nav2 拒绝目标请求 (%s)", type.c_str());
                goal_in_progress_ = false;
            }
            else
            {
                current_goal_ = handle;
                RCLCPP_INFO(this->get_logger(), "✅ Nav2 接受目标 (%s)，开始导航...", type.c_str());
            }
        };

        options.result_callback = [this, type](const GoalHandleNavigateToPose::WrappedResult &result)
        {
            goal_in_progress_ = false;
            current_goal_.reset();

            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "🎯 [%s] 导航成功！", type.c_str());
                if (type == "CUBE")
                    process_next_cube_goal();
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_WARN(this->get_logger(), "⚠️ [%s] 导航被中止", type.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "⏹️ [%s] 导航被取消", type.c_str());
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "❌ [%s] 未知导航结果", type.c_str());
                break;
            }
        };

        // ✅ 改进版反馈滤波
        options.feedback_callback = [this, type](auto, auto feedback)
        {
            double dist = feedback->distance_remaining;
            if (dist < 0.05 && goal_in_progress_)
            {
                if (last_valid_distance_ > 0.05)
                    dist = last_valid_distance_;
            }
            else
            {
                last_valid_distance_ = dist;
            }

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "📊 [%s] 导航中 | 剩余距离: %.2f 米", type.c_str(), dist);
        };

        nav2_client_->async_send_goal(goal, options);
        last_sent_goal_ = pose;

        RCLCPP_INFO(this->get_logger(),
                    "📤 发送%s目标 (%.2f, %.2f)",
                    type.c_str(), pose.pose.position.x, pose.pose.position.y);
    }

    // ---------------- 处理 Cube 队列 ----------------
    void process_next_cube_goal()
    {
        if (goal_in_progress_)
        {
            RCLCPP_DEBUG(this->get_logger(), "⚙️ 上一个Cube任务尚未完成，等待...");
            return;
        }

        if (cube_queue_.empty())
        {
            RCLCPP_INFO(this->get_logger(), "📭 Cube目标队列已空 → 恢复FOLLOW模式");
            current_mode_ = Mode::FOLLOWING;
            return;
        }

        auto goal = cube_queue_.front();
        cube_queue_.pop();
        RCLCPP_INFO(this->get_logger(),
                    "🎯 执行Cube目标 | 队列剩余: %ld",
                    cube_queue_.size());

        send_goal(goal, "CUBE");
    }
    void cancel_current_goal()
    {
        if (goal_in_progress_ && current_goal_)
        {
            RCLCPP_WARN(this->get_logger(), "⏹️ 取消当前导航任务");
            // 仅发起取消请求，不处理回调
            nav2_client_->async_cancel_goal(current_goal_);
            // 临时标记状态为false，避免重复取消（最终以result_callback为准）
            goal_in_progress_ = false;
            current_goal_.reset();
        }
    }



};

// ---------------- 主函数 ----------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationTargetGoalNode>());
    rclcpp::shutdown();
    return 0;
}
