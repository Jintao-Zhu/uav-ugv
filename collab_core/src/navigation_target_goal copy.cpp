#include <rclcpp/rclcpp.hpp> 
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
/*此代码新增判断：若当前处于 Cube 导航模式且任务正在执行，直接忽略跟随目标，避免打断 Cube 导航。*/
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class NavigationTargetGoalNode : public rclcpp::Node
{
public:
    NavigationTargetGoalNode()
        : Node("navigation_target_goal_node"),
          current_mode_(Mode::FOLLOWING),
          goal_in_progress_(false)
    {
        RCLCPP_INFO(this->get_logger(), "🚀 启动 navigation_target_goal 节点（双模式仲裁 + 持续导航）");

        // 启用仿真时钟
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        RCLCPP_INFO(this->get_logger(), "✅ 已启用仿真时钟（use_sim_time=true）");

        // 参数声明
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

        // 订阅器
        cube_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            cube_goal_topic_, rclcpp::QoS(10),
            std::bind(&NavigationTargetGoalNode::cube_goal_callback, this, std::placeholders::_1));

        follow_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            follow_goal_topic_, rclcpp::QoS(10),
            std::bind(&NavigationTargetGoalNode::follow_goal_callback, this, std::placeholders::_1));

        // Nav2 Action Client
        nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav2_action_name_);
        if (!nav2_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_FATAL(this->get_logger(),
                         "❌ 无法连接 Nav2 服务 '%s'，请确认导航栈已启动。", nav2_action_name_.c_str());
            rclcpp::shutdown();
        }

        // 定时器：仲裁逻辑
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(check_interval_),
            std::bind(&NavigationTargetGoalNode::arbitrate, this));

        RCLCPP_INFO(this->get_logger(),
                    "✅ 初始化完成 | Cube超时: %.1fs | Follow超时: %.1fs | 队列上限: %d",
                    cube_timeout_, follow_timeout_, queue_max_size_);
    }

private:
    enum class Mode
    {
        FOLLOWING,   // 跟随模式
        NAVIGATING,  // 导航到Cube目标模式
        IDLE         // 空闲模式
    };

    // ========================== 成员变量 ==========================
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

    // ========================== 工具函数 ==========================
    // 计算两点在XY平面的距离
    double distance_xy(const geometry_msgs::msg::Pose &a, const geometry_msgs::msg::Pose &b)
    {
        return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
    }

    // 判断Cube目标是否有效
    bool cube_available()
    {
        // 若当前正在执行Cube导航，认为目标仍有效
        if (current_mode_ == Mode::NAVIGATING && goal_in_progress_)
            return true;

        if (cube_goal_time_.nanoseconds() == 0)
            return false;

        // 检查是否在超时时间内
        return (this->get_clock()->now() - cube_goal_time_).seconds() <= cube_timeout_;
    }

    // 判断跟随目标是否有效
    bool follow_available()
    {
        if (follow_goal_time_.nanoseconds() == 0)
            return false;
        return (this->get_clock()->now() - follow_goal_time_).seconds() <= follow_timeout_;
    }

    // ========================== 回调函数 ==========================
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
    }

    // ========================== 核心仲裁逻辑 ==========================
    void arbitrate()
    {
        std::lock_guard<std::mutex> lock(mtx_);

        bool cube_valid = cube_available();
        bool follow_valid = follow_available();

        // 优先级 1：Cube 导航（最高优先级，任何时候都可以打断其他模式）
        if (cube_valid)
        {
            if (current_mode_ != Mode::NAVIGATING)
            {
                RCLCPP_WARN(this->get_logger(), "🔄 检测到Cube目标，切换 FOLLOW → NAVIGATE 模式");
                cancel_current_goal();  // 取消当前跟随任务
                current_mode_ = Mode::NAVIGATING;
            }

            // 如果没有正在执行的目标，处理下一个Cube目标
            if (!goal_in_progress_)
                process_next_cube_goal();

            return;
        }

        // 优先级 2：无人机跟随（仅在非Cube导航模式下处理）
        if (follow_valid)
        {
            // 若当前正在执行Cube导航，忽略跟随目标（核心修改点）
            if (current_mode_ == Mode::NAVIGATING && goal_in_progress_)
            {
                RCLCPP_DEBUG(this->get_logger(), "⏸️ 正在执行Cube导航，暂不处理跟随目标");
                return;
            }

            // 切换到跟随模式（如果当前不是）
            if (current_mode_ != Mode::FOLLOWING)
            {
                RCLCPP_INFO(this->get_logger(), "🔄 Cube任务完成，切换 NAVIGATE → FOLLOW 模式");
                current_mode_ = Mode::FOLLOWING;
            }

            // 防抖处理：目标位置变化超过阈值才更新
            double dist = distance_xy(latest_follow_goal_.pose, last_follow_goal_.pose);
            if (dist < follow_debounce_)
                return;

            // 发布新的跟随目标（允许覆盖旧的跟随目标）
            last_follow_goal_ = latest_follow_goal_;
            cancel_current_goal();  // 取消旧的跟随任务
            send_goal(latest_follow_goal_, "FOLLOW");
            return;
        }

        // 优先级 3：无有效目标
        if (current_mode_ != Mode::IDLE)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ 无有效目标，进入 IDLE 模式");
            cancel_current_goal();
            current_mode_ = Mode::IDLE;
        }
    }

    // ========================== 发送目标到Nav2 ==========================
    void send_goal(const geometry_msgs::msg::PoseStamped &pose, const std::string &type)
    {
        goal_in_progress_ = true;

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
                    process_next_cube_goal();  // Cube目标完成后处理下一个
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

        options.feedback_callback = [this](auto, auto feedback)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "📊 导航中 | 剩余距离: %.2f 米", feedback->distance_remaining);
        };

        nav2_client_->async_send_goal(goal, options);
        last_sent_goal_ = pose;

        RCLCPP_INFO(this->get_logger(),
                    "📤 发送%s目标 (%.2f, %.2f)",
                    type.c_str(), pose.pose.position.x, pose.pose.position.y);
    }

    // ========================== 处理Cube目标队列 ==========================
    void process_next_cube_goal()
    {
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

    // ========================== 取消当前任务 ==========================
    void cancel_current_goal()
    {
        if (goal_in_progress_ && current_goal_)
        {
            RCLCPP_WARN(this->get_logger(), "⏹️ 取消当前导航任务");
            nav2_client_->async_cancel_goal(current_goal_);
        }
        goal_in_progress_ = false;
        current_goal_.reset();
    }
};

// ========================== 主函数 ==========================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationTargetGoalNode>());
    rclcpp::shutdown();
    return 0;
}
