#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mutex>
#include <deque>
#include <cmath>
#include <chrono>

class NavigationToRedCubeNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavigationToRedCubeNode() : Node("navigation_to_red_cube_node"), navigating_(false)
    {
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        RCLCPP_INFO(this->get_logger(), "启动导航节点：navigation_to_red_cube_node...");

        // 参数
        this->declare_parameter<std::string>("target_topic", "/red_cube/target_position");
        this->declare_parameter<std::string>("base_frame_id", "map");
        this->declare_parameter<std::string>("action_name", "navigate_to_pose");
        this->declare_parameter<int>("max_queue_size", 10);
        this->declare_parameter<double>("min_goal_distance", 0.5); // m（小于此值不入队）

        target_topic_ = this->get_parameter("target_topic").as_string();
        base_frame_id_ = this->get_parameter("base_frame_id").as_string();
        action_name_ = this->get_parameter("action_name").as_string();
        max_queue_size_ = this->get_parameter("max_queue_size").as_int();
        min_goal_distance_ = this->get_parameter("min_goal_distance").as_double();

        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);

        RCLCPP_INFO(this->get_logger(), "⏳ 等待 Nav2 动作服务器...");
        while (!nav_client_->wait_for_action_server(std::chrono::seconds(2)))
        {
            RCLCPP_WARN(this->get_logger(), "Nav2 未连接，继续等待...");
        }
        RCLCPP_INFO(this->get_logger(), "✅ Nav2 动作服务器已连接");

        // 订阅 tracker 的目标点
        target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            target_topic_,
            10,
            std::bind(&NavigationToRedCubeNode::target_callback, this, std::placeholders::_1));

        // 定时器：定期检查是否空闲可发送下一个目标
        check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&NavigationToRedCubeNode::check_and_send_next_goal, this));

        RCLCPP_INFO(this->get_logger(), "📡 初始化完成，等待目标点消息...");
    }

private:
    // 动作客户端与订阅器
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
    rclcpp::TimerBase::SharedPtr check_timer_;

    // 参数
    std::string target_topic_, base_frame_id_, action_name_;
    int max_queue_size_;
    double min_goal_distance_;

    // 队列 & 状态
    std::deque<geometry_msgs::msg::PointStamped> goal_queue_;
    std::mutex queue_mutex_;
    bool navigating_;
    geometry_msgs::msg::Point last_goal_; // 防抖用

    // 计算两点距离
    double distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
    {
        return std::hypot(a.x - b.x, a.y - b.y);
    }

    // 收到目标点 → 入队
    void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);

        // 防抖：距离上个点太近则忽略
        if (!goal_queue_.empty() || navigating_)
        {
            if (distance(msg->point, last_goal_) < min_goal_distance_)
            {
                RCLCPP_DEBUG(this->get_logger(),
                             "忽略重复目标: 距离 %.2f < 阈值 %.2f",
                             distance(msg->point, last_goal_), min_goal_distance_);
                return;
            }
        }

        if ((int)goal_queue_.size() >= max_queue_size_)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ 队列已满（%d/%d），丢弃新目标",
                        (int)goal_queue_.size(), max_queue_size_);
            return;
        }

        goal_queue_.push_back(*msg);
        last_goal_ = msg->point;

        RCLCPP_INFO(this->get_logger(),
                    "📍 新目标加入队列 (%.2f, %.2f, %.2f)，当前队列大小：%zu",
                    msg->point.x, msg->point.y, msg->point.z, goal_queue_.size());
    }

    // 定期检查是否可以执行下一个目标
    void check_and_send_next_goal()
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (navigating_ || goal_queue_.empty())
        {
            return;
        }

        auto next_goal = goal_queue_.front();
        goal_queue_.pop_front();
        navigating_ = true;

        RCLCPP_INFO(this->get_logger(), "🚀 开始执行队列目标: (%.2f, %.2f, %.2f)",
                    next_goal.point.x, next_goal.point.y, next_goal.point.z);

        send_navigation_goal(next_goal);
    }

    // 发送导航目标
    void send_navigation_goal(const geometry_msgs::msg::PointStamped &msg)
    {
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = base_frame_id_;
        goal_msg.pose.header.stamp = this->get_clock()->now();
        goal_msg.pose.pose.position = msg.point;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        goal_msg.pose.pose.orientation = tf2::toMsg(q);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](auto goal_handle)
        {
            if (!goal_handle)
                RCLCPP_ERROR(this->get_logger(), "❌ Nav2 拒绝目标请求");
            else
                RCLCPP_INFO(this->get_logger(), "✅ 目标被 Nav2 接受");
        };

        send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult &result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "🎯 导航成功到达目标点");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_WARN(this->get_logger(), "⚠️ 导航被中止");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "⏹️ 导航被取消");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "❌ 未知导航结果");
                break;
            }

            std::lock_guard<std::mutex> lock(queue_mutex_);
            navigating_ = false;

            if (!goal_queue_.empty())
            {
                RCLCPP_INFO(this->get_logger(), "⏭️ 队列中还有 %zu 个目标，准备执行下一个",
                            goal_queue_.size());
            }
        };

        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }
};

// -------------------- 主函数 --------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationToRedCubeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
