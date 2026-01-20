#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <vector>
#include <string>
#include <random>
#include <sstream>

/*1.19 模拟无人机发现目标点：仅发布任务通知，不调用任何服务（修正RL决策冲突）*/
using namespace std::chrono_literals;
using StringMsg = std_msgs::msg::String;

// 定义航点数据结构
struct WaypointInfo
{
    std::string cube_name;
    std::string waypoint;
    std::string x;
    std::string y;
    std::string z;
    int priority;
};

class AutoSendWaypointsNode : public rclcpp::Node
{
public:
    AutoSendWaypointsNode() : Node("auto_send_waypoints_node"), current_waypoint_idx_(0)
    {
        // 仅创建/task_monitor/start发布者（删除服务客户端）
        task_monitor_pub_ = this->create_publisher<StringMsg>("/task_monitor/start", 10);

        // 初始化航点列表（保持不变）
        waypoints_ = {
            {"red_cube_n14", "n14", "80.84", "-28.52", "0.5", 0},
            {"red_cube_n13", "n13", "84.44", "-4.94", "0.5", 0},
            {"red_cube_n23", "n23", "182.80", "-42.30", "0.5", 0},
            {"red_cube_s08", "s08", "96.61", "-50.50", "0.5", 0},
            {"red_cube_s10", "s10", "122.10", "-46.68", "0.5", 0},
            {"red_cube_west_koi_pond", "west_koi_pond", "34.32", "-10.13", "0.5", 0},
            {"red_cube_n08", "n08", "59.61", "-7.42", "0.5", 0},
            {"red_cube_junction_south_west", "junction_south_west", "84.56", "-38.81", "0.5", 0}};

        // 创建定时器（每隔20秒发布一个任务通知）
        timer_ = this->create_wall_timer(
            20s, 
            std::bind(&AutoSendWaypointsNode::publish_task_notification, this));

        RCLCPP_INFO(this->get_logger(), "✅ 自动任务通知节点已启动！");
        RCLCPP_INFO(this->get_logger(), "📝 每隔20秒发布一个任务到/task_monitor/start（等待RL决策）");
        RCLCPP_INFO(this->get_logger(), "📊 共%d个任务待发布", (int)waypoints_.size());
    }

private:
    // 生成带red_cube_前缀的唯一任务ID
    std::string generate_task_id(const std::string &cube_name)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(100000, 999999);
        return cube_name + "_" + std::to_string(dis(gen)); // 格式：red_cube_n14_123456
    }

    // 修正后的核心逻辑：仅发布任务通知，不调用任何服务
    void publish_task_notification()
    {
        // 1. 检查是否所有任务发布完毕
        if (current_waypoint_idx_ >= waypoints_.size())
        {
            RCLCPP_INFO(this->get_logger(), "🎉 所有任务通知已发布完成！");
            timer_->cancel(); 
            return;
        }

        // 2. 获取当前任务信息
        WaypointInfo &info = waypoints_[current_waypoint_idx_];
        std::string task_id = generate_task_id(info.cube_name);

        // 3. 仅发布/task_monitor/start消息（核心：只通知，不执行）
        StringMsg monitor_msg;
        monitor_msg.data = task_id + "," + info.waypoint;
        task_monitor_pub_->publish(monitor_msg);

        // 4. 打印日志（明确是“通知”，不是“发送执行任务”）
        RCLCPP_INFO(
            this->get_logger(),
            "📤 发布第%d个任务通知：task_id=%s, 目标航点=%s (等待RL节点决策派车)",
            current_waypoint_idx_ + 1,
            task_id.c_str(),
            info.waypoint.c_str());

        // 5. 索引自增
        current_waypoint_idx_++;
    }

    // 成员变量（删除服务客户端相关）
    rclcpp::TimerBase::SharedPtr timer_;              
    std::vector<WaypointInfo> waypoints_;             
    size_t current_waypoint_idx_;                     
    rclcpp::Publisher<StringMsg>::SharedPtr task_monitor_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoSendWaypointsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
