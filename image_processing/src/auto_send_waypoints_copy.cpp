#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp> // 新增：发布任务到全局队列
#include <rmf_custom_tasks/srv/single_nav_task.hpp>
#include <chrono>
#include <vector>
#include <string>
/*19改向全局队列提交任务。模拟无人机发现目标点：每隔指定时间，自动调用 /submit_single_nav_task 服务，逐个发送指定航点给 deliveryRobot 车队，模拟无人车发现目标点并执行任务的过程。*/
using namespace std::chrono_literals;
using SingleNavTask = rmf_custom_tasks::srv::SingleNavTask;
using StringMsg = std_msgs::msg::String; // 新增：队列消息类型

// 定义航点数据结构：立方体名称、目标航点（截取cube后缀）、x/y/z（备用）、优先级
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
        // ========== 修改：不再调用服务，改为发布到全局队列 ==========
        // 原：client_ = this->create_client<SingleNavTask>("/submit_single_nav_task");
        // 新：创建发布器，发布任务到全局队列
        queue_pub_ = this->create_publisher<StringMsg>("/task_queue/submit", 10);

        // 2. 初始化航点列表（从cube名称提取waypoint，比如red_cube_s10 → s10）
        waypoints_ = {
            {"red_cube_n14", "n14", "80.84", "-28.52", "0.5", 0},
            {"red_cube_n13", "n13", "84.44", "-4.94", "0.5", 0},
            {"red_cube_n23", "n23", "182.80", "-42.30", "0.5", 0},
            {"red_cube_west_koi_pond", "west_koi_pond", "34.32", "-10.13", "0.5", 0},
            /*{"red_cube_s08", "s08", "96.61", "-51.94", "0.5", 0},
            {"red_cube_s10", "s10", "122.10", "-46.68", "0.5", 0},
            {"red_cube_s11", "s11", "152.73", "-42.86", "0.5", 0},
            {"red_cube_junction_south_west", "junction_south_west", "84.56", "-38.81", "0.5", 0}*/
        };

        // 3. 创建定时器（每隔10秒发送一个航点，可修改时间）
        timer_ = this->create_wall_timer(
            15s, // 发送间隔：10秒，可改为5s/15s等
            std::bind(&AutoSendWaypointsNode::send_waypoint_callback, this));

        RCLCPP_INFO(this->get_logger(), "自动发送航点节点已启动！每隔15秒发送一个航点到全局队列");
        RCLCPP_INFO(this->get_logger(), "共%d个航点待发送", (int)waypoints_.size());
    }

private:
    // 定时器回调：发送当前航点到全局队列
    void send_waypoint_callback()
    {
        // 1. 检查是否所有航点发送完毕
        if (current_waypoint_idx_ >= waypoints_.size())
        {
            RCLCPP_INFO(this->get_logger(), "所有航点已发送完成！");
            timer_->cancel(); // 停止定时器
            return;
        }

        // 2. 构造队列消息（格式：task_id,target_waypoint,fleet_name,priority）
        WaypointInfo &info = waypoints_[current_waypoint_idx_];
        // 生成唯一task_id
        std::string task_id = "auto_nav_" + std::to_string(rclcpp::Clock().now().nanoseconds() / 1000000);
        StringMsg queue_msg;
        queue_msg.data = task_id + "," + info.waypoint + ",deliveryRobot," + std::to_string(info.priority);

        // 3. 发布到全局队列
        queue_pub_->publish(queue_msg);

        // 4. 打印发送日志
        RCLCPP_INFO(
            this->get_logger(),
            "发送第%d个航点到全局队列：cube=%s, waypoint=%s, fleet=deliveryRobot, priority=%d, task_id=%s",
            current_waypoint_idx_ + 1,
            info.cube_name.c_str(),
            info.waypoint.c_str(),
            info.priority,
            task_id.c_str());

        // 5. 索引自增，准备下一个航点
        current_waypoint_idx_++;
    }

    // 成员变量（修改：替换服务客户端为队列发布器）
    // rclcpp::Client<SingleNavTask>::SharedPtr client_;  // 移除
    rclcpp::Publisher<StringMsg>::SharedPtr queue_pub_; // 新增
    rclcpp::TimerBase::SharedPtr timer_;                // 定时器
    std::vector<WaypointInfo> waypoints_;               // 航点列表
    size_t current_waypoint_idx_;                       // 当前发送的航点索引
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoSendWaypointsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
