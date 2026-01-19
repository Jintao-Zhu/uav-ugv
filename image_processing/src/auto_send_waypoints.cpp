#include <rclcpp/rclcpp.hpp>
#include <rmf_custom_tasks_self/srv/single_nav_task.hpp>
#include <std_msgs/msg/string.hpp>  // 新增：发布/task_monitor/start话题
#include <chrono>
#include <vector>
#include <string>
#include <random>  // 新增：生成唯一ID
#include <sstream> // 新增：拼接字符串
/*1.19 模拟无人机发现目标点：修复task_id前缀为red_cube_，匹配RL/监控节点过滤逻辑*/
using namespace std::chrono_literals;
using SingleNavTask = rmf_custom_tasks_self::srv::SingleNavTask;
using StringMsg = std_msgs::msg::String;  // 新增：任务监控话题消息类型

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
        // 1. 创建服务客户端（调用/submit_single_nav_task）
        client_ = this->create_client<SingleNavTask>("/submit_single_nav_task");

        // 新增：创建/task_monitor/start发布者（和RL/监控节点对齐）
        task_monitor_pub_ = this->create_publisher<StringMsg>("/task_monitor/start", 10);

        // 2. 初始化航点列表（保持不变）
        waypoints_ = {
            {"red_cube_n14", "n14", "80.84", "-28.52", "0.5", 0},
            {"red_cube_n13", "n13", "84.44", "-4.94", "0.5", 0},
            {"red_cube_n23", "n23", "182.80", "-42.30", "0.5", 0},
            {"red_cube_s08", "s08", "96.61", "-50.50", "0.5", 0},
            {"red_cube_s10", "s10", "122.10", "-46.68", "0.5", 0},
            {"red_cube_west_koi_pond", "west_koi_pond", "34.32", "-10.13", "0.5", 0},
            {"red_cube_n08", "n08", "59.61", "-7.42", "0.5", 0},
            {"red_cube_junction_south_west", "junction_south_west", "84.56", "-38.81", "0.5", 0}};

        // 3. 创建定时器（每隔20秒发送一个航点，注释修正）
        timer_ = this->create_wall_timer(
            20s, 
            std::bind(&AutoSendWaypointsNode::send_waypoint_callback, this));

        RCLCPP_INFO(this->get_logger(), "自动发送航点节点已启动！每隔20秒发送一个航点给deliveryRobot");
        RCLCPP_INFO(this->get_logger(), "共%d个航点待发送", (int)waypoints_.size());
    }

private:
    // 新增：生成带red_cube_前缀的唯一任务ID
    std::string generate_task_id(const std::string &cube_name)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(100000, 999999);
        return cube_name + "_" + std::to_string(dis(gen)); // 格式：red_cube_n14_123456
    }

    // 定时器回调：发送当前航点（核心修改）
    void send_waypoint_callback()
    {
        // 1. 检查是否所有航点发送完毕
        if (current_waypoint_idx_ >= waypoints_.size())
        {
            RCLCPP_INFO(this->get_logger(), "所有航点已发送完成！");
            timer_->cancel(); 
            return;
        }

        // 2. 检查服务是否可用
        if (!client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "服务/submit_single_nav_task不可用，等待重试...");
            return;
        }

        // 3. 构造服务请求
        auto request = std::make_shared<SingleNavTask::Request>();
        WaypointInfo &info = waypoints_[current_waypoint_idx_];
        request->target_waypoint = info.waypoint;
        request->fleet_name = "deliveryRobot"; 
        request->priority = info.priority;
        // 注意：不指定robot_name，让RMF原生调度选车

        // 4. 生成red_cube_前缀的任务ID（核心修复）
        std::string task_id = generate_task_id(info.cube_name);
        std::string current_waypoint = info.waypoint; // 临时变量，供lambda捕获

        // 5. 发送异步请求（用lambda替代bind，修复参数不匹配问题）
        auto future = client_->async_send_request(
            request,
            [this, task_id, current_waypoint](rclcpp::Client<SingleNavTask>::SharedFuture future) {
                this->response_callback(future, task_id, current_waypoint);
            });

        // 6. 提前发布/task_monitor/start消息（让监控节点先收到任务）
        StringMsg monitor_msg;
        monitor_msg.data = task_id + "," + info.waypoint;
        task_monitor_pub_->publish(monitor_msg);
        RCLCPP_INFO(this->get_logger(), "📤 发布监控任务: %s -> %s", task_id.c_str(), info.waypoint.c_str());

        // 7. 打印发送日志
        RCLCPP_INFO(
            this->get_logger(),
            "发送第%d个航点：cube=%s, waypoint=%s, fleet=deliveryRobot, priority=%d, task_id=%s",
            current_waypoint_idx_ + 1,
            info.cube_name.c_str(),
            info.waypoint.c_str(),
            info.priority,
            task_id.c_str());

        // 8. 索引自增
        current_waypoint_idx_++;
    }

    // 服务响应回调（参数不变，仅调用方式改为lambda）
    void response_callback(
        rclcpp::Client<SingleNavTask>::SharedFuture future, 
        const std::string &task_id, 
        const std::string &waypoint)
    {
        try
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "航点发送成功！自定义task_id=%s, RMF返回task_id=%s, 消息=%s",
                    task_id.c_str(),
                    response->task_id.c_str(),
                    response->message.c_str());
            }
            else
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "航点发送失败！自定义task_id=%s, 错误消息：%s",
                    task_id.c_str(),
                    response->message.c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "服务调用异常：%s", e.what());
        }
    }


    // 成员变量
    rclcpp::Client<SingleNavTask>::SharedPtr client_; 
    rclcpp::TimerBase::SharedPtr timer_;              
    std::vector<WaypointInfo> waypoints_;             
    size_t current_waypoint_idx_;                     
    rclcpp::Publisher<StringMsg>::SharedPtr task_monitor_pub_;  // 新增：监控话题发布者
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoSendWaypointsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
