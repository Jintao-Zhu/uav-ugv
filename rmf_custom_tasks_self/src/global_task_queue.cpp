#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_custom_tasks/srv/single_nav_task.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <map>
#include <vector>
#include <string>
#include <cmath>
#include <mutex>

using namespace std::chrono_literals;
using SingleNavTask = rmf_custom_tasks::srv::SingleNavTask;

/*
维护线程安全的任务队列；
监听小车状态和位置；
贪心策略选择距离最近的任务；
调用自定义服务发布任务给 RMF；
任务发布失败时自动回退到队列，确保不丢失。
【修改版 V6 终极版】：
1. 修复假空闲导致的日志刷屏问题
2. 识破假空闲后，强制将小车内部状态置为 BUSY，保护当前任务不被覆盖
*/

struct Task
{
    std::string task_id;
    std::string target_waypoint;
    std::string fleet_name;
    int priority;
    rclcpp::Time submit_time;
};

class GlobalTaskQueueNode : public rclcpp::Node
{
public:
    GlobalTaskQueueNode() : Node("global_task_queue_node")
    {
        if (!this->has_parameter("use_sim_time"))
        {
            this->declare_parameter("use_sim_time", true);
        }
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        RCLCPP_INFO(this->get_logger(), "初始化全局任务队列节点（智能防覆盖模式）...");

        init_waypoint_coords();

        task_sub_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/task_queue/submit", 10, std::bind(&GlobalTaskQueueNode::task_submit_callback, this, std::placeholders::_1));

        robot_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot/status", 10, std::bind(&GlobalTaskQueueNode::robot_status_callback, this, std::placeholders::_1));

        fleet_state_sub_ = this->create_subscription<rmf_fleet_msgs::msg::FleetState>(
            "/fleet_states", 10, std::bind(&GlobalTaskQueueNode::fleet_state_callback, this, std::placeholders::_1));

        nav_client_ = this->create_client<SingleNavTask>("/submit_single_nav_task");

        scan_timer_ = this->create_wall_timer(
            1000ms, std::bind(&GlobalTaskQueueNode::scan_queue_callback, this));

        RCLCPP_INFO(this->get_logger(), "🚀 节点启动就绪");
    }

private:
    void init_waypoint_coords()
    {
        auto add_pt = [&](std::string name, double x, double y)
        {
            geometry_msgs::msg::Point p;
            p.x = x;
            p.y = y;
            waypoint_coords_[name] = p;
        };
        add_pt("junction_n01", 1.57, -45.93);
        add_pt("n08", 59.61, -7.42);
        add_pt("n14", 80.84, -28.52);
        add_pt("n13", 84.44, -4.94);
        add_pt("n23", 182.80, -42.30);
        add_pt("west_koi_pond", 34.32, -10.13);
        add_pt("s08", 96.61, -45.94);
        add_pt("s10", 122.10, -46.68);
        add_pt("s11", 152.73, -42.86);
        add_pt("junction_south_west", 84.56, -38.81);
    }

    void task_submit_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        try
        {
            std::string data = msg->data;
            size_t pos1 = data.find(',');
            size_t pos2 = data.find(',', pos1 + 1);
            size_t pos3 = data.find(',', pos2 + 1);

            if (pos1 == std::string::npos || pos2 == std::string::npos || pos3 == std::string::npos)
                return;

            Task task;
            task.task_id = data.substr(0, pos1);
            task.target_waypoint = data.substr(pos1 + 1, pos2 - pos1 - 1);
            task.fleet_name = data.substr(pos2 + 1, pos3 - pos2 - 1);
            task.priority = std::stoi(data.substr(pos3 + 1));
            task.submit_time = this->now();

            if (waypoint_coords_.find(task.target_waypoint) == waypoint_coords_.end())
            {
                RCLCPP_ERROR(this->get_logger(), "未知航点：%s", task.target_waypoint.c_str());
                return;
            }

            task_queue_.push_back(task);
            RCLCPP_INFO(this->get_logger(), "📥 任务[%s]入队 -> %s (当前队列: %zu)",
                        task.task_id.c_str(), task.target_waypoint.c_str(), task_queue_.size());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "任务解析异常: %s", e.what());
        }
    }

    void robot_status_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        try
        {
            std::string data = msg->data;
            size_t pos = data.find(',');
            if (pos == std::string::npos)
                return;

            std::string robot_name = data.substr(0, pos);
            std::string new_status = data.substr(pos + 1);
            std::string old_status = robot_status_[robot_name];

            // 保护逻辑 1：时间锁 (15秒内无视 Idle)
            if (new_status == "idle" && last_assignment_time_.count(robot_name))
            {
                auto now = this->now();
                double seconds_since_assign = (now - last_assignment_time_[robot_name]).seconds();

                if (seconds_since_assign < 15.0)
                {
                    RCLCPP_DEBUG(this->get_logger(), "🛡️ 忽略小车 %s 的空闲信号 (刚出发 %.1fs < 15s)",
                                 robot_name.c_str(), seconds_since_assign);
                    return;
                }
            }

            robot_status_[robot_name] = new_status;

            if (old_status != "idle" && new_status == "idle")
            {
                robot_cooldowns_[robot_name] = this->now();
                RCLCPP_INFO(this->get_logger(), "🔵 小车 %s 报告空闲，进入3秒冷却", robot_name.c_str());
            }
        }
        catch (...)
        {
        }
    }

    void fleet_state_callback(const rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
    {
        if (msg->name != "deliveryRobot")
            return;
        std::lock_guard<std::mutex> lock(pos_mutex_);
        for (const auto &robot : msg->robots)
        {
            geometry_msgs::msg::Point pos;
            pos.x = robot.location.x;
            pos.y = robot.location.y;
            pos.z = 0.0;
            robot_positions_[robot.name] = pos;
        }
    }

    void scan_queue_callback()
    {
        std::vector<std::string> idle_robots;
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            for (const auto &[robot_name, status] : robot_status_)
            {
                if (status == "idle")
                    idle_robots.push_back(robot_name);
            }
        }

        if (idle_robots.empty())
            return;

        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (task_queue_.empty())
            return;

        for (const auto &robot_name : idle_robots)
        {
            if (task_queue_.empty())
                break;

            if (robot_cooldowns_.count(robot_name))
            {
                double time_diff = (this->now() - robot_cooldowns_[robot_name]).seconds();
                if (time_diff < 3.0)
                    continue;
            }

            // 保护逻辑 2：距离验证 (核心修改)
            if (last_target_name_.count(robot_name) && last_assignment_time_.count(robot_name))
            {
                std::string last_target = last_target_name_[robot_name];

                geometry_msgs::msg::Point robot_pos;
                bool has_pos = false;
                {
                    std::lock_guard<std::mutex> pos_lock(pos_mutex_);
                    if (robot_positions_.count(robot_name))
                    {
                        robot_pos = robot_positions_[robot_name];
                        has_pos = true;
                    }
                }

                if (has_pos && waypoint_coords_.count(last_target))
                {
                    geometry_msgs::msg::Point target_pos = waypoint_coords_[last_target];
                    double dx = target_pos.x - robot_pos.x;
                    double dy = target_pos.y - robot_pos.y;
                    double dist = std::sqrt(dx * dx + dy * dy);
                    double time_since = (this->now() - last_assignment_time_[robot_name]).seconds();

                    // 如果确实是假空闲 (离目标远 且 时间短)
                    if (dist > 5.0 && time_since < 60.0)
                    {
                        RCLCPP_INFO(this->get_logger(), "🚫 识破小车 %s 假空闲 (距 %s 还有 %.1f米)，强制标记为 BUSY",
                                    robot_name.c_str(), last_target.c_str(), dist);

                        // 【核心修改】强制设为 busy，防止循环刷屏，并让出分配机会给其他车
                        {
                            std::lock_guard<std::mutex> lock(status_mutex_);
                            robot_status_[robot_name] = "busy";
                        }
                        continue;
                    }
                }
            }

            int best_task_idx = select_best_task(robot_name);
            if (best_task_idx < 0)
                continue;

            Task best_task = task_queue_[best_task_idx];
            task_queue_.erase(task_queue_.begin() + best_task_idx);

            publish_task_to_rmf(best_task);

            {
                std::lock_guard<std::mutex> lock(status_mutex_);
                robot_status_[robot_name] = "busy";
            }

            last_assignment_time_[robot_name] = this->now();
            robot_cooldowns_[robot_name] = this->now();
            last_target_name_[robot_name] = best_task.target_waypoint;

            RCLCPP_INFO(this->get_logger(), "🚀 分配任务[%s] -> %s (目标:%s)",
                        best_task.task_id.c_str(), robot_name.c_str(), best_task.target_waypoint.c_str());
        }
    }

    int select_best_task(const std::string &robot_name)
    {
        std::lock_guard<std::mutex> lock(pos_mutex_);
        if (robot_positions_.find(robot_name) == robot_positions_.end())
            return 0;

        geometry_msgs::msg::Point robot_pos = robot_positions_[robot_name];
        double min_distance = 1e9;
        int best_idx = -1;

        for (int i = 0; i < task_queue_.size(); i++)
        {
            const Task &task = task_queue_[i];
            geometry_msgs::msg::Point target_pos = waypoint_coords_[task.target_waypoint];

            double dx = target_pos.x - robot_pos.x;
            double dy = target_pos.y - robot_pos.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            double weighted_distance = distance / (task.priority + 1);

            if (weighted_distance < min_distance)
            {
                min_distance = weighted_distance;
                best_idx = i;
            }
        }
        return best_idx >= 0 ? best_idx : 0;
    }

    void publish_task_to_rmf(const Task &task)
    {
        if (!nav_client_->wait_for_service(1s))
        {
            RCLCPP_ERROR(this->get_logger(), "服务不可用，任务回退");
            std::lock_guard<std::mutex> lock(queue_mutex_);
            task_queue_.insert(task_queue_.begin(), task);
            return;
        }

        auto request = std::make_shared<SingleNavTask::Request>();
        request->target_waypoint = task.target_waypoint;
        request->fleet_name = task.fleet_name;
        request->priority = task.priority;

        nav_client_->async_send_request(request, [this, task](rclcpp::Client<SingleNavTask>::SharedFuture future)
                                        {
            try {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "✅ RMF接收任务: %s", response->task_id.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "❌ RMF拒绝: %s", response->message.c_str());
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    task_queue_.insert(task_queue_.begin(), task);
                }
            } catch (...) {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                task_queue_.insert(task_queue_.begin(), task);
            } });
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_sub_;
    rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_state_sub_;
    rclcpp::Client<SingleNavTask>::SharedPtr nav_client_;
    rclcpp::TimerBase::SharedPtr scan_timer_;

    std::mutex queue_mutex_;
    std::vector<Task> task_queue_;

    std::mutex status_mutex_;
    std::map<std::string, std::string> robot_status_;

    std::mutex pos_mutex_;
    std::map<std::string, geometry_msgs::msg::Point> robot_positions_;
    std::map<std::string, geometry_msgs::msg::Point> waypoint_coords_;

    std::map<std::string, rclcpp::Time> robot_cooldowns_;
    std::map<std::string, rclcpp::Time> last_assignment_time_;
    std::map<std::string, std::string> last_target_name_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalTaskQueueNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}