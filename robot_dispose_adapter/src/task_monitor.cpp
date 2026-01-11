#include <rclcpp/rclcpp.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <rmf_task_msgs/msg/api_request.hpp>
#include <nlohmann/json.hpp>
#include <limits> // 必须保留，用于数值限制
// 1.9 可以实现多任务监听，现在加一个新节点，全局任务队列
using json = nlohmann::json;

#include <map>
#include <string>
#include <cmath>
#include <chrono>
#include <memory>
#include <utility>
#include <functional>

using namespace std::chrono_literals;
using ApiRequestMsg = rmf_task_msgs::msg::ApiRequest;

// 简化任务状态枚举（仅保留执行中/已完成）
enum TaskStatus
{
    EXECUTING,  // 执行中（替代原有PENDING/ASSIGNED）
    COMPLETING, // 处置中
    COMPLETED   // 已完成
};

// 简化任务信息结构体（新增：连续满足距离的计时字段）
struct TaskInfo
{
    geometry_msgs::msg::Point target_coords; // 任务目标航点坐标
    TaskStatus status;                       // 任务状态
    std::string target_waypoint;             // 目标航点名称
    rclcpp::Time arrival_time;               // 抵达时间
    std::string nearest_robot;               // 仅记录最近小车（不绑定）
    double nearest_distance;                 // 最近小车到目标的距离

    // 新增：连续满足距离条件的计时相关
    std::string qualified_robot;         // 当前满足距离<2米的小车
    rclcpp::Time distance_ok_start_time; // 首次满足距离<2米的时间
    bool is_distance_ok;                 // 是否处于距离<2米状态
};

// 存储小车的RMF任务ID映射（仅用于发送完成信号）
std::map<std::string, std::string> robot_to_rmf_task_id;

// 辅助函数：将TaskStatus转为字符串
std::string task_status_to_string(TaskStatus status)
{
    switch (status)
    {
    case EXECUTING:
        return "EXECUTING(执行中)";
    case COMPLETING:
        return "COMPLETING(处置中)";
    case COMPLETED:
        return "COMPLETED(已完成)";
    default:
        return "UNKNOWN(未知)";
    }
}

class TaskMonitor : public rclcpp::Node
{
public:
    TaskMonitor() : Node("task_monitor")
    {
        RCLCPP_INFO(this->get_logger(), "初始化任务监控器（纯距离检测+5秒驻留模式）...");

        // 初始化航点坐标字典（保留原有硬编码）
        initWaypointCoords();

        // 订阅车队状态（完全保留原有逻辑，确保能获取小车位置）
        fleet_state_sub_ = this->create_subscription<rmf_fleet_msgs::msg::FleetState>(
            "/fleet_states", 10,
            std::bind(&TaskMonitor::fleet_state_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "✅ 已订阅话题：/fleet_states");

        // 订阅开始监控任务的请求
        start_monitoring_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/task_monitor/start",
            10,
            std::bind(&TaskMonitor::start_monitoring_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "✅ 已订阅话题：/task_monitor/start");

        // 订阅RMF任务分发请求（保留原有，兼容旧逻辑）
        dispatch_sub_ = this->create_subscription<ApiRequestMsg>(
            "/task_api_requests",
            10,
            std::bind(&TaskMonitor::dispatch_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "✅ 已订阅话题：/task_api_requests");

        // 创建任务完成信号发布器
        completion_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/custom_task_completion", 10);
        RCLCPP_INFO(this->get_logger(), "✅ 已创建话题：/custom_task_completion");

        // 创建Gazebo删除服务客户端
        delete_entity_client_ = this->create_client<gazebo_msgs::srv::DeleteEntity>(
            "/delete_entity");
        RCLCPP_INFO(this->get_logger(), "✅ 已创建Gazebo删除服务客户端");

        // 创建RMF任务完成请求发布者
        task_complete_pub_ = this->create_publisher<ApiRequestMsg>(
            "/task_api_requests",
            rclcpp::QoS(10)
                .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
                .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE));
        RCLCPP_INFO(this->get_logger(), "✅ 已创建RMF任务完成请求发布者");

        // 创建监控定时器（1秒检查一次任务）
        monitor_timer_ = this->create_wall_timer(
            1000ms, std::bind(&TaskMonitor::monitor_tasks, this));

        // 15秒打印一次任务队列状态
        status_print_timer_ = this->create_wall_timer(
            15000ms, std::bind(&TaskMonitor::print_task_queue_status, this));

        RCLCPP_INFO(this->get_logger(), "🚀 任务监控器初始化完成，等待任务...");
    }

private:
    // 初始化航点坐标字典（完全保留原有）
    void initWaypointCoords()
    {
        waypoint_coords_["junction_n01"] = geometry_msgs::msg::Point();
        waypoint_coords_["junction_n01"].x = 1.57;
        waypoint_coords_["junction_n01"].y = -45.93;

        waypoint_coords_["n08"] = geometry_msgs::msg::Point();
        waypoint_coords_["n08"].x = 59.61;
        waypoint_coords_["n08"].y = -7.42;

        waypoint_coords_["n14"] = geometry_msgs::msg::Point();
        waypoint_coords_["n14"].x = 80.84;
        waypoint_coords_["n14"].y = -28.52;

        waypoint_coords_["n13"] = geometry_msgs::msg::Point();
        waypoint_coords_["n13"].x = 84.44;
        waypoint_coords_["n13"].y = -4.94;

        waypoint_coords_["n23"] = geometry_msgs::msg::Point();
        waypoint_coords_["n23"].x = 182.80;
        waypoint_coords_["n23"].y = -42.30;

        waypoint_coords_["west_koi_pond"] = geometry_msgs::msg::Point();
        waypoint_coords_["west_koi_pond"].x = 34.32;
        waypoint_coords_["west_koi_pond"].y = -10.13;

        waypoint_coords_["s08"] = geometry_msgs::msg::Point();
        waypoint_coords_["s08"].x = 96.61;
        waypoint_coords_["s08"].y = -50.50;

        waypoint_coords_["s10"] = geometry_msgs::msg::Point();
        waypoint_coords_["s10"].x = 122.10;
        waypoint_coords_["s10"].y = -46.68;

        waypoint_coords_["s11"] = geometry_msgs::msg::Point();
        waypoint_coords_["s11"].x = 152.73;
        waypoint_coords_["s11"].y = -43.00;

        waypoint_coords_["junction_south_west"] = geometry_msgs::msg::Point();
        waypoint_coords_["junction_south_west"].x = 84.56;
        waypoint_coords_["junction_south_west"].y = -38.81;

        RCLCPP_INFO(this->get_logger(), "📌 已初始化 %zu 个航点坐标", waypoint_coords_.size());
    }

    // 修复日志打印逻辑：增加小车列表为空的判断，补全距离打印
    void print_task_queue_status()
    {
        RCLCPP_INFO(this->get_logger(), "======================= 任务队列状态 =======================");

        if (active_tasks_.empty())
        {
            RCLCPP_INFO(this->get_logger(), "📭 当前无活跃任务");
            RCLCPP_INFO(this->get_logger(), "============================================================");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "📊 活跃任务总数：%zu", active_tasks_.size());

        // 遍历所有任务，打印每个任务的所有小车距离
        for (const auto &[task_id, task_info] : active_tasks_)
        {
            RCLCPP_INFO(this->get_logger(), "┌────────────────────────────────────────────────────────");
            RCLCPP_INFO(this->get_logger(), "│ 任务ID: %s", task_id.c_str());
            RCLCPP_INFO(this->get_logger(), "│ 目标航点: %s (X: %.2f, Y: %.2f)",
                        task_info.target_waypoint.c_str(),
                        task_info.target_coords.x, task_info.target_coords.y);
            RCLCPP_INFO(this->get_logger(), "│ 任务状态: %s", task_status_to_string(task_info.status).c_str());

            // 修复：判断是否有小车数据，避免打印最大值
            if (task_info.nearest_robot.empty() || robot_positions_.empty())
            {
                RCLCPP_INFO(this->get_logger(), "│ 最近小车: 无 (距离: 无有效小车数据)");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "│ 最近小车: %s (距离: %.2f米)",
                            task_info.nearest_robot.c_str(), task_info.nearest_distance);
            }

            // 新增：打印距离满足状态和计时
            if (task_info.is_distance_ok)
            {
                double elapsed = (this->now() - task_info.distance_ok_start_time).seconds();
                RCLCPP_INFO(this->get_logger(), "│ 距离状态: 满足<2米 | 持续时间: %.1f秒 (需累计5秒)", elapsed);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "│ 距离状态: 不满足<2米 | 持续时间: 0秒");
            }

            // 修复：打印所有小车到该任务的距离（增加空判断）
            RCLCPP_INFO(this->get_logger(), "│ 所有小车到目标距离：");
            if (robot_positions_.empty())
            {
                RCLCPP_INFO(this->get_logger(), "│   - 暂无小车位置数据");
            }
            else
            {
                for (const auto &[robot_name, robot_pos] : robot_positions_)
                {
                    double dx = task_info.target_coords.x - robot_pos.x;
                    double dy = task_info.target_coords.y - robot_pos.y;
                    double distance = std::sqrt(dx * dx + dy * dy);
                    RCLCPP_INFO(this->get_logger(), "│   - %s: %.2f米", robot_name.c_str(), distance);
                }
            }
            RCLCPP_INFO(this->get_logger(), "└────────────────────────────────────────────────────────");
        }

        // 打印小车基础信息（辅助排查）
        RCLCPP_INFO(this->get_logger(), "🤖 当前小车信息：");
        if (robot_positions_.empty())
        {
            RCLCPP_INFO(this->get_logger(), "│ 暂无小车位置数据");
        }
        else
        {
            for (const auto &[robot_name, pos] : robot_positions_)
            {
                std::string robot_task_id = "无任务";
                if (robot_to_rmf_task_id.count(robot_name))
                {
                    robot_task_id = robot_to_rmf_task_id[robot_name];
                }
                RCLCPP_INFO(this->get_logger(), "│ %s: (X: %.2f, Y: %.2f) | RMF任务ID: %s",
                            robot_name.c_str(), pos.x, pos.y, robot_task_id.c_str());
            }
        }

        RCLCPP_INFO(this->get_logger(), "============================================================");
    }

    // 完全保留原有fleet_state回调逻辑（确保能获取小车位置）
    void fleet_state_callback(const rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
    {
        if (msg->name != "deliveryRobot")
        {
            return;
        }

        for (const auto &robot : msg->robots)
        {
            std::string robot_name = robot.name;
            geometry_msgs::msg::Point robot_position;
            robot_position.x = robot.location.x;
            robot_position.y = robot.location.y;
            robot_position.z = 0.0;

            std::string rmf_task_id = robot.task_id;
            robot_positions_[robot_name] = robot_position;
            robot_to_rmf_task_id[robot_name] = rmf_task_id; // 记录小车当前执行的RMF任务ID

            RCLCPP_DEBUG(this->get_logger(),
                         "🔍 小车[%s] 位置: (%.2f, %.2f) | RMF任务ID: %s",
                         robot_name.c_str(), robot_position.x, robot_position.y, rmf_task_id.c_str());
        }
    }

    // 保留原有dispatch回调（兼容旧逻辑）
    void dispatch_callback(const ApiRequestMsg::SharedPtr msg)
    {
        try
        {
            json task_json = json::parse(msg->json_msg);
            std::string waypoint = task_json["request"]["description"]["phases"][0]["activity"]["description"];
            rmf_task_to_waypoint_[msg->request_id] = waypoint;

            RCLCPP_INFO(this->get_logger(),
                        "📌 RMF任务ID [%s] 映射到航点 [%s]",
                        msg->request_id.c_str(), waypoint.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(),
                        "⚠️ 解析任务JSON失败：%s，消息内容：%s",
                        e.what(), msg->json_msg.c_str());
        }
    }

    // 接收任务请求：仅创建任务，不绑定任何小车（初始化新增的计时字段）
    void start_monitoring_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string data = msg->data;
        size_t comma_pos = data.find(',');

        if (comma_pos == std::string::npos)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "❌ 无效的消息格式，应为'task_id,target_waypoint'，收到: %s",
                         data.c_str());
            return;
        }

        std::string task_id = data.substr(0, comma_pos);
        std::string target_waypoint = data.substr(comma_pos + 1);

        RCLCPP_INFO(this->get_logger(),
                    "📥 收到监控请求: 任务ID=[%s], 目标航点=[%s]",
                    task_id.c_str(), target_waypoint.c_str());

        if (waypoint_coords_.find(target_waypoint) == waypoint_coords_.end())
        {
            RCLCPP_ERROR(this->get_logger(), "❌ 未知航点: %s", target_waypoint.c_str());
            return;
        }

        if (active_tasks_.find(task_id) != active_tasks_.end())
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ 任务 [%s] 已在监控中", task_id.c_str());
            return;
        }

        // 初始化任务信息（无绑定，仅记录目标坐标 + 新增计时字段初始化）
        TaskInfo task_info;
        task_info.target_coords = waypoint_coords_[target_waypoint];
        task_info.status = EXECUTING;
        task_info.target_waypoint = target_waypoint;
        task_info.nearest_robot = "";
        task_info.nearest_distance = 0.0; // 初始化0，避免最大值

        // 新增：计时字段初始化
        task_info.qualified_robot = "";
        task_info.distance_ok_start_time = this->now(); // 初始化为当前时间
        task_info.is_distance_ok = false;

        active_tasks_[task_id] = task_info;

        RCLCPP_INFO(this->get_logger(),
                    "📌 开始监控任务 [%s] -> 航点 [%s] (坐标: %.2f, %.2f)",
                    task_id.c_str(), target_waypoint.c_str(),
                    task_info.target_coords.x, task_info.target_coords.y);
    }

    // 核心逻辑：纯距离检测 + 5秒连续满足距离才触发删除
    void monitor_tasks()
    {
        auto now = this->now();
        const double DISTANCE_THRESHOLD = 2.0; // 距离阈值2米
        const double DURATION_THRESHOLD = 5.0; // 连续满足时间阈值5秒

        // 遍历所有任务，计算距离并判断完成
        for (auto &[task_id, task_info] : active_tasks_)
        {
            if (task_info.status == COMPLETED || task_info.status == COMPLETING)
            {
                continue;
            }

            // 初始化最近距离和小车（修复：用合理默认值）
            double min_distance = 10000.0; // 和原代码保持一致的默认值
            std::string nearest_robot = "";
            std::string current_qualified_robot = ""; // 当前满足距离<2米的小车

            // 只有小车列表非空时才计算距离
            if (!robot_positions_.empty())
            {
                // 遍历所有小车，计算到任务航点的距离
                for (const auto &[robot_name, robot_pos] : robot_positions_)
                {
                    double dx = task_info.target_coords.x - robot_pos.x;
                    double dy = task_info.target_coords.y - robot_pos.y;
                    double distance = std::sqrt(dx * dx + dy * dy);

                    // 更新最近小车和距离
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                        nearest_robot = robot_name;
                    }

                    // 判断：当前小车是否满足距离<2米
                    if (distance < DISTANCE_THRESHOLD)
                    {
                        current_qualified_robot = robot_name;
                    }
                }
            }

            // 更新任务的最近距离和小车（仅记录，不绑定）
            task_info.nearest_robot = nearest_robot;
            task_info.nearest_distance = min_distance;

            // ========== 新增核心逻辑：连续5秒距离<2米判定 ==========
            if (!current_qualified_robot.empty())
            {
                // 情况1：当前有小车满足距离<2米
                if (!task_info.is_distance_ok)
                {
                    // 首次满足，记录开始时间和小车
                    task_info.is_distance_ok = true;
                    task_info.qualified_robot = current_qualified_robot;
                    task_info.distance_ok_start_time = now;
                    RCLCPP_INFO(this->get_logger(),
                                "⏱️  任务[%s]：小车[%s]进入目标区域（距离: %.2f米），开始计时...",
                                task_id.c_str(), current_qualified_robot.c_str(),
                                std::sqrt(std::pow(task_info.target_coords.x - robot_positions_[current_qualified_robot].x, 2) +
                                          std::pow(task_info.target_coords.y - robot_positions_[current_qualified_robot].y, 2)));
                }
                else
                {
                    // 持续满足，检查是否累计达到5秒
                    double elapsed_time = (now - task_info.distance_ok_start_time).seconds();
                    if (elapsed_time >= DURATION_THRESHOLD)
                    {
                        // 满足5秒，触发任务完成
                        RCLCPP_INFO(this->get_logger(),
                                    "🎯 任务[%s]完成触发：小车[%s]在目标区域驻留%.1f秒（≥5秒），距离: %.2f米",
                                    task_id.c_str(), task_info.qualified_robot.c_str(),
                                    elapsed_time,
                                    std::sqrt(std::pow(task_info.target_coords.x - robot_positions_[task_info.qualified_robot].x, 2) +
                                              std::pow(task_info.target_coords.y - robot_positions_[task_info.qualified_robot].y, 2)));

                        task_info.arrival_time = now;
                        task_info.status = COMPLETING;

                        // 执行立方体删除
                        execute_disposal_sequence(task_id, task_info, task_info.qualified_robot);

                        // 向RMF发送任务完成信号（使用触发小车的RMF任务ID）
                        if (robot_to_rmf_task_id.count(task_info.qualified_robot))
                        {
                            send_rmf_task_complete(robot_to_rmf_task_id[task_info.qualified_robot]);
                        }
                    }
                    else
                    {
                        // 未达到5秒，打印计时日志（DEBUG级别，避免刷屏）
                        RCLCPP_DEBUG(this->get_logger(),
                                     "⏳ 任务[%s]：小车[%s]已驻留%.1f秒（需5秒），距离: %.2f米",
                                     task_id.c_str(), task_info.qualified_robot.c_str(),
                                     elapsed_time,
                                     std::sqrt(std::pow(task_info.target_coords.x - robot_positions_[task_info.qualified_robot].x, 2) +
                                               std::pow(task_info.target_coords.y - robot_positions_[task_info.qualified_robot].y, 2)));
                    }
                }
            }
            else
            {
                // 情况2：当前无小车满足距离<2米，重置计时状态
                if (task_info.is_distance_ok)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "🔄 任务[%s]：小车[%s]离开目标区域，重置计时",
                                task_id.c_str(), task_info.qualified_robot.c_str());
                    task_info.is_distance_ok = false;
                    task_info.qualified_robot = "";
                }
            }
        }
    }

    // 向RMF发送任务完成请求（保留原有）
    void send_rmf_task_complete(const std::string &rmf_task_id)
    {
        if (rmf_task_id.empty())
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ RMF task_id为空，跳过发送完成请求");
            return;
        }

        json complete_json;
        complete_json["type"] = "cancel_task_request";
        complete_json["request"]["task_id"] = rmf_task_id;
        complete_json["request"]["reason"] = "task_completed_successfully";

        ApiRequestMsg msg;
        msg.request_id = "complete_" + rmf_task_id;
        msg.json_msg = complete_json.dump();

        task_complete_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "📤 向RMF发送完成请求：%s", rmf_task_id.c_str());
    }

    // 执行立方体删除逻辑（新增completed_robot参数）
    void execute_disposal_sequence(const std::string &task_id, TaskInfo &task_info, const std::string &completed_robot)
    {
        RCLCPP_INFO(this->get_logger(), "🔧 开始执行任务 [%s] 的处置序列（触发小车：%s）...",
                    task_id.c_str(), completed_robot.c_str());

        // 模型名拼接逻辑（保留原有）
        std::string cube_name = "red_cube_" + task_info.target_waypoint;
        delete_gazebo_model(cube_name, task_id);

        RCLCPP_INFO(this->get_logger(), "⏳ 任务 [%s] 开始5秒停留...", task_id.c_str());

        // 5秒后发布完成信号
        auto timer = this->create_wall_timer(
            5000ms,
            [this, task_id, completed_robot, task_info]()
            {
                this->publish_completion(task_id, completed_robot, task_info.target_waypoint);
            });

        completion_timers_[task_id] = timer;
    }

    // 删除Gazebo模型（保留原有逻辑）
    void delete_gazebo_model(const std::string &model_name, const std::string &task_id)
    {
        if (!delete_entity_client_->wait_for_service(3s))
        {
            RCLCPP_ERROR(this->get_logger(),
                         "❌ 任务 [%s]：Gazebo删除服务不可用，无法删除模型: %s",
                         task_id.c_str(), model_name.c_str());
            return;
        }

        auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
        request->name = model_name;

        delete_entity_client_->async_send_request(request,
                                                  [this, model_name, task_id](rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedFuture future)
                                                  {
                                                      auto response = future.get();
                                                      if (response->success)
                                                      {
                                                          RCLCPP_INFO(this->get_logger(),
                                                                      "✅ 任务 [%s]：成功删除模型: %s",
                                                                      task_id.c_str(), model_name.c_str());
                                                      }
                                                      else
                                                      {
                                                          RCLCPP_ERROR(this->get_logger(),
                                                                       "❌ 任务 [%s]：删除模型失败: %s",
                                                                       task_id.c_str(), model_name.c_str());
                                                      }
                                                  });

        RCLCPP_INFO(this->get_logger(), "🗑️  任务 [%s]：发送删除请求: %s", task_id.c_str(), model_name.c_str());
    }

    // 发布任务完成信号（调整参数）
    void publish_completion(const std::string &task_id, const std::string &robot_name, const std::string &waypoint)
    {
        auto it = active_tasks_.find(task_id);
        if (it == active_tasks_.end())
        {
            RCLCPP_ERROR(this->get_logger(), "❌ 任务 [%s] 不存在，无法发布完成信号", task_id.c_str());
            return;
        }

        // 构造完成消息
        std_msgs::msg::String msg;
        auto now_sec = this->now().seconds();
        msg.data = task_id + "," + robot_name + "," + waypoint + "," + std::to_string(now_sec);

        completion_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(),
                    "📤 任务 [%s] 完成信号已发布: %s",
                    task_id.c_str(), msg.data.c_str());

        // 更新任务状态并清理
        it->second.status = COMPLETED;
        if (completion_timers_.find(task_id) != completion_timers_.end())
        {
            completion_timers_[task_id]->cancel();
            completion_timers_.erase(task_id);
        }

        // 任务完成后删除，避免内存堆积
        active_tasks_.erase(it);
    }

    // 成员变量（完全保留原有，确保兼容）
    rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_monitoring_sub_;
    rclcpp::Subscription<ApiRequestMsg>::SharedPtr dispatch_sub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr completion_pub_;
    rclcpp::Publisher<ApiRequestMsg>::SharedPtr task_complete_pub_;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_entity_client_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    rclcpp::TimerBase::SharedPtr status_print_timer_;

    std::map<std::string, TaskInfo> active_tasks_;
    std::map<std::string, geometry_msgs::msg::Point> waypoint_coords_;
    std::map<std::string, geometry_msgs::msg::Point> robot_positions_;
    std::map<std::string, std::string> rmf_task_to_waypoint_;
    std::map<std::string, rclcpp::TimerBase::SharedPtr> completion_timers_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
