//1.16复刻python脚本逻辑，使用仿真时间，go_to_place任务格式
// RL 调度器负责 “决策派哪个机器人去哪个点”，这个 C++ 节点负责 “把RL决策转换成 RMF 能执行的标准任务”
#include <rclcpp/rclcpp.hpp>
#include <rmf_task_msgs/msg/api_request.hpp>
#include <rmf_task_msgs/msg/api_response.hpp>
#include <std_msgs/msg/string.hpp>
#include <rmf_custom_tasks_self/srv/single_nav_task.hpp>

#include <string>
#include <cstdlib>
#include <ctime>
#include <random>
#include <sstream>
#include <chrono>
#include <iostream>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

using namespace rclcpp;
using ApiRequestMsg = rmf_task_msgs::msg::ApiRequest;
using SingleNavSrv = rmf_custom_tasks_self::srv::SingleNavTask;
using StringMsg = std_msgs::msg::String;

class RMFCustomSingleNavServer : public Node
{
public:
  RMFCustomSingleNavServer() : Node("rmf_custom_single_nav_server",
                                    rclcpp::NodeOptions()
                                        .allow_undeclared_parameters(true)
                                        .automatically_declare_parameters_from_overrides(true))
  {
    // 1. 强制开启仿真时间 (与 Python 脚本一致)
    // Python: param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
    if (!this->has_parameter("use_sim_time")) {
      this->declare_parameter("use_sim_time", true);
    }
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    std::srand(std::time(nullptr));

    // 2. RMF标准QoS配置 (与 Python 脚本一致: Transient Local)
    auto qos = QoS(10)
                   .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
                   .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                   .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

    _task_pub = this->create_publisher<ApiRequestMsg>("/task_api_requests", qos);
    _monitor_start_pub = this->create_publisher<StringMsg>("/task_monitor/start", QoS(10));
    
    _nav_srv = this->create_service<SingleNavSrv>(
        "/submit_single_nav_task",
        std::bind(&RMFCustomSingleNavServer::handle_nav_request, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "✅ RMF自定义单点导航接口已启动！(Sim Time Mode) [Using 'go_to_place' schema]");
  }

private:
  // --- 辅助函数：生成 UUID ---
  std::string generate_uuid()
  {
    std::stringstream ss;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 15);
    for (int i = 0; i < 8; i++) ss << std::hex << dis(gen);
    ss << "-";
    for (int i = 0; i < 4; i++) ss << std::hex << dis(gen);
    ss << "-4"; 
    for (int i = 0; i < 3; i++) ss << std::hex << dis(gen);
    ss << "-";
    for (int i = 0; i < 12; i++) ss << std::hex << dis(gen);
    return ss.str();
  }

  // --- 核心修改：完全复刻 Python 逻辑 ---
  std::string build_go_to_place_json(
      const std::string &target,
      const std::string &fleet,
      const std::string &robot_name,
      int32_t priority)
  {
    json payload;

    // 1. 判断任务类型
    // Python: if self.args.robot and self.args.fleet:
    if (!robot_name.empty() && !fleet.empty()) {
      payload["type"] = "robot_task_request";
      payload["robot"] = robot_name;
      payload["fleet"] = fleet;
    } else {
      payload["type"] = "dispatch_task_request";
    }

    // 2. 【关键修正】使用 ROS Sim Time (与 Python 脚本保持 100% 一致)
    // Python: now = self.get_clock().now() ... start_time = now.sec * 1000 + ...
    // C++: this->now() 获取的就是当前的仿真时间 (因为我们设置了 use_sim_time=true)
    rclcpp::Time now = this->now();
    long long start_time = now.nanoseconds() / 1000000; // 纳秒转毫秒

    // 3. 构建 "go_to_place"
    json go_to_description;
    go_to_description["waypoint"] = target;

    json go_to_activity;
    go_to_activity["category"] = "go_to_place";
    go_to_activity["description"] = go_to_description;

    // 4. 构建 Compose 任务
    json rmf_task_request;
    rmf_task_request["category"] = "compose";
    // 这里不再用 0，而是用真实的仿真时间戳
    rmf_task_request["unix_millis_earliest_start_time"] = start_time; 
    
    if (priority > 0) {
        json priority_obj;
        priority_obj["value"] = priority;
        rmf_task_request["priority"] = priority_obj;
    }

    json phase;
    phase["activity"] = go_to_activity;
    
    json task_description;
    task_description["category"] = "go_to_place";
    task_description["phases"] = json::array({phase});
    
    rmf_task_request["description"] = task_description;
    
    // 只有在 Dispatch 模式下，才可能需要把 fleet 放到 request 内部
    // Python 脚本其实并没有显式处理 dispatch 时的 fleet，但加上更保险
    if (payload["type"] == "dispatch_task_request" && !fleet.empty()) {
        rmf_task_request["fleet_name"] = fleet;
    }

    payload["request"] = rmf_task_request;

    return payload.dump();
  }

  void handle_nav_request(
      const std::shared_ptr<SingleNavSrv::Request> req,
      std::shared_ptr<SingleNavSrv::Response> res)
  {
    if (req->target_waypoint.empty() || req->fleet_name.empty()) {
      res->success = false;
      res->message = "错误：目标点或车队名不能为空！";
      RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
      return;
    }

    // Python 使用 "direct_" 前缀，我们也改回来，以防万一 RMF 对前缀有正则匹配
    std::string request_id = "direct_" + generate_uuid();

    ApiRequestMsg msg;
    msg.request_id = request_id;

    msg.json_msg = build_go_to_place_json(
        req->target_waypoint, 
        req->fleet_name, 
        req->robot_name,
        req->priority
    );

    _task_pub->publish(msg);
    
    // 打印日志，包含时间戳以便调试
    RCLCPP_INFO(this->get_logger(), "📤 任务发送 [ID: %s] [机器人: %s]", 
                request_id.c_str(), 
                req->robot_name.empty() ? "自动调度" : req->robot_name.c_str());

    StringMsg monitor_msg;
    monitor_msg.data = request_id + "," + req->target_waypoint;
    _monitor_start_pub->publish(monitor_msg);

    res->success = true;
    res->task_id = request_id;
    res->message = "Compose 任务已发送 -> " + req->target_waypoint;
  }

  rclcpp::Publisher<ApiRequestMsg>::SharedPtr _task_pub;
  rclcpp::Service<SingleNavSrv>::SharedPtr _nav_srv;
  rclcpp::Publisher<StringMsg>::SharedPtr _monitor_start_pub;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RMFCustomSingleNavServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

