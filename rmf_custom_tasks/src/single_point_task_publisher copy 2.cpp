#include <rclcpp/rclcpp.hpp> // 12.11 成功实现单点导航
#include <rmf_task_msgs/msg/api_request.hpp>
#include <string>
#include <cstdlib>
#include <ctime>
#include "rmf_custom_tasks/srv/single_nav_task.hpp"
// 新增：添加String消息头文件
#include <std_msgs/msg/string.hpp>

using namespace rclcpp;
using ApiRequestMsg = rmf_task_msgs::msg::ApiRequest;
using SingleNavSrv = rmf_custom_tasks::srv::SingleNavTask;
// 新增：定义String消息类型
using StringMsg = std_msgs::msg::String;

class RMFCustomSingleNavServer : public Node
{
public:
  RMFCustomSingleNavServer() : Node("rmf_custom_single_nav_server", 
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true))
  {
    // 1. 强制开启仿真时间
    if (!this->has_parameter("use_sim_time")) {
        this->declare_parameter("use_sim_time", true);
    }
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    std::srand(std::time(nullptr));

    // 2. 标准 QoS 配置 (配合 RMF 的 Transient Local)
    auto qos = QoS(10)
      .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST); 
    
    _task_pub = this->create_publisher<ApiRequestMsg>("/task_api_requests", qos);

    // 新增：创建监控启动消息发布者（匹配监控节点的订阅话题）
    _monitor_start_pub = this->create_publisher<StringMsg>("/task_monitor/start", QoS(10));
    RCLCPP_INFO(this->get_logger(), "✅ 已创建监控启动消息发布者：/task_monitor/start");

    _nav_srv = this->create_service<SingleNavSrv>(
      "/submit_single_nav_task", 
      std::bind(
        &RMFCustomSingleNavServer::handle_nav_request,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );

    RCLCPP_INFO(this->get_logger(), "✅ RMF自定义单点导航接口已启动！(Sim Time Mode)");
  }

private:
  // 构建任务JSON
  std::string build_raw_json(
      const std::string& target, 
      const std::string& fleet, 
      int32_t priority)
  {
      // 获取仿真时间 (毫秒)
      int64_t start_time_ms = this->now().nanoseconds() / 1000000;
      if (start_time_ms == 0) start_time_ms = (int64_t)std::time(nullptr) * 1000;
      
      // 关键：在JSON中明确指定finishing_request
      std::string json_str = 
          "{"
          "\"type\":\"dispatch_task_request\","
          "\"request\":{"
            "\"unix_millis_earliest_start_time\":" + std::to_string(start_time_ms) + ","
            "\"priority\":{\"value\":" + std::to_string(priority) + "},"
            "\"category\":\"compose\","
            "\"fleet_name\":\"" + fleet + "\","
            "\"finishing_request\":{\"type\":\"nothing\"},"  // 关键！明确指定不返回
            "\"description\":{"
              "\"category\":\"compose\","
              "\"phases\":[{"
                "\"activity\":{"
                  "\"category\":\"go_to_place\","
                  "\"description\":\"" + target + "\""
                "}"
              "}]"
            "}"
          "}"
          "}";

      return json_str;
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

    std::string task_id = "nav_" + std::to_string(std::rand());

    ApiRequestMsg msg;
    msg.request_id = task_id;
    // 使用新的字符串构建函数
    msg.json_msg = build_raw_json(
      req->target_waypoint, 
      req->fleet_name, 
      req->priority
    );
    
    // 1. 发布RMF任务请求
    _task_pub->publish(msg);

    // 打印发出的 JSON 以便调试
    RCLCPP_INFO(this->get_logger(), "📤 发送 JSON: %s", msg.json_msg.c_str());

    // 2. 发送监控启动消息（格式：task_id,target_waypoint）
    // 注意：这里发送的是简化的格式，与监控节点的start_monitoring_callback匹配
    StringMsg monitor_msg;
    monitor_msg.data = task_id + "," + req->target_waypoint;
    _monitor_start_pub->publish(monitor_msg);
    
    RCLCPP_INFO(this->get_logger(), 
               "📢 发送监控启动消息：%s", 
               monitor_msg.data.c_str());

    res->success = true;
    res->task_id = task_id;
    res->message = "任务已发送 -> " + req->target_waypoint;
  }

  // 成员变量
  rclcpp::Publisher<ApiRequestMsg>::SharedPtr _task_pub;
  rclcpp::Service<SingleNavSrv>::SharedPtr _nav_srv;
  // 新增：监控启动消息发布者
  rclcpp::Publisher<StringMsg>::SharedPtr _monitor_start_pub;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RMFCustomSingleNavServer>();
  rclcpp::spin(node); 
  rclcpp::shutdown();
  return 0;
}