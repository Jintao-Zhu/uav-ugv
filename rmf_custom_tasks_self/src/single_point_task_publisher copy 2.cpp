#include <rclcpp/rclcpp.hpp>
#include <rmf_task_msgs/msg/api_request.hpp>
#include <string>
#include <cstdlib>
#include <ctime>
#include "rmf_custom_tasks_self/srv/single_nav_task.hpp"
#include <std_msgs/msg/string.hpp>
#include <random>  // 随机数相关（可选，替换rand()更规范）
#include <sstream> // 字符串流（可选，优化JSON构建）
#include <chrono>  // 时间戳（可选，替换unix_millis_earliest_start_time=0）
/*1.16 可以正常运行，启动这个节点之后，可以发布单点导航任务 */
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
    // 1. 强制开启仿真时间（原有逻辑保留）
    if (!this->has_parameter("use_sim_time"))
    {
      this->declare_parameter("use_sim_time", true);
    }
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    std::srand(std::time(nullptr));

    // 2. RMF标准QoS配置（原有逻辑保留）
    auto qos = QoS(10)
                   .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
                   .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                   .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

    _task_pub = this->create_publisher<ApiRequestMsg>("/task_api_requests", qos);

    // 监控启动消息发布者（原有逻辑保留）
    _monitor_start_pub = this->create_publisher<StringMsg>("/task_monitor/start", QoS(10));
    RCLCPP_INFO(this->get_logger(), "✅ 已创建监控启动消息发布者：/task_monitor/start");

    // 单点导航服务（原有逻辑保留）
    _nav_srv = this->create_service<SingleNavSrv>(
        "/submit_single_nav_task",
        std::bind(
            &RMFCustomSingleNavServer::handle_nav_request,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "✅ RMF自定义单点导航接口已启动！(Sim Time Mode)");
  }

private:
  // 恢复你原有能正常执行的JSON格式，添加队列兼容逻辑
  std::string build_raw_json(
      const std::string &target,
      const std::string &fleet,
      int32_t priority)
  {
    // 【修改点1】直接生成一个新的唯一ID，用于内部追踪
    // 注意：需包含 <random> 和 <sstream> 头文件，或者直接用传入的 req_id 如果有的话
    // 这里为了简单，我们不传 ID，RMF 会自动处理，或者我们在外部 msg 层的 request_id 够用了

    // 构造 JSON
    // 我们改用 "patrol" 模式，这是最稳妥的单点导航方式
    std::string json_str =
        "{"
        "\"type\": \"dispatch_task_request\"," // 改为 dispatch_task_request
        "\"request\": {"
        "\"unix_millis_earliest_start_time\": 0," // 【核心修改】0 代表 ASAP (立即)
        "\"priority\": {\"value\": " +
        std::to_string(priority) + "},"
                                   "\"category\": \"patrol\"," // 【修改点2】改为 patrol 类型
                                   "\"fleet_name\": \"" +
        fleet + "\","
                "\"description\": {"
                "\"places\": [\"" +
        target + "\"],"          // 目标点
                 "\"rounds\": 1" // 巡逻 1 次等于去那里停下
                 "}"
                 "}"
                 "}";

    return json_str;

  }

  void handle_nav_request(
      const std::shared_ptr<SingleNavSrv::Request> req,
      std::shared_ptr<SingleNavSrv::Response> res)
  {
    // 参数校验（原有逻辑保留）
    if (req->target_waypoint.empty() || req->fleet_name.empty())
    {
      res->success = false;
      res->message = "错误：目标点或车队名不能为空！";
      RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
      return;
    }

    // 生成任务ID（原有逻辑保留）
    std::string task_id = "nav_" + std::to_string(std::rand());

    // 构建任务消息（恢复原有能执行的格式）
    ApiRequestMsg msg;
    msg.request_id = task_id;
    msg.json_msg = build_raw_json(
        req->target_waypoint,
        req->fleet_name,
        req->priority);

    // 发布到RMF任务接口（原有逻辑）
    _task_pub->publish(msg);
    RCLCPP_INFO(this->get_logger(), "📤 任务[%s]已发送到RMF -> 目标：%s",
                task_id.c_str(), req->target_waypoint.c_str());

    // 发布监控启动消息（原有逻辑保留）
    StringMsg monitor_msg;
    monitor_msg.data = task_id + "," + req->target_waypoint;
    _monitor_start_pub->publish(monitor_msg);
    RCLCPP_INFO(this->get_logger(), "📢 已发送监控启动消息：%s", monitor_msg.data.c_str());

    // 返回结果（恢复原有提示）
    res->success = true;
    res->task_id = task_id;
    res->message = "任务已发送 -> " + req->target_waypoint;
  }

  // 成员变量（移除调度定时器，恢复原有）
  rclcpp::Publisher<ApiRequestMsg>::SharedPtr _task_pub;
  rclcpp::Service<SingleNavSrv>::SharedPtr _nav_srv;
  rclcpp::Publisher<StringMsg>::SharedPtr _monitor_start_pub;
};

// 主函数（原有逻辑保留）
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RMFCustomSingleNavServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
