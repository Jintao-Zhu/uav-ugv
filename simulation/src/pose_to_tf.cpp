#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "rclcpp/qos.hpp"
#include "builtin_interfaces/msg/time.hpp"

class PoseToTFNode : public rclcpp::Node
{
public:
  PoseToTFNode() : Node("pose_to_tf_node")
  {
    // 声明参数
    this->declare_parameter<std::string>("pose_topic", "/drone/pose");
    this->declare_parameter<std::string>("world_frame", "map");
    this->declare_parameter<std::string>("child_frame", "drone_base_link");

    pose_topic_ = this->get_parameter("pose_topic").as_string();
    world_frame_ = this->get_parameter("world_frame").as_string();
    child_frame_ = this->get_parameter("child_frame").as_string();

    // ? 使用 Best Effort QoS，防止订阅不到 MAVROS 的话题
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // 订阅 UAV 位姿
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, qos,
        std::bind(&PoseToTFNode::pose_callback, this, std::placeholders::_1));

    // TF 广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(),
                "? pose_to_tf_node 已启动，订阅 %s，发布 TF: %s → %s",
                pose_topic_.c_str(), world_frame_.c_str(), child_frame_.c_str());
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // 获取当前时间
    rclcpp::Time now = this->get_clock()->now();
    t.header.stamp.sec = static_cast<int32_t>(now.seconds());
    t.header.stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000ULL);

    t.header.frame_id = world_frame_;
    t.child_frame_id = child_frame_;

    t.transform.translation.x = msg->pose.position.x;
    t.transform.translation.y = msg->pose.position.y;
    t.transform.translation.z = msg->pose.position.z;

    t.transform.rotation = msg->pose.orientation;

    tf_broadcaster_->sendTransform(t);
  }

  std::string pose_topic_;
  std::string world_frame_;
  std::string child_frame_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseToTFNode>());
  rclcpp::shutdown();
  return 0;
}
