#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/qos.hpp"

class TestPoseSub : public rclcpp::Node
{
public:
    TestPoseSub() : Node("test_pose_sub")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/drone/pose", qos,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                RCLCPP_INFO(this->get_logger(), "ÊÕµ½ /drone/pose: (%.2f, %.2f, %.2f)",
                            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            });
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestPoseSub>());
    rclcpp::shutdown();
    return 0;
}
