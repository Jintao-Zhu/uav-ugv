#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <mutex>
#include <cmath>
#include <chrono>
// 接收红色立方体的目标点坐标，输出机器人导航的目标位姿
class NavigationToRedCubeNode : public rclcpp::Node
{
public:
    NavigationToRedCubeNode()
        : Node("navigation_to_red_cube_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          last_pub_time_(this->now() - rclcpp::Duration::from_seconds(10.0)),
          has_last_pose_(false)
    {
        RCLCPP_INFO(this->get_logger(), "启动 navigation_to_red_cube_node（任务生成层）：订阅目标点 -> 发布任务位姿");

        // ---------------- 参数声明 ----------------
        this->declare_parameter<std::string>("input_topic", "/red_cube/target_position");
        this->declare_parameter<std::string>("output_topic", "/navigation/cube_goal");
        this->declare_parameter<std::string>("global_frame_id", "map");
        this->declare_parameter<std::string>("robot_frame_id", "ugv_base_link"); // ✅ 修改为你的实际TF
        this->declare_parameter<double>("approach_distance", 0.5);               // 前方停靠距离
        this->declare_parameter<double>("min_goal_separation", 0.5);             // 与上次任务的最小间隔
        this->declare_parameter<double>("publish_interval", 0.5);                // 最小发布周期
        this->declare_parameter<double>("near_margin", 0.2);                     // 过近时安全边界

        // ---------------- 参数读取 ----------------
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        global_frame_id_ = this->get_parameter("global_frame_id").as_string();
        robot_frame_id_ = this->get_parameter("robot_frame_id").as_string();
        approach_distance_ = this->get_parameter("approach_distance").as_double();
        min_goal_separation_ = this->get_parameter("min_goal_separation").as_double();
        publish_interval_ = this->get_parameter("publish_interval").as_double();
        near_margin_ = this->get_parameter("near_margin").as_double();

        // 参数边界检查
        if (approach_distance_ < 0.0)
            approach_distance_ = 0.0;
        if (min_goal_separation_ < 0.0)
            min_goal_separation_ = 0.0;
        if (publish_interval_ < 0.05)
            publish_interval_ = 0.05;
        if (near_margin_ < 0.0)
            near_margin_ = 0.0;

        // ---------------- ROS I/O ----------------
        sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            input_topic_, rclcpp::QoS(10).reliable(),
            std::bind(&NavigationToRedCubeNode::onTargetPoint, this, std::placeholders::_1));

        pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            output_topic_, rclcpp::QoS(10).reliable());

        RCLCPP_INFO(this->get_logger(),
                    "参数：input=%s, output=%s, frames=[%s <- %s], "
                    "approach=%.2fm, sep=%.2fm, interval=%.2fs",
                    input_topic_.c_str(), output_topic_.c_str(),
                    global_frame_id_.c_str(), robot_frame_id_.c_str(),
                    approach_distance_, min_goal_separation_, publish_interval_);
    }

private:
    // ---------------- 成员变量 ----------------
    std::string input_topic_, output_topic_;
    std::string global_frame_id_, robot_frame_id_;
    double approach_distance_;
    double min_goal_separation_;
    double publish_interval_;
    double near_margin_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;

    rclcpp::Time last_pub_time_;
    geometry_msgs::msg::Pose last_pub_pose_;
    bool has_last_pose_;
    std::mutex mtx_;

    // ---------------- 工具函数 ----------------
    static double distanceXY(const geometry_msgs::msg::Point &a,
                             const geometry_msgs::msg::Point &b)
    {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        return std::hypot(dx, dy);
    }

    bool lookupRobotPose(geometry_msgs::msg::Pose &pose_out)
    {
        try
        {
            geometry_msgs::msg::TransformStamped tf =
                tf_buffer_.lookupTransform(global_frame_id_, robot_frame_id_, tf2::TimePointZero);

            pose_out.position.x = tf.transform.translation.x;
            pose_out.position.y = tf.transform.translation.y;
            pose_out.position.z = tf.transform.translation.z;
            pose_out.orientation = tf.transform.rotation;
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000 /*ms*/,
                                 "TF 获取失败 [%s->%s]: %s",
                                 global_frame_id_.c_str(), robot_frame_id_.c_str(), ex.what());
            return false;
        }
    }

    // ---------------- 核心逻辑 ----------------
    void onTargetPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (msg->header.frame_id != global_frame_id_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "收到的目标点 frame_id=%s != %s（忽略）",
                                 msg->header.frame_id.c_str(), global_frame_id_.c_str());
            return;
        }

        std::lock_guard<std::mutex> lock(mtx_);
        const rclcpp::Time now = this->now();

        // 限频
        if ((now - last_pub_time_).seconds() < publish_interval_)
            return;

        geometry_msgs::msg::Pose robot_pose;
        const bool have_robot = lookupRobotPose(robot_pose);

        // 计算目标与小车相对位置
        const double tx = msg->point.x;
        const double ty = msg->point.y;
        double rx = 0.0, ry = 0.0;

        if (have_robot)
        {
            rx = robot_pose.position.x;
            ry = robot_pose.position.y;
        }

        const double vx = tx - rx;
        const double vy = ty - ry;
        const double dist = std::hypot(vx, vy);

        // 计算朝向
        double yaw = 0.0;
        if (have_robot && dist > 1e-6)
            yaw = std::atan2(vy, vx);

        // 计算停靠点偏移
        double use_offset = approach_distance_;
        if (have_robot)
        {
            if (dist <= near_margin_)
                use_offset = 0.0;
            else if (dist <= approach_distance_ + near_margin_)
                use_offset = std::max(0.0, dist - near_margin_);
        }
        else
        {
            use_offset = 0.0;
        }

        double px = tx;
        double py = ty;
        if (have_robot && use_offset > 1e-6 && dist > 1e-6)
        {
            const double ux = vx / dist;
            const double uy = vy / dist;
            px = tx - ux * use_offset;
            py = ty - uy * use_offset;
        }

        geometry_msgs::msg::PoseStamped pose_out;
        pose_out.header.stamp = now;
        pose_out.header.frame_id = global_frame_id_;
        pose_out.pose.position.x = px;
        pose_out.pose.position.y = py;
        pose_out.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose_out.pose.orientation = tf2::toMsg(q);

        // 防抖：与上次任务相距太近则忽略
        if (has_last_pose_)
        {
            if (distanceXY(pose_out.pose.position, last_pub_pose_.position) < min_goal_separation_)
                return;
        }

        // 发布
        pub_->publish(pose_out);
        last_pub_pose_ = pose_out.pose;
        last_pub_time_ = now;
        has_last_pose_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "🧭 任务生成：停靠点(%.2f, %.2f), yaw=%.1f°, "
                    "原始目标(%.2f, %.2f), 偏移=%.2fm",
                    px, py, yaw * 180.0 / M_PI, tx, ty, use_offset);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationToRedCubeNode>());
    rclcpp::shutdown();
    return 0;
}
