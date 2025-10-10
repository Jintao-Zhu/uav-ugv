#include "rclcpp/rclcpp.hpp"
#include "image_processing/msg/red_cube_detection.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <vector>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <chrono>

// 订阅 /red_cube/detections 话题，对检测结果进行去重和聚合处理，最终发布更稳定的目标位置信息到新话题

struct DetectionPoint
{
    double x, y, z;
    double confidence;
};

class RedCubeTrackerNode : public rclcpp::Node
{
public:
    RedCubeTrackerNode() : Node("red_cube_tracker_node")
    {
        RCLCPP_INFO(this->get_logger(), "初始化红色立方体跟踪节点...");

        // 参数声明
        this->declare_parameter<double>("merge_distance_threshold", 2.0);
        this->declare_parameter<double>("publish_interval", 1.0);
        this->declare_parameter<std::string>("world_frame_id", "map");
        this->declare_parameter<float>("confidence_threshold", 0.6);
        this->declare_parameter<int>("target_class_id", 0);
        this->declare_parameter<double>("z_min", -3.0);
        this->declare_parameter<double>("z_max", 3.0);

        // 参数读取
        this->get_parameter("merge_distance_threshold", merge_distance_threshold_);
        this->get_parameter("publish_interval", publish_interval_);
        this->get_parameter("world_frame_id", world_frame_);
        this->get_parameter("confidence_threshold", conf_threshold_);
        this->get_parameter("target_class_id", target_class_id_);
        this->get_parameter("z_min", z_min_);
        this->get_parameter("z_max", z_max_);

        // ROS 接口
        det_sub_ = this->create_subscription<image_processing::msg::RedCubeDetection>(
            "/red_cube/detections", rclcpp::QoS(10).reliable(),
            std::bind(&RedCubeTrackerNode::det_callback, this, std::placeholders::_1));

        target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/red_cube/target_position", rclcpp::QoS(10).reliable());

        publish_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(publish_interval_),
            std::bind(&RedCubeTrackerNode::aggregate_and_publish, this));

        RCLCPP_INFO(this->get_logger(),
                    "启动成功，merge_distance_threshold=%.2fm, Z范围[%.2f, %.2f], 发布周期=%.1fs",
                    merge_distance_threshold_, z_min_, z_max_, publish_interval_);
    }

private:
    // ROS 成员
    rclcpp::Subscription<image_processing::msg::RedCubeDetection>::SharedPtr det_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::mutex data_mutex_;

    // 缓存与参数
    std::vector<DetectionPoint> buffer_;
    double merge_distance_threshold_;
    double publish_interval_;
    std::string world_frame_;
    float conf_threshold_;
    int target_class_id_;
    double z_min_, z_max_;

    // ========== 接收检测 ==========
    void det_callback(const image_processing::msg::RedCubeDetection::SharedPtr msg)
    {
        if (msg->class_id != target_class_id_ || msg->confidence < conf_threshold_)
            return;
        if (std::isnan(msg->world_x) || std::isnan(msg->world_y))
            return;
        if (msg->world_z < z_min_ || msg->world_z > z_max_)
            return;

        std::lock_guard<std::mutex> lock(data_mutex_);
        buffer_.push_back({msg->world_x, msg->world_y, msg->world_z, msg->confidence});
        if (buffer_.size() > 200)
            buffer_.erase(buffer_.begin(), buffer_.begin() + 50);
    }

    // ========== 聚类 + 二次合并 + 发布 ==========
    void aggregate_and_publish()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (buffer_.empty())
            return;

        // 第一步：初步分组
        std::vector<std::vector<DetectionPoint>> clusters;
        for (const auto &det : buffer_)
        {
            bool merged = false;
            for (auto &cluster : clusters)
            {
                auto [cx, cy] = compute_cluster_center(cluster);
                double dx = det.x - cx;
                double dy = det.y - cy;
                if (std::sqrt(dx * dx + dy * dy) < merge_distance_threshold_)
                {
                    cluster.push_back(det);
                    merged = true;
                    break;
                }
            }
            if (!merged)
                clusters.push_back({det});
        }

        // 第二步：二次聚类（合并相互接近的簇）
        bool merged_any = true;
        while (merged_any)
        {
            merged_any = false;
            for (size_t i = 0; i < clusters.size(); ++i)
            {
                for (size_t j = i + 1; j < clusters.size(); ++j)
                {
                    auto [cx1, cy1] = compute_cluster_center(clusters[i]);
                    auto [cx2, cy2] = compute_cluster_center(clusters[j]);
                    double dx = cx1 - cx2, dy = cy1 - cy2;
                    if (std::sqrt(dx * dx + dy * dy) < merge_distance_threshold_)
                    {
                        // 合并簇
                        clusters[i].insert(clusters[i].end(), clusters[j].begin(), clusters[j].end());
                        clusters.erase(clusters.begin() + j);
                        merged_any = true;
                        break;
                    }
                }
                if (merged_any)
                    break;
            }
        }

        // 发布每个聚类中心
        for (const auto &cluster : clusters)
        {
            if (cluster.empty())
                continue;

            double wx = 0.0, wy = 0.0, wz = 0.0, total_w = 0.0;
            for (const auto &p : cluster)
            {
                double w = p.confidence;
                wx += w * p.x;
                wy += w * p.y;
                wz += w * p.z;
                total_w += w;
            }
            wx /= total_w;
            wy /= total_w;
            wz /= total_w;

            geometry_msgs::msg::PointStamped msg;
            msg.header.stamp = this->get_clock()->now();
            msg.header.frame_id = world_frame_;
            msg.point.x = wx;
            msg.point.y = wy;
            msg.point.z = wz;

            target_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(),
                        "📡 发布聚类目标: (%.2f, %.2f, %.2f), 聚类成员=%zu",
                        wx, wy, wz, cluster.size());
        }

        buffer_.clear();
    }

    // 工具函数：计算簇中心
    std::pair<double, double> compute_cluster_center(const std::vector<DetectionPoint> &cluster)
    {
        double cx = 0.0, cy = 0.0;
        for (const auto &p : cluster)
        {
            cx += p.x;
            cy += p.y;
        }
        cx /= cluster.size();
        cy /= cluster.size();
        return {cx, cy};
    }
};

// ========== 主函数 ==========
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RedCubeTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
