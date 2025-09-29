#include <rclcpp/rclcpp.hpp> // 最初的避障节点
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <chrono>  // 确保包含chrono头文件

using namespace std::chrono_literals;

class ObstacleAvoidanceNode : public rclcpp::Node
{
public:
    // 关键修复：使用成员初始化列表初始化Duration
    ObstacleAvoidanceNode() : Node("obstacle_avoidance_node"),
        follow_cmd_timeout_(std::chrono::milliseconds(500))
    {
        // 声明参数
        this->declare_parameter("obstacle_distance_threshold", 1.5);
        this->declare_parameter("critical_distance", 0.8);
        this->declare_parameter("safety_distance", 1.2);
        this->declare_parameter("max_linear_vel", 1.0);
        this->declare_parameter("max_angular_vel", 2.0);
        this->declare_parameter("vfh_sector_angle", 5.0);
        this->declare_parameter("wide_valley_threshold", 30.0);
        this->declare_parameter("obstacle_certainty_threshold", 3);
        this->declare_parameter("enable_visualization", true);
        this->declare_parameter("vehicle_namespace", "yahboomcar");

        // 获取车辆命名空间
        std::string vehicle_ns = this->get_parameter("vehicle_namespace").as_string();

        // 初始化订阅器
        std::string follow_cmd_topic = "/" + vehicle_ns + "/follow_cmd_vel";
        follow_cmd_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            follow_cmd_topic,
            rclcpp::QoS(10).reliable().durability_volatile(),
            std::bind(&ObstacleAvoidanceNode::follow_cmd_callback, this, std::placeholders::_1));

        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/yahboomcar/scan",
            10,
            std::bind(&ObstacleAvoidanceNode::laser_callback, this, std::placeholders::_1));

        // 初始化发布器
        std::string cmd_vel_topic = "/" + vehicle_ns + "/cmd_vel";
        safe_cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
        
        obstacle_detected_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/obstacle_avoidance/obstacle_detected", 10);
        min_distance_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/obstacle_avoidance/min_distance", 10);

        if (this->get_parameter("enable_visualization").as_bool())
        {
            marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/obstacle_avoidance/markers", 10);
        }

        // 定时器
        timer_ = this->create_wall_timer(
            50ms,
            std::bind(&ObstacleAvoidanceNode::timer_callback, this));

        // 初始化状态变量
        obstacle_detected_ = false;
        min_obstacle_distance_ = std::numeric_limits<float>::max();
        laser_data_received_ = false;
        follow_cmd_received_ = false;
        last_laser_time_ = this->get_clock()->now();
        // 注意：follow_cmd_timeout_已在成员初始化列表中初始化，此处无需重复

        // 打印初始化信息
        RCLCPP_INFO(this->get_logger(), "🛡️ 避障节点初始化完成");
        RCLCPP_INFO(this->get_logger(), "车辆命名空间: %s", vehicle_ns.c_str());
        RCLCPP_INFO(this->get_logger(), "订阅跟随指令话题: %s", follow_cmd_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "发布控制指令话题: %s", cmd_vel_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "参数设置:");
        RCLCPP_INFO(this->get_logger(), "  障碍物检测距离: %.2f m", this->get_parameter("obstacle_distance_threshold").as_double());
        RCLCPP_INFO(this->get_logger(), "  紧急停止距离: %.2f m", this->get_parameter("critical_distance").as_double());
        RCLCPP_INFO(this->get_logger(), "  安全距离: %.2f m", this->get_parameter("safety_distance").as_double());
        RCLCPP_INFO(this->get_logger(), "  跟随指令超时时间: %ld ms", follow_cmd_timeout_.nanoseconds() / 1000000);
    }

private:
    // 订阅器/发布器/定时器
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr follow_cmd_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_detected_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr min_distance_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 状态变量
    geometry_msgs::msg::Twist latest_follow_cmd_;
    sensor_msgs::msg::LaserScan latest_scan_;
    bool follow_cmd_received_;
    bool laser_data_received_;
    bool obstacle_detected_;
    float min_obstacle_distance_;
    rclcpp::Time last_laser_time_;
    rclcpp::Time last_follow_cmd_time_;
    rclcpp::Duration follow_cmd_timeout_;  // 仅声明，不在此处初始化
    std::vector<float> filtered_ranges_;
    std::vector<int> histogram_;

    // VFH谷值结构体
    struct Valley
    {
        int start_idx;
        int end_idx;
        int width;
        float center_angle;
    };

    // 激光雷达回调
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_scan_ = *msg;
        laser_data_received_ = true;
        last_laser_time_ = this->get_clock()->now();

        preprocess_laser_data();
        build_vfh_histogram();
        detect_obstacles();
    }

    // 跟随指令回调
    void follow_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        latest_follow_cmd_ = *msg;
        follow_cmd_received_ = true;
        last_follow_cmd_time_ = this->get_clock()->now();
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "接收跟随指令：线速度=%.2f m/s, 角速度=%.2f rad/s",
                             msg->linear.x, msg->angular.z);
    }

    // 定时器回调（核心控制逻辑）
    void timer_callback()
    {
        // 激光数据超时检查
        if (!laser_data_received_ || (this->get_clock()->now() - last_laser_time_).seconds() > 1.0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "⚠️ 激光数据无效（未接收/超时）！停止车辆");
            publish_stop_cmd();
            return;
        }

        // 跟随指令超时检查
        if (!follow_cmd_received_ || (this->get_clock()->now() - last_follow_cmd_time_) > follow_cmd_timeout_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "⚠️ 跟随指令超时或未接收！停止车辆");
            publish_stop_cmd();
            return;
        }

        // 读取参数
        const double critical_dist = this->get_parameter("critical_distance").as_double();
        const double safety_dist = this->get_parameter("safety_distance").as_double();
        const double max_lin_vel = this->get_parameter("max_linear_vel").as_double();
        const double max_ang_vel = this->get_parameter("max_angular_vel").as_double();

        // 初始化最终指令
        geometry_msgs::msg::Twist final_cmd = latest_follow_cmd_;

        // 障碍物处理
        if (obstacle_detected_)
        {
            if (min_obstacle_distance_ < critical_dist)
            {
                // 紧急后退 - 忽略跟随指令
                final_cmd.linear.x = -0.3;
                final_cmd.angular.z = 0.0;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                     "🛑 紧急后退！障碍物距离=%.2f m", min_obstacle_distance_);
            }
            else
            {
                // 计算目标方向（基于跟随指令的角速度）
                float target_angle = 0.0;
                if (std::abs(latest_follow_cmd_.angular.z) > 0.1)
                {
                    target_angle = latest_follow_cmd_.angular.z > 0 ? 30.0 : -30.0;
                }

                // VFH避障逻辑
                std::vector<Valley> valleys = find_valleys();
                float best_angle = select_best_direction(valleys, target_angle);
                float angle_error = normalize_angle_deg(best_angle);

                // 调整角速度 - 融合跟随指令和避障需求
                final_cmd.angular.z = -angle_error * M_PI / 180.0 * 2.0;
                final_cmd.angular.z = std::clamp(final_cmd.angular.z, -max_ang_vel, max_ang_vel);

                // 调整线速度 - 根据障碍物距离动态调整
                float speed_factor = (min_obstacle_distance_ - critical_dist) / (safety_dist - critical_dist);
                speed_factor = std::clamp(speed_factor, 0.1f, 1.0f);
                final_cmd.linear.x = std::min(latest_follow_cmd_.linear.x * speed_factor, max_lin_vel);

                // 大角度转向降速
                if (std::abs(angle_error) > 30.0)
                {
                    final_cmd.linear.x *= 0.5;
                }

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "🔄 避障融合：线速度=%.2f, 角速度=%.2f, 避障角度=%.1f°",
                                     final_cmd.linear.x, final_cmd.angular.z, best_angle);
            }
        }
        // 无障碍物时，直接使用跟随指令但限制最大速度
        else
        {
            final_cmd.linear.x = std::clamp(final_cmd.linear.x, -max_lin_vel, max_lin_vel);
            final_cmd.angular.z = std::clamp(final_cmd.angular.z, -max_ang_vel, max_ang_vel);
        }

        // 发布最终指令
        safe_cmd_publisher_->publish(final_cmd);
    }

    // 激光数据预处理
    void preprocess_laser_data()
    {
        filtered_ranges_.clear();
        filtered_ranges_.resize(latest_scan_.ranges.size());

        for (size_t i = 0; i < latest_scan_.ranges.size(); ++i)
        {
            const float range = latest_scan_.ranges[i];
            if (std::isnan(range) || std::isinf(range) || range < latest_scan_.range_min || range > latest_scan_.range_max)
            {
                filtered_ranges_[i] = latest_scan_.range_max;
            }
            else
            {
                filtered_ranges_[i] = range;
            }
        }
    }

    // 构建VFH直方图
    void build_vfh_histogram()
    {
        const double sector_angle = this->get_parameter("vfh_sector_angle").as_double();
        const int num_sectors = static_cast<int>(360.0 / sector_angle);
        const double obs_threshold = this->get_parameter("obstacle_distance_threshold").as_double();
        const int certainty_thresh = this->get_parameter("obstacle_certainty_threshold").as_int();

        histogram_.clear();
        histogram_.resize(num_sectors, 0);

        for (size_t i = 0; i < filtered_ranges_.size(); ++i)
        {
            if (filtered_ranges_[i] >= obs_threshold)
                continue;

            float angle_rad = latest_scan_.angle_min + i * latest_scan_.angle_increment;
            float angle_deg = angle_rad * 180.0 / M_PI;
            while (angle_deg < 0) angle_deg += 360.0;
            while (angle_deg >= 360.0) angle_deg -= 360.0;

            int sector_idx = static_cast<int>(angle_deg / sector_angle);
            sector_idx = std::clamp(sector_idx, 0, num_sectors - 1);

            int certainty = static_cast<int>((obs_threshold - filtered_ranges_[i]) / obs_threshold * 10);
            histogram_[sector_idx] += certainty;
        }

        for (int &val : histogram_)
        {
            val = (val >= certainty_thresh) ? 1 : 0;
        }
    }

    // 检测障碍物
    void detect_obstacles()
    {
        const double obs_threshold = this->get_parameter("obstacle_distance_threshold").as_double();
        min_obstacle_distance_ = std::numeric_limits<float>::max();
        obstacle_detected_ = false;

        // 定义前方区域（激光雷达数据的25%~75%，即前方180°）
        const int front_start = static_cast<int>(filtered_ranges_.size() * 0.25);
        const int front_end = static_cast<int>(filtered_ranges_.size() * 0.75);

        for (int i = front_start; i < front_end; ++i)
        {
            if (filtered_ranges_[i] < obs_threshold && filtered_ranges_[i] < min_obstacle_distance_)
            {
                obstacle_detected_ = true;
                min_obstacle_distance_ = filtered_ranges_[i];
            }
        }

        // 发布障碍物状态
        std_msgs::msg::Bool obs_msg;
        obs_msg.data = obstacle_detected_;
        obstacle_detected_publisher_->publish(obs_msg);

        // 发布最近距离
        std_msgs::msg::Float32 dist_msg;
        dist_msg.data = (min_obstacle_distance_ == std::numeric_limits<float>::max()) ? 
                        latest_scan_.range_max : min_obstacle_distance_;
        min_distance_publisher_->publish(dist_msg);

        // 可视化
        if (this->get_parameter("enable_visualization").as_bool() && marker_publisher_)
        {
            visualize_obstacles();
        }
    }

    // 查找安全谷值
    std::vector<Valley> find_valleys()
    {
        std::vector<Valley> valleys;
        bool in_valley = false;
        Valley current_valley;
        const double sector_angle = this->get_parameter("vfh_sector_angle").as_double();
        const int num_sectors = histogram_.size();

        for (int i = 0; i < num_sectors; ++i)
        {
            if (histogram_[i] == 0)
            {
                if (!in_valley)
                {
                    in_valley = true;
                    current_valley.start_idx = i;
                }
            }
            else
            {
                if (in_valley)
                {
                    in_valley = false;
                    current_valley.end_idx = i - 1;
                    current_valley.width = current_valley.end_idx - current_valley.start_idx + 1;
                    current_valley.center_angle = (current_valley.start_idx + current_valley.end_idx) * 0.5 * sector_angle;
                    valleys.push_back(current_valley);
                }
            }
        }

        // 处理环形边界
        if (in_valley)
        {
            current_valley.end_idx = num_sectors - 1;
            current_valley.width = current_valley.end_idx - current_valley.start_idx + 1;
            current_valley.center_angle = (current_valley.start_idx + current_valley.end_idx) * 0.5 * sector_angle;
            valleys.push_back(current_valley);
        }

        return valleys;
    }

    // 选择最佳避障方向
    float select_best_direction(const std::vector<Valley> &valleys, float target_angle)
    {
        if (valleys.empty())
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ 无安全谷值！默认向前");
            return 0.0;
        }

        const double wide_thresh = this->get_parameter("wide_valley_threshold").as_double();
        const double sector_angle = this->get_parameter("vfh_sector_angle").as_double();
        std::vector<Valley> wide_valleys;

        for (const auto &valley : valleys)
        {
            const double valley_width_deg = valley.width * sector_angle;
            if (valley_width_deg >= wide_thresh)
            {
                wide_valleys.push_back(valley);
            }
        }

        const auto &target_valleys = wide_valleys.empty() ? valleys : wide_valleys;

        float best_angle = target_valleys[0].center_angle;
        float min_angle_diff = std::abs(normalize_angle_deg(target_valleys[0].center_angle - target_angle));

        for (const auto &valley : target_valleys)
        {
            const float angle_diff = std::abs(normalize_angle_deg(valley.center_angle - target_angle));
            if (angle_diff < min_angle_diff)
            {
                min_angle_diff = angle_diff;
                best_angle = valley.center_angle;
            }
        }

        return best_angle;
    }

    // 角度归一化
    float normalize_angle_deg(float angle)
    {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    // 障碍物可视化
    void visualize_obstacles()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "obstacles";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.color.a = 1.0;

        const double obs_threshold = this->get_parameter("obstacle_distance_threshold").as_double();

        for (size_t i = 0; i < filtered_ranges_.size(); ++i)
        {
            if (filtered_ranges_[i] >= obs_threshold)
                continue;

            const float angle_rad = latest_scan_.angle_min + i * latest_scan_.angle_increment;
            geometry_msgs::msg::Point p;
            p.x = filtered_ranges_[i] * cos(angle_rad);
            p.y = filtered_ranges_[i] * sin(angle_rad);
            p.z = 0.0;
            marker.points.push_back(p);

            std_msgs::msg::ColorRGBA color;
            const float ratio = filtered_ranges_[i] / obs_threshold;
            color.r = 1.0 - ratio;
            color.g = ratio;
            color.b = 0.0;
            color.a = 1.0;
            marker.colors.push_back(color);
        }

        if (!marker.points.empty())
        {
            marker_publisher_->publish(marker);
        }
    }

    // 发布停止指令
    void publish_stop_cmd()
    {
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        safe_cmd_publisher_->publish(stop_cmd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleAvoidanceNode>();
    RCLCPP_INFO(rclcpp::get_logger("main"), "🚀 避障节点启动成功！");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
