#include "rclcpp/rclcpp.hpp"
#include "image_processing/msg/red_cube_detection.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <vector>
#include <mutex>
#include <chrono>
#include <numeric>
#include <algorithm>
#include <cmath>

// ========================== 目标历史记录结构体 ==========================
struct TargetRecord
{
    geometry_msgs::msg::PointStamped world_point; // 世界系目标位置
    rclcpp::Time timestamp;                       // 记录时间戳
    float confidence;                             // 检测置信度
};

// ========================== 主节点类定义 ==========================
class RedCubeTrackerNode : public rclcpp::Node
{
public:
    RedCubeTrackerNode() : Node("red_cube_tracker_node")
    {
        RCLCPP_INFO(this->get_logger(), "初始化红色立方体跟踪节点...");

        declare_parameters();
        get_parameters();
        init_subscribers();
        init_publishers();
        init_timers();

        RCLCPP_INFO(this->get_logger(), "红色立方体跟踪节点启动成功！");
        RCLCPP_INFO(this->get_logger(),
                    "配置参数: 重复距离阈值=%.2fm, 最小置信度=%.2f, 连续检测次数=%d, 历史记录超时=%.1fs",
                    duplicate_threshold_, conf_threshold_, min_continuous_count_, history_timeout_);
    }

private:
    // ========================== 成员变量 ==========================
    rclcpp::Subscription<image_processing::msg::RedCubeDetection>::SharedPtr det_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;

    rclcpp::TimerBase::SharedPtr history_clean_timer_;
    rclcpp::TimerBase::SharedPtr target_verify_timer_;

    std::vector<TargetRecord> target_history_;
    std::mutex history_mutex_;

    // 基础参数（保持你的命名）
    double duplicate_threshold_; // 基础距离阈值（米）
    float conf_threshold_;       // 误检过滤阈值
    int min_continuous_count_;   // 连续检测次数
    double history_timeout_;     // 历史清理超时（秒）
    std::string world_frame_;
    int target_class_id_;

    // 新增/可调参数（提供合理默认）
    double time_window_sec_; // 判定同一目标的时间窗（秒）
    double conf_influence_;  // 置信度对阈值影响（0~1）
    double ema_alpha_;       // EMA合并系数（0~1）
    int max_history_size_;   // 历史长度上限（防止无限增长）

    // ========================== 初始化函数 ==========================
    void declare_parameters()
    {
        this->declare_parameter<double>("duplicate_distance_threshold", 1.0); // 默认 1m
        this->declare_parameter<float>("confidence_threshold", 0.6);
        this->declare_parameter<int>("min_continuous_detections", 2);
        this->declare_parameter<double>("history_timeout", 3.0);
        this->declare_parameter<std::string>("world_frame_id", "map");
        this->declare_parameter<int>("target_class_id", 0);

        // 新增可调
        this->declare_parameter<double>("time_window_sec", 3.0); // 多帧比对窗口
        this->declare_parameter<double>("conf_influence", 0.6);  // 阈值随置信度变化幅度
        this->declare_parameter<double>("ema_alpha", 0.35);      // 合并的EMA权重
        this->declare_parameter<int>("max_history_size", 100);   // 历史最大条数
    }

    void get_parameters()
    {
        this->get_parameter("duplicate_distance_threshold", duplicate_threshold_);
        this->get_parameter("confidence_threshold", conf_threshold_);
        this->get_parameter("min_continuous_detections", min_continuous_count_);
        this->get_parameter("history_timeout", history_timeout_);
        this->get_parameter("world_frame_id", world_frame_);
        this->get_parameter("target_class_id", target_class_id_);

        this->get_parameter("time_window_sec", time_window_sec_);
        this->get_parameter("conf_influence", conf_influence_);
        this->get_parameter("ema_alpha", ema_alpha_);
        this->get_parameter("max_history_size", max_history_size_);
        if (max_history_size_ < 10)
            max_history_size_ = 10; // 安全下限
        if (ema_alpha_ < 0.0)
            ema_alpha_ = 0.0;
        if (ema_alpha_ > 1.0)
            ema_alpha_ = 1.0;
    }

    void init_subscribers()
    {
        auto qos = rclcpp::QoS(10).reliable();
        det_sub_ = this->create_subscription<image_processing::msg::RedCubeDetection>(
            "/red_cube/detections", qos,
            std::bind(&RedCubeTrackerNode::det_callback, this, std::placeholders::_1));
    }

    void init_publishers()
    {
        target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/red_cube/target_position", rclcpp::QoS(10).reliable());
    }

    void init_timers()
    {
        history_clean_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RedCubeTrackerNode::clean_expired_history, this));

        target_verify_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&RedCubeTrackerNode::verify_and_publish_target, this));
    }

    // ========================== 核心逻辑 ==========================
    void det_callback(const image_processing::msg::RedCubeDetection::SharedPtr det_msg)
    {
        if (det_msg->class_id != target_class_id_ || det_msg->confidence < conf_threshold_)
            return;
        if (std::isnan(det_msg->world_x) || std::isnan(det_msg->world_y))
            return;

        geometry_msgs::msg::PointStamped world_point;
        world_point.header.stamp = det_msg->header.stamp;
        world_point.header.frame_id = world_frame_;
        world_point.point.x = det_msg->world_x;
        world_point.point.y = det_msg->world_y;
        world_point.point.z = det_msg->world_z;

        std::lock_guard<std::mutex> lock(history_mutex_);

        // 改进：多帧 + 时间加权 + 动态阈值；若重复则“合并到最佳记录并移动到队尾”
        if (is_duplicate_and_merge(world_point, det_msg->confidence))
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "合并重复目标: (%.2f, %.2f, %.2f)",
                         world_point.point.x, world_point.point.y, world_point.point.z);
            return;
        }

        // 新目标：直接入队（队尾为最新）
        TargetRecord record;
        record.world_point = world_point;
        record.timestamp = this->get_clock()->now();
        record.confidence = det_msg->confidence;
        target_history_.push_back(record);

        // 控制历史长度
        if (static_cast<int>(target_history_.size()) > max_history_size_)
            target_history_.erase(target_history_.begin(),
                                  target_history_.begin() + (target_history_.size() - max_history_size_));

        RCLCPP_INFO(this->get_logger(),
                    "✅ 新目标记录: (%.2f, %.2f, %.2f), 置信度=%.2f, 历史=%zu",
                    world_point.point.x, world_point.point.y, world_point.point.z,
                    det_msg->confidence, target_history_.size());
    }

    // ========================== 改进的重复检测（并合并） ==========================
    bool is_duplicate_and_merge(const geometry_msgs::msg::PointStamped &new_point, float new_conf)
    {
        if (target_history_.empty())
            return false;

        auto now = this->get_clock()->now();

        // 先找到“最佳匹配”（分数最小）
        int best_idx = -1;
        double best_score = 1e9;

        for (int i = static_cast<int>(target_history_.size()) - 1; i >= 0; --i)
        {
            const auto &rec = target_history_[i];
            double dt = (now - rec.timestamp).seconds();
            if (dt > time_window_sec_)
                continue; // 仅考虑时间窗内的历史

            // 欧氏距离（仅XY）
            double dx = new_point.point.x - rec.world_point.point.x;
            double dy = new_point.point.y - rec.world_point.point.y;
            double dist = std::sqrt(dx * dx + dy * dy);

            // 动态阈值：基础阈值 * (1 + k * avg_conf)
            double avg_conf = 0.5 * (std::clamp<double>(new_conf, 0.0, 1.0) +
                                     std::clamp<double>(rec.confidence, 0.0, 1.0));
            double conf_scale = 1.0 + conf_influence_ * avg_conf;
            double adaptive_thresh = duplicate_threshold_ * conf_scale;

            // 时间加权（越旧越难匹配）：time_weight ∈ (0,1]，使用 1/time_weight 放大距离
            double time_weight = std::exp(-dt / std::max(0.2, time_window_sec_));
            double effective_dist = dist / std::max(0.15, time_weight); // 防止除0，最低0.15

            // 得分：小于1则可视为同一目标；越小越好
            double score = effective_dist / std::max(1e-6, adaptive_thresh);

            if (score < best_score)
            {
                best_score = score;
                best_idx = i;
            }
        }

        // 阈值判断：best_score < 1 认为重复
        if (best_idx >= 0 && best_score < 1.0)
        {
            // 用 EMA 平滑合并，并把该条目“移动到队尾”（表示它是最新）
            TargetRecord merged = target_history_[best_idx];

            // EMA 合并位置
            merged.world_point.point.x = (1.0 - ema_alpha_) * merged.world_point.point.x + ema_alpha_ * new_point.point.x;
            merged.world_point.point.y = (1.0 - ema_alpha_) * merged.world_point.point.y + ema_alpha_ * new_point.point.y;
            merged.world_point.point.z = (1.0 - ema_alpha_) * merged.world_point.point.z + ema_alpha_ * new_point.point.z;

            // 更新时间戳 & 置信度
            merged.timestamp = now;
            merged.confidence = std::max(merged.confidence, new_conf);

            // 替换到队尾：先擦除旧位置，再 push_back（保证“最近有效记录”在末尾）
            target_history_.erase(target_history_.begin() + best_idx);
            target_history_.push_back(merged);

            return true;
        }

        return false;
    }

    // ========================== 连续验证与发布（加权） ==========================
    void verify_and_publish_target()
    {
        std::lock_guard<std::mutex> lock(history_mutex_);
        if (target_history_.empty())
            return;

        auto now = this->get_clock()->now();

        // 仅统计“时间窗 & 置信度合格”的记录
        int count = 0;
        double sum_w = 0.0;
        double wx = 0.0, wy = 0.0, wz = 0.0;

        for (int i = static_cast<int>(target_history_.size()) - 1; i >= 0; --i)
        {
            const auto &rec = target_history_[i];
            double dt = (now - rec.timestamp).seconds();
            if (dt > time_window_sec_)
                break; // 更早的基本不参与

            if (rec.confidence < conf_threshold_)
                continue;

            // 权重：时间衰减 × 置信度（0.5~1.0增强一点）
            double time_w = std::exp(-dt / std::max(0.2, time_window_sec_));
            double conf_w = 0.5 + 0.5 * std::clamp<double>(rec.confidence, 0.0, 1.0);
            double w = time_w * conf_w;

            wx += w * rec.world_point.point.x;
            wy += w * rec.world_point.point.y;
            wz += w * rec.world_point.point.z;
            sum_w += w;
            count++;
        }

        if (count >= min_continuous_count_ && sum_w > 1e-6)
        {
            geometry_msgs::msg::PointStamped final_target;
            final_target.header.stamp = now; // 用本地时间戳，避免时源不一致
            final_target.header.frame_id = world_frame_;
            final_target.point.x = wx / sum_w;
            final_target.point.y = wy / sum_w;
            final_target.point.z = wz / sum_w;

            target_pub_->publish(final_target);

            RCLCPP_INFO(this->get_logger(),
                        "📡 发布目标: (%.2f, %.2f, %.2f), 连续检测=%d",
                        final_target.point.x, final_target.point.y, final_target.point.z,
                        count);
        }
    }

    // ========================== 清理与工具函数 ==========================
    void clean_expired_history()
    {
        std::lock_guard<std::mutex> lock(history_mutex_);
        auto now = this->get_clock()->now();
        size_t old_size = target_history_.size();

        target_history_.erase(
            std::remove_if(target_history_.begin(), target_history_.end(),
                           [&](const TargetRecord &rec)
                           { return (now - rec.timestamp).seconds() > history_timeout_; }),
            target_history_.end());

        if (target_history_.size() < old_size)
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "🧹 清理过期记录: %zu → %zu",
                         old_size, target_history_.size());
        }
    }
};

// ========================== 主函数 ==========================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RedCubeTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
