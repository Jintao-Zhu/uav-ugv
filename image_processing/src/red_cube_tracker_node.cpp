#include "rclcpp/rclcpp.hpp"
#include "image_processing/msg/red_cube_detection.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/LinearMath/Vector3.h"
#include <vector>
#include <mutex>
#include <chrono>
#include <numeric>
#include <algorithm>

// 目标历史记录结构体（用于重复检测过滤和时间连续性验证）
struct TargetRecord {
    geometry_msgs::msg::PointStamped world_point;  // 世界系目标位置
    rclcpp::Time timestamp;                        // 记录时间戳
    float confidence;                              // 检测置信度
};

class RedCubeTrackerNode : public rclcpp::Node
{
public:
    RedCubeTrackerNode() : Node("red_cube_tracker_node")
    {
        RCLCPP_INFO(this->get_logger(), "初始化红色立方体跟踪节点...");
        
        // 1. 声明并获取可配置参数（灵活适配不同场景）
        declare_parameters();
        get_parameters();

        // 2. 初始化订阅器（订阅 detector 输出的原始检测结果）
        init_subscribers();

        // 3. 初始化发布器（发布过滤后的最终目标位置）
        init_publishers();

        // 4. 初始化定时器（定期清理过期历史记录、验证目标连续性）
        init_timers();

        RCLCPP_INFO(this->get_logger(), "红色立方体跟踪节点启动成功！");
        RCLCPP_INFO(this->get_logger(), "配置参数: 重复距离阈值=%.2fm, 最小置信度=%.2f, 连续检测次数=%d, 历史记录超时=%.1fs",
                    duplicate_threshold_, conf_threshold_, min_continuous_count_, history_timeout_);
    }

private:
    // ========================== 成员变量 ==========================
    // 订阅器/发布器
    rclcpp::Subscription<image_processing::msg::RedCubeDetection>::SharedPtr det_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr history_clean_timer_;  // 清理过期历史记录
    rclcpp::TimerBase::SharedPtr target_verify_timer_;  // 验证目标连续性并发布

    // 目标历史记录（线程安全访问）
    std::vector<TargetRecord> target_history_;
    std::mutex history_mutex_;

    // 可配置参数
    double duplicate_threshold_;    // 重复检测距离阈值（米）
    float conf_threshold_;          // 误检过滤：最小置信度阈值
    int min_continuous_count_;      // 误检过滤：最小连续检测次数
    double history_timeout_;        // 历史记录超时时间（秒）
    std::string world_frame_;       // 世界坐标系ID（与 detector 保持一致）
    int target_class_id_;           // 目标类别ID（默认0，与 detector 配置匹配）

    // ========================== 初始化函数 ==========================
    void declare_parameters()
    {
        // 重复检测过滤参数
        this->declare_parameter<double>("duplicate_distance_threshold", 0.5);  // 0.5米内视为同一目标
        // 误检过滤参数
        this->declare_parameter<float>("confidence_threshold", 0.6);           // 过滤置信度<0.6的低信度结果
        this->declare_parameter<int>("min_continuous_detections", 2);         // 至少连续检测2次才视为有效目标
        // 历史记录管理参数
        this->declare_parameter<double>("history_timeout", 3.0);              // 3秒前的记录视为过期
        // 坐标系与目标类别参数
        this->declare_parameter<std::string>("world_frame_id", "map");        // 与 detector 的 world_frame 一致
        this->declare_parameter<int>("target_class_id", 0);                   // 与 detector 的 target_class_id 一致
    }

    void get_parameters()
    {
        this->get_parameter("duplicate_distance_threshold", duplicate_threshold_);
        this->get_parameter("confidence_threshold", conf_threshold_);
        this->get_parameter("min_continuous_detections", min_continuous_count_);
        this->get_parameter("history_timeout", history_timeout_);
        this->get_parameter("world_frame_id", world_frame_);
        this->get_parameter("target_class_id", target_class_id_);
    }

    void init_subscribers()
    {
        // QoS 配置：可靠传输（确保检测结果不丢失），队列大小10
        auto qos = rclcpp::QoS(10).reliable();
        
        det_sub_ = this->create_subscription<image_processing::msg::RedCubeDetection>(
            "/red_cube/detections",  // 订阅 detector 发布的原始检测话题
            qos,
            std::bind(&RedCubeTrackerNode::det_callback, this, std::placeholders::_1)
        );
    }

    void init_publishers()
    {
        // 发布过滤后的最终目标位置（话题与需求一致）
        target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/red_cube/target_position",  // 需求指定的输出话题
            rclcpp::QoS(10).reliable()   // 可靠传输，确保后续节点能接收
        );
    }

    void init_timers()
    {
        // 1秒清理一次过期历史记录
        history_clean_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RedCubeTrackerNode::clean_expired_history, this)
        );

        // 500ms验证一次目标连续性并发布（与 detector 检测频率匹配）
        target_verify_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&RedCubeTrackerNode::verify_and_publish_target, this)
        );
    }

    // ========================== 核心逻辑函数 ==========================
    /**
     * @brief 订阅 detector 原始检测结果的回调函数
     * 功能：1. 过滤低置信度误检 2. 过滤非目标类别 3. 过滤重复检测 4. 记录有效目标到历史
     */
    void det_callback(const image_processing::msg::RedCubeDetection::SharedPtr det_msg)
    {
        // 1. 过滤非目标类别（仅保留配置的 target_class_id）
        if (det_msg->class_id != target_class_id_)
        {
            RCLCPP_DEBUG(this->get_logger(), "跳过非目标类别: %s (ID=%d)", 
                        det_msg->class_name.c_str(), det_msg->class_id);
            return;
        }

        // 2. 过滤低置信度误检（低于阈值的结果视为不可靠）
        if (det_msg->confidence < conf_threshold_)
        {
            RCLCPP_DEBUG(this->get_logger(), "过滤低置信度结果: 置信度=%.3f < 阈值=%.2f",
                        det_msg->confidence, conf_threshold_);
            return;
        }

        // 3. 过滤无效世界坐标（detector 变换失败的结果）
        if (std::isnan(det_msg->world_x) || std::isnan(det_msg->world_y))
        {
            RCLCPP_WARN(this->get_logger(), "跳过无效世界坐标（NaN），TF变换可能失败");
            return;
        }

        // 4. 构造世界系目标点（与 detector 的坐标系保持一致）
        geometry_msgs::msg::PointStamped world_point;
        world_point.header.stamp = det_msg->header.stamp;
        world_point.header.frame_id = world_frame_;
        world_point.point.x = det_msg->world_x;
        world_point.point.y = det_msg->world_y;
        world_point.point.z = det_msg->world_z;  // 地面目标，Z应接近0

        // 5. 过滤重复检测（与历史记录对比，距离过近视为同一目标）
        std::lock_guard<std::mutex> lock(history_mutex_);
        if (is_duplicate(world_point))
        {
            RCLCPP_DEBUG(this->get_logger(), "过滤重复检测: 与历史目标距离=%.2fm < 阈值=%.2fm",
                        calculate_distance(world_point.point, target_history_.back().world_point.point),
                        duplicate_threshold_);
            return;
        }

        // 6. 记录有效目标到历史队列
        TargetRecord record;
        record.world_point = world_point;
        record.timestamp = this->get_clock()->now();
        record.confidence = det_msg->confidence;
        target_history_.push_back(record);

        RCLCPP_DEBUG(this->get_logger(), "添加有效目标到历史: 位置(%.2f,%.2f,%.2f), 置信度=%.3f, 历史记录数=%zu",
                    world_point.point.x, world_point.point.y, world_point.point.z,
                    det_msg->confidence, target_history_.size());
    }

    /**
     * @brief 验证目标连续性并发布最终结果
     * 逻辑：仅当历史记录中连续有效目标数 ≥ 配置阈值时，才发布目标（过滤瞬时误检）
     */
    void verify_and_publish_target()
    {
        std::lock_guard<std::mutex> lock(history_mutex_);

        // 1. 无历史记录，直接返回
        if (target_history_.empty())
        {
            RCLCPP_DEBUG(this->get_logger(), "无历史目标记录，跳过发布");
            return;
        }

        // 2. 统计最近的连续有效目标数（按时间倒序检查）
        int continuous_count = 0;
        std::vector<geometry_msgs::msg::Point> recent_points;  // 用于计算平均位置（减少噪声）
        auto now = this->get_clock()->now();

        for (auto it = target_history_.rbegin(); it != target_history_.rend(); ++it)
        {
            // 仅考虑未过期、高置信度的记录
            double time_diff = (now - it->timestamp).seconds();
            if (time_diff > history_timeout_ || it->confidence < conf_threshold_)
            {
                break;  // 时间过期或置信度不足，停止统计
            }

            continuous_count++;
            recent_points.push_back(it->world_point.point);
        }

        // 3. 连续检测数达标，发布平均目标位置（减少噪声影响）
        if (continuous_count >= min_continuous_count_)
        {
            auto final_target = calculate_average_point(recent_points);
            final_target.header.stamp = this->get_clock()->now();
            final_target.header.frame_id = world_frame_;

            target_pub_->publish(final_target);
            RCLCPP_INFO(this->get_logger(), "发布最终目标位置: 世界系(%.2f,%.2f,%.2f), 连续检测数=%d, 基于最近%zu个记录的平均值",
                        final_target.point.x, final_target.point.y, final_target.point.z,
                        continuous_count, recent_points.size());  // %d → %zu

        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "连续目标数不足（%d/%d），跳过发布",
                        continuous_count, min_continuous_count_);
        }
    }

    /**
     * @brief 清理过期的历史记录（避免内存泄漏和过时数据干扰）
     */
    void clean_expired_history()
    {
        std::lock_guard<std::mutex> lock(history_mutex_);
        auto now = this->get_clock()->now();
        size_t old_size = target_history_.size();

        // 移除超时的记录（按时间戳过滤）
        target_history_.erase(
            std::remove_if(target_history_.begin(), target_history_.end(),
                [this, now](const TargetRecord& rec) {
                    return (now - rec.timestamp).seconds() > history_timeout_;
                }),
            target_history_.end()
        );

        // 打印清理日志（仅当有记录被清理时）
        if (target_history_.size() < old_size)
        {
            RCLCPP_DEBUG(this->get_logger(), "清理过期历史记录: 原大小=%zu → 新大小=%zu",
                        old_size, target_history_.size());
        }
    }

    // ========================== 工具函数 ==========================
    /**
     * @brief 判断当前目标是否与历史记录中的目标重复
     */
    bool is_duplicate(const geometry_msgs::msg::PointStamped& new_point)
    {
        // 无历史记录，直接视为非重复
        if (target_history_.empty())
            return false;

        // 与最近的历史目标计算距离
        auto& latest = target_history_.back();
        double distance = calculate_distance(new_point.point, latest.world_point.point);

        // 距离 < 阈值 → 视为重复
        return distance < duplicate_threshold_;
    }

    /**
     * @brief 计算两个三维点在XY平面的欧氏距离（忽略Z轴，地面目标高度无意义）
     */
    double calculate_distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    /**
     * @brief 计算多个点的平均位置（减少噪声，提高目标稳定性）
     */
    geometry_msgs::msg::PointStamped calculate_average_point(const std::vector<geometry_msgs::msg::Point>& points)
    {
        geometry_msgs::msg::PointStamped avg_point;
        avg_point.point.x = std::accumulate(points.begin(), points.end(), 0.0,
            [](double sum, const geometry_msgs::msg::Point& p) { return sum + p.x; }) / points.size();
        avg_point.point.y = std::accumulate(points.begin(), points.end(), 0.0,
            [](double sum, const geometry_msgs::msg::Point& p) { return sum + p.y; }) / points.size();
        avg_point.point.z = std::accumulate(points.begin(), points.end(), 0.0,
            [](double sum, const geometry_msgs::msg::Point& p) { return sum + p.z; }) / points.size();

        return avg_point;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RedCubeTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
