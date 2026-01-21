#include "rclcpp/rclcpp.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "onnxruntime_cxx_api.h"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "image_processing/msg/red_cube_detection.hpp"

// TF2相关头文件
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Transform.h"

#include <filesystem>
#include <string>
#include <vector>
#include <memory>
#include <unordered_set>
#include <chrono>
#include <fstream>
#include <mutex>
#include <Eigen/Dense>
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <cmath>
#include <limits>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace fs = std::filesystem;
using namespace std::chrono_literals;

class RedCubeDetectorNode : public rclcpp::Node
{
public:
    // 航点结构体（真实坐标）
    struct Waypoint
    {
        std::string name;
        float x;
        float y;
        float z;

        Waypoint(const std::string& n, float x_val, float y_val, float z_val)
            : name(n), x(x_val), y(y_val), z(z_val) {}
    };
    struct RealWorldWaypoint
    {
        std::string name;
        float x;
        float y;
        float z;

        RealWorldWaypoint(const std::string &n, float px, float py, float pz)
            : name(n), x(px), y(py), z(pz) {}
        
        // 计算到目标点的欧几里得距离
        float distance_to(float target_x, float target_y, float target_z) const
        {
            float dx = x - target_x;
            float dy = y - target_y;
            float dz = z - target_z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }
    };
    // 检测结果结构体
    struct Detection
    {
        cv::Rect box;
        float confidence;
        int class_id;
        std::string class_name;

        float adapted_fx;
        float adapted_fy;
        float adapted_cx;
        float adapted_cy;

        Detection() = default;
        Detection(const cv::Rect &b, float conf, int id, const std::string &name)
            : box(b), confidence(conf), class_id(id), class_name(name),
              adapted_fx(0.0f), adapted_fy(0.0f), adapted_cx(0.0f), adapted_cy(0.0f) {}
    };

    // 位姿数据结构体
    struct PoseData
    {
        std::string timestamp;
        float pos_x = 0.0f, pos_y = 0.0f, pos_z = 0.0f;
        float ori_x = 0.0f, ori_y = 0.0f, ori_z = 0.0f, ori_w = 1.0f;
        int target_x = 0, target_y = 0;
        int64_t ros_timestamp_sec = 0;
        int64_t ros_timestamp_nsec = 0;

        PoseData() = default;
    };

    // 相机内参结构体
    struct CameraIntrinsics
    {
        float fx, fy, cx, cy;

        CameraIntrinsics(float fx_val, float fy_val, float cx_val, float cy_val)
            : fx(fx_val), fy(fy_val), cx(cx_val), cy(cy_val) {}
    };

    RedCubeDetectorNode() : Node("red_cube_detector_node")
    {
        RCLCPP_INFO(this->get_logger(), "初始化红色立方体检测器节点...");
         // 初始化真实坐标点（在initialize_parameters之前）
        initialize_real_waypoints();

        try
        {
            initialize_ground_truth_waypoints();
            initialize_parameters();
            initialize_tf();
            initialize_paths();
            initialize_onnx_runtime();
            initialize_publishers_and_timers();
            load_processed_files();

            RCLCPP_INFO(this->get_logger(), "红色立方体检测器节点启动成功！");
            RCLCPP_INFO(this->get_logger(), "TF配置: %s -> %s", camera_frame_.c_str(), world_frame_.c_str());
            RCLCPP_INFO(this->get_logger(), "已加载 %zu 个真实坐标航点", ground_truth_waypoints_.size());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "节点初始化失败: %s", e.what());
            rclcpp::shutdown();
        }
    }

    ~RedCubeDetectorNode()
    {
        RCLCPP_INFO(this->get_logger(), "关闭红色立方体检测器节点...");
        cleanup_resources();
    }

private:
    // 常量定义
    static constexpr float DEFAULT_DEPTH_SCALE = 1000.0f;
    static constexpr int DEFAULT_BORDER_VALUE = 114;
    static constexpr float MIN_VALID_DEPTH = 0.01f;
    static constexpr float MAX_VALID_DEPTH = 50.0f;
    static constexpr size_t MAX_LOG_FILE_SIZE = 1024 * 1024;

    const std::vector<std::string> class_names_ = {"red_cube"};

    // 真实坐标航点列表
    std::vector<Waypoint> ground_truth_waypoints_;

    // 初始化真实坐标航点
    void initialize_ground_truth_waypoints()
    {
        ground_truth_waypoints_ = {
            Waypoint("red_cube_west_koi_pond", 34.32f, -10.13f, 5.0f),
            Waypoint("red_cube_n14", 80.84f, -28.52f, 5.0f),
            Waypoint("red_cube_n13", 84.44f, -4.94f, 5.0f),
            Waypoint("red_cube_junction_south_west", 84.56f, -38.81f, 5.0f),
            Waypoint("red_cube_s08", 96.61f, -50.50f, 5.0f),
            Waypoint("red_cube_s10", 122.10f, -46.68f, 5.0f)
        };
    }

    // 查找最近的真实坐标
    Waypoint find_nearest_waypoint(float x, float y, float z) const
    {
        if (ground_truth_waypoints_.empty())
        {
            throw std::runtime_error("真实坐标航点列表为空");
        }

        float min_distance = std::numeric_limits<float>::max();
        size_t nearest_index = 0;

        for (size_t i = 0; i < ground_truth_waypoints_.size(); ++i)
        {
            const auto& wp = ground_truth_waypoints_[i];
            float dx = x - wp.x;
            float dy = y - wp.y;
            float dz = z - wp.z;
            float distance = std::sqrt(dx*dx + dy*dy + dz*dz);

            if (distance < min_distance)
            {
                min_distance = distance;
                nearest_index = i;
            }
        }

        RCLCPP_INFO(this->get_logger(), 
                    "找到最近的真实坐标: %s, 距离=%.3fm",
                    ground_truth_waypoints_[nearest_index].name.c_str(), 
                    min_distance);

        return ground_truth_waypoints_[nearest_index];
    }
    // 初始化真实坐标点
    void initialize_real_waypoints()
    {
        real_waypoints_.clear();
        real_waypoints_.reserve(6);

        real_waypoints_.emplace_back("red_cube_west_koi_pond", 34.32f, -10.13f, 5.0f);
        real_waypoints_.emplace_back("red_cube_n14", 80.84f, -28.52f, 5.0f);
        real_waypoints_.emplace_back("red_cube_n13", 84.44f, -4.94f, 5.0f);
        real_waypoints_.emplace_back("red_cube_junction_south_west", 84.56f, -38.81f, 5.0f);
        real_waypoints_.emplace_back("red_cube_s08", 96.61f, -50.50f, 5.0f);
        real_waypoints_.emplace_back("red_cube_s10", 122.10f, -46.68f, 5.0f);

        RCLCPP_INFO(this->get_logger(), "已加载 %zu 个真实坐标点", real_waypoints_.size());
    }

    void initialize_parameters()
    {
        this->declare_parameter<std::string>("model_path", "models/red_cube_yolov11.onnx");
        this->declare_parameter<float>("confidence_threshold", 0.5f);
        this->declare_parameter<float>("nms_threshold", 0.45f);
        this->declare_parameter<int>("target_class_id", 0);
        this->declare_parameter<std::string>("image_save_path", "image_save");
        this->declare_parameter<int>("check_interval", 2000);

        this->declare_parameter<float>("camera_fx", 454.69f);
        this->declare_parameter<float>("camera_fy", 454.69f);
        this->declare_parameter<float>("camera_cx", 424.50f);
        this->declare_parameter<float>("camera_cy", 240.50f);

        this->declare_parameter<float>("depth_scale", 1.0f);
        this->declare_parameter<float>("min_depth", 0.01f);
        this->declare_parameter<float>("max_depth", 50.0f);

        this->declare_parameter<std::string>("camera_frame", "drone_camera_link");
        this->declare_parameter<std::string>("world_frame", "map");
        this->declare_parameter<double>("transform_timeout", 5.0);

        this->declare_parameter<bool>("enable_fallback_mode", true);
        this->declare_parameter<bool>("publish_static_tf", false);
        this->declare_parameter<bool>("debug_coordinates", true);
        this->declare_parameter<bool>("wait_for_tf", true);
        this->declare_parameter<double>("tf_wait_timeout", 10.0);

        this->get_parameter("confidence_threshold", conf_threshold_);
        this->get_parameter("nms_threshold", nms_threshold_);
        this->get_parameter("target_class_id", target_class_id_);
        this->get_parameter("camera_frame", camera_frame_);
        this->get_parameter("world_frame", world_frame_);
        this->get_parameter("transform_timeout", transform_timeout_);
        this->get_parameter("enable_fallback_mode", enable_fallback_mode_);
        this->get_parameter("publish_static_tf", publish_static_tf_);
        this->get_parameter("debug_coordinates", debug_coordinates_);
        this->get_parameter("wait_for_tf", wait_for_tf_);
        this->get_parameter("tf_wait_timeout", tf_wait_timeout_);

        int check_interval;
        this->get_parameter("check_interval", check_interval);
        check_interval_ms_ = std::chrono::milliseconds(check_interval);

        this->get_parameter("depth_scale", depth_scale_);
        this->get_parameter("min_depth", min_depth_);
        this->get_parameter("max_depth", max_depth_);

        validate_parameters();

        float fx, fy, cx, cy;
        this->get_parameter("camera_fx", fx);
        this->get_parameter("camera_fy", fy);
        this->get_parameter("camera_cx", cx);
        this->get_parameter("camera_cy", cy);
        camera_intrinsics_ = std::make_unique<CameraIntrinsics>(fx, fy, cx, cy);

        RCLCPP_INFO(this->get_logger(), "相机内参: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx, fy, cx, cy);
        RCLCPP_INFO(this->get_logger(), "深度处理参数: scale=%.1f, min_depth=%.2fm, max_depth=%.1fm",
                    depth_scale_, min_depth_, max_depth_);
        RCLCPP_INFO(this->get_logger(), "TF参数: camera_frame=%s, world_frame=%s, timeout=%.1fs",
                    camera_frame_.c_str(), world_frame_.c_str(), transform_timeout_);
    }

    void initialize_tf()
    {
        RCLCPP_INFO(this->get_logger(), "初始化TF2系统...");

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        if (enable_fallback_mode_)
        {
            RCLCPP_INFO(this->get_logger(), "后备模式已启用：当TF变换不可用时，将使用相机坐标系");
        }
    }

    void validate_parameters()
    {
        if (conf_threshold_ <= 0.0f || conf_threshold_ >= 1.0f)
        {
            throw std::invalid_argument("置信度阈值必须在 (0, 1) 范围内");
        }

        if (nms_threshold_ <= 0.0f || nms_threshold_ >= 1.0f)
        {
            throw std::invalid_argument("NMS阈值必须在 (0, 1) 范围内");
        }

        if (target_class_id_ < 0 || static_cast<size_t>(target_class_id_) >= class_names_.size())
        {
            throw std::invalid_argument("目标类别ID超出范围");
        }

        if (depth_scale_ <= 0.0f)
        {
            throw std::invalid_argument("深度缩放因子必须大于0");
        }

        if (min_depth_ <= 0.0f || min_depth_ >= max_depth_)
        {
            throw std::invalid_argument("深度范围参数无效");
        }

        if (transform_timeout_ <= 0.0)
        {
            throw std::invalid_argument("变换超时时间必须大于0");
        }
    }

    void initialize_paths()
    {
        std::string model_rel_path, image_save_rel_path;
        this->get_parameter("model_path", model_rel_path);
        this->get_parameter("image_save_path", image_save_rel_path);

        std::vector<std::string> candidate_paths = {
            "/home/suda/drone_ugv_ws/src/image_processing",
            fs::current_path().string() + "/src/image_processing",
            fs::current_path().string(),
        };

        try
        {
            std::string install_path = ament_index_cpp::get_package_prefix("image_processing");
            candidate_paths.push_back(install_path);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "无法获取ament_index路径: %s", e.what());
        }

        bool model_found = false;
        for (const auto &base_path : candidate_paths)
        {
            std::string candidate_model_path = base_path + "/" + model_rel_path;
            if (fs::exists(candidate_model_path))
            {
                model_path_ = candidate_model_path;
                model_found = true;
                RCLCPP_INFO(this->get_logger(), "找到模型文件: %s", model_path_.c_str());
                break;
            }
        }

        if (!model_found)
        {
            RCLCPP_ERROR(this->get_logger(), "在以下位置均未找到模型文件 '%s':", model_rel_path.c_str());
            for (const auto &base_path : candidate_paths)
            {
                RCLCPP_ERROR(this->get_logger(), "  - %s/%s", base_path.c_str(), model_rel_path.c_str());
            }
            throw std::runtime_error("模型文件不存在");
        }

        bool image_dir_found = false;
        for (const auto &base_path : candidate_paths)
        {
            std::string candidate_image_path = base_path + "/" + image_save_rel_path;
            if (fs::exists(candidate_image_path) && fs::is_directory(candidate_image_path))
            {
                image_save_path_ = candidate_image_path;
                image_dir_found = true;
                RCLCPP_INFO(this->get_logger(), "找到图像保存目录: %s", image_save_path_.c_str());
                break;
            }
        }

        if (!image_dir_found)
        {
            std::string default_image_path = candidate_paths[0] + "/" + image_save_rel_path;
            try
            {
                fs::create_directories(default_image_path);
                image_save_path_ = default_image_path;
                RCLCPP_INFO(this->get_logger(), "创建图像保存目录: %s", image_save_path_.c_str());
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "无法创建图像保存目录 %s: %s",
                             default_image_path.c_str(), e.what());
                throw std::runtime_error("无法创建图像保存目录");
            }
        }
    }

    void initialize_onnx_runtime()
    {
        RCLCPP_INFO(this->get_logger(), "初始化ONNX Runtime...");

        env_ = std::make_unique<Ort::Env>(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "RedCubeDetector");

        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(1);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        configure_execution_providers(session_options);

        try
        {
            session_ = std::make_unique<Ort::Session>(*env_, model_path_.c_str(), session_options);
            setup_model_io_info();
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error("创建ONNX会话失败: " + std::string(e.what()));
        }
    }

    void configure_execution_providers(Ort::SessionOptions &session_options)
    {
        auto available_providers = Ort::GetAvailableProviders();
        std::unordered_set<std::string> providers(available_providers.begin(), available_providers.end());

        RCLCPP_INFO(this->get_logger(), "可用的执行提供程序:");
        for (const auto &provider : available_providers)
        {
            RCLCPP_INFO(this->get_logger(), "  - %s", provider.c_str());
        }

        if (providers.count("CUDAExecutionProvider"))
        {
            try
            {
                OrtCUDAProviderOptions cuda_options{};
                cuda_options.device_id = 0;
                cuda_options.gpu_mem_limit = 2ULL * 1024 * 1024 * 1024;
                session_options.AppendExecutionProvider_CUDA(cuda_options);
                RCLCPP_INFO(this->get_logger(), "已启用CUDA加速推理");
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(this->get_logger(), "CUDA初始化失败，使用CPU: %s", e.what());
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "CUDA不可用，使用CPU推理");
        }
    }

    void setup_model_io_info()
    {
        Ort::AllocatedStringPtr input_name_alloc = session_->GetInputNameAllocated(0, allocator_);
        input_name_ = input_name_alloc.release();

        Ort::TypeInfo input_type_info = session_->GetInputTypeInfo(0);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        input_dims_ = input_tensor_info.GetShape();

        if (input_dims_.size() < 4)
        {
            throw std::runtime_error("输入维度格式错误，期望4维NCHW格式");
        }

        input_height_ = static_cast<int>(input_dims_[2]);
        input_width_ = static_cast<int>(input_dims_[3]);

        RCLCPP_INFO(this->get_logger(), "模型输入: %s, 尺寸: %dx%d",
                    input_name_, input_width_, input_height_);

        size_t output_count = session_->GetOutputCount();
        output_names_.reserve(output_count);

        for (size_t i = 0; i < output_count; ++i)
        {
            Ort::AllocatedStringPtr output_name_alloc = session_->GetOutputNameAllocated(i, allocator_);
            output_names_.push_back(output_name_alloc.release());
            RCLCPP_INFO(this->get_logger(), "输出节点[%zu]: %s", i, output_names_[i]);
        }
    }

    void initialize_publishers_and_timers()
    {
        detection_pub_ = this->create_publisher<image_processing::msg::RedCubeDetection>(
            "/red_cube/detections", 10);

        check_timer_ = this->create_wall_timer(
            check_interval_ms_,
            std::bind(&RedCubeDetectorNode::check_new_files, this));
    }

    void cleanup_resources()
    {
        save_processed_files();

        if (input_name_)
        {
            allocator_.Free(const_cast<char *>(input_name_));
            input_name_ = nullptr;
        }

        for (const char *name : output_names_)
        {
            if (name)
            {
                allocator_.Free(const_cast<char *>(name));
            }
        }
        output_names_.clear();
    }

    geometry_msgs::msg::TransformStamped build_historical_tf(const PoseData &pose_data) const
    {
        geometry_msgs::msg::TransformStamped drone_base_to_map;
        drone_base_to_map.header.frame_id = "map";
        drone_base_to_map.child_frame_id = "drone_base_link";
        drone_base_to_map.transform.translation.x = pose_data.pos_x;
        drone_base_to_map.transform.translation.y = pose_data.pos_y;
        drone_base_to_map.transform.translation.z = pose_data.pos_z;
        drone_base_to_map.transform.rotation.x = pose_data.ori_x;
        drone_base_to_map.transform.rotation.y = pose_data.ori_y;
        drone_base_to_map.transform.rotation.z = pose_data.ori_z;
        drone_base_to_map.transform.rotation.w = pose_data.ori_w;
        drone_base_to_map.header.stamp.sec = pose_data.ros_timestamp_sec;
        drone_base_to_map.header.stamp.nanosec = pose_data.ros_timestamp_nsec;

        geometry_msgs::msg::TransformStamped camera_to_drone_base;
        camera_to_drone_base.header.frame_id = "drone_base_link";
        camera_to_drone_base.child_frame_id = "drone_camera_link";
        camera_to_drone_base.transform.translation.x = 0.1;
        camera_to_drone_base.transform.translation.y = 0.0;
        camera_to_drone_base.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, -0.785, 0);
        camera_to_drone_base.transform.rotation.x = q.x();
        camera_to_drone_base.transform.rotation.y = q.y();
        camera_to_drone_base.transform.rotation.z = q.z();
        camera_to_drone_base.transform.rotation.w = q.w();

        geometry_msgs::msg::TransformStamped camera_to_map;
        camera_to_map.header.frame_id = "map";
        camera_to_map.child_frame_id = "drone_camera_link";
        camera_to_map.header.stamp = drone_base_to_map.header.stamp;

        tf2::Vector3 camera_translation_camera_link(
            camera_to_drone_base.transform.translation.x,
            camera_to_drone_base.transform.translation.y,
            camera_to_drone_base.transform.translation.z
        );

        tf2::Quaternion drone_orientation;
        tf2::fromMsg(drone_base_to_map.transform.rotation, drone_orientation);
        drone_orientation.normalize();
        tf2::Quaternion drone_orientation_inv = drone_orientation.inverse();
        drone_orientation_inv.normalize();
        tf2::Vector3 camera_translation_map = tf2::quatRotate(drone_orientation_inv, camera_translation_camera_link);

        camera_to_map.transform.translation.x = drone_base_to_map.transform.translation.x + camera_translation_map.x();
        camera_to_map.transform.translation.y = drone_base_to_map.transform.translation.y + camera_translation_map.y();
        camera_to_map.transform.translation.z = drone_base_to_map.transform.translation.z + camera_translation_map.z();

        tf2::Quaternion camera_orientation;
        tf2::fromMsg(camera_to_drone_base.transform.rotation, camera_orientation);
        camera_orientation.normalize();

        tf2::Quaternion camera_orientation_map = camera_orientation * drone_orientation;
        camera_orientation_map.normalize();
        camera_to_map.transform.rotation = tf2::toMsg(camera_orientation_map);

        if (debug_coordinates_)
        {
            RCLCPP_INFO(this->get_logger(), "历史TF构建完成:");
            RCLCPP_INFO(this->get_logger(), "  无人机拍摄时位置: (%.3f, %.3f, %.3f)",
                        pose_data.pos_x, pose_data.pos_y, pose_data.pos_z);
            RCLCPP_INFO(this->get_logger(), "  相机历史位置: (%.3f, %.3f, %.3f)",
                        camera_to_map.transform.translation.x,
                        camera_to_map.transform.translation.y,
                        camera_to_map.transform.translation.z);
        }

        return camera_to_map;
    }

    void process_detections_with_historical_tf(const std::vector<Detection> &detections,
                                               const cv::Mat &depth_image,
                                               const PoseData &pose_data,
                                               const std::string &base_name,
                                               const geometry_msgs::msg::TransformStamped &historical_tf)
    {
        for (const auto &det : detections)
        {
            if (det.class_id != target_class_id_)
            {
                RCLCPP_DEBUG(this->get_logger(), "跳过非目标类别: %s (ID=%d)", det.class_name.c_str(), det.class_id);
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "检测到目标: %s (ID=%d), 置信度=%.3f",
                        det.class_name.c_str(), det.class_id, det.confidence);

            const int center_x = det.box.x + det.box.width / 2;
            const int center_y = det.box.y + det.box.height / 2;

            if (!is_valid_pixel_coord(center_x, center_y, depth_image))
            {
                RCLCPP_WARN(this->get_logger(), "像素坐标超出图像范围: (%d, %d)", center_x, center_y);
                continue;
            }

            const float depth = get_depth_value(center_x, center_y, depth_image);
            if (depth <= min_depth_ || depth > max_depth_)
            {
                RCLCPP_WARN(this->get_logger(), "深度值超出范围: %.3f m (有效范围: %.2f-%.1f m)",
                            depth, min_depth_, max_depth_);
                continue;
            }

            const Eigen::Vector3f cam_coords_opencv = pixel_to_camera(center_x, center_y, depth,
                                                                      det.adapted_fx, det.adapted_fy,
                                                                      det.adapted_cx, det.adapted_cy);
            if (std::isnan(cam_coords_opencv.x()))
            {
                RCLCPP_WARN(this->get_logger(), "相机系坐标计算失败");
                continue;
            }

            Eigen::Vector3f cam_coords_ros;
            cam_coords_ros.x() = cam_coords_opencv.z();
            cam_coords_ros.y() = cam_coords_opencv.x();
            cam_coords_ros.z() = -cam_coords_opencv.y();

            geometry_msgs::msg::PointStamped cam_point, world_point;
            cam_point.header.frame_id = camera_frame_;
            cam_point.header.stamp = historical_tf.header.stamp;
            cam_point.point.x = cam_coords_ros.x();
            cam_point.point.y = cam_coords_ros.y();
            cam_point.point.z = cam_coords_ros.z();

            try
            {
                tf2::doTransform(cam_point, world_point, historical_tf);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "TF变换失败: %s", ex.what());
                continue;
            }

            const float z_threshold = 1.0f;
            if (std::abs(world_point.point.z) > z_threshold)
            {
                RCLCPP_WARN(this->get_logger(), "世界坐标Z轴异常: %.3f m（地面目标应接近0，阈值±%.1f m）",
                            world_point.point.z, z_threshold);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "世界坐标验证通过（Z轴接近地面）");
            }

            // 查找最近的真实坐标（使用TF变换后的世界坐标）
            Waypoint nearest_waypoint = find_nearest_waypoint(
                world_point.point.x, 
                world_point.point.y, 
                world_point.point.z
            );

            // 创建世界坐标向量（确保所有地方使用同一个坐标）
            Eigen::Vector3f world_coords_vec(world_point.point.x, world_point.point.y, world_point.point.z);

            RCLCPP_INFO(this->get_logger(),
                        "目标信息: 像素(%d,%d) | 深度%.3f m | "
                        "OpenCV相机系(%.3f,%.3f,%.3f) | "
                        "ROS相机系(%.3f,%.3f,%.3f) | "
                        "世界系(%.3f,%.3f,%.3f) | "
                        "真实坐标: %s (%.3f,%.3f,%.3f)",
                        center_x, center_y, depth,
                        cam_coords_opencv.x(), cam_coords_opencv.y(), cam_coords_opencv.z(),
                        cam_coords_ros.x(), cam_coords_ros.y(), cam_coords_ros.z(),
                        world_coords_vec.x(), world_coords_vec.y(), world_coords_vec.z(),
                        nearest_waypoint.name.c_str(), 
                        nearest_waypoint.x, nearest_waypoint.y, nearest_waypoint.z);

            int target_index = &det - &detections[0];
            save_detection_result(
                base_name,
                det,
                cam_coords_ros,
                world_coords_vec,
                pose_data,
                target_index,
                nearest_waypoint
            );

            publish_detection_result(det, center_x, center_y, cam_coords_ros,
                                     world_coords_vec,
                                     pose_data, base_name, historical_tf.header.stamp, true,
                                     nearest_waypoint);
        }
    }

    void load_processed_files()
    {
        std::lock_guard<std::mutex> lock(processed_files_mutex_);
        processed_files_.clear();

        const std::string processed_file_path = image_save_path_ + "/processed_files.txt";
        if (!fs::exists(processed_file_path))
        {
            return;
        }

        std::ifstream file(processed_file_path);
        if (!file.is_open())
        {
            RCLCPP_WARN(this->get_logger(), "无法打开已处理文件列表: %s", processed_file_path.c_str());
            return;
        }

        std::string line;
        while (std::getline(file, line))
        {
            if (!line.empty())
            {
                processed_files_.insert(line);
            }
        }

        RCLCPP_INFO(this->get_logger(), "已加载 %zu 个已处理文件记录", processed_files_.size());
    }

    void save_processed_files()
    {
        std::lock_guard<std::mutex> lock(processed_files_mutex_);

        const std::string processed_file_path = image_save_path_ + "/processed_files.txt";

        try
        {
            fs::create_directories(fs::path(processed_file_path).parent_path());
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "无法创建目录: %s", e.what());
        }

        std::ofstream file(processed_file_path);

        if (!file.is_open())
        {
            RCLCPP_WARN(this->get_logger(), "无法保存已处理文件列表: %s", processed_file_path.c_str());
            return;
        }

        for (const std::string &filename : processed_files_)
        {
            file << filename << "\n";
        }

        RCLCPP_INFO(this->get_logger(), "已保存 %zu 个已处理文件记录", processed_files_.size());
    }

    void add_processed_file(const std::string &base_name)
    {
        std::lock_guard<std::mutex> lock(processed_files_mutex_);
        processed_files_.insert(base_name);
    }

    bool is_processed(const std::string &base_name) const
    {
        std::lock_guard<std::mutex> lock(processed_files_mutex_);
        return processed_files_.find(base_name) != processed_files_.end();
    }

    void check_new_files()
    {
        RCLCPP_DEBUG(this->get_logger(), "检查新文件...");

        try
        {
            const auto base_names = scan_for_image_files();

            RCLCPP_DEBUG(this->get_logger(), "找到 %zu 组图像文件", base_names.size());

            for (const std::string &base_name : base_names)
            {
                if (!is_processed(base_name))
                {
                    if (process_file_group(base_name))
                    {
                        add_processed_file(base_name);
                    }
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "检查新文件时出错: %s", e.what());
        }
    }

    std::unordered_set<std::string> scan_for_image_files() const
    {
        std::unordered_set<std::string> base_names;

        if (!fs::exists(image_save_path_) || !fs::is_directory(image_save_path_))
        {
            RCLCPP_DEBUG(this->get_logger(), "图像保存目录不存在或不是目录: %s", image_save_path_.c_str());
            return base_names;
        }

        try
        {
            for (const auto &entry : fs::directory_iterator(image_save_path_))
            {
                if (!entry.is_regular_file())
                    continue;

                const fs::path file_path = entry.path();
                const std::string filename = file_path.filename().string();
                const std::string file_ext = file_path.extension().string();

                if (filename.substr(0, 6) == "color_" && file_ext == ".jpg")
                {
                    std::string base_name = file_path.stem().string().substr(6);
                    std::string depth_xml_path = image_save_path_ + "/depth_" + base_name + ".xml";
                    if (fs::exists(depth_xml_path))
                    {
                        RCLCPP_DEBUG(this->get_logger(), "提取base_name: %s（对应XML深度图存在）", base_name.c_str());
                        base_names.insert(base_name);
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "base_name: %s 对应的XML深度图不存在，跳过", base_name.c_str());
                    }
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "扫描图像文件时出错: %s", e.what());
        }

        return base_names;
    }

    bool process_file_group(const std::string &base_name)
    {
        RCLCPP_INFO(this->get_logger(), "处理文件组: %s", base_name.c_str());

        try
        {
            const auto file_paths = build_file_paths(base_name);

            if (!validate_file_existence(file_paths))
            {
                return false;
            }

            const auto images = load_images(file_paths);
            if (images.first.empty() || images.second.empty())
            {
                return false;
            }

            const PoseData pose_data = load_pose_data(file_paths.pose_path);
            const auto detections = detect_red_cube(images.first);

            geometry_msgs::msg::TransformStamped historical_tf;
            try
            {
                historical_tf = build_historical_tf(pose_data);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "构建历史TF失败，跳过文件组: %s", e.what());
                return false;
            }

            process_detections_with_historical_tf(detections, images.second, pose_data, base_name, historical_tf);
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "处理文件组时出错: %s", e.what());
            return false;
        }
    }

    struct FilePaths
    {
        std::string color_path;
        std::string depth_path;
        std::string result_path;
        std::string pose_path;
    };

    FilePaths build_file_paths(const std::string &base_name) const
    {
        FilePaths paths;
        const std::string base_path = image_save_path_ + "/";

        paths.color_path = base_path + "color_" + base_name + ".jpg";
        paths.depth_path = base_path + "depth_" + base_name + ".xml";
        paths.result_path = base_path + "result_" + base_name + ".jpg";
        paths.pose_path = base_path + "pose_" + base_name + ".txt";

        return paths;
    }

    bool validate_file_existence(const FilePaths &paths) const
    {
        const std::vector<std::pair<std::string, std::string>> required_files = {
            {paths.color_path, "彩色图像"},
            {paths.depth_path, "深度图像"},
            {paths.pose_path, "位姿文件"}};

        for (const auto &[path, description] : required_files)
        {
            if (!fs::exists(path))
            {
                RCLCPP_WARN(this->get_logger(), "%s不存在: %s", description.c_str(), path.c_str());
                return false;
            }
        }
        return true;
    }

    std::pair<cv::Mat, cv::Mat> load_images(const FilePaths &paths) const
    {
        cv::Mat color_image = cv::imread(paths.color_path);
        if (color_image.empty())
        {
            RCLCPP_WARN(this->get_logger(), "无法加载彩色图像: %s", paths.color_path.c_str());
        }

        cv::Mat depth_image;
        cv::FileStorage fs(paths.depth_path, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            RCLCPP_WARN(this->get_logger(), "无法加载深度图像XML文件: %s", paths.depth_path.c_str());
        }
        else
        {
            fs["depth_image"] >> depth_image;
            fs.release();

            if (depth_image.type() != CV_32FC1)
            {
                RCLCPP_WARN(this->get_logger(), "深度图格式异常（实际%d），强制转换为32FC1", depth_image.type());
                depth_image.convertTo(depth_image, CV_32FC1);
            }
            RCLCPP_DEBUG(this->get_logger(), "成功加载XML深度图，尺寸: %dx%d, 格式: 32FC1",
                         depth_image.cols, depth_image.rows);
        }

        return {color_image, depth_image};
    }

    cv::Mat preprocess_image(const cv::Mat &image, float &scale, int &pad_w, int &pad_h) const
    {
        if (image.empty())
        {
            throw std::runtime_error("输入图像为空");
        }

        const int original_w = image.cols;
        const int original_h = image.rows;

        const float scale_x = static_cast<float>(input_width_) / original_w;
        const float scale_y = static_cast<float>(input_height_) / original_h;
        scale = std::min(scale_x, scale_y);

        const int new_w = static_cast<int>(original_w * scale);
        const int new_h = static_cast<int>(original_h * scale);

        pad_w = (input_width_ - new_w) / 2;
        pad_h = (input_height_ - new_h) / 2;

        cv::Mat resized;
        cv::resize(image, resized, cv::Size(new_w, new_h));

        cv::Mat padded;
        cv::copyMakeBorder(resized, padded,
                           pad_h, input_height_ - new_h - pad_h,
                           pad_w, input_width_ - new_w - pad_w,
                           cv::BORDER_CONSTANT,
                           cv::Scalar(DEFAULT_BORDER_VALUE, DEFAULT_BORDER_VALUE, DEFAULT_BORDER_VALUE));

        cv::Mat rgb;
        cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);
        rgb.convertTo(rgb, CV_32FC3, 1.0 / 255.0);

        return rgb;
    }

    std::vector<Ort::Value> infer(const cv::Mat &input_image) const
    {
        const size_t input_size = 1 * 3 * input_height_ * input_width_;
        std::vector<float> input_data(input_size);

        for (int c = 0; c < 3; ++c)
        {
            for (int h = 0; h < input_height_; ++h)
            {
                for (int w = 0; w < input_width_; ++w)
                {
                    const size_t index = c * input_height_ * input_width_ + h * input_width_ + w;
                    input_data[index] = input_image.at<cv::Vec3f>(h, w)[c];
                }
            }
        }

        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
            OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, input_data.data(), input_data.size(),
            input_dims_.data(), input_dims_.size());

        const auto start_time = std::chrono::high_resolution_clock::now();

        auto outputs = session_->Run(Ort::RunOptions{nullptr},
                                     &input_name_, &input_tensor, 1,
                                     output_names_.data(), output_names_.size());

        const auto end_time = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        RCLCPP_DEBUG(this->get_logger(), "推理完成，耗时: %ld ms", duration.count());

        return outputs;
    }

    std::vector<Detection> postprocess(
        std::vector<Ort::Value> &outputs,
        int original_w, int original_h,
        float scale, int pad_w, int pad_h, float adapted_fx,
        float adapted_fy,
        float adapted_cx,
        float adapted_cy
    ) const
    {
        if (outputs.empty())
        {
            RCLCPP_WARN(this->get_logger(), "推理输出为空");
            return {};
        }

        float *output_data = outputs[0].GetTensorMutableData<float>();
        auto output_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();

        if (output_shape.size() != 3)
        {
            RCLCPP_ERROR(this->get_logger(), "输出形状不正确，期望3维，实际%d维",
                         static_cast<int>(output_shape.size()));
            return {};
        }

        const int num_detections = static_cast<int>(output_shape[2]);
        const int num_classes = static_cast<int>(output_shape[1]) - 4;

        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        std::vector<int> class_ids;

        boxes.reserve(num_detections);
        scores.reserve(num_detections);
        class_ids.reserve(num_detections);

        for (int i = 0; i < num_detections; ++i)
        {
            const float cx = output_data[i];
            const float cy = output_data[num_detections + i];
            const float w = output_data[2 * num_detections + i];
            const float h = output_data[3 * num_detections + i];

            float max_score = 0.0f;
            int max_class = -1;

            for (int c = 0; c < num_classes; ++c)
            {
                const float score = output_data[(4 + c) * num_detections + i];
                if (score > max_score)
                {
                    max_score = score;
                    max_class = c;
                }
            }

            if (max_score > conf_threshold_)
            {
                const auto box = convert_to_original_coords(cx, cy, w, h, scale, pad_w, pad_h,
                                                            original_w, original_h);
                if (box.width > 0 && box.height > 0)
                {
                    boxes.push_back(box);
                    scores.push_back(max_score);
                    class_ids.push_back(max_class);
                }
            }
        }

        const auto keep_indices = apply_nms(boxes, scores, nms_threshold_);

        std::vector<Detection> detections;
        detections.reserve(keep_indices.size());
        for (const int idx : keep_indices)
        {
            const std::string class_name = (static_cast<size_t>(class_ids[idx]) < class_names_.size())
                                               ? class_names_[class_ids[idx]]
                                               : "unknown";
            Detection det(boxes[idx], scores[idx], class_ids[idx], class_name);
            det.adapted_fx = adapted_fx;
            det.adapted_fy = adapted_fy;
            det.adapted_cx = adapted_cx;
            det.adapted_cy = adapted_cy;
            detections.push_back(det);
        }

        return detections;
    }

    cv::Rect convert_to_original_coords(float cx, float cy, float w, float h,
                                        float scale, int pad_w, int pad_h,
                                        int original_w, int original_h) const
    {
        const float x1 = (cx - w * 0.5f - pad_w) / scale;
        const float y1 = (cy - h * 0.5f - pad_h) / scale;
        const float x2 = (cx + w * 0.5f - pad_w) / scale;
        const float y2 = (cy + h * 0.5f - pad_h) / scale;

        const int final_x1 = std::max(0, std::min(original_w, static_cast<int>(x1)));
        const int final_y1 = std::max(0, std::min(original_h, static_cast<int>(y1)));
        const int final_x2 = std::max(0, std::min(original_w, static_cast<int>(x2)));
        const int final_y2 = std::max(0, std::min(original_h, static_cast<int>(y2)));

        return cv::Rect(final_x1, final_y1, final_x2 - final_x1, final_y2 - final_y1);
    }

    std::vector<int> apply_nms(const std::vector<cv::Rect> &boxes,
                               const std::vector<float> &scores,
                               float threshold) const
    {
        std::vector<int> indices(boxes.size());
        std::iota(indices.begin(), indices.end(), 0);

        std::sort(indices.begin(), indices.end(),
                  [&scores](int i1, int i2)
                  { return scores[i1] > scores[i2]; });

        std::vector<bool> suppressed(boxes.size(), false);
        std::vector<int> keep;

        for (size_t i = 0; i < indices.size(); ++i)
        {
            const int idx = indices[i];
            if (suppressed[idx])
                continue;

            keep.push_back(idx);

            for (size_t j = i + 1; j < indices.size(); ++j)
            {
                const int idx2 = indices[j];
                if (suppressed[idx2])
                    continue;

                const cv::Rect intersection = boxes[idx] & boxes[idx2];
                const float inter_area = intersection.area();
                const float union_area = boxes[idx].area() + boxes[idx2].area() - inter_area;

                if (union_area > 0 && inter_area / union_area > threshold)
                {
                    suppressed[idx2] = true;
                }
            }
        }

        return keep;
    }

    std::vector<Detection> detect_red_cube(const cv::Mat &color_image) const
    {
        int original_width = color_image.cols;
        int original_height = color_image.rows;
        RCLCPP_DEBUG(this->get_logger(), "图像原始分辨率: %dx%d", original_width, original_height);

        float adapted_fx = camera_intrinsics_->fx;
        float adapted_fy = camera_intrinsics_->fy;
        float adapted_cx = camera_intrinsics_->cx;
        float adapted_cy = camera_intrinsics_->cy;

        RCLCPP_DEBUG(this->get_logger(), "适配后内参(深度相机实际值): fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                     adapted_fx, adapted_fy, adapted_cx, adapted_cy);

        float scale;
        int pad_w, pad_h;

        const cv::Mat input_image = preprocess_image(color_image, scale, pad_w, pad_h);
        auto outputs = infer(input_image);

        return postprocess(outputs, original_width, original_height, scale, pad_w, pad_h,
                           adapted_fx, adapted_fy, adapted_cx, adapted_cy);
    }

    Eigen::Vector3f pixel_to_camera(float x_pixel, float y_pixel, float depth,
                                    float adapted_fx,
                                    float adapted_fy,
                                    float adapted_cx,
                                    float adapted_cy
    ) const
    {
        if (depth <= min_depth_ || depth > max_depth_)
        {
            if (debug_coordinates_)
            {
                RCLCPP_WARN(this->get_logger(), "深度值超出范围: %.3f m (范围: %.2f - %.1f m)",
                            depth, min_depth_, max_depth_);
            }
            return Eigen::Vector3f(NAN, NAN, NAN);
        }

        const float Z_cam = depth;
        const float X_cam = (x_pixel - adapted_cx) * Z_cam / adapted_fx;
        const float Y_cam = (y_pixel - adapted_cy) * Z_cam / adapted_fy;

        if (debug_coordinates_)
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "像素坐标(%.1f, %.1f)，深度%.3fm -> 相机坐标(%.3f, %.3f, %.3f)m",
                         x_pixel, y_pixel, depth, X_cam, Y_cam, Z_cam);
        }

        return Eigen::Vector3f(X_cam, Y_cam, Z_cam);
    }
    std::pair<RealWorldWaypoint, float> find_nearest_waypoint(
        float world_x, float world_y, float world_z) const
    {
        if (real_waypoints_.empty())
        {
            throw std::runtime_error("真实坐标点列表为空");
        }

        float min_distance = std::numeric_limits<float>::max();
        size_t nearest_index = 0;

        for (size_t i = 0; i < real_waypoints_.size(); ++i)
        {
            float distance = real_waypoints_[i].distance_to(world_x, world_y, world_z);
            if (distance < min_distance)
            {
                min_distance = distance;
                nearest_index = i;
            }
        }

        return {real_waypoints_[nearest_index], min_distance};
    }
    
    void save_detection_result(const std::string &base_name,
                               const Detection &det,
                               const Eigen::Vector3f &cam_coords,
                               const Eigen::Vector3f &world_coords,
                               const PoseData &pose_data,
                               int target_index)
    {
        // 查找最近的真实坐标点
        std::pair<RealWorldWaypoint, float> nearest;
        try
        {
            nearest = find_nearest_waypoint(world_coords.x(), world_coords.y(), world_coords.z());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "查找最近真实坐标失败: %s", e.what());
            return;
        }

        // 单个目标的结果文件（加索引）
        std::string result_file = image_save_path_ + "/detection_result_" + base_name + "_" + std::to_string(target_index) + ".txt";
        std::ofstream file(result_file);

        if (file.is_open())
        {
            file << "目标索引: " << target_index << "\n";
            file << "类别: " << det.class_name << " (ID: " << det.class_id << ")\n";
            file << "置信度: " << det.confidence << "\n";
            file << "边界框: [" << det.box.x << ", " << det.box.y << ", "
                 << det.box.width << ", " << det.box.height << "]\n";
            
            file << "\n相机坐标系(米):\n";
            file << "X: " << cam_coords.x() << "\n";
            file << "Y: " << cam_coords.y() << "\n";
            file << "Z: " << cam_coords.z() << "\n";
            
            file << "\n世界坐标系(米):\n";
            file << "X: " << world_coords.x() << "\n";
            file << "Y: " << world_coords.y() << "\n";
            file << "Z: " << world_coords.z() << "\n";
            
            // ✅ 新增：真实坐标信息
            file << "\n真实坐标(米):\n";
            file << "名称: " << nearest.first.name << "\n";
            file << "X: " << nearest.first.x << "\n";
            file << "Y: " << nearest.first.y << "\n";
            file << "Z: " << nearest.first.z << "\n";
            file << "距离世界坐标的误差: " << nearest.second << " 米\n";
            
            file << "\n无人机位置:\n";
            file << "位置: (" << pose_data.pos_x << ", " << pose_data.pos_y
                 << ", " << pose_data.pos_z << ")\n";
            file << "姿态(四元数): (" << pose_data.ori_w << ", " << pose_data.ori_x
                 << ", " << pose_data.ori_y << ", " << pose_data.ori_z << ")\n";

            RCLCPP_INFO(this->get_logger(), 
                "目标%d结果已保存 | 最近真实坐标: %s (误差%.2fm)", 
                target_index, nearest.first.name.c_str(), nearest.second);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开文件保存检测结果: %s", result_file.c_str());
        }
    }

    bool is_valid_pixel_coord(int x, int y, const cv::Mat &image) const
    {
        return x >= 0 && x < image.cols && y >= 0 && y < image.rows;
    }

    float get_depth_value(int x, int y, const cv::Mat &depth_image) const
    {
        const int kernel = 5;
        const int half_kernel = kernel / 2;
        std::vector<float> valid_depths;

        for (int dy = -half_kernel; dy <= half_kernel; dy++)
        {
            for (int dx = -half_kernel; dx <= half_kernel; dx++)
            {
                const int nx = x + dx;
                const int ny = y + dy;
                if (nx >= 0 && nx < depth_image.cols && ny >= 0 && ny < depth_image.rows)
                {
                    const float d = depth_image.at<float>(ny, nx);
                    if (d > min_depth_ && d < max_depth_ && !std::isnan(d) && !std::isinf(d))
                    {
                        valid_depths.push_back(d);
                    }
                }
            }
        }

        if (valid_depths.empty())
        {
            RCLCPP_WARN(this->get_logger(), "目标区域无有效深度值");
            return 0.0f;
        }

        std::sort(valid_depths.begin(), valid_depths.end());
        float median_depth;
        size_t n = valid_depths.size();
        if (n % 2 == 1)
        {
            median_depth = valid_depths[n / 2];
        }
        else
        {
            median_depth = (valid_depths[n / 2 - 1] + valid_depths[n / 2]) / 2.0f;
        }

        RCLCPP_DEBUG(this->get_logger(), "像素(%d,%d)：%zu个有效深度的中位数=%.3fm",
                     x, y, valid_depths.size(), median_depth);
        return median_depth;
    }

    PoseData load_pose_data(const std::string &pose_path) const
    {
        PoseData pose_data;
        std::ifstream file(pose_path);

        if (!file.is_open())
        {
            throw std::runtime_error("无法打开位姿文件: " + pose_path);
        }

        std::string line;
        while (std::getline(file, line))
        {
            parse_pose_line(line, pose_data);
        }

        RCLCPP_DEBUG(this->get_logger(), "从 %s 读取位姿: pos=(%.3f,%.3f,%.3f), ori=(%.3f,%.3f,%.3f,%.3f)",
                     pose_path.c_str(),
                     pose_data.pos_x, pose_data.pos_y, pose_data.pos_z,
                     pose_data.ori_x, pose_data.ori_y, pose_data.ori_z, pose_data.ori_w);

        if (pose_data.pos_x == 0.0f && pose_data.pos_y == 0.0f && pose_data.pos_z == 0.0f)
        {
            RCLCPP_WARN(this->get_logger(), "位姿数据可能无效(全为0),请检查 pose 文件格式: %s", pose_path.c_str());
        }

        return pose_data;
    }

    void parse_pose_line(const std::string &line, PoseData &pose_data) const
    {
        if (line.find("位置(x,y,z):") != std::string::npos)
        {
            parse_field(line, "位置(x,y,z):", [&](const std::string &data)
                        {
                int count = sscanf(data.c_str(), "%f,%f,%f", 
                                &pose_data.pos_x, &pose_data.pos_y, &pose_data.pos_z);
                RCLCPP_DEBUG(this->get_logger(), "位置解析: (%f,%f,%f), 成功字段数=%d",
                            pose_data.pos_x, pose_data.pos_y, pose_data.pos_z, count); });
        }
        else if (line.find("姿态(四元数w,x,y,z):") != std::string::npos)
        {
            parse_field(line, "姿态(四元数w,x,y,z):", [&](const std::string &data)
                        {
                int count = sscanf(data.c_str(), "%f,%f,%f,%f",
                                &pose_data.ori_w, &pose_data.ori_x, &pose_data.ori_y, &pose_data.ori_z);
                RCLCPP_DEBUG(this->get_logger(), "姿态解析: (%f,%f,%f,%f), 成功字段数=%d",
                            pose_data.ori_w, pose_data.ori_x, pose_data.ori_y, pose_data.ori_z, count); });
        }
        else if (line.find("图像时间戳:") != std::string::npos)
        {
            parse_field(line, "图像时间戳:", [&](const std::string &data)
                        {
                pose_data.timestamp = data;
                RCLCPP_DEBUG(this->get_logger(), "图像时间戳解析: %s", data.c_str()); });
        }
        else if (line.find("目标像素坐标:") != std::string::npos)
        {
            parse_field(line, "目标像素坐标:", [&](const std::string &data)
                        {
                int count = sscanf(data.c_str(), "(%d,%d)", &pose_data.target_x, &pose_data.target_y);
                RCLCPP_DEBUG(this->get_logger(), "像素坐标解析: (%d,%d), 成功字段数=%d",
                            pose_data.target_x, pose_data.target_y, count); });
        }
        else if (line.find("ROS时间戳:") != std::string::npos)
        {
            parse_field(line, "ROS时间戳:", [&](const std::string &data)
                        {
                int count = sscanf(data.c_str(), "%ld %ld", 
                                &pose_data.ros_timestamp_sec, &pose_data.ros_timestamp_nsec);
                RCLCPP_DEBUG(this->get_logger(), "ROS时间戳解析: %ld %ld, 成功字段数=%d",
                            pose_data.ros_timestamp_sec, pose_data.ros_timestamp_nsec, count); });
        }
    }

    void parse_field(const std::string &line, const std::string &keyword,
                     std::function<void(const std::string &)> callback) const
    {
        size_t keyword_pos = line.find(keyword);
        if (keyword_pos == std::string::npos)
            return;

        size_t data_start = keyword_pos + keyword.length();
        std::string data = line.substr(data_start);

        size_t trim_start = data.find_first_not_of(" \t");
        if (trim_start != std::string::npos)
        {
            data = data.substr(trim_start);
        }

        callback(data);
    }

    void publish_detection_result(const Detection &det, int center_x, int center_y,
                                  const Eigen::Vector3f &cam_coords,
                                  const Eigen::Vector3f &world_coords,
                                  const PoseData &pose_data,
                                  const std::string & /* base_name */,
                                  const rclcpp::Time &timestamp,
                                  bool world_coords_valid,
                                  const Waypoint &nearest_waypoint) const
    {
        auto detection_msg = image_processing::msg::RedCubeDetection();

        detection_msg.header.stamp = timestamp;
        detection_msg.header.frame_id = world_coords_valid ? world_frame_ : camera_frame_;

        detection_msg.class_id = det.class_id;
        detection_msg.class_name = det.class_name;
        detection_msg.confidence = det.confidence;
        detection_msg.xmin = det.box.x;
        detection_msg.ymin = det.box.y;
        detection_msg.xmax = det.box.x + det.box.width;
        detection_msg.ymax = det.box.y + det.box.height;
        detection_msg.center_x = center_x;
        detection_msg.center_y = center_y;

        detection_msg.cam_x = cam_coords.x();
        detection_msg.cam_y = cam_coords.y();
        detection_msg.cam_z = cam_coords.z();

        if (world_coords_valid)
        {
            detection_msg.world_x = world_coords.x();
            detection_msg.world_y = world_coords.y();
            detection_msg.world_z = world_coords.z();
        }
        else
        {
            detection_msg.world_x = std::numeric_limits<float>::quiet_NaN();
            detection_msg.world_y = std::numeric_limits<float>::quiet_NaN();
            detection_msg.world_z = std::numeric_limits<float>::quiet_NaN();
        }

        detection_msg.uav_x = pose_data.pos_x;
        detection_msg.uav_y = pose_data.pos_y;
        detection_msg.uav_z = pose_data.pos_z;
        detection_msg.uav_qx = pose_data.ori_x;
        detection_msg.uav_qy = pose_data.ori_y;
        detection_msg.uav_qz = pose_data.ori_z;
        detection_msg.uav_qw = pose_data.ori_w;

        detection_pub_->publish(detection_msg);

        std::string result_status = world_coords_valid ? "成功" : (enable_fallback_mode_ ? "后备模式" : "失败");

        RCLCPP_INFO(this->get_logger(), "已发布检测结果: %s, 世界坐标变换%s, 最近真实坐标: %s",
                    det.class_name.c_str(), result_status.c_str(), nearest_waypoint.name.c_str());
    }

    // 成员变量
    std::unique_ptr<CameraIntrinsics> camera_intrinsics_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::string camera_frame_;
    std::string world_frame_;
    double transform_timeout_;

    bool tf_status_reported_ = false;

    bool enable_fallback_mode_ = true;
    bool publish_static_tf_ = false;
    bool debug_coordinates_ = true;
    bool wait_for_tf_ = true;
    double tf_wait_timeout_ = 10.0;

    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    Ort::AllocatorWithDefaultOptions allocator_;
    const char *input_name_ = nullptr;
    std::vector<const char *> output_names_;
    std::vector<int64_t> input_dims_;
    int input_width_ = 0;
    int input_height_ = 0;

    std::string model_path_;
    std::string image_save_path_;

    std::unordered_set<std::string> processed_files_;
    mutable std::mutex processed_files_mutex_;

    rclcpp::TimerBase::SharedPtr check_timer_;
    rclcpp::Publisher<image_processing::msg::RedCubeDetection>::SharedPtr detection_pub_;

    float conf_threshold_ = 0.5f;
    float nms_threshold_ = 0.45f;
    int target_class_id_ = 0;
    std::chrono::milliseconds check_interval_ms_{2000};

    float depth_scale_ = 1.0f;
    float min_depth_ = 0.1f;
    float max_depth_ = 50.0f;
    // 深度处理参数
    float depth_scale_ = DEFAULT_DEPTH_SCALE;
    float min_depth_ = 0.1f;
    float max_depth_ = 50.0f;

    // 真实坐标点列表
    std::vector<RealWorldWaypoint> real_waypoints_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<RedCubeDetectorNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "节点运行失败: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}