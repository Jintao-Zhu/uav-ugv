#include "rclcpp/rclcpp.hpp" //检测xml格式的深度图   11.8 加入无人机位姿利用
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

namespace fs = std::filesystem;
using namespace std::chrono_literals;

class RedCubeDetectorNode : public rclcpp::Node
{
public:
    // 检测结果结构体
    struct Detection
    {
        cv::Rect box;
        float confidence;
        int class_id;
        std::string class_name;

        // 新增：适配后的内参（供后续计算相机坐标用）
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
        // 新增：拍摄时的ROS时间戳（秒+纳秒）
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

        try
        {
            initialize_parameters();
            initialize_tf();
            initialize_paths();
            initialize_onnx_runtime();
            initialize_publishers_and_timers();
            load_processed_files();

            RCLCPP_INFO(this->get_logger(), "红色立方体检测器节点启动成功！");
            RCLCPP_INFO(this->get_logger(), "TF配置: %s -> %s", camera_frame_.c_str(), world_frame_.c_str());
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
    static constexpr float DEFAULT_DEPTH_SCALE = 1000.0f;    // 深度图缩放因子（毫米到米）
    static constexpr int DEFAULT_BORDER_VALUE = 114;         // 图像填充值
    static constexpr float MIN_VALID_DEPTH = 0.01f;          // 最小有效深度(米) - 降低到1cm
    static constexpr float MAX_VALID_DEPTH = 50.0f;          // 最大有效深度(米)
    static constexpr size_t MAX_LOG_FILE_SIZE = 1024 * 1024; // 1MB

    // COCO类别名称（简化版，只包含常用类别）
    const std::vector<std::string> class_names_ = {"red_cube"};

    // 初始化参数
    void initialize_parameters()
    {
        // 声明参数
        this->declare_parameter<std::string>("model_path", "models/red_cube_yolov11.onnx");
        this->declare_parameter<float>("confidence_threshold", 0.5f);
        this->declare_parameter<float>("nms_threshold", 0.45f);
        this->declare_parameter<int>("target_class_id", 0);
        this->declare_parameter<std::string>("image_save_path", "image_save");
        this->declare_parameter<int>("check_interval", 2000);

        // 修改后（深度相机内参）
        this->declare_parameter<float>("camera_fx", 454.69f); // 原615.0f
        this->declare_parameter<float>("camera_fy", 454.69f); // 原615.0f
        this->declare_parameter<float>("camera_cx", 424.50f); // 原320.0f
        this->declare_parameter<float>("camera_cy", 240.50f); // 原240.0f

        // 深度处理参数 - 修正默认深度缩放因子
        this->declare_parameter<float>("depth_scale", DEFAULT_DEPTH_SCALE); // 毫米到米的转换
        this->declare_parameter<float>("min_depth", 0.01f);                 // 最小有效深度(米) - 降低到1cm
        this->declare_parameter<float>("max_depth", 50.0f);                 // 最大有效深度(米)
        this->declare_parameter<bool>("auto_detect_depth_scale", true);     // 自动检测深度缩放因子

        // TF变换相关参数 - 基于launch文件的配置
        this->declare_parameter<std::string>("camera_frame", "drone_camera_link");
        this->declare_parameter<std::string>("world_frame", "map");
        this->declare_parameter<double>("transform_timeout", 5.0);

        // 新增参数
        this->declare_parameter<bool>("enable_fallback_mode", true);
        this->declare_parameter<bool>("publish_static_tf", false);
        this->declare_parameter<bool>("debug_coordinates", true); // 新增调试参数
        this->declare_parameter<bool>("wait_for_tf", true);       // 是否等待TF可用
        this->declare_parameter<double>("tf_wait_timeout", 10.0); // 等待TF的超时时间

        // 获取参数
        this->get_parameter("confidence_threshold", conf_threshold_);
        this->get_parameter("nms_threshold", nms_threshold_);
        this->get_parameter("target_class_id", target_class_id_);
        this->get_parameter("camera_frame", camera_frame_);
        this->get_parameter("world_frame", world_frame_);
        this->get_parameter("transform_timeout", transform_timeout_);
        this->get_parameter("enable_fallback_mode", enable_fallback_mode_);
        this->get_parameter("publish_static_tf", publish_static_tf_);
        this->get_parameter("debug_coordinates", debug_coordinates_);
        this->get_parameter("auto_detect_depth_scale", auto_detect_depth_scale_);
        this->get_parameter("wait_for_tf", wait_for_tf_);
        this->get_parameter("tf_wait_timeout", tf_wait_timeout_);

        int check_interval;
        this->get_parameter("check_interval", check_interval);
        check_interval_ms_ = std::chrono::milliseconds(check_interval);

        // 获取深度处理参数
        this->get_parameter("depth_scale", depth_scale_);
        this->get_parameter("min_depth", min_depth_);
        this->get_parameter("max_depth", max_depth_);

        // 验证参数合法性
        validate_parameters();

        // 初始化相机内参
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
        RCLCPP_INFO(this->get_logger(), "模式参数: fallback_mode=%s, debug_coordinates=%s, wait_for_tf=%s",
                    enable_fallback_mode_ ? "enabled" : "disabled",
                    debug_coordinates_ ? "enabled" : "disabled",
                    wait_for_tf_ ? "enabled" : "disabled");
    }

    // 初始化TF2
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

    // 验证参数合法性
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

    // 初始化路径
    void initialize_paths()
    {
        std::string model_rel_path, image_save_rel_path;
        this->get_parameter("model_path", model_rel_path);
        this->get_parameter("image_save_path", image_save_rel_path);

        // 尝试多个可能的路径位置
        std::vector<std::string> candidate_paths = {
            "/home/suda/drone_ugv_ws/src/image_processing",        // 源代码目录
            fs::current_path().string() + "/src/image_processing", // 当前目录下的src
            fs::current_path().string(),                           // 当前目录
        };

        // 尝试从ament_index获取install路径作为额外候选
        try
        {
            std::string install_path = ament_index_cpp::get_package_prefix("image_processing");
            candidate_paths.push_back(install_path);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "无法获取ament_index路径: %s", e.what());
        }

        // 查找模型文件
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

        // 查找或创建图像保存目录
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

        // 如果没有找到现有目录，尝试在源代码目录创建
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

    // 初始化ONNX Runtime
    void initialize_onnx_runtime()
    {
        RCLCPP_INFO(this->get_logger(), "初始化ONNX Runtime...");

        env_ = std::make_unique<Ort::Env>(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "RedCubeDetector");

        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(1);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        // 配置执行提供程序
        configure_execution_providers(session_options);

        // 创建会话
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

    // 配置执行提供程序
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
                cuda_options.gpu_mem_limit = 2ULL * 1024 * 1024 * 1024; // 2GB
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

    // 设置模型输入输出信息
    void setup_model_io_info()
    {
        // 获取输入信息
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

        // 获取输出信息
        size_t output_count = session_->GetOutputCount();
        output_names_.reserve(output_count);

        for (size_t i = 0; i < output_count; ++i)
        {
            Ort::AllocatedStringPtr output_name_alloc = session_->GetOutputNameAllocated(i, allocator_);
            output_names_.push_back(output_name_alloc.release());
            RCLCPP_INFO(this->get_logger(), "输出节点[%zu]: %s", i, output_names_[i]);
        }
    }

    // 初始化发布者和定时器
    void initialize_publishers_and_timers()
    {
        detection_pub_ = this->create_publisher<image_processing::msg::RedCubeDetection>(
            "/red_cube/detections", 10);

        check_timer_ = this->create_wall_timer(
            check_interval_ms_,
            std::bind(&RedCubeDetectorNode::check_new_files, this));
    }

    // 资源清理
    void cleanup_resources()
    {
        save_processed_files();

        // 清理ONNX资源
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

    // 新增：基于拍摄时的无人机位姿，构建历史TF变换（drone_camera_link → map）
    geometry_msgs::msg::TransformStamped build_historical_tf(const PoseData &pose_data) const
    {
        // 1. 构建：drone_base_link → map 的变换（拍摄时无人机在世界系的位置）
        geometry_msgs::msg::TransformStamped drone_base_to_map;
        drone_base_to_map.header.frame_id = "map";
        drone_base_to_map.child_frame_id = "drone_base_link";
        // 从pose_data读取拍摄时的无人机位姿
        drone_base_to_map.transform.translation.x = pose_data.pos_x;
        drone_base_to_map.transform.translation.y = pose_data.pos_y;
        drone_base_to_map.transform.translation.z = pose_data.pos_z;
        drone_base_to_map.transform.rotation.x = pose_data.ori_x;
        drone_base_to_map.transform.rotation.y = pose_data.ori_y;
        drone_base_to_map.transform.rotation.z = pose_data.ori_z;
        drone_base_to_map.transform.rotation.w = pose_data.ori_w;
        // 设置时间戳为拍摄时间
        drone_base_to_map.header.stamp.sec = pose_data.ros_timestamp_sec;
        drone_base_to_map.header.stamp.nanosec = pose_data.ros_timestamp_nsec;

        // 2. 构建：drone_camera_link → drone_base_link 的固定变换（与launch一致）
        geometry_msgs::msg::TransformStamped camera_to_drone_base;
        camera_to_drone_base.header.frame_id = "drone_base_link";
        camera_to_drone_base.child_frame_id = "drone_camera_link";
        // 相机相对于无人机本体的固定参数（与launch中的static_tf_drone_camera一致）
        camera_to_drone_base.transform.translation.x = 0.1;   // x偏移0.1m
        camera_to_drone_base.transform.translation.y = 0.0;   // y偏移0m
        camera_to_drone_base.transform.translation.z = 0.0; // z偏移-0.05m（相机在无人机下方）
        // 相机姿态：pitch=0.785rad（45°向下），yaw=0，roll=0 → 转换为四元数
        tf2::Quaternion q;
        q.setRPY(0.0, -0.785, 0.0); // roll, pitch, yaw
        camera_to_drone_base.transform.rotation.x = q.x();
        camera_to_drone_base.transform.rotation.y = q.y();
        camera_to_drone_base.transform.rotation.z = q.z();
        camera_to_drone_base.transform.rotation.w = q.w();
        camera_to_drone_base.header.stamp = drone_base_to_map.header.stamp; // 时间戳同步

        // 3. 合并变换：drone_camera_link → drone_base_link → map
        geometry_msgs::msg::TransformStamped camera_to_map;
        try
        {
            // 手动合并两个变换（tf2库函数）
            tf2::doTransform(camera_to_drone_base, camera_to_map, drone_base_to_map);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "合并历史TF变换失败: %s", ex.what());
            throw;
        }

        // 打印调试信息
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
        // 删掉汇总相关的变量（valid_detections 和 valid_world_coords），无需记录

        // 遍历所有检测到的目标，逐个处理
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

            // 像素→相机系→ROS相机系→世界系的转换逻辑（不变）
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
            cam_coords_ros.y() = -cam_coords_opencv.x();
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
            // Z轴异常警告
            const float z_threshold = 1.0f; // 可调整阈值，比如地面目标Z轴应接近0，超出±1m视为异常
            if (std::abs(world_point.point.z) > z_threshold)
            {
                RCLCPP_WARN(this->get_logger(), "世界坐标Z轴异常: %.3f m（地面目标应接近0，阈值±%.1f m）",
                            world_point.point.z, z_threshold);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "世界坐标验证通过（Z轴接近地面）");
            }

            // 打印当前目标的坐标信息（不变）
            RCLCPP_INFO(this->get_logger(),
                        "目标信息: 像素(%d,%d) | 深度%.3f m | "
                        "OpenCV相机系(%.3f,%.3f,%.3f) | "
                        "ROS相机系(%.3f,%.3f,%.3f) | "
                        "世界系(%.3f,%.3f,%.3f)",
                        center_x, center_y, depth,
                        cam_coords_opencv.x(), cam_coords_opencv.y(), cam_coords_opencv.z(),
                        cam_coords_ros.x(), cam_coords_ros.y(), cam_coords_ros.z(),
                        world_point.point.x, world_point.point.y, world_point.point.z);

            // 关键修复：正确调用save_detection_result（传目标索引，避免文件覆盖）
            int target_index = &det - &detections[0]; // 计算当前目标的序号（0,1,2...）
            save_detection_result(
                base_name, // 基础名称（函数内自动加索引）
                det,
                cam_coords_ros,
                Eigen::Vector3f(world_point.point.x, world_point.point.y, world_point.point.z),
                pose_data,
                target_index // 传入目标索引（修复参数不匹配）
            );

            // 发布当前目标的结果（不变）
            publish_detection_result(det, center_x, center_y, cam_coords_ros,
                                     Eigen::Vector3f(world_point.point.x, world_point.point.y, world_point.point.z),
                                     pose_data, base_name, historical_tf.header.stamp, true);
        }
    }

    // 文件管理相关方法
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

        // 确保目录存在
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

    // 文件检查和处理

    void check_new_files()
    {
        RCLCPP_DEBUG(this->get_logger(), "检查新文件...");

        try
        {
            const auto base_names = scan_for_image_files();

            RCLCPP_DEBUG(this->get_logger(), "找到 %zu 组图像文件", base_names.size());

            // 处理未处理的文件组（这部分保留，不变）
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

                // 仅处理"color_"开头且后缀为".jpg"的文件
                if (filename.substr(0, 6) == "color_" && file_ext == ".jpg")
                {
                    std::string base_name = file_path.stem().string().substr(6);
                    // 【新增日志】验证base_name对应的XML深度图是否存在
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

            // 新增：构建历史TF变换（关键修改，替代实时TF）
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

            // 关键修改：传入历史TF，而非实时时间戳
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
        // 1. 读取彩色图（不变）
        cv::Mat color_image = cv::imread(paths.color_path);
        if (color_image.empty())
        {
            RCLCPP_WARN(this->get_logger(), "无法加载彩色图像: %s", paths.color_path.c_str());
        }

        // 2. 【关键修改】读取XML格式的32FC1深度图
        cv::Mat depth_image;
        cv::FileStorage fs(paths.depth_path, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            RCLCPP_WARN(this->get_logger(), "无法加载深度图像XML文件: %s", paths.depth_path.c_str());
        }
        else
        {
            fs["depth_image"] >> depth_image; // 读取32FC1矩阵
            fs.release();

            // 验证深度图格式是否为32FC1
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

    // 图像处理和检测相关方法
    cv::Mat preprocess_image(const cv::Mat &image, float &scale, int &pad_w, int &pad_h) const
    {
        if (image.empty())
        {
            throw std::runtime_error("输入图像为空");
        }

        const int original_w = image.cols;
        const int original_h = image.rows;

        // 计算缩放比例和填充
        const float scale_x = static_cast<float>(input_width_) / original_w;
        const float scale_y = static_cast<float>(input_height_) / original_h;
        scale = std::min(scale_x, scale_y);

        const int new_w = static_cast<int>(original_w * scale);
        const int new_h = static_cast<int>(original_h * scale);

        pad_w = (input_width_ - new_w) / 2;
        pad_h = (input_height_ - new_h) / 2;

        // 调整图像大小
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(new_w, new_h));

        // 添加填充
        cv::Mat padded;
        cv::copyMakeBorder(resized, padded,
                           pad_h, input_height_ - new_h - pad_h,
                           pad_w, input_width_ - new_w - pad_w,
                           cv::BORDER_CONSTANT,
                           cv::Scalar(DEFAULT_BORDER_VALUE, DEFAULT_BORDER_VALUE, DEFAULT_BORDER_VALUE));

        // 转换为RGB并归一化
        cv::Mat rgb;
        cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);
        rgb.convertTo(rgb, CV_32FC3, 1.0 / 255.0);

        return rgb;
    }

    std::vector<Ort::Value> infer(const cv::Mat &input_image) const
    {
        const size_t input_size = 1 * 3 * input_height_ * input_width_;
        std::vector<float> input_data(input_size);

        // 转换为NCHW格式
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
        float scale, int pad_w, int pad_h, float adapted_fx, // 新增：适配后的fx
        float adapted_fy,                                    // 新增：适配后的fy
        float adapted_cx,                                    // 新增：适配后的cx
        float adapted_cy                                     // 新增：适配后的cy
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
            // 解析边界框坐标
            const float cx = output_data[i];
            const float cy = output_data[num_detections + i];
            const float w = output_data[2 * num_detections + i];
            const float h = output_data[3 * num_detections + i];

            // 找到最高置信度的类别
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

        // 应用NMS
        const auto keep_indices = apply_nms(boxes, scores, nms_threshold_);

        // 2. 构造检测结果时，记录适配内参（供后续计算相机坐标用）
        std::vector<Detection> detections;
        detections.reserve(keep_indices.size());
        for (const int idx : keep_indices)
        {
            const std::string class_name = (static_cast<size_t>(class_ids[idx]) < class_names_.size())
                                               ? class_names_[class_ids[idx]]
                                               : "unknown";
            // 存储适配内参到Detection结构体（需先扩展结构体）
            Detection det(boxes[idx], scores[idx], class_ids[idx], class_name);
            det.adapted_fx = adapted_fx; // 需在Detection结构体中添加这些成员
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
        // 转换为 (x1, y1, x2, y2) 并还原到原图尺寸
        const float x1 = (cx - w * 0.5f - pad_w) / scale;
        const float y1 = (cy - h * 0.5f - pad_h) / scale;
        const float x2 = (cx + w * 0.5f - pad_w) / scale;
        const float y2 = (cy + h * 0.5f - pad_h) / scale;

        // 限制边界框在图像范围内
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
        // 获取图像原始分辨率（848×480，与深度图一致）
        int original_width = color_image.cols;
        int original_height = color_image.rows;
        RCLCPP_DEBUG(this->get_logger(), "图像原始分辨率: %dx%d", original_width, original_height);

        // 【关键修改：基于深度相机的实际内参，无需额外缩放】
        // 原因：深度相机内参（fx=454.69, cx=424.5）已与848×480分辨率匹配，无需再乘以比例
        float adapted_fx = camera_intrinsics_->fx; // 直接使用深度相机的fx=454.69
        float adapted_fy = camera_intrinsics_->fy; // 直接使用深度相机的fy=454.69
        float adapted_cx = camera_intrinsics_->cx; // 直接使用深度相机的cx=424.5（非848/2=424，更精准）
        float adapted_cy = camera_intrinsics_->cy; // 直接使用深度相机的cy=240.5（非480/2=240）

        // 打印适配后的真实内参（验证是否与深度相机一致）
        RCLCPP_DEBUG(this->get_logger(), "适配后内参(深度相机实际值): fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                     adapted_fx, adapted_fy, adapted_cx, adapted_cy);

        float scale;
        int pad_w, pad_h;

        const cv::Mat input_image = preprocess_image(color_image, scale, pad_w, pad_h);
        auto outputs = infer(input_image);

        // 传递深度相机的实际内参到postprocess，供后续相机坐标计算
        return postprocess(outputs, original_width, original_height, scale, pad_w, pad_h,
                           adapted_fx, adapted_fy, adapted_cx, adapted_cy);
    }

    // 像素到相机坐标转换
    Eigen::Vector3f pixel_to_camera(float x_pixel, float y_pixel, float depth,
                                    float adapted_fx, // 适配后的fx
                                    float adapted_fy, // 适配后的fy
                                    float adapted_cx, // 适配后的cx
                                    float adapted_cy  // 适配后的cy
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

        // 2. 使用适配后的内参计算相机坐标（核心修改）
        const float Z_cam = depth;
        const float X_cam = (x_pixel - adapted_cx) * Z_cam / adapted_fx; // 用adapted_cx和adapted_fx
        const float Y_cam = (y_pixel - adapted_cy) * Z_cam / adapted_fy; // 用adapted_cy和adapted_fy

        if (debug_coordinates_)
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "像素坐标(%.1f, %.1f)，深度%.3fm -> 相机坐标(%.3f, %.3f, %.3f)m",
                         x_pixel, y_pixel, depth, X_cam, Y_cam, Z_cam);
        }

        return Eigen::Vector3f(X_cam, Y_cam, Z_cam);
    }

    // 保存检测结果到文件
    void save_detection_result(const std::string &base_name,
                               const Detection &det,
                               const Eigen::Vector3f &cam_coords,
                               const Eigen::Vector3f &world_coords,
                               const PoseData &pose_data,
                               int target_index) // 新增：目标索引（区分多个目标）
    {
        // 单个目标的结果文件（加索引）
        std::string result_file = image_save_path_ + "/detection_result_" + base_name + "_" + std::to_string(target_index) + ".txt";
        std::ofstream file(result_file);

        if (file.is_open())
        {
            file << "目标索引: " << target_index << "\n"; // 标记是第几个目标
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
            file << "\n无人机位置:\n";
            file << "位置: (" << pose_data.pos_x << ", " << pose_data.pos_y
                 << ", " << pose_data.pos_z << ")\n";
            file << "姿态(四元数): (" << pose_data.ori_w << ", " << pose_data.ori_x
                 << ", " << pose_data.ori_y << ", " << pose_data.ori_z << ")\n";

            RCLCPP_DEBUG(this->get_logger(), "目标%d结果已保存到: %s", target_index, result_file.c_str());
        }
    }

    bool is_valid_pixel_coord(int x, int y, const cv::Mat &image) const
    {
        return x >= 0 && x < image.cols && y >= 0 && y < image.rows;
    }

    float get_depth_value(int x, int y, const cv::Mat &depth_image) const
    {
        const int kernel = 5; // 5x5区域，可根据目标大小调整
        const int half_kernel = kernel / 2;
        std::vector<float> valid_depths; // 存储区域内所有有效深度值

        // 收集区域内的有效深度
        for (int dy = -half_kernel; dy <= half_kernel; dy++)
        {
            for (int dx = -half_kernel; dx <= half_kernel; dx++)
            {
                const int nx = x + dx;
                const int ny = y + dy;
                if (nx >= 0 && nx < depth_image.cols && ny >= 0 && ny < depth_image.rows)
                {
                    const float d = depth_image.at<float>(ny, nx);
                    // 筛选有效深度（排除NaN、无穷大、超出范围的值）
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

        // 对有效深度排序，取中位数
        std::sort(valid_depths.begin(), valid_depths.end());
        float median_depth;
        size_t n = valid_depths.size();
        if (n % 2 == 1)
        {
            median_depth = valid_depths[n / 2]; // 奇数个元素，取中间值
        }
        else
        {
            median_depth = (valid_depths[n / 2 - 1] + valid_depths[n / 2]) / 2.0f; // 偶数个元素，取中间两值平均
        }

        RCLCPP_DEBUG(this->get_logger(), "像素(%d,%d)：%zu个有效深度的中位数=%.3fm",
                     x, y, valid_depths.size(), median_depth);
        return median_depth;
    }

    // 位姿数据加载
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

        // 新增调试：打印读取的位姿数据，确认是否正确
        RCLCPP_DEBUG(this->get_logger(), "从 %s 读取位姿: pos=(%.3f,%.3f,%.3f), ori=(%.3f,%.3f,%.3f,%.3f)",
                     pose_path.c_str(),
                     pose_data.pos_x, pose_data.pos_y, pose_data.pos_z,
                     pose_data.ori_x, pose_data.ori_y, pose_data.ori_z, pose_data.ori_w);

        // 验证位姿是否有效（非零值）
        if (pose_data.pos_x == 0.0f && pose_data.pos_y == 0.0f && pose_data.pos_z == 0.0f)
        {
            RCLCPP_WARN(this->get_logger(), "位姿数据可能无效(全为0),请检查 pose 文件格式: %s", pose_path.c_str());
        }

        return pose_data;
    }

    void parse_pose_line(const std::string &line, PoseData &pose_data) const
    {
        // 1. 解析位置(x,y,z)
        if (line.find("位置(x,y,z):") != std::string::npos)
        {
            parse_field(line, "位置(x,y,z):", [&](const std::string &data)
                        {
                int count = sscanf(data.c_str(), "%f,%f,%f", 
                                &pose_data.pos_x, &pose_data.pos_y, &pose_data.pos_z);
                RCLCPP_DEBUG(this->get_logger(), "位置解析: (%f,%f,%f), 成功字段数=%d",
                            pose_data.pos_x, pose_data.pos_y, pose_data.pos_z, count); });
        }
        // 2. 解析姿态(四元数w,x,y,z)
        else if (line.find("姿态(四元数w,x,y,z):") != std::string::npos)
        {
            parse_field(line, "姿态(四元数w,x,y,z):", [&](const std::string &data)
                        {
                int count = sscanf(data.c_str(), "%f,%f,%f,%f",
                                &pose_data.ori_w, &pose_data.ori_x, &pose_data.ori_y, &pose_data.ori_z);
                RCLCPP_DEBUG(this->get_logger(), "姿态解析: (%f,%f,%f,%f), 成功字段数=%d",
                            pose_data.ori_w, pose_data.ori_x, pose_data.ori_y, pose_data.ori_z, count); });
        }
        // 3. 解析图像时间戳
        else if (line.find("图像时间戳:") != std::string::npos)
        {
            parse_field(line, "图像时间戳:", [&](const std::string &data)
                        {
                pose_data.timestamp = data;
                RCLCPP_DEBUG(this->get_logger(), "图像时间戳解析: %s", data.c_str()); });
        }
        // 4. 解析目标像素坐标
        else if (line.find("目标像素坐标:") != std::string::npos)
        {
            parse_field(line, "目标像素坐标:", [&](const std::string &data)
                        {
                int count = sscanf(data.c_str(), "(%d,%d)", &pose_data.target_x, &pose_data.target_y);
                RCLCPP_DEBUG(this->get_logger(), "像素坐标解析: (%d,%d), 成功字段数=%d",
                            pose_data.target_x, pose_data.target_y, count); });
        }
        // 5. 解析ROS时间戳
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

    // 辅助函数：通用字段解析（提取关键字后的有效数据，自动跳过空格）
    void parse_field(const std::string &line, const std::string &keyword,
                     std::function<void(const std::string &)> callback) const
    {
        size_t keyword_pos = line.find(keyword);
        if (keyword_pos == std::string::npos)
            return;

        // 从关键字后开始提取（跳过关键字本身）
        size_t data_start = keyword_pos + keyword.length();
        std::string data = line.substr(data_start);

        // 跳过开头的所有空格和制表符
        size_t trim_start = data.find_first_not_of(" \t");
        if (trim_start != std::string::npos)
        {
            data = data.substr(trim_start);
        }

        // 执行具体解析逻辑
        callback(data);
    }

    // 发布检测结果
    void publish_detection_result(const Detection &det, int center_x, int center_y,
                                  const Eigen::Vector3f &cam_coords,
                                  const Eigen::Vector3f &world_coords,
                                  const PoseData &pose_data,
                                  const std::string & /* base_name */,
                                  const rclcpp::Time &timestamp,
                                  bool world_coords_valid) const
    {
        auto detection_msg = image_processing::msg::RedCubeDetection();

        // 设置时间戳和坐标系
        detection_msg.header.stamp = timestamp;
        detection_msg.header.frame_id = world_coords_valid ? world_frame_ : camera_frame_;

        // 设置检测结果
        detection_msg.class_id = det.class_id;
        detection_msg.class_name = det.class_name;
        detection_msg.confidence = det.confidence;
        detection_msg.xmin = det.box.x;
        detection_msg.ymin = det.box.y;
        detection_msg.xmax = det.box.x + det.box.width;
        detection_msg.ymax = det.box.y + det.box.height;
        detection_msg.center_x = center_x;
        detection_msg.center_y = center_y;

        // 相机坐标系下的位置
        detection_msg.cam_x = cam_coords.x();
        detection_msg.cam_y = cam_coords.y();
        detection_msg.cam_z = cam_coords.z();

        // 世界坐标系下的位置（如果变换成功）
        if (world_coords_valid)
        {
            detection_msg.world_x = world_coords.x();
            detection_msg.world_y = world_coords.y();
            detection_msg.world_z = world_coords.z();
        }
        else
        {
            // 如果变换失败，设置为NaN
            detection_msg.world_x = std::numeric_limits<float>::quiet_NaN();
            detection_msg.world_y = std::numeric_limits<float>::quiet_NaN();
            detection_msg.world_z = std::numeric_limits<float>::quiet_NaN();
        }

        // 设置无人机位姿信息
        detection_msg.uav_x = pose_data.pos_x;
        detection_msg.uav_y = pose_data.pos_y;
        detection_msg.uav_z = pose_data.pos_z;
        detection_msg.uav_qx = pose_data.ori_x;
        detection_msg.uav_qy = pose_data.ori_y;
        detection_msg.uav_qz = pose_data.ori_z;
        detection_msg.uav_qw = pose_data.ori_w;

        detection_pub_->publish(detection_msg);

        std::string result_status = world_coords_valid ? "成功" : (enable_fallback_mode_ ? "后备模式" : "失败");

        RCLCPP_INFO(this->get_logger(), "已发布检测结果: %s, 世界坐标变换%s",
                    det.class_name.c_str(), result_status.c_str());
    }

    // 成员变量
    std::unique_ptr<CameraIntrinsics> camera_intrinsics_;

    // TF2相关
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::string camera_frame_;
    std::string world_frame_;
    double transform_timeout_;

    // TF状态监控
    bool tf_status_reported_ = false;

    // 新增功能参数
    bool enable_fallback_mode_ = true;
    bool publish_static_tf_ = false;
    bool debug_coordinates_ = true;
    bool auto_detect_depth_scale_ = false;
    bool wait_for_tf_ = true;       // 是否等待TF可用
    double tf_wait_timeout_ = 10.0; // 等待TF的超时时间

    // ONNX Runtime相关
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    Ort::AllocatorWithDefaultOptions allocator_;
    const char *input_name_ = nullptr;
    std::vector<const char *> output_names_;
    std::vector<int64_t> input_dims_;
    int input_width_ = 0;
    int input_height_ = 0;

    // 路径
    std::string model_path_;
    std::string image_save_path_;

    // 文件管理
    std::unordered_set<std::string> processed_files_;
    mutable std::mutex processed_files_mutex_;

    // ROS2组件
    rclcpp::TimerBase::SharedPtr check_timer_;
    rclcpp::Publisher<image_processing::msg::RedCubeDetection>::SharedPtr detection_pub_;

    // 配置参数
    float conf_threshold_ = 0.5f;
    float nms_threshold_ = 0.45f;
    int target_class_id_ = 0;
    std::chrono::milliseconds check_interval_ms_{2000};

    // 深度处理参数
    float depth_scale_ = DEFAULT_DEPTH_SCALE;
    float min_depth_ = 0.1f;
    float max_depth_ = 50.0f;
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