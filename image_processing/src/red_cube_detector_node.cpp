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

        Detection() = default;
        Detection(const cv::Rect &b, float conf, int id, const std::string &name)
            : box(b), confidence(conf), class_id(id), class_name(name) {}
    };

    // 位姿数据结构体
    struct PoseData
    {
        std::string timestamp;
        float pos_x = 0.0f, pos_y = 0.0f, pos_z = 0.0f;
        float ori_x = 0.0f, ori_y = 0.0f, ori_z = 0.0f, ori_w = 1.0f;
        int target_x = 0, target_y = 0;

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
    const std::vector<std::string> class_names_ = {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
        "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
        "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
        "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
        "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
        "hair drier", "toothbrush"};

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

        // 相机内参
        this->declare_parameter<float>("camera_fx", 615.0f);
        this->declare_parameter<float>("camera_fy", 615.0f);
        this->declare_parameter<float>("camera_cx", 320.0f);
        this->declare_parameter<float>("camera_cy", 240.0f);

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

        // 等待变换可用
        RCLCPP_INFO(this->get_logger(), "等待TF变换可用: %s -> %s",
                    camera_frame_.c_str(), world_frame_.c_str());

        if (wait_for_tf_)
        {
            wait_for_transform();
        }

        if (enable_fallback_mode_)
        {
            RCLCPP_INFO(this->get_logger(), "后备模式已启用：当TF变换不可用时，将使用相机坐标系");
        }

        // 启动TF检查定时器
        tf_check_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&RedCubeDetectorNode::check_tf_status, this));
    }

    // 等待TF变换可用
    void wait_for_transform()
    {
        auto start_time = std::chrono::steady_clock::now();
        bool transform_available = false;

        while (rclcpp::ok() && !transform_available)
        {
            try
            {
                // 尝试查找变换
                auto transform = tf_buffer_->lookupTransform(
                    world_frame_, camera_frame_,
                    tf2::TimePointZero,
                    tf2::durationFromSec(1.0));

                transform_available = true;
                RCLCPP_INFO(this->get_logger(), "TF变换链已就绪: %s -> %s",
                            camera_frame_.c_str(), world_frame_.c_str());

                // 输出变换信息
                auto t = transform.transform.translation;
                auto r = transform.transform.rotation;
                RCLCPP_INFO(this->get_logger(),
                            "初始相机位置 - 平移: (%.3f, %.3f, %.3f), 旋转: (%.3f, %.3f, %.3f, %.3f)",
                            t.x, t.y, t.z, r.x, r.y, r.z, r.w);
            }
            catch (const tf2::TransformException &ex)
            {
                auto elapsed = std::chrono::steady_clock::now() - start_time;
                auto elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

                if (elapsed_sec > tf_wait_timeout_)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "等待TF变换超时(%.1fs)，继续运行。错误: %s",
                                tf_wait_timeout_, ex.what());
                    break;
                }

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "等待TF变换中... (已等待 %ld 秒)", elapsed_sec);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

    // 检查TF状态
    void check_tf_status()
    {
        try
        {
            // 检查完整的TF链
            std::vector<std::string> tf_chain = {"map", "drone_base_link", "drone_camera_link"};

            // 检查每个变换
            for (size_t i = 0; i < tf_chain.size() - 1; ++i)
            {
                try
                {
                    auto transform = tf_buffer_->lookupTransform(
                        tf_chain[i], tf_chain[i + 1], tf2::TimePointZero);

                    if (debug_coordinates_ && !tf_status_reported_)
                    {
                        auto t = transform.transform.translation;
                        auto r = transform.transform.rotation;
                        RCLCPP_INFO(this->get_logger(),
                                    "TF: %s -> %s: 平移(%.3f,%.3f,%.3f) 旋转(%.3f,%.3f,%.3f,%.3f)",
                                    tf_chain[i].c_str(), tf_chain[i + 1].c_str(),
                                    t.x, t.y, t.z, r.x, r.y, r.z, r.w);
                    }
                }
                catch (const tf2::TransformException &ex)
                {
                    RCLCPP_DEBUG(this->get_logger(), "TF变换不可用: %s -> %s",
                                 tf_chain[i].c_str(), tf_chain[i + 1].c_str());
                }
            }

            // 检查完整链
            auto full_transform = tf_buffer_->lookupTransform(world_frame_, camera_frame_, tf2::TimePointZero);

            if (!tf_status_reported_)
            {
                RCLCPP_INFO(this->get_logger(), "TF变换链完整可用: %s -> %s",
                            camera_frame_.c_str(), world_frame_.c_str());

                if (debug_coordinates_)
                {
                    auto t = full_transform.transform.translation;
                    auto r = full_transform.transform.rotation;
                    RCLCPP_INFO(this->get_logger(),
                                "完整变换 - 相机在世界系中位置: (%.3f, %.3f, %.3f), 姿态: (%.3f, %.3f, %.3f, %.3f)",
                                t.x, t.y, t.z, r.x, r.y, r.z, r.w);
                }
                tf_status_reported_ = true;
            }
        }
        catch (const tf2::TransformException &ex)
        {
            if (tf_status_reported_ || (this->now() - this->get_clock()->now()).seconds() > 30)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                     "TF变换不可用: %s -> %s, 错误: %s",
                                     camera_frame_.c_str(), world_frame_.c_str(), ex.what());
                tf_status_reported_ = false;
            }
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

    // 改进的坐标变换方法
    bool transform_to_world(const Eigen::Vector3f &cam_coords,
                            const rclcpp::Time &timestamp,
                            Eigen::Vector3f &world_coords)
    {
        geometry_msgs::msg::PointStamped cam_point, world_point;
        cam_point.header.frame_id = camera_frame_;
        cam_point.point.x = cam_coords.x();
        cam_point.point.y = cam_coords.y();
        cam_point.point.z = cam_coords.z();

        try
        {
            // 对于离线处理，使用最新可用的变换
            if (timestamp.seconds() == 0.0)
            {
                // 使用最新变换
                auto transform = tf_buffer_->lookupTransform(
                    world_frame_, camera_frame_,
                    tf2::TimePointZero,
                    tf2::durationFromSec(transform_timeout_));

                cam_point.header.stamp = transform.header.stamp;
                tf2::doTransform(cam_point, world_point, transform);

                if (debug_coordinates_)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "坐标变换成功(最新): 相机系(%.3f,%.3f,%.3f) -> 世界系(%.3f,%.3f,%.3f)",
                                cam_coords.x(), cam_coords.y(), cam_coords.z(),
                                world_point.point.x, world_point.point.y, world_point.point.z);
                }
            }
            else
            {
                // 尝试使用指定时间戳
                cam_point.header.stamp = timestamp;
                world_point = tf_buffer_->transform(
                    cam_point, world_frame_, tf2::durationFromSec(transform_timeout_));

                if (debug_coordinates_)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "坐标变换成功(时间戳): 相机系(%.3f,%.3f,%.3f) -> 世界系(%.3f,%.3f,%.3f)",
                                cam_coords.x(), cam_coords.y(), cam_coords.z(),
                                world_point.point.x, world_point.point.y, world_point.point.z);
                }
            }

            world_coords.x() = world_point.point.x;
            world_coords.y() = world_point.point.y;
            world_coords.z() = world_point.point.z;

            // 合理性检查
            if (debug_coordinates_)
            {
                // 检查高度是否合理（无人机一般在0-100米高度）
                if (world_coords.z() < -5.0f || world_coords.z() > 100.0f)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "世界坐标Z值可能异常: %.3f m", world_coords.z());
                }
            }

            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            if (enable_fallback_mode_)
            {
                RCLCPP_DEBUG(this->get_logger(), "TF变换失败，使用后备模式: %s", ex.what());
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "TF变换失败: %s", ex.what());
            }
            return false;
        }
    }

    bool check_transform_available() const
    {
        try
        {
            tf_buffer_->lookupTransform(world_frame_, camera_frame_, tf2::TimePointZero,
                                        tf2::durationFromSec(0.1));
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            return false;
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

        // 检查TF变换是否可用
        bool tf_available = check_transform_available();

        if (!tf_available && !enable_fallback_mode_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                 "TF变换不可用: %s -> %s, 跳过处理",
                                 camera_frame_.c_str(), world_frame_.c_str());
            return;
        }

        if (!tf_available && enable_fallback_mode_)
        {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                                  "TF变换不可用，使用后备模式（仅相机坐标系）");
        }

        try
        {
            const auto base_names = scan_for_image_files();

            RCLCPP_DEBUG(this->get_logger(), "找到 %zu 组图像文件", base_names.size());

            // 处理未处理的文件组
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

        // 检查目录是否存在
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
                {
                    continue;
                }

                const std::string filename = entry.path().filename().string();
                if (filename.substr(0, 6) == "color_" && filename.size() >= 4 &&
                    filename.substr(filename.size() - 4) == ".jpg")
                {
                    std::string base_name = filename.substr(6);
                    const size_t ext_pos = base_name.find(".jpg");
                    if (ext_pos != std::string::npos)
                    {
                        base_name = base_name.substr(0, ext_pos);
                        base_names.insert(base_name);
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

            // 对离线处理使用Time(0,0)以获取最新TF
            rclcpp::Time timestamp = rclcpp::Time(0, 0);

            process_detections(detections, images.second, pose_data, base_name, timestamp);
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
        paths.depth_path = base_path + "depth_" + base_name + ".png";
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
        cv::Mat depth_image = cv::imread(paths.depth_path, cv::IMREAD_ANYDEPTH);

        if (color_image.empty())
        {
            RCLCPP_WARN(this->get_logger(), "无法加载彩色图像: %s", paths.color_path.c_str());
        }
        if (depth_image.empty())
        {
            RCLCPP_WARN(this->get_logger(), "无法加载深度图像: %s", paths.depth_path.c_str());
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

    std::vector<Detection> postprocess(std::vector<Ort::Value> &outputs,
                                       int original_w, int original_h,
                                       float scale, int pad_w, int pad_h) const
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

        // 构造最终检测结果
        std::vector<Detection> detections;
        detections.reserve(keep_indices.size());

        for (const int idx : keep_indices)
        {
            const std::string class_name = (static_cast<size_t>(class_ids[idx]) < class_names_.size())
                                               ? class_names_[class_ids[idx]]
                                               : "unknown";
            detections.emplace_back(boxes[idx], scores[idx], class_ids[idx], class_name);
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
        float scale;
        int pad_w, pad_h;

        const cv::Mat input_image = preprocess_image(color_image, scale, pad_w, pad_h);
        auto outputs = infer(input_image);

        return postprocess(outputs, color_image.cols, color_image.rows, scale, pad_w, pad_h);
    }

    // 像素到相机坐标转换
    Eigen::Vector3f pixel_to_camera(float x_pixel, float y_pixel, float depth) const
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

        // 标准相机坐标系转换：Z轴向前，X轴向右，Y轴向下
        const float Z_cam = depth;
        const float X_cam = (x_pixel - camera_intrinsics_->cx) * Z_cam / camera_intrinsics_->fx;
        const float Y_cam = (y_pixel - camera_intrinsics_->cy) * Z_cam / camera_intrinsics_->fy;

        if (debug_coordinates_)
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "像素坐标(%.1f, %.1f)，深度%.3fm -> 相机坐标(%.3f, %.3f, %.3f)m",
                         x_pixel, y_pixel, depth, X_cam, Y_cam, Z_cam);
        }

        return Eigen::Vector3f(X_cam, Y_cam, Z_cam);
    }

    void process_detections(const std::vector<Detection> &detections,
                            const cv::Mat &depth_image,
                            const PoseData &pose_data,
                            const std::string &base_name,
                            const rclcpp::Time &timestamp)
    {
        for (const auto &det : detections)
        {
            if (det.class_id != target_class_id_)
            {
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
            if (depth <= min_depth_)
            {
                RCLCPP_WARN(this->get_logger(), "深度值太小: %.3f m (最小值: %.2f m)", depth, min_depth_);
                continue;
            }
            if (depth > max_depth_)
            {
                RCLCPP_WARN(this->get_logger(), "深度值太大: %.3f m (最大值: %.1f m)", depth, max_depth_);
                continue;
            }

            const Eigen::Vector3f cam_coords = pixel_to_camera(center_x, center_y, depth);
            if (std::isnan(cam_coords.x()))
            {
                RCLCPP_WARN(this->get_logger(), "无法计算相机坐标系下的坐标");
                continue;
            }

            // 尝试转换到世界坐标系
            Eigen::Vector3f world_coords;
            bool transform_success = transform_to_world(cam_coords, timestamp, world_coords);

            RCLCPP_INFO(this->get_logger(),
                        "目标中心点: (%d, %d), 深度: %.3f m, 相机坐标: (%.3f, %.3f, %.3f) m",
                        center_x, center_y, depth, cam_coords.x(), cam_coords.y(), cam_coords.z());

            if (transform_success)
            {
                RCLCPP_INFO(this->get_logger(), "世界坐标: (%.3f, %.3f, %.3f) m",
                            world_coords.x(), world_coords.y(), world_coords.z());

                // 保存结果到文件
                save_detection_result(base_name, det, cam_coords, world_coords, pose_data);
            }
            else if (enable_fallback_mode_)
            {
                RCLCPP_INFO(this->get_logger(), "使用后备模式，仅输出相机坐标系结果");
            }

            publish_detection_result(det, center_x, center_y, cam_coords, world_coords,
                                     pose_data, base_name, timestamp, transform_success);
        }
    }

    // 保存检测结果到文件
    void save_detection_result(const std::string &base_name,
                               const Detection &det,
                               const Eigen::Vector3f &cam_coords,
                               const Eigen::Vector3f &world_coords,
                               const PoseData &pose_data) const
    {
        std::string result_file = image_save_path_ + "/detection_results_" + base_name + ".txt";
        std::ofstream file(result_file);

        if (file.is_open())
        {
            file << "检测结果:\n";
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

            RCLCPP_DEBUG(this->get_logger(), "检测结果已保存到: %s", result_file.c_str());
        }
    }

    bool is_valid_pixel_coord(int x, int y, const cv::Mat &image) const
    {
        return x >= 0 && x < image.cols && y >= 0 && y < image.rows;
    }

    float get_depth_value(int x, int y, const cv::Mat &depth_image) const
    {
        // 检查边界
        if (x < 0 || x >= depth_image.cols || y < 0 || y >= depth_image.rows)
        {
            return 0.0f;
        }

        uint16_t raw_depth = depth_image.at<uint16_t>(y, x);

        // 检查无效深度值（通常为0）
        if (raw_depth == 0)
        {
            if (debug_coordinates_)
            {
                RCLCPP_WARN(this->get_logger(), "像素(%d,%d)深度值为0，尝试邻域搜索", x, y);
            }

            // 尝试在3x3邻域内找到有效深度值
            std::vector<uint16_t> valid_depths;
            for (int dy = -1; dy <= 1; dy++)
            {
                for (int dx = -1; dx <= 1; dx++)
                {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx >= 0 && nx < depth_image.cols && ny >= 0 && ny < depth_image.rows)
                    {
                        uint16_t neighbor_depth = depth_image.at<uint16_t>(ny, nx);
                        if (neighbor_depth > 0)
                        {
                            valid_depths.push_back(neighbor_depth);
                        }
                    }
                }
            }

            if (!valid_depths.empty())
            {
                // 使用中值作为估计值
                std::sort(valid_depths.begin(), valid_depths.end());
                raw_depth = valid_depths[valid_depths.size() / 2];
                if (debug_coordinates_)
                {
                    RCLCPP_INFO(this->get_logger(), "使用邻域中值深度: %u", raw_depth);
                }
            }
            else
            {
                if (debug_coordinates_)
                {
                    RCLCPP_WARN(this->get_logger(), "邻域内无有效深度值");
                }
                return 0.0f;
            }
        }

        float depth_meters;
        if (auto_detect_depth_scale_)
        {
            // 自动检测深度缩放因子
            depth_meters = auto_scale_depth(raw_depth);
        }
        else
        {
            depth_meters = raw_depth / depth_scale_;
        }

        // 添加调试信息
        if (debug_coordinates_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "像素(%d,%d): 原始深度=%u, 缩放因子=%.1f, 转换后深度=%.3fm",
                        x, y, raw_depth, depth_scale_, depth_meters);
        }

        return depth_meters;
    }

    // 自动检测深度缩放并转换
    float auto_scale_depth(uint16_t raw_depth) const
    {
        // 常见的深度图格式：
        // 1. 毫米单位：缩放因子1000 (raw_depth / 1000.0)
        // 2. 0.1毫米单位：缩放因子10000 (raw_depth / 10000.0)
        // 3. 微米单位：缩放因子1000000 (raw_depth / 1000000.0)
        // 4. 直接米单位：缩放因子1 (raw_depth / 1.0)

        float depth_mm = raw_depth / 1000.0f;     // 假设毫米
        float depth_0_1mm = raw_depth / 10000.0f; // 假设0.1毫米
        float depth_um = raw_depth / 1000000.0f;  // 假设微米
        float depth_m = raw_depth / 1.0f;         // 假设直接米

        // 基于合理性选择最佳缩放因子
        // 一般相机检测距离在0.1-10米范围内比较合理
        std::vector<std::pair<float, std::string>> candidates = {
            {depth_mm, "毫米单位(÷1000)"},
            {depth_0_1mm, "0.1毫米单位(÷10000)"},
            {depth_um, "微米单位(÷1000000)"},
            {depth_m, "米单位(÷1)"}};

        for (const auto &[depth, desc] : candidates)
        {
            if (depth >= 0.05f && depth <= 20.0f) // 5cm到20m的合理范围
            {
                if (debug_coordinates_)
                {
                    RCLCPP_DEBUG(this->get_logger(),
                                 "自动选择深度格式: %s, 原始值%u -> %.3fm",
                                 desc.c_str(), raw_depth, depth);
                }
                return depth;
            }
        }

        // 如果都不在合理范围内，使用默认的毫米转米
        if (debug_coordinates_)
        {
            RCLCPP_WARN(this->get_logger(),
                        "深度值%u无法自动识别格式，使用默认毫米单位，结果: %.3fm",
                        raw_depth, depth_mm);
        }
        return depth_mm;
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

        return pose_data;
    }

    void parse_pose_line(const std::string &line, PoseData &pose_data) const
    {
        if (line.substr(0, 11) == "位置(x,y,z):")
        {
            const std::string data = line.substr(12);
            sscanf(data.c_str(), "%f,%f,%f", &pose_data.pos_x, &pose_data.pos_y, &pose_data.pos_z);
        }
        else if (line.substr(0, 19) == "姿态(四元数w,x,y,z):")
        {
            const std::string data = line.substr(20);
            sscanf(data.c_str(), "%f,%f,%f,%f",
                   &pose_data.ori_w, &pose_data.ori_x, &pose_data.ori_y, &pose_data.ori_z);
        }
        else if (line.substr(0, 6) == "图像时间戳:")
        {
            pose_data.timestamp = line.substr(7);
        }
        else if (line.substr(0, 6) == "目标像素坐标:")
        {
            const std::string data = line.substr(7);
            sscanf(data.c_str(), "(%d,%d)", &pose_data.target_x, &pose_data.target_y);
        }
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
    rclcpp::TimerBase::SharedPtr tf_check_timer_;
    bool tf_status_reported_ = false;

    // 新增功能参数
    bool enable_fallback_mode_ = true;
    bool publish_static_tf_ = false;
    bool debug_coordinates_ = true;
    bool auto_detect_depth_scale_ = true;
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