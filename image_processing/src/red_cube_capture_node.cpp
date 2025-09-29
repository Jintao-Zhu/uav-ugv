#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "opencv4/opencv2/opencv.hpp"
#include "onnxruntime_cxx_api.h"
#include "ament_index_cpp/get_package_prefix.hpp"
#include <filesystem>
#include <string>
#include <vector>
#include <memory>
#include <unordered_set>
#include <chrono>
#include <numeric>
#include <algorithm>
#include <thread>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <fstream>

namespace fs = std::filesystem;
using ImageConstPtr = sensor_msgs::msg::Image::ConstSharedPtr;
using namespace std::chrono_literals;

typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image>
    ImageSyncPolicy;

class YoloDetectorNode : public rclcpp::Node
{
public:
    struct Detection
    {
        cv::Rect box;
        float confidence;
        int class_id;
        std::string class_name;
    };

    YoloDetectorNode() : Node("yolo_detector_node")
    {
        RCLCPP_INFO(this->get_logger(), "开始初始化YOLO检测器节点...");

        try
        {
            // 1. 获取功能包安装路径，再反向推导源路径
            std::string package_path;
            try
            {
                std::string package_prefix = ament_index_cpp::get_package_prefix("image_processing");
                size_t pos = package_prefix.find("/install/");
                if (pos != std::string::npos)
                {
                    package_path = package_prefix.substr(0, pos) + "/src/image_processing/";
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "无法自动推导源目录，使用默认src路径");
                    package_path = "/home/suda/drone_ugv_ws/src/image_processing";
                }
                RCLCPP_INFO(this->get_logger(), "当前功能包源目录路径: %s", package_path.c_str());
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "获取功能包路径失败: %s", e.what());
                throw std::runtime_error("获取功能包路径失败");
            }

            // 2. 拼接图像保存路径
            this->declare_parameter<std::string>("image_save_path", "image_save/");
            std::string rel_save_path;
            this->get_parameter("image_save_path", rel_save_path);
            image_save_path_ = package_path + "/" + rel_save_path;

            RCLCPP_INFO(this->get_logger(), "最终图像保存路径: %s", image_save_path_.c_str());

            // 修改模型路径处理
            std::string model_rel_path;
            this->declare_parameter<std::string>("model_path", "models/red_cube_yolov11.onnx");
            this->get_parameter("model_path", model_rel_path);
            std::string model_path = package_path + "/" + model_rel_path;

            // 声明和获取参数
            this->declare_parameter<float>("confidence_threshold", 0.5f);
            this->declare_parameter<float>("nms_threshold", 0.45f);
            this->declare_parameter<int>("target_class_id", 0);
            this->declare_parameter<std::string>("color_topic", "/camera/image_raw");
            this->declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
            this->declare_parameter<int>("queue_size", 10);

            std::string color_topic, depth_topic;
            int queue_size;

            this->get_parameter("confidence_threshold", conf_threshold_);
            this->get_parameter("nms_threshold", nms_threshold_);
            this->get_parameter("target_class_id", target_class_id_);
            this->get_parameter("color_topic", color_topic);
            this->get_parameter("depth_topic", depth_topic);
            this->get_parameter("queue_size", queue_size);

            RCLCPP_INFO(this->get_logger(), "参数获取完成:");
            RCLCPP_INFO(this->get_logger(), "  - 模型路径: %s", model_path.c_str());
            RCLCPP_INFO(this->get_logger(), "  - 图像保存路径: %s", image_save_path_.c_str());
            RCLCPP_INFO(this->get_logger(), "  - 置信度阈值: %.2f", conf_threshold_);
            RCLCPP_INFO(this->get_logger(), "  - NMS阈值: %.2f", nms_threshold_);
            RCLCPP_INFO(this->get_logger(), "  - 输入尺寸: 待从模型动态获取");
            RCLCPP_INFO(this->get_logger(), "  - 目标类别ID: %d", target_class_id_);
            RCLCPP_INFO(this->get_logger(), "  - 彩色图话题: %s", color_topic.c_str());
            RCLCPP_INFO(this->get_logger(), "  - 深度图话题: %s", depth_topic.c_str());

            // 创建保存目录
            if (!fs::exists(image_save_path_))
            {
                RCLCPP_INFO(this->get_logger(), "创建图像保存目录: %s", image_save_path_.c_str());
                if (!fs::create_directories(image_save_path_))
                {
                    RCLCPP_ERROR(this->get_logger(), "无法创建保存目录: %s", image_save_path_.c_str());
                    throw std::runtime_error("无法创建保存目录");
                }
            }

            // 初始化ONNX Runtime
            RCLCPP_INFO(this->get_logger(), "开始初始化ONNX Runtime...");
            init_onnx_runtime(model_path);
            RCLCPP_INFO(this->get_logger(), "ONNX Runtime初始化完成");

            // 初始化消息计数器和定时器
            color_msg_count_ = 0;
            depth_msg_count_ = 0;
            sync_msg_count_ = 0;
            last_color_time_ = std::chrono::steady_clock::now();
            last_depth_time_ = std::chrono::steady_clock::now();
            last_sync_time_ = std::chrono::steady_clock::now();

            // 创建监控定时器
            monitor_timer_ = this->create_wall_timer(
                5s, std::bind(&YoloDetectorNode::monitor_callback, this));

            // 初始化订阅器和同步器
            RCLCPP_INFO(this->get_logger(), "创建图像订阅器...");
            color_sub_.subscribe(this, color_topic, rmw_qos_profile_sensor_data);
            depth_sub_.subscribe(this, depth_topic, rmw_qos_profile_sensor_data);

            RCLCPP_INFO(this->get_logger(), "创建消息同步器...");
            sync_ = std::make_shared<message_filters::Synchronizer<ImageSyncPolicy>>(
                ImageSyncPolicy(queue_size), color_sub_, depth_sub_);

            sync_->registerCallback(
                std::bind(
                    &YoloDetectorNode::image_callback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

            // 创建监控订阅器
            color_monitor_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                color_topic, rclcpp::SensorDataQoS(),
                std::bind(&YoloDetectorNode::color_monitor_callback, this, std::placeholders::_1));

            depth_monitor_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                depth_topic, rclcpp::SensorDataQoS(),
                std::bind(&YoloDetectorNode::depth_monitor_callback, this, std::placeholders::_1));

            // 【修改】创建无人机位姿订阅器（使用SensorDataQoS匹配发布者）
            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/drone/pose",
                rclcpp::SensorDataQoS(), // 改用SensorDataQoS
                std::bind(&YoloDetectorNode::pose_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "已创建无人机位姿订阅器，订阅话题: /drone/pose");

            // 初始化CSV索引文件
            pose_csv_filename_ = image_save_path_ + "image_poses.csv";
            if (!fs::exists(pose_csv_filename_))
            {
                std::ofstream csv_file(pose_csv_filename_);
                if (csv_file.is_open())
                {
                    csv_file << "image_count,timestamp,pos_x,pos_y,pos_z,ori_x,ori_y,ori_z,ori_w,target_x,target_y\n";
                    csv_file.close();
                }
            }

            // 初始化图像计数
            image_count_ = 0;

            // 【新增】等待位姿数据（最多5秒）
            RCLCPP_INFO(this->get_logger(), "等待无人机位姿数据...");
            auto start_wait = std::chrono::steady_clock::now();
            while (!pose_received_ && rclcpp::ok())
            {
                rclcpp::spin_some(this->get_node_base_interface());

                auto elapsed = std::chrono::steady_clock::now() - start_wait;
                if (elapsed > 5s)
                {
                    RCLCPP_WARN(this->get_logger(), "5秒内未接收到位姿数据，节点将继续运行但会跳过无效位姿的保存");
                    break;
                }

                std::this_thread::sleep_for(100ms);
            }

            if (pose_received_)
            {
                RCLCPP_INFO(this->get_logger(), "✓ 已接收到位姿数据，节点就绪！");
            }

            RCLCPP_INFO(this->get_logger(), "YOLO检测器节点启动成功！");
            check_topics_availability(color_topic, depth_topic);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "节点初始化失败: %s", e.what());
            rclcpp::shutdown();
        }
    }

    ~YoloDetectorNode()
    {
        RCLCPP_INFO(this->get_logger(), "正在关闭YOLO检测器节点...");
        if (input_name_)
        {
            allocator_.Free(const_cast<char *>(input_name_));
        }
        for (const char *name : output_names_)
        {
            if (name)
            {
                allocator_.Free(const_cast<char *>(name));
            }
        }
        RCLCPP_INFO(this->get_logger(), "YOLO检测器节点已关闭");
    }

private:
    std::vector<std::string> class_names = {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
        "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
        "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
        "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
        "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
        "hair drier", "toothbrush"};

    void check_topics_availability(const std::string &color_topic, const std::string &depth_topic)
    {
        RCLCPP_INFO(this->get_logger(), "检查话题可用性...");

        auto topic_names = this->get_topic_names_and_types();
        bool color_found = false, depth_found = false;

        for (const auto &[topic_name, topic_types] : topic_names)
        {
            if (topic_name == color_topic)
                color_found = true;
            if (topic_name == depth_topic)
                depth_found = true;
        }

        if (!color_found)
            RCLCPP_WARN(this->get_logger(), "未找到彩色图话题: %s", color_topic.c_str());
        if (!depth_found)
            RCLCPP_WARN(this->get_logger(), "未找到深度图话题: %s", depth_topic.c_str());
    }

    void monitor_callback()
    {
        auto now = std::chrono::steady_clock::now();

        RCLCPP_INFO(this->get_logger(), "=== 状态监控 ===");
        RCLCPP_INFO(this->get_logger(), "彩色图消息接收数: %zu", color_msg_count_.load());
        RCLCPP_INFO(this->get_logger(), "深度图消息接收数: %zu", depth_msg_count_.load());
        RCLCPP_INFO(this->get_logger(), "同步消息处理数: %zu", sync_msg_count_.load());
        RCLCPP_INFO(this->get_logger(), "已保存图像数: %zu", image_count_.load());
        RCLCPP_INFO(this->get_logger(), "位姿接收状态: %s", pose_received_ ? "正常" : "未接收");

        auto color_elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_color_time_).count();
        auto depth_elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_depth_time_).count();
        auto sync_elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_sync_time_).count();

        if (color_elapsed > 10)
            RCLCPP_WARN(this->get_logger(), "彩色图消息超过10秒未接收!");
        if (depth_elapsed > 10)
            RCLCPP_WARN(this->get_logger(), "深度图消息超过10秒未接收!");
        if (sync_elapsed > 10 && color_msg_count_ > 0 && depth_msg_count_ > 0)
            RCLCPP_WARN(this->get_logger(), "同步消息超过10秒未处理!");
    }

    void color_monitor_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        color_msg_count_++;
        last_color_time_ = std::chrono::steady_clock::now();

        if (color_msg_count_.load() % 50 == 1)
        {
            RCLCPP_INFO(this->get_logger(), "彩色图 #%zu: %dx%d, 编码: %s",
                        color_msg_count_.load(), msg->width, msg->height, msg->encoding.c_str());
        }
    }

    void depth_monitor_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        depth_msg_count_++;
        last_depth_time_ = std::chrono::steady_clock::now();

        if (depth_msg_count_.load() % 50 == 1)
        {
            RCLCPP_INFO(this->get_logger(), "深度图 #%zu: %dx%d, 编码: %s",
                        depth_msg_count_.load(), msg->width, msg->height, msg->encoding.c_str());
        }
    }

    void init_onnx_runtime(const std::string &model_path)
    {
        RCLCPP_INFO(this->get_logger(), "检查模型文件: %s", model_path.c_str());
        if (!fs::exists(model_path))
        {
            RCLCPP_ERROR(this->get_logger(), "模型文件不存在: %s", model_path.c_str());
            throw std::runtime_error("模型文件不存在: " + model_path);
        }

        RCLCPP_INFO(this->get_logger(), "创建ONNX Runtime环境...");
        env_ = std::make_unique<Ort::Env>(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "YoloDetector");

        RCLCPP_INFO(this->get_logger(), "配置会话选项...");
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(1);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

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
                cuda_options.gpu_mem_limit = 2 * 1024 * 1024 * 1024ULL;
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

        RCLCPP_INFO(this->get_logger(), "加载模型会话...");
        try
        {
            session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), session_options);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "创建模型会话失败: %s", e.what());
            throw;
        }

        try
        {
            Ort::AllocatedStringPtr input_name_alloc = session_->GetInputNameAllocated(0, allocator_);
            input_name_ = input_name_alloc.release();
            RCLCPP_INFO(this->get_logger(), "输入节点名称: %s", input_name_);

            Ort::TypeInfo input_type_info = session_->GetInputTypeInfo(0);
            auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
            input_dims_ = input_tensor_info.GetShape();

            RCLCPP_INFO(this->get_logger(), "输入维度:");
            for (size_t i = 0; i < input_dims_.size(); ++i)
            {
                RCLCPP_INFO(this->get_logger(), "  维度[%zu]: %ld", i, static_cast<long>(input_dims_[i]));
            }

            if (input_dims_.size() >= 4)
            {
                input_height_ = static_cast<int>(input_dims_[2]);
                input_width_ = static_cast<int>(input_dims_[3]);
                RCLCPP_INFO(this->get_logger(), "从模型动态获取输入尺寸: %dx%d", input_width_, input_height_);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "输入维度格式错误，无法提取宽高");
                throw std::runtime_error("输入维度格式错误");
            }

            size_t output_count = session_->GetOutputCount();
            RCLCPP_INFO(this->get_logger(), "输出节点数量: %zu", output_count);

            for (size_t i = 0; i < output_count; ++i)
            {
                Ort::AllocatedStringPtr output_name_alloc = session_->GetOutputNameAllocated(i, allocator_);
                const char *output_name = output_name_alloc.release();
                output_names_.push_back(output_name);
                RCLCPP_INFO(this->get_logger(), "输出节点[%zu]: %s", i, output_name);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "获取模型信息失败: %s", e.what());
            throw;
        }
    }

    cv::Mat preprocess_image(const cv::Mat &image, float &scale, int &pad_w, int &pad_h)
    {
        if (image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "输入图像为空");
            throw std::runtime_error("输入图像为空");
        }

        int original_w = image.cols;
        int original_h = image.rows;
        RCLCPP_DEBUG(this->get_logger(), "原始图像尺寸: %dx%d", original_w, original_h);

        float scale_x = static_cast<float>(input_width_) / original_w;
        float scale_y = static_cast<float>(input_height_) / original_h;
        scale = std::min(scale_x, scale_y);

        int new_w = static_cast<int>(original_w * scale);
        int new_h = static_cast<int>(original_h * scale);

        pad_w = (input_width_ - new_w) / 2;
        pad_h = (input_height_ - new_h) / 2;

        RCLCPP_DEBUG(this->get_logger(), "缩放比例: %.3f, 新尺寸: %dx%d, 填充: (%d, %d)",
                     scale, new_w, new_h, pad_w, pad_h);

        try
        {
            cv::Mat resized;
            cv::resize(image, resized, cv::Size(new_w, new_h));

            cv::Mat padded;
            cv::copyMakeBorder(resized, padded, pad_h, input_height_ - new_h - pad_h,
                               pad_w, input_width_ - new_w - pad_w,
                               cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

            cv::Mat rgb;
            cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);
            rgb.convertTo(rgb, CV_32FC3, 1.0 / 255.0);

            return rgb;
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "图像预处理失败: %s", e.what());
            throw;
        }
    }

    std::vector<int> nms(const std::vector<cv::Rect> &boxes, const std::vector<float> &scores, float threshold)
    {
        std::vector<int> indices(boxes.size());
        std::iota(indices.begin(), indices.end(), 0);

        std::sort(indices.begin(), indices.end(), [&scores](int i1, int i2)
                  { return scores[i1] > scores[i2]; });

        std::vector<bool> suppressed(boxes.size(), false);
        std::vector<int> keep;

        for (size_t i = 0; i < indices.size(); ++i)
        {
            int idx = indices[i];
            if (suppressed[idx])
                continue;

            keep.push_back(idx);

            for (size_t j = i + 1; j < indices.size(); ++j)
            {
                int idx2 = indices[j];
                if (suppressed[idx2])
                    continue;

                cv::Rect intersection = boxes[idx] & boxes[idx2];
                float inter_area = intersection.area();
                float union_area = boxes[idx].area() + boxes[idx2].area() - inter_area;

                if (inter_area / union_area > threshold)
                {
                    suppressed[idx2] = true;
                }
            }
        }

        return keep;
    }

    std::vector<Ort::Value> infer(const cv::Mat &input_image)
    {
        try
        {
            RCLCPP_DEBUG(this->get_logger(), "开始推理，输入图像: %dx%dx%d",
                         input_image.rows, input_image.cols, input_image.channels());

            std::vector<float> input_data(1 * 3 * input_height_ * input_width_);
            for (int c = 0; c < 3; ++c)
            {
                for (int h = 0; h < input_height_; ++h)
                {
                    for (int w = 0; w < input_width_; ++w)
                    {
                        input_data[c * input_height_ * input_width_ + h * input_width_ + w] =
                            input_image.at<cv::Vec3f>(h, w)[c];
                    }
                }
            }

            Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
                OrtAllocatorType::OrtArenaAllocator,
                OrtMemType::OrtMemTypeDefault);

            Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                memory_info,
                input_data.data(),
                input_data.size(),
                input_dims_.data(),
                input_dims_.size());

            RCLCPP_DEBUG(this->get_logger(), "执行推理...");
            auto start_time = std::chrono::high_resolution_clock::now();

            auto outputs = session_->Run(Ort::RunOptions{nullptr},
                                         &input_name_, &input_tensor, 1,
                                         output_names_.data(), output_names_.size());

            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            RCLCPP_DEBUG(this->get_logger(), "推理完成，耗时: %ld ms", duration.count());

            return outputs;
        }
        catch (const Ort::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "ONNX推理失败: %s", e.what());
            throw;
        }
    }

    std::vector<Detection> postprocess(std::vector<Ort::Value> &outputs,
                                       int original_w, int original_h,
                                       float scale, int pad_w, int pad_h)
    {
        std::vector<Detection> detections;

        try
        {
            if (outputs.empty())
            {
                RCLCPP_WARN(this->get_logger(), "推理输出为空");
                return detections;
            }

            float *output_data = outputs[0].GetTensorMutableData<float>();
            auto output_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();

            if (output_shape.size() != 3)
            {
                RCLCPP_ERROR(this->get_logger(), "输出形状不正确，期望3维，实际%d维",
                             static_cast<int>(output_shape.size()));
                return detections;
            }

            int num_detections = static_cast<int>(output_shape[2]);
            int num_classes = static_cast<int>(output_shape[1]) - 4;

            RCLCPP_DEBUG(this->get_logger(), "后处理开始，检测数量: %d, 类别数: %d",
                         num_detections, num_classes);

            std::vector<cv::Rect> boxes;
            std::vector<float> scores;
            std::vector<int> class_ids;

            for (int i = 0; i < num_detections; ++i)
            {
                float cx = output_data[i];
                float cy = output_data[num_detections + i];
                float w = output_data[2 * num_detections + i];
                float h = output_data[3 * num_detections + i];

                float max_score = 0.0f;
                int max_class = -1;
                for (int c = 0; c < num_classes; ++c)
                {
                    float score = output_data[(4 + c) * num_detections + i];
                    if (score > max_score)
                    {
                        max_score = score;
                        max_class = c;
                    }
                }

                if (max_score > conf_threshold_)
                {
                    float x1 = (cx - w * 0.5f - pad_w) / scale;
                    float y1 = (cy - h * 0.5f - pad_h) / scale;
                    float x2 = (cx + w * 0.5f - pad_w) / scale;
                    float y2 = (cy + h * 0.5f - pad_h) / scale;

                    x1 = std::max(0.0f, std::min(static_cast<float>(original_w), x1));
                    y1 = std::max(0.0f, std::min(static_cast<float>(original_h), y1));
                    x2 = std::max(0.0f, std::min(static_cast<float>(original_w), x2));
                    y2 = std::max(0.0f, std::min(static_cast<float>(original_h), y2));

                    if (x2 > x1 && y2 > y1)
                    {
                        boxes.emplace_back(static_cast<int>(x1), static_cast<int>(y1),
                                           static_cast<int>(x2 - x1), static_cast<int>(y2 - y1));
                        scores.push_back(max_score);
                        class_ids.push_back(max_class);
                    }
                }
            }

            RCLCPP_DEBUG(this->get_logger(), "NMS前检测框数量: %zu", boxes.size());

            std::vector<int> keep_indices = nms(boxes, scores, nms_threshold_);

            for (int idx : keep_indices)
            {
                Detection det;
                det.box = boxes[idx];
                det.confidence = scores[idx];
                det.class_id = class_ids[idx];
                det.class_name = (static_cast<size_t>(class_ids[idx]) < class_names.size()) ? class_names[class_ids[idx]] : "unknown";
                detections.push_back(det);
            }

            RCLCPP_DEBUG(this->get_logger(), "NMS后检测框数量: %zu", detections.size());
            return detections;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "后处理失败: %s", e.what());
            throw;
        }
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &color_msg,
                        const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg)
    {
        sync_msg_count_++;
        last_sync_time_ = std::chrono::steady_clock::now();

        RCLCPP_DEBUG(this->get_logger(), "收到同步消息对 #%zu", sync_msg_count_.load());

        try
        {
            cv_bridge::CvImagePtr cv_color = cv_bridge::toCvCopy(
                color_msg, sensor_msgs::image_encodings::BGR8);
            cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy(
                depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);

            if (cv_color->image.empty() || cv_depth->image.empty())
            {
                RCLCPP_WARN(this->get_logger(), "图像转换后为空，跳过处理");
                return;
            }

            int original_w = cv_color->image.cols;
            int original_h = cv_color->image.rows;
            float scale;
            int pad_w, pad_h;
            cv::Mat input_image = preprocess_image(cv_color->image, scale, pad_w, pad_h);

            auto outputs = infer(input_image);

            std::vector<Detection> detections = postprocess(outputs, original_w, original_h, scale, pad_w, pad_h);

            RCLCPP_INFO(this->get_logger(), "检测到 %zu 个目标", detections.size());

            std::vector<Detection> target_detections;
            for (const auto &det : detections)
            {
                RCLCPP_INFO(this->get_logger(), "检测到目标: 类别=%s (ID=%d), 置信度=%.3f",
                            det.class_name.c_str(), det.class_id, det.confidence);

                if (det.class_id == target_class_id_)
                {
                    target_detections.push_back(det);
                    RCLCPP_INFO(this->get_logger(), "匹配到目标类别 %d", target_class_id_);
                }
            }

            if (target_detections.empty())
            {
                RCLCPP_DEBUG(this->get_logger(), "未检测到目标类别 %d，跳过保存", target_class_id_);
                return;
            }

            // 【新增】检查位姿有效性
            if (!pose_received_)
            {
                RCLCPP_WARN(this->get_logger(), "⚠ 尚未接收到无人机位姿，跳过本次保存");
                return;
            }

            cv::Mat result_image = cv_color->image.clone();
            for (const auto &det : target_detections)
            {
                cv::rectangle(result_image, det.box, cv::Scalar(0, 255, 0), 2);
                std::string label = det.class_name + ": " +
                                    std::to_string(static_cast<int>(det.confidence * 100)) + "%";
                cv::putText(result_image, label,
                            cv::Point(det.box.x, det.box.y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            }

            int target_x = target_detections[0].box.x + target_detections[0].box.width / 2;
            int target_y = target_detections[0].box.y + target_detections[0].box.height / 2;

            geometry_msgs::msg::PoseStamped processing_pose;
            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                processing_pose = current_pose_;
            }

            save_images(color_msg, cv_color->image, cv_depth->image, result_image, target_x, target_y, processing_pose);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge错误: %s", e.what());
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV错误: %s", e.what());
        }
        catch (const Ort::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "ONNX Runtime错误: %s", e.what());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "处理错误: %s", e.what());
        }
    }

    void save_images(const sensor_msgs::msg::Image::ConstSharedPtr color_msg,
                     const cv::Mat &color_image, const cv::Mat &depth_image,
                     const cv::Mat &result_image, int target_x, int target_y,
                     const geometry_msgs::msg::PoseStamped &processing_pose)
    {
        try
        {
            // 【新增】保存前再次验证位姿有效性（双重检查）
            RCLCPP_INFO(this->get_logger(), "准备保存图像 - 位姿有效性: %s, 位置: (%.2f, %.2f, %.2f)",
                        pose_received_ ? "✓有效" : "✗无效",
                        processing_pose.pose.position.x,
                        processing_pose.pose.position.y,
                        processing_pose.pose.position.z);

            std::string timestamp = std::to_string(color_msg->header.stamp.sec) +
                                    "_" + std::to_string(color_msg->header.stamp.nanosec / 1000000);

            std::string color_filename = image_save_path_ + "color_" + timestamp + "_" +
                                         std::to_string(image_count_) + ".jpg";
            if (!cv::imwrite(color_filename, color_image))
            {
                RCLCPP_ERROR(this->get_logger(), "彩色图保存失败: %s", color_filename.c_str());
                return;
            }

            std::string depth_filename = image_save_path_ + "depth_" + timestamp + "_" +
                                         std::to_string(image_count_) + ".png";
            cv::imwrite(depth_filename, depth_image);

            std::string result_filename = image_save_path_ + "result_" + timestamp + "_" +
                                          std::to_string(image_count_) + ".jpg";
            cv::imwrite(result_filename, result_image);

            std::string pose_filename = image_save_path_ + "pose_" + timestamp + "_" + std::to_string(image_count_) + ".txt";
            std::ofstream pose_file(pose_filename);
            if (pose_file.is_open())
            {
                pose_file << "图像时间戳: " << timestamp << "\n";
                pose_file << "位置(x,y,z): " << processing_pose.pose.position.x << "," << processing_pose.pose.position.y << "," << processing_pose.pose.position.z << "\n";
                pose_file << "姿态(四元数w,x,y,z): " << processing_pose.pose.orientation.w << "," << processing_pose.pose.orientation.x << "," << processing_pose.pose.orientation.y << "," << processing_pose.pose.orientation.z << "\n";
                pose_file << "目标像素坐标: (" << target_x << "," << target_y << ")\n";
                pose_file.close();
            }

            std::ofstream csv_file(pose_csv_filename_, std::ios::app);
            if (csv_file.is_open())
            {
                csv_file << image_count_ << "," << timestamp << ","
                         << processing_pose.pose.position.x << "," << processing_pose.pose.position.y << "," << processing_pose.pose.position.z << ","
                         << processing_pose.pose.orientation.x << "," << processing_pose.pose.orientation.y << "," << processing_pose.pose.orientation.z << "," << processing_pose.pose.orientation.w << ","
                         << target_x << "," << target_y << "\n";
                csv_file.close();
            }

            RCLCPP_INFO(this->get_logger(), "✓ 已保存图像 #%zu: 目标像素(%d,%d) | 无人机位置(%.2f,%.2f,%.2f)",
                        image_count_.load(), target_x, target_y,
                        processing_pose.pose.position.x,
                        processing_pose.pose.position.y,
                        processing_pose.pose.position.z);

            image_count_++;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "保存图像时出错: %s", e.what());
        }
    }

    // 【修改】无人机位姿回调（添加接收标志）
    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_ = *msg;

        if (!pose_received_)
        {
            pose_received_ = true;
            RCLCPP_INFO(this->get_logger(), "✓ 首次接收到无人机位姿: (%.2f, %.2f, %.2f)",
                        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        }

        static int pose_count = 0;
        if (++pose_count % 50 == 0)
        {
            RCLCPP_DEBUG(this->get_logger(), "收到无人机位姿 #%d: 位置(%.2f, %.2f, %.2f)",
                         pose_count, msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        }
    }

    // ONNX Runtime相关成员
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    Ort::AllocatorWithDefaultOptions allocator_;
    const char *input_name_ = nullptr;
    std::vector<const char *> output_names_;
    std::vector<int64_t> input_dims_;

    message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy>> sync_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_monitor_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_monitor_sub_;

    rclcpp::TimerBase::SharedPtr monitor_timer_;

    std::atomic<size_t> color_msg_count_;
    std::atomic<size_t> depth_msg_count_;
    std::atomic<size_t> sync_msg_count_;
    std::chrono::steady_clock::time_point last_color_time_;
    std::chrono::steady_clock::time_point last_depth_time_;
    std::chrono::steady_clock::time_point last_sync_time_;

    std::string image_save_path_;
    float conf_threshold_;
    float nms_threshold_;
    int input_width_;
    int input_height_;
    int target_class_id_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    geometry_msgs::msg::PoseStamped current_pose_;
    std::mutex pose_mutex_;
    std::string pose_csv_filename_;

    std::atomic<size_t> image_count_;

    // 【新增】位姿接收标志
    std::atomic<bool> pose_received_{false};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YoloDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}