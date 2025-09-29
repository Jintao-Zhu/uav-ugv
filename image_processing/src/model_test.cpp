#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
//测试onnx是否可用
class YOLOv11Detector
{
private:
    Ort::Env env;
    Ort::Session session;
    Ort::SessionOptions session_options;
    std::vector<std::string> input_names;
    std::vector<std::string> output_names;
    std::vector<std::vector<int64_t>> input_shapes;
    std::vector<std::vector<int64_t>> output_shapes;

    // 模型参数
    int input_width = 640;
    int input_height = 640;
    float conf_threshold = 0.25f;
    float iou_threshold = 0.45f;

    // COCO类别名称（根据你的模型调整）
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

public:
    struct Detection
    {
        cv::Rect box;
        float confidence;
        int class_id;
        std::string class_name;
    };

    YOLOv11Detector(const std::string &model_path)
        : env(ORT_LOGGING_LEVEL_WARNING, "YOLOv11"),
          session(nullptr)
    {

        // 配置会话选项
        session_options.SetIntraOpNumThreads(1);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        try
        {
            // 创建会话
            session = Ort::Session(env, model_path.c_str(), session_options);

            // 获取输入信息
            size_t num_inputs = session.GetInputCount();
            input_names.resize(num_inputs);
            input_shapes.resize(num_inputs);

            for (size_t i = 0; i < num_inputs; i++)
            {
                Ort::AllocatorWithDefaultOptions allocator;
                input_names[i] = session.GetInputNameAllocated(i, allocator).get();
                input_shapes[i] = session.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();

                // 获取实际的输入尺寸
                if (input_shapes[i].size() >= 4)
                {
                    input_height = static_cast<int>(input_shapes[i][2]);
                    input_width = static_cast<int>(input_shapes[i][3]);
                }
            }

            // 获取输出信息
            size_t num_outputs = session.GetOutputCount();
            output_names.resize(num_outputs);
            output_shapes.resize(num_outputs);

            for (size_t i = 0; i < num_outputs; i++)
            {
                Ort::AllocatorWithDefaultOptions allocator;
                output_names[i] = session.GetOutputNameAllocated(i, allocator).get();
                output_shapes[i] = session.GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
            }

            std::cout << "模型加载成功!" << std::endl;
            std::cout << "输入尺寸: " << input_width << "x" << input_height << std::endl;
        }
        catch (const Ort::Exception &e)
        {
            std::cerr << "加载模型时出错: " << e.what() << std::endl;
            throw;
        }
    }

    // 修正后的图像预处理函数
    cv::Mat preprocess(const cv::Mat &image, float &scale, int &pad_x, int &pad_y)
    {
        int orig_width = image.cols;
        int orig_height = image.rows;

        // 计算缩放比例（保持宽高比）
        float scale_x = static_cast<float>(input_width) / orig_width;
        float scale_y = static_cast<float>(input_height) / orig_height;
        scale = std::min(scale_x, scale_y); // 使用统一的缩放比例

        // 计算新的尺寸
        int new_width = static_cast<int>(orig_width * scale);
        int new_height = static_cast<int>(orig_height * scale);

        // 计算padding
        pad_x = (input_width - new_width) / 2;
        pad_y = (input_height - new_height) / 2;

        // 缩放图像
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(new_width, new_height));

        // 添加padding
        cv::Mat padded;
        cv::copyMakeBorder(resized, padded, pad_y, input_height - new_height - pad_y,
                           pad_x, input_width - new_width - pad_x, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

        // 转换为RGB并归一化
        cv::Mat rgb;
        cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);
        rgb.convertTo(rgb, CV_32F, 1.0 / 255.0);

        return rgb;
    }

    // 非最大抑制
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

    // 修正后的主推理函数
    std::vector<Detection> detect(const cv::Mat &image)
    {
        // 预处理 - 使用修正后的参数
        float scale; // 统一的缩放比例
        int pad_x, pad_y;
        cv::Mat processed = preprocess(image, scale, pad_x, pad_y);

        // 准备输入tensor
        std::vector<float> input_data;
        input_data.resize(1 * 3 * input_height * input_width);

        // 将图像数据复制到输入向量 (NCHW格式)
        for (int c = 0; c < 3; ++c)
        {
            for (int h = 0; h < input_height; ++h)
            {
                for (int w = 0; w < input_width; ++w)
                {
                    input_data[c * input_height * input_width + h * input_width + w] =
                        processed.at<cv::Vec3f>(h, w)[c];
                }
            }
        }

        // 创建输入tensor
        std::vector<int64_t> input_shape = {1, 3, input_height, input_width};
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, input_data.data(), input_data.size(), input_shape.data(), input_shape.size());

        // 准备输入和输出名称
        std::vector<const char *> input_names_char;
        std::vector<const char *> output_names_char;

        for (const auto &name : input_names)
        {
            input_names_char.push_back(name.c_str());
        }
        for (const auto &name : output_names)
        {
            output_names_char.push_back(name.c_str());
        }

        // 执行推理
        auto output_tensors = session.Run(Ort::RunOptions{nullptr},
                                          input_names_char.data(), &input_tensor, 1,
                                          output_names_char.data(), output_names.size());

        // 解析输出
        std::vector<Detection> detections;

        if (!output_tensors.empty())
        {
            float *output_data = output_tensors[0].GetTensorMutableData<float>();
            auto output_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();

            // YOLOv11输出格式: [batch, num_classes + 4, num_detections]
            int num_detections = static_cast<int>(output_shape[2]);
            int num_classes = static_cast<int>(output_shape[1]) - 4;

            // 添加调试信息
            std::cout << "输出形状: [" << output_shape[0] << ", " << output_shape[1]
                      << ", " << output_shape[2] << "]" << std::endl;
            std::cout << "检测数量: " << num_detections << ", 类别数: " << num_classes << std::endl;
            std::cout << "缩放比例: " << scale << ", 填充: (" << pad_x << ", " << pad_y << ")" << std::endl;

            std::vector<cv::Rect> boxes;
            std::vector<float> scores;
            std::vector<int> class_ids;

            for (int i = 0; i < num_detections; ++i)
            {
                // 获取边界框坐标 (cx, cy, w, h)
                float cx = output_data[i];
                float cy = output_data[num_detections + i];
                float w = output_data[2 * num_detections + i];
                float h = output_data[3 * num_detections + i];

                // 找到最高置信度的类别
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

                // 过滤低置信度检测
                if (max_score > conf_threshold)
                {
                    // 转换为 (x1, y1, x2, y2) 格式并还原到原图尺寸
                    // 关键修正：使用统一的scale进行逆变换
                    float x1 = (cx - w * 0.5f - pad_x) / scale;
                    float y1 = (cy - h * 0.5f - pad_y) / scale;
                    float x2 = (cx + w * 0.5f - pad_x) / scale;
                    float y2 = (cy + h * 0.5f - pad_y) / scale;

                    // 添加调试信息（可选，调试时启用）
                    /*
                    std::cout << "检测 " << i << ": cx=" << cx << ", cy=" << cy
                              << ", w=" << w << ", h=" << h << ", score=" << max_score << std::endl;
                    std::cout << "映射前: (" << (cx - w * 0.5f) << ", " << (cy - h * 0.5f)
                              << ", " << (cx + w * 0.5f) << ", " << (cy + h * 0.5f) << ")" << std::endl;
                    std::cout << "映射后: (" << x1 << ", " << y1 << ", " << x2 << ", " << y2 << ")" << std::endl;
                    */

                    // 限制边界框在图像范围内
                    x1 = std::max(0.0f, std::min(static_cast<float>(image.cols), x1));
                    y1 = std::max(0.0f, std::min(static_cast<float>(image.rows), y1));
                    x2 = std::max(0.0f, std::min(static_cast<float>(image.cols), x2));
                    y2 = std::max(0.0f, std::min(static_cast<float>(image.rows), y2));

                    // 确保边界框有效
                    if (x2 > x1 && y2 > y1)
                    {
                        boxes.emplace_back(static_cast<int>(x1), static_cast<int>(y1),
                                           static_cast<int>(x2 - x1), static_cast<int>(y2 - y1));
                        scores.push_back(max_score);
                        class_ids.push_back(max_class);
                    }
                }
            }

            // 应用NMS
            std::vector<int> keep_indices = nms(boxes, scores, iou_threshold);

            for (int idx : keep_indices)
            {
                Detection det;
                det.box = boxes[idx];
                det.confidence = scores[idx];
                det.class_id = class_ids[idx];
                det.class_name = (class_ids[idx] < class_names.size()) ? class_names[class_ids[idx]] : "unknown";
                detections.push_back(det);
            }
        }

        return detections;
    }

    // 在图像上绘制检测结果
    void draw_detections(cv::Mat &image, const std::vector<Detection> &detections)
    {
        for (const auto &det : detections)
        {
            // 绘制边界框
            cv::rectangle(image, det.box, cv::Scalar(0, 255, 0), 2);

            // 准备标签文本
            std::string label = det.class_name + ": " +
                                std::to_string(static_cast<int>(det.confidence * 100)) + "%";

            // 获取文本尺寸
            int baseline;
            cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);

            // 绘制标签背景
            cv::rectangle(image,
                          cv::Point(det.box.x, det.box.y - text_size.height - 10),
                          cv::Point(det.box.x + text_size.width, det.box.y),
                          cv::Scalar(0, 255, 0), -1);

            // 绘制标签文本
            cv::putText(image, label, cv::Point(det.box.x, det.box.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        }
    }

    // 设置检测阈值
    void set_thresholds(float conf_thresh, float iou_thresh)
    {
        conf_threshold = conf_thresh;
        iou_threshold = iou_thresh;
    }

    // 获取模型输入尺寸
    std::pair<int, int> get_input_size() const
    {
        return {input_width, input_height};
    }
};

// 主函数示例
int main()
{
    try
    {
        // 初始化检测器
        YOLOv11Detector detector("/home/suda/drone_ugv_ws/src/image_processing/models/red_cube_yolov11.onnx");

        // 设置阈值（可选）
        detector.set_thresholds(0.25f, 0.45f);

        // 加载图像
        cv::Mat image = cv::imread("/home/suda/drone_ugv_ws/src/image_processing/camera/color_images/001.png");
        if (image.empty())
        {
            std::cerr << "无法加载图像!" << std::endl;
            return -1;
        }

        std::cout << "原图尺寸: " << image.cols << "x" << image.rows << std::endl;

        // 执行检测
        auto detections = detector.detect(image);

        // 输出检测结果
        std::cout << "检测到 " << detections.size() << " 个目标:" << std::endl;
        for (const auto &det : detections)
        {
            std::cout << "类别: " << det.class_name
                      << ", 置信度: " << std::fixed << std::setprecision(3) << det.confidence
                      << ", 位置: (" << det.box.x << ", " << det.box.y
                      << ", " << det.box.width << ", " << det.box.height << ")" << std::endl;
        }

        // 在图像上绘制检测结果
        detector.draw_detections(image, detections);

        // 保存结果
        cv::imwrite("detection_result.jpg", image);
        std::cout << "检测结果已保存到 detection_result.jpg" << std::endl;

        // 显示结果（可选）
        cv::imshow("YOLOv11 Detection", image);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
    catch (const std::exception &e)
    {
        std::cerr << "错误: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}