#include "opencv4/opencv2/opencv.hpp"
#include "opencv4/opencv2/core/core.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <string>
#include <filesystem>
//获取无人机飞行过程中拍摄的所有图像
namespace fs = std::filesystem;
using namespace std::chrono_literals;

class ImageCaptureNode : public rclcpp::Node
{
public:
    ImageCaptureNode() : Node("image_capture_node")
    {
        try {
            // -------------------------- 优化：参数化传递路径（保留默认值） --------------------------
            // 1. 声明路径参数：默认值为原代码的路径，确保不修改命令时行为一致
            this->declare_parameter<std::string>(
                "src_package_path",  // 参数名（运行时通过此名称修改）
                "/home/suda/drone_ugv_ws/src/image_processing/"  // 默认路径（原代码路径）
            );

            // 2. 获取参数值：优先使用命令行传入的路径，无传入则用默认值
            std::string src_package_path;
            this->get_parameter("src_package_path", src_package_path);
            
            // 3. 检查源码包路径是否存在（避免路径错误导致保存失败）
            if (!fs::exists(src_package_path)) {
                throw std::runtime_error("源码包路径不存在！当前路径：" + src_package_path + 
                                        "\n提示：可通过 --ros-args -p src_package_path:=新路径 修改");
            }

            // 4. 设置图像保存路径（逻辑与原代码一致，基于参数路径拼接）
            color_image_path_ = src_package_path + "camera/color_images/";
            depth_image_path_ = src_package_path + "camera/depth_images/";
            
            // 创建目录（不存在则创建，支持多级目录）
            if (!fs::exists(color_image_path_)) {
                fs::create_directories(color_image_path_);
                RCLCPP_INFO(this->get_logger(), "创建彩色图像目录(源码包下): %s", color_image_path_.c_str());
            }
            
            if (!fs::exists(depth_image_path_)) {
                fs::create_directories(depth_image_path_);
                RCLCPP_INFO(this->get_logger(), "创建深度图像目录(源码包下): %s", depth_image_path_.c_str());
            }

            // 打印路径信息（明确当前使用的路径，方便调试）
            RCLCPP_INFO(this->get_logger(), "当前使用的源码包路径: %s", src_package_path.c_str());
            RCLCPP_INFO(this->get_logger(), "（若需修改路径，启动时添加参数：--ros-args -p src_package_path:=你的路径）");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "路径配置失败: %s", e.what());
            rclcpp::shutdown();
        }

        // 订阅逻辑保持不变
        color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&ImageCaptureNode::color_image_callback, this, std::placeholders::_1));
        
        depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10,
            std::bind(&ImageCaptureNode::depth_image_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "图像捕获节点已启动");
        RCLCPP_INFO(this->get_logger(), "彩色图像保存至(源码包下): %s", color_image_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "深度图像保存至(源码包下): %s", depth_image_path_.c_str());
    }

private:
    // 彩色图像回调（完全不变，与原代码逻辑一致）
    void color_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            static int color_count = 0;
            std::string filename = color_image_path_ + "color_image_" + std::to_string(color_count++) + ".jpg";
            
            if (cv::imwrite(filename, cv_ptr->image)) {
                if (color_count % 10 == 0) {
                    RCLCPP_INFO(this->get_logger(), "已保存彩色图像: %s", filename.c_str());
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "彩色图像保存失败: %s", filename.c_str());
            }
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge错误 (彩色图像): %s", e.what());
        }
        catch (cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV错误 (彩色图像): %s", e.what());
        }
    }

    // 深度图像回调（完全不变，与原代码逻辑一致）
    void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // 深度图像通常是16位或32位单通道图像，保持原格式转换
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
            
            static int depth_count = 0;
            std::string filename = depth_image_path_ + "depth_image_" + std::to_string(depth_count++) + ".png";
            
            // 保存深度图像（PNG格式保留原始深度值，逻辑不变）
            if (cv::imwrite(filename, cv_ptr->image)) {
                if (depth_count % 10 == 0) {
                    RCLCPP_INFO(this->get_logger(), "已保存深度图像: %s", filename.c_str());
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "深度图像保存失败: %s", filename.c_str());
            }
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge错误 (深度图像): %s", e.what());
        }
        catch (cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV错误 (深度图像): %s", e.what());
        }
    }

    // 成员变量保持不变
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    std::string color_image_path_;
    std::string depth_image_path_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageCaptureNode>());
    rclcpp::shutdown();
    return 0;
}
