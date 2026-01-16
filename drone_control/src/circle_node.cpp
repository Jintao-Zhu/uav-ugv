#include <px4_msgs/msg/offboard_control_mode.hpp> // 在原来的自定义场景环形飞行
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <array>
#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("circle_flight_node")
    {
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        offboard_setpoint_counter_ = 0;

        this->declare_parameter<double>("circle_radius_", 35.0);   // 圆的半径（米）15
        this->declare_parameter<double>("angular_velocity_", 0.05); // 角速度（弧度/秒），决定飞行速度 10.30 注意 由0.05改为0.035
        this->declare_parameter<double>("center_x_", 0.0);   // 圆心x坐标
        this->declare_parameter<double>("center_y_", 0.0);   // 圆心y坐标

        // 获取参数（支持启动时动态修改）
        circle_radius_ = this->get_parameter("circle_radius_").as_double(); 
        angular_velocity_ = this->get_parameter("angular_velocity_").as_double();
        center_x_ = this->get_parameter("center_x_").as_double();  
        center_y_ = this->get_parameter("center_y_").as_double();  

        // 圆形飞行参数
        flight_height_ = -5.0;   // 飞行高度（米，负值表示相对于起飞点向上）
        current_angle_ = 0.0;    // 当前角度


        // 飞行阶段
        flight_phase_ = TAKEOFF;
        takeoff_timer_ = 0;
        circle_complete_ = false;
        land_height_ = flight_height_; // 着陆高度变量
        landing_started_ = false;      // 着陆命令是否已发送

        auto timer_callback = [this]() -> void
        {
            if (offboard_setpoint_counter_ == 10)
            {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                // Arm the vehicle
                this->arm();
            }

            // offboard_control_mode needs to be paired with trajectory_setpoint
            publish_offboard_control_mode();
            publish_trajectory_setpoint();

            // stop the counter after reaching 11
            if (offboard_setpoint_counter_ < 11)
            {
                offboard_setpoint_counter_++;
            }
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

    void arm();
    void disarm();
    void land(); // 新增着陆命令函数

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    std::atomic<uint64_t> timestamp_;    //!< common synced timestamped
    uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent

    // 圆形飞行参数
    double circle_radius_;       // 圆的半径
    double flight_height_;       // 飞行高度
    double angular_velocity_;    // 角速度
    double current_angle_;       // 当前角度
    double center_x_, center_y_; // 圆心坐标

    // 着陆相关变量
    float land_height_;    // 当前着陆高度
    bool landing_started_; // 是否已开始着陆流程

    // 飞行阶段枚举
    enum FlightPhase
    {
        TAKEOFF,       // 起飞阶段
        HOVER,         // 悬停阶段
        CIRCLE_FLIGHT, // 圆形飞行阶段
        RETURN_HOME,   // 返回起点
        LAND,          // 降落阶段
        LANDED         // 已着陆
    };

    FlightPhase flight_phase_;
    int takeoff_timer_;
    bool circle_complete_;

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Send a command to Land the vehicle
 */
void OffboardControl::land()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(this->get_logger(), "Land command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        实现起飞、悬停、圆形飞行、返回、着陆的完整流程
 */
void OffboardControl::publish_trajectory_setpoint()
{
    TrajectorySetpoint msg{};

    switch (flight_phase_)
    {
    case TAKEOFF:
        // 起飞阶段：保持在原点上升到指定高度
        msg.position = {0.0, 0.0, static_cast<float>(flight_height_)};
        msg.yaw = 0.0;
        takeoff_timer_++;

        // 起飞完成后进入悬停阶段
        if (takeoff_timer_ > 50)
        { // 5秒后（50 * 100ms）
            flight_phase_ = HOVER;
            takeoff_timer_ = 0;
            RCLCPP_INFO(this->get_logger(), "起飞完成，开始悬停");
        }
        break;

    case HOVER:
        // 悬停阶段：在起点悬停2秒
        msg.position = {0.0, 0.0, static_cast<float>(flight_height_)};
        msg.yaw = 0.0;
        takeoff_timer_++;

        if (takeoff_timer_ > 20)
        { // 2秒后开始圆形飞行
            flight_phase_ = CIRCLE_FLIGHT;
            current_angle_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "开始圆形飞行");
        }
        break;

    case CIRCLE_FLIGHT:
        // 圆形飞行阶段
        {
            double x = center_x_ + circle_radius_ * cos(current_angle_);
            double y = center_y_ + circle_radius_ * sin(current_angle_);

            msg.position = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(flight_height_)};

            // 让无人机朝向运动方向
            double yaw = current_angle_ + M_PI / 2; // 切线方向
            msg.yaw = static_cast<float>(yaw);

            // 更新角度
            current_angle_ += angular_velocity_ * 0.1; // 0.1秒的时间步长

            // 完成一圈后进入返回阶段
            if (current_angle_ >= 2 * M_PI)
            {
                flight_phase_ = RETURN_HOME;
                takeoff_timer_ = 0; // 重置计时器
                RCLCPP_INFO(this->get_logger(), "圆形飞行完成，返回起点");
            }
        }
        break;

    case RETURN_HOME:
        // 返回起点
        msg.position = {0.0, 0.0, static_cast<float>(flight_height_)};
        msg.yaw = 0.0;
        takeoff_timer_++;

        if (takeoff_timer_ > 30)
        { // 3秒后开始降落
            flight_phase_ = LAND;
            takeoff_timer_ = 0; // 重置计时器用于着陆计时
            RCLCPP_INFO(this->get_logger(), "开始降落流程");
        }
        break;

    case LAND:
        // 降落阶段：逐步降低高度直到着陆
        {
            // 缓慢下降，每个周期下降3cm
            land_height_ += 0.03f; // 向0靠近（从负值变小）

            // 如果接近地面（-0.3米），发送着陆命令
            if (land_height_ >= -0.3f && !landing_started_)
            {
                this->land(); // 发送着陆命令
                landing_started_ = true;
                RCLCPP_INFO(this->get_logger(), "发送着陆命令");
            }

            // 继续控制位置直到完全着陆
            if (land_height_ >= 0.2f)
            {                        // 着陆完成
                land_height_ = 0.2f; // 限制在地面以上20cm
                flight_phase_ = LANDED;
                RCLCPP_INFO(this->get_logger(), "着陆完成");
            }

            msg.position = {0.0, 0.0, land_height_};
            msg.yaw = 0.0;
        }
        break;

    case LANDED:
        // 已着陆状态：保持在地面并解除武装
        msg.position = {0.0, 0.0, 0.2f}; // 保持在地面以上20cm
        msg.yaw = 0.0;

        takeoff_timer_++;
        if (takeoff_timer_ > 20)
        { // 2秒后解除武装
            this->disarm();
            RCLCPP_INFO(this->get_logger(), "任务完成，无人机已解除武装");
            // 可以选择关闭节点或停止发布
            takeoff_timer_ = 0; // 重置以避免重复解除武装
        }
        break;
    }

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
    std::cout << "Starting circle flight node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}