#include <px4_msgs/msg/offboard_control_mode.hpp>
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

        // =================================================================
        // 【配置区域】 这里填写 RMF/Gazebo 的真实全局坐标，所见即所得
        // =================================================================
        
        // 1. 飞行参数
        circle_radius_ = 35.0;     // 圆半径 (米)
        flight_height_ = -5.0;     // 飞行高度 (PX4 NED坐标，负数为向上 5米)
        angular_velocity_ = 0.035; // 角速度 (rad/s)
        current_angle_ = 0.0;      // 初始角度

        // 2. 圆心坐标 (你的目标中心点)
        // 对应 RMF 地图上的 (40, -35)
        center_x_ = 40.0;         
        center_y_ = -35.0;        

        // 3. 起飞原点 (你的 Launch 文件里设定的生成点)
        // 对应 RMF 地图上的 (40, -40)
        // 代码会自动计算： 起飞点 -> 圆心 = 向北飞 5 米
        takeoff_origin_x_ = 40.0; 
        takeoff_origin_y_ = -40.0; 

        // =================================================================

        flight_phase_ = TAKEOFF;
        takeoff_timer_ = 0;
        circle_complete_ = false;
        land_height_ = flight_height_; 
        landing_started_ = false;      

        auto timer_callback = [this]() -> void
        {
            if (offboard_setpoint_counter_ == 10)
            {
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->arm();
            }

            publish_offboard_control_mode();
            publish_trajectory_setpoint();

            if (offboard_setpoint_counter_ < 11)
            {
                offboard_setpoint_counter_++;
            }
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

    void arm();
    void disarm();
    void land(); 

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    std::atomic<uint64_t> timestamp_;    
    uint64_t offboard_setpoint_counter_; 

    // 坐标变量
    double circle_radius_;    
    double flight_height_;    
    double angular_velocity_; 
    double current_angle_;    
    double center_x_, center_y_; 
    double takeoff_origin_x_;    
    double takeoff_origin_y_;    

    float land_height_;    
    bool landing_started_; 

    enum FlightPhase
    {
        TAKEOFF,       
        HOVER,         
        CIRCLE_FLIGHT, 
        RETURN_HOME,   
        LAND,          
        LANDED         
    };

    FlightPhase flight_phase_;
    int takeoff_timer_;
    bool circle_complete_;

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

void OffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void OffboardControl::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void OffboardControl::land()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(this->get_logger(), "Land command send");
}

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
 * @brief 核心函数：实现全局坐标(ENU)到局部坐标(NED)的自动转换
 */
void OffboardControl::publish_trajectory_setpoint()
{
    TrajectorySetpoint msg{};

    // 1. 定义临时变量：目标点的【全局 ENU 坐标】
    //    我们在 Switch 里面只思考 "在地图的哪个位置"，不需要思考 "相对位移"
    double target_x_enu = 0.0;
    double target_y_enu = 0.0;

    switch (flight_phase_)
    {
    case TAKEOFF:
        // 起飞阶段：目标就是起飞点本身 (40, -40)
        target_x_enu = takeoff_origin_x_;
        target_y_enu = takeoff_origin_y_;
        
        takeoff_timer_++;
        if (takeoff_timer_ > 50) // 5秒
        { 
            flight_phase_ = HOVER;
            takeoff_timer_ = 0;
            RCLCPP_INFO(this->get_logger(), "起飞完成，开始悬停");
        }
        break;

    case HOVER:
        // 悬停阶段：位置不变
        target_x_enu = takeoff_origin_x_;
        target_y_enu = takeoff_origin_y_;
        
        takeoff_timer_++;
        if (takeoff_timer_ > 20) // 2秒
        { 
            flight_phase_ = CIRCLE_FLIGHT;
            // 计算初始角度，使无人机从当前位置平滑切入圆形
            // 这里简单从 0 开始
            current_angle_ = 0.0; 
            RCLCPP_INFO(this->get_logger(), "开始圆形飞行");
        }
        break;

    case CIRCLE_FLIGHT:
        {
            // 圆形飞行：基于圆心 center_x/y 计算目标点
            target_x_enu = center_x_ + circle_radius_ * cos(current_angle_);
            target_y_enu = center_y_ + circle_radius_ * sin(current_angle_);

            // 计算朝向 (ENU 下的偏航角)
            // 切线方向 = 角度 + 90度
            double yaw_enu = current_angle_ + M_PI / 2;

            // 转换 Yaw：ENU 转 NED
            // NED North (0) = ENU North (90) -> 关系是: yaw_ned = -yaw_enu + PI/2
            msg.yaw = static_cast<float>(-yaw_enu + M_PI / 2);

            // 更新角度
            current_angle_ += angular_velocity_ * 0.1;

            if (current_angle_ >= 2 * M_PI)
            {
                flight_phase_ = RETURN_HOME;
                takeoff_timer_ = 0; 
                RCLCPP_INFO(this->get_logger(), "圆形飞行完成，返回起点");
            }
        }
        break;

    case RETURN_HOME:
        // 返回起飞点 (40, -40)
        target_x_enu = takeoff_origin_x_;
        target_y_enu = takeoff_origin_y_;
        
        takeoff_timer_++;
        if (takeoff_timer_ > 30)
        { 
            flight_phase_ = LAND;
            takeoff_timer_ = 0; 
            RCLCPP_INFO(this->get_logger(), "开始降落流程");
        }
        break;

    case LAND:
        // 降落：水平位置保持在起飞点
        target_x_enu = takeoff_origin_x_;
        target_y_enu = takeoff_origin_y_;

        // 高度处理 (使用 NED 高度逻辑)
        land_height_ += 0.03f; // 逐渐变大 (从 -5 向 0 靠近)

        if (land_height_ >= -0.3f && !landing_started_)
        {
            this->land(); 
            landing_started_ = true;
            RCLCPP_INFO(this->get_logger(), "发送着陆命令");
        }

        if (land_height_ >= 0.2f)
        {                    
            land_height_ = 0.2f; 
            flight_phase_ = LANDED;
            RCLCPP_INFO(this->get_logger(), "着陆完成");
        }
        
        // 特殊处理：降落阶段我们直接控制 msg.position[2]
        msg.position[2] = land_height_; 
        break;

    case LANDED:
        target_x_enu = takeoff_origin_x_;
        target_y_enu = takeoff_origin_y_;
        msg.position[2] = 0.2f; // 地面

        takeoff_timer_++;
        if (takeoff_timer_ > 20)
        { 
            this->disarm();
            RCLCPP_INFO(this->get_logger(), "任务完成，无人机已解除武装");
            takeoff_timer_ = 0; 
        }
        break;
    }

    // =========================================================
    // 【核心转换逻辑】 Gazebo ENU -> PX4 NED
    // =========================================================
    
    // 1. 计算【相对位移】 (目标点 - 起飞点)
    double offset_x_enu = target_x_enu - takeoff_origin_x_;
    double offset_y_enu = target_y_enu - takeoff_origin_y_;

    // 2. 坐标轴映射
    // PX4 North (x) = Gazebo North (y)
    msg.position[0] = static_cast<float>(offset_y_enu);

    // PX4 East (y)  = Gazebo East (x)
    msg.position[1] = static_cast<float>(offset_x_enu);

    // PX4 Down (z)  = Gazebo Up (-z)
    // 如果不是降落阶段（降落阶段上面已经手动设置了），则使用默认飞行高度
    if (flight_phase_ != LAND && flight_phase_ != LANDED) {
        msg.position[2] = static_cast<float>(flight_height_);
    }

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

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