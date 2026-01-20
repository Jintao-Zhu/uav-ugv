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
#include <vector>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    struct Waypoint
    {
        std::string name;
        double x, y, z;
        
        Waypoint(const std::string& n, double x_val, double y_val, double z_val)
            : name(n), x(x_val), y(y_val), z(z_val) {}
    };

    OffboardControl() : Node("circular_inspection_node")
    {
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        offboard_setpoint_counter_ = 0;

        // =================================================================
        // 【配置区域 - 环绕式巡检】
        // =================================================================
        
        // 1. 飞行参数
        flight_height_ = -6.0;           // 飞行高度 5米
        approach_speed_ = 300;           // 接近目标的速度 (10秒)
        
        // 2. 环绕参数 (关键配置)
        circle_radius_ = 1;            // 环绕半径 6米
        angular_velocity_ = 0.20;        // 角速度 (rad/s) - 控制环绕速度
        current_circle_angle_ = 0.0;     // 当前环绕角度
        
        // 3. 起飞原点
        takeoff_origin_x_ = 40.0; 
        takeoff_origin_y_ = -40.0; 

        // 4. 巡检航点列表
        waypoints_ = {
            Waypoint("red_cube_west_koi_pond", 34.32, -10.13, 5.0),
            Waypoint("red_cube_n14", 80.84, -28.52, 5.0),
            Waypoint("red_cube_n13", 84.44, -4.94, 5.0),
            Waypoint("red_cube_junction_south_west", 84.56, -38.81, 5.0),
            Waypoint("red_cube_s08", 96.61, -50.50, 5.0),
            Waypoint("red_cube_s10", 122.10, -46.68, 5.0)
        };

        // =================================================================

        current_waypoint_index_ = 0;
        flight_phase_ = TAKEOFF;
        takeoff_timer_ = 0;
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

        RCLCPP_INFO(this->get_logger(), "=== 环绕式巡检任务初始化 ===");
        RCLCPP_INFO(this->get_logger(), "起飞点: (%.2f, %.2f)", takeoff_origin_x_, takeoff_origin_y_);
        RCLCPP_INFO(this->get_logger(), "巡检目标: %zu 个", waypoints_.size());
        RCLCPP_INFO(this->get_logger(), "环绕半径: %.1f 米", circle_radius_);
        RCLCPP_INFO(this->get_logger(), "环绕速度: %.3f rad/s", angular_velocity_);
        
        // 计算环绕一圈的时间
        double circle_time = 2 * M_PI / angular_velocity_ * 0.1;
        RCLCPP_INFO(this->get_logger(), "环绕一圈时间: %.1f 秒", circle_time);
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

    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_index_;

    double flight_height_;    
    double takeoff_origin_x_;    
    double takeoff_origin_y_;    
    int approach_speed_;
    
    // 环绕相关参数
    double circle_radius_;
    double angular_velocity_;
    double current_circle_angle_;

    float land_height_;    
    bool landing_started_; 

    enum FlightPhase
    {
        TAKEOFF,              // 起飞
        INITIAL_HOVER,        // 起飞后悬停
        APPROACHING_TARGET,   // 接近目标点
        CIRCLING_TARGET,      // 环绕目标点
        RETURN_HOME,          // 返回起点
        LAND,                 // 降落
        LANDED                // 已着陆
    };

    FlightPhase flight_phase_;
    int takeoff_timer_;

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

void OffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "✈️ 解锁命令已发送");
}

void OffboardControl::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "🔒 上锁命令已发送");
}

void OffboardControl::land()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(this->get_logger(), "🛬 着陆命令已发送");
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

void OffboardControl::publish_trajectory_setpoint()
{
    TrajectorySetpoint msg{};
    double target_x_enu = 0.0;
    double target_y_enu = 0.0;

    switch (flight_phase_)
    {
    case TAKEOFF:
        target_x_enu = takeoff_origin_x_;
        target_y_enu = takeoff_origin_y_;
        
        takeoff_timer_++;
        if (takeoff_timer_ > 50)  // 5秒
        { 
            flight_phase_ = INITIAL_HOVER;
            takeoff_timer_ = 0;
            RCLCPP_INFO(this->get_logger(), "✅ 起飞完成");
        }
        break;

    case INITIAL_HOVER:
        target_x_enu = takeoff_origin_x_;
        target_y_enu = takeoff_origin_y_;
        
        takeoff_timer_++;
        if (takeoff_timer_ > 20)  // 2秒
        { 
            if (current_waypoint_index_ < waypoints_.size())
            {
                flight_phase_ = APPROACHING_TARGET;
                takeoff_timer_ = 0;
                RCLCPP_INFO(this->get_logger(), "🎯 飞往目标 %zu: %s", 
                    current_waypoint_index_ + 1,
                    waypoints_[current_waypoint_index_].name.c_str());
            }
            else
            {
                flight_phase_ = RETURN_HOME;
                takeoff_timer_ = 0;
            }
        }
        break;

    case APPROACHING_TARGET:
        {
            // 飞往目标点附近
            const Waypoint& wp = waypoints_[current_waypoint_index_];
            target_x_enu = wp.x;
            target_y_enu = wp.y;
            
            // 朝向目标点
            double dx = target_x_enu - takeoff_origin_x_;
            double dy = target_y_enu - takeoff_origin_y_;
            double yaw_enu = std::atan2(dy, dx);
            msg.yaw = static_cast<float>(-yaw_enu + M_PI / 2);
            
            takeoff_timer_++;
            if (takeoff_timer_ > approach_speed_)
            {
                // 到达目标点,开始环绕
                flight_phase_ = CIRCLING_TARGET;
                current_circle_angle_ = 0.0;  // 从0度开始环绕
                takeoff_timer_ = 0;
                RCLCPP_INFO(this->get_logger(), "📍 到达 %s 附近，开始环绕巡检", 
                    wp.name.c_str());
            }
        }
        break;

    case CIRCLING_TARGET:
        {
            // =========================================================
            // 【核心逻辑】环绕目标点飞行,相机朝向圆心
            // =========================================================
            const Waypoint& wp = waypoints_[current_waypoint_index_];
            
            // 1. 计算环绕轨迹上的位置 (以目标点为圆心)
            target_x_enu = wp.x + circle_radius_ * cos(current_circle_angle_);
            target_y_enu = wp.y + circle_radius_ * sin(current_circle_angle_);
            
            // 2. 计算偏航角 (朝向圆心)
            // 从当前位置指向圆心的方向
            double dx_to_center = wp.x - target_x_enu;
            double dy_to_center = wp.y - target_y_enu;
            double yaw_to_center_enu = std::atan2(dy_to_center, dx_to_center);
            
            // 转换为 NED 坐标系
            msg.yaw = static_cast<float>(-yaw_to_center_enu + M_PI / 2);
            
            // 3. 更新环绕角度
            current_circle_angle_ += angular_velocity_ * 0.1;  // 每0.1秒更新一次
            
            // 4. 完成一圈后进入下一个目标
            if (current_circle_angle_ >= 2 * M_PI)
            {
                RCLCPP_INFO(this->get_logger(), "✅ 完成 %s 的环绕巡检", 
                    wp.name.c_str());
                
                current_waypoint_index_++;
                current_circle_angle_ = 0.0;
                
                if (current_waypoint_index_ < waypoints_.size())
                {
                    flight_phase_ = APPROACHING_TARGET;
                    takeoff_timer_ = 0;
                    RCLCPP_INFO(this->get_logger(), "🎯 飞往下一个目标: %s", 
                        waypoints_[current_waypoint_index_].name.c_str());
                }
                else
                {
                    flight_phase_ = RETURN_HOME;
                    takeoff_timer_ = 0;
                    RCLCPP_INFO(this->get_logger(), "🏠 所有目标巡检完成，返回起点");
                }
            }
            
            // 5. 每45度(π/4)打印一次进度
            if (fmod(current_circle_angle_, M_PI / 4) < angular_velocity_ * 0.1)
            {
                double progress = current_circle_angle_ / (2 * M_PI) * 100.0;
                RCLCPP_INFO(this->get_logger(), "🔄 环绕进度: %.0f%%", progress);
            }
        }
        break;

    case RETURN_HOME:
        target_x_enu = takeoff_origin_x_;
        target_y_enu = takeoff_origin_y_;
        
        takeoff_timer_++;
        if (takeoff_timer_ > 100)  // 10秒返回
        { 
            flight_phase_ = LAND;
            takeoff_timer_ = 0; 
            RCLCPP_INFO(this->get_logger(), "🛬 开始降落");
        }
        break;

    case LAND:
        target_x_enu = takeoff_origin_x_;
        target_y_enu = takeoff_origin_y_;

        land_height_ += 0.03f;

        if (land_height_ >= -0.3f && !landing_started_)
        {
            this->land(); 
            landing_started_ = true;
            RCLCPP_INFO(this->get_logger(), "📡 发送着陆命令");
        }

        if (land_height_ >= 0.2f)
        {                    
            land_height_ = 0.2f; 
            flight_phase_ = LANDED;
            RCLCPP_INFO(this->get_logger(), "✅ 着陆完成");
        }
        
        msg.position[2] = land_height_; 
        break;

    case LANDED:
        target_x_enu = takeoff_origin_x_;
        target_y_enu = takeoff_origin_y_;
        msg.position[2] = 0.2f;

        takeoff_timer_++;
        if (takeoff_timer_ > 20)
        { 
            this->disarm();
            RCLCPP_INFO(this->get_logger(), "🎉 任务完成! 共巡检 %zu 个目标", waypoints_.size());
            takeoff_timer_ = 0; 
        }
        break;
    }

    // =========================================================
    // 【坐标转换】 Gazebo ENU -> PX4 NED
    // =========================================================
    
    double offset_x_enu = target_x_enu - takeoff_origin_x_;
    double offset_y_enu = target_y_enu - takeoff_origin_y_;

    msg.position[0] = static_cast<float>(offset_y_enu);  // North
    msg.position[1] = static_cast<float>(offset_x_enu);  // East

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
    std::cout << "Starting circular inspection node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}