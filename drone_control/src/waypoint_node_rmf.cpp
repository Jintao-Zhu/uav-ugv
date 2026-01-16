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
    // 航点结构体
    struct Waypoint
    {
        std::string name;
        double x;  // 全局X坐标(米)
        double y;  // 全局Y坐标(米)
        double z;  // 目标高度(米,地面以上)
        
        Waypoint(const std::string& n, double x_val, double y_val, double z_val)
            : name(n), x(x_val), y(y_val), z(z_val) {}
    };

    OffboardControl() : Node("waypoint_inspection_node")
    {
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        offboard_setpoint_counter_ = 0;

        // =================================================================
        // 【配置区域】
        // =================================================================
        
        // 1. 飞行参数
        flight_height_ = -5.0;      // 飞行高度 (PX4 NED坐标，负数为向上 5米)
        hover_duration_ = 100;      // 每个航点悬停时长(×100ms = 10秒) ← 增加到10秒
        waypoint_threshold_ = 2.0;  // 到达航点的距离阈值(米)
        flight_speed_ = 400;         // 飞行到航点的时间(×100ms = 10秒) ← 放慢飞行速度

        // 2. 起飞原点
        takeoff_origin_x_ = 40.0; 
        takeoff_origin_y_ = -40.0; 

        // 3. 定义巡检航点列表 (按顺序飞行)
        waypoints_ = {
            Waypoint("red_cube_n14", 80.84, -28.52, 5.0),
            Waypoint("red_cube_n13", 84.44, -4.94, 5.0),
            // Waypoint("red_cube_n23", 182.80, -42.30, 5.0),
            Waypoint("red_cube_west_koi_pond", 34.32, -10.13, 5.0),
            Waypoint("red_cube_s08", 96.61, -50.50, 5.0),
            Waypoint("red_cube_s10", 122.10, -46.68, 5.0),
            //Waypoint("red_cube_s11", 152.73, -43.00, 5.0),
            Waypoint("red_cube_junction_south_west", 84.56, -38.81, 5.0)
        };

        // =================================================================

        current_waypoint_index_ = 0;
        flight_phase_ = TAKEOFF;
        takeoff_timer_ = 0;
        hover_timer_ = 0;
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

        RCLCPP_INFO(this->get_logger(), "=== 多点巡检任务初始化完成 ===");
        RCLCPP_INFO(this->get_logger(), "起飞点: (%.2f, %.2f)", takeoff_origin_x_, takeoff_origin_y_);
        RCLCPP_INFO(this->get_logger(), "巡检航点数量: %zu", waypoints_.size());
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

    // 航点相关
    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_index_;
    double waypoint_threshold_;

    // 坐标变量
    double flight_height_;    
    double takeoff_origin_x_;    
    double takeoff_origin_y_;    
    int hover_duration_;
    int flight_speed_;  // 飞行速度(时间)

    float land_height_;    
    bool landing_started_; 

    enum FlightPhase
    {
        TAKEOFF,           // 起飞阶段
        INITIAL_HOVER,     // 起飞后初始悬停
        FLYING_TO_WAYPOINT,// 飞往航点
        HOVERING_AT_WAYPOINT, // 在航点悬停
        RETURN_HOME,       // 返回起点
        LAND,              // 降落阶段
        LANDED             // 已着陆
    };

    FlightPhase flight_phase_;
    int takeoff_timer_;
    int hover_timer_;

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    
    // 计算当前位置到目标点的距离
    double distance_to_target(double target_x, double target_y);
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

double OffboardControl::distance_to_target(double target_x, double target_y)
{
    // 注意: 这里简化处理,实际应该订阅无人机当前位置
    // 为了简化,我们假设已经到达目标点附近
    // 在实际应用中,应该订阅 /fmu/out/vehicle_local_position 来获取当前位置
    return 0.0; // 简化处理
}

void OffboardControl::publish_trajectory_setpoint()
{
    TrajectorySetpoint msg{};
    double target_x_enu = 0.0;
    double target_y_enu = 0.0;

    switch (flight_phase_)
    {
    case TAKEOFF:
        // 起飞: 垂直上升到指定高度
        target_x_enu = takeoff_origin_x_;
        target_y_enu = takeoff_origin_y_;
        
        takeoff_timer_++;
        if (takeoff_timer_ > 50) // 5秒
        { 
            flight_phase_ = INITIAL_HOVER;
            takeoff_timer_ = 0;
            RCLCPP_INFO(this->get_logger(), "✅ 起飞完成，初始悬停");
        }
        break;

    case INITIAL_HOVER:
        // 起飞后短暂悬停
        target_x_enu = takeoff_origin_x_;
        target_y_enu = takeoff_origin_y_;
        
        takeoff_timer_++;
        if (takeoff_timer_ > 20) // 2秒
        { 
            if (current_waypoint_index_ < waypoints_.size())
            {
                flight_phase_ = FLYING_TO_WAYPOINT;
                RCLCPP_INFO(this->get_logger(), "🎯 开始飞往第 %zu 个航点: %s (%.2f, %.2f)", 
                    current_waypoint_index_ + 1,
                    waypoints_[current_waypoint_index_].name.c_str(),
                    waypoints_[current_waypoint_index_].x,
                    waypoints_[current_waypoint_index_].y);
            }
            else
            {
                flight_phase_ = RETURN_HOME;
                RCLCPP_INFO(this->get_logger(), "🏠 所有航点已完成，返回起点");
            }
            takeoff_timer_ = 0;
        }
        break;

    case FLYING_TO_WAYPOINT:
        {
            // 飞往当前航点
            const Waypoint& wp = waypoints_[current_waypoint_index_];
            target_x_enu = wp.x;
            target_y_enu = wp.y;
            
            // 计算朝向目标点的偏航角
            double dx = target_x_enu - takeoff_origin_x_;
            double dy = target_y_enu - takeoff_origin_y_;
            double yaw_enu = std::atan2(dy, dx);
            msg.yaw = static_cast<float>(-yaw_enu + M_PI / 2); // ENU转NED
            
            // 使用可配置的飞行速度参数
            takeoff_timer_++;
            if (takeoff_timer_ > flight_speed_) // 6秒 (原来3秒)
            {
                flight_phase_ = HOVERING_AT_WAYPOINT;
                hover_timer_ = 0;
                RCLCPP_INFO(this->get_logger(), "📍 到达航点: %s，开始悬停观测", 
                    wp.name.c_str());
                takeoff_timer_ = 0;
            }
        }
        break;

    case HOVERING_AT_WAYPOINT:
        {
            // 在航点悬停
            const Waypoint& wp = waypoints_[current_waypoint_index_];
            target_x_enu = wp.x;
            target_y_enu = wp.y;
            
            hover_timer_++;
            if (hover_timer_ >= hover_duration_) // 悬停10秒 (原来5秒)
            {
                current_waypoint_index_++;
                hover_timer_ = 0;
                
                if (current_waypoint_index_ < waypoints_.size())
                {
                    flight_phase_ = FLYING_TO_WAYPOINT;
                    RCLCPP_INFO(this->get_logger(), "🎯 飞往下一个航点: %s (%.2f, %.2f)", 
                        waypoints_[current_waypoint_index_].name.c_str(),
                        waypoints_[current_waypoint_index_].x,
                        waypoints_[current_waypoint_index_].y);
                }
                else
                {
                    flight_phase_ = RETURN_HOME;
                    takeoff_timer_ = 0;
                    RCLCPP_INFO(this->get_logger(), "🏠 所有航点已完成，返回起点");
                }
            }
        }
        break;

    case RETURN_HOME:
        // 返回起飞点
        target_x_enu = takeoff_origin_x_;
        target_y_enu = takeoff_origin_y_;
        
        takeoff_timer_++;
        if (takeoff_timer_ > 30) // 3秒
        { 
            flight_phase_ = LAND;
            takeoff_timer_ = 0; 
            RCLCPP_INFO(this->get_logger(), "🛬 开始降落流程");
        }
        break;

    case LAND:
        // 降落: 保持水平位置，逐渐降低高度
        target_x_enu = takeoff_origin_x_;
        target_y_enu = takeoff_origin_y_;

        land_height_ += 0.03f; // 每次上升0.03m (从-5向0靠近)

        if (land_height_ >= -0.3f && !landing_started_)
        {
            this->land(); 
            landing_started_ = true;
            RCLCPP_INFO(this->get_logger(), "📡 发送PX4着陆命令");
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
        if (takeoff_timer_ > 20) // 2秒后解锁
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
    std::cout << "Starting waypoint inspection node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}