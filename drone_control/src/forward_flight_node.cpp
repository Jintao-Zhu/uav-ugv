#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <array> 
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class ForwardFlightControl : public rclcpp::Node
{
public:
	ForwardFlightControl() : Node("forward_flight_node")
	{
		// 创建发布者
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		// 初始化计数器和位置变量
		offboard_setpoint_counter_ = 0;
		current_y_ = 0.0;  // 改为跟踪Y轴位置（东向）
		forward_speed_ = 0.5; // 前向飞行速度 m/s
		takeoff_altitude_ = -5.0; // 起飞高度（负值表示在机体坐标系下方）

		// 设置定时器回调函数，100ms执行一次
		auto timer_callback = [this]() -> void {
			// 发送10个setpoint后切换到offboard模式并解锁
			if (offboard_setpoint_counter_ == 10) {
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				this->arm();
			}

			// 发布控制模式和轨迹点
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			// 更新计数器，超过11后不再增加
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			} else {
				// 解锁后开始向东移动（y轴正向，因为Y轴指向东）
				current_y_ += forward_speed_ * 0.1; // 0.1是100ms的时间（秒）
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	std::atomic<uint64_t> timestamp_;   // 时间戳
	uint64_t offboard_setpoint_counter_;   // setpoint计数器
	double current_y_; // 当前y轴位置（东向）
	double forward_speed_; // 前向飞行速度
	double takeoff_altitude_; // 起飞高度

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

/**
 * @brief 发送解锁指令
 */
void ForwardFlightControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

/**
 * @brief 发送上锁指令
 */
void ForwardFlightControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

/**
 * @brief 发布offboard控制模式
 *        这里使用位置控制模式
 */
void ForwardFlightControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;    // 位置控制
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief 发布轨迹点
 *        持续更新y轴位置（东向），保持x轴和高度不变，保持固定偏航角
 */
void ForwardFlightControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	// y轴持续增加（向东飞行），x轴保持0，固定高度
	msg.position = {0.0, static_cast<float>(current_y_), static_cast<float>(takeoff_altitude_)};
	msg.yaw = 0.0; // 偏航角0度（面朝北，可根据需要调整）
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
	
	// 打印当前位置信息
	if (offboard_setpoint_counter_ >= 11) {
		RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
			"Current position: x=0.0, y=%.2f, z=%.2f", current_y_, takeoff_altitude_);
	}
}

/**
 * @brief 发布车辆指令
 * @param command   指令代码
 * @param param1    指令参数1
 * @param param2    指令参数2
 */
void ForwardFlightControl::publish_vehicle_command(uint16_t command, float param1, float param2)
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
	std::cout << "Starting forward flight node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ForwardFlightControl>());

	rclcpp::shutdown();
	return 0;
}
