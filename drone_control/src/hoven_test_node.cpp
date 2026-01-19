#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;

/* ================================================================
 * Hover + Vertical Takeoff Test Node
 * ================================================================ */

class HoverTestNode : public rclcpp::Node
{
public:
  HoverTestNode()
  : Node("hover_test_node")
  {
    /* ---------------- Publishers ---------------- */

    offboard_control_mode_pub_ =
        create_publisher<OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

    trajectory_setpoint_pub_ =
        create_publisher<TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

    vehicle_command_pub_ =
        create_publisher<VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

    /* ---------------- Parameters ---------------- */

    declare_parameter<double>("takeoff_z", -7.0);

    declare_parameter<double>("target_x", 0.0);
    declare_parameter<double>("target_y", 0.0);
    declare_parameter<double>("target_z", -7.0);

    takeoff_z_ = get_parameter("takeoff_z").as_double();

    target_x_  = get_parameter("target_x").as_double();
    target_y_  = get_parameter("target_y").as_double();
    target_z_  = get_parameter("target_z").as_double();

    RCLCPP_INFO(get_logger(),
      "Takeoff at (0, 0) to z=%.2f\n"
      "Target hover: (%.2f, %.2f, %.2f)",
      takeoff_z_,
      target_x_, target_y_, target_z_);

    /* ---------------- Timer ---------------- */

    timer_ = create_wall_timer(
        100ms,
        std::bind(&HoverTestNode::timer_callback, this));
  }

private:
  /* ---------------- Flight phase ---------------- */

  enum class FlightPhase
  {
    TAKEOFF,
    GOTO,
    HOVER
  };

  FlightPhase phase_{FlightPhase::TAKEOFF};

  /* ---------------- ROS ---------------- */

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr
      offboard_control_mode_pub_;

  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr
      trajectory_setpoint_pub_;

  rclcpp::Publisher<VehicleCommand>::SharedPtr
      vehicle_command_pub_;

  uint64_t offboard_counter_{0};
  uint64_t phase_counter_{0};

  /* ---------------- Coordinates ---------------- */

  static constexpr float START_X = 0.0f;
  static constexpr float START_Y = 0.0f;

  float takeoff_z_;

  float target_x_;
  float target_y_;
  float target_z_;

  /* ---------------- Timer callback ---------------- */

  void timer_callback()
  {
    publish_offboard_control_mode();

    /* PX4 要求：先发 10 次 setpoint */
    if (offboard_counter_ < 10)
    {
      publish_takeoff_setpoint();
      offboard_counter_++;
      return;
    }

    if (offboard_counter_ == 10)
    {
      set_offboard_mode();
      arm();
      RCLCPP_INFO(get_logger(), "Offboard enabled, vehicle armed");
      offboard_counter_++;
      return;
    }

    /* ---------------- Flight logic ---------------- */

    switch (phase_)
    {
      case FlightPhase::TAKEOFF:
        publish_takeoff_setpoint();
        phase_counter_++;

        if (phase_counter_ > 50)  // ~5 秒
        {
          RCLCPP_INFO(get_logger(), "Takeoff complete, flying horizontally");
          phase_ = FlightPhase::GOTO;
          phase_counter_ = 0;
        }
        break;

      case FlightPhase::GOTO:
        publish_goto_setpoint();
        phase_counter_++;

        if (phase_counter_ > 80)  // ~8 秒
        {
          RCLCPP_INFO(get_logger(), "Reached target, hovering");
          phase_ = FlightPhase::HOVER;
        }
        break;

      case FlightPhase::HOVER:
        publish_hover_setpoint();
        break;
    }
  }

  /* ---------------- Setpoints ---------------- */

  void publish_takeoff_setpoint()
  {
    TrajectorySetpoint msg{};
    msg.position = {START_X, START_Y, takeoff_z_};
    msg.yaw = 0.0f;
    msg.timestamp = now_us();
    trajectory_setpoint_pub_->publish(msg);
  }

  void publish_goto_setpoint()
  {
    TrajectorySetpoint msg{};
    msg.position = {target_x_, target_y_, takeoff_z_};
    msg.yaw = 0.0f;
    msg.timestamp = now_us();
    trajectory_setpoint_pub_->publish(msg);
  }

  void publish_hover_setpoint()
  {
    TrajectorySetpoint msg{};
    msg.position = {target_x_, target_y_, target_z_};
    msg.yaw = 0.0f;
    msg.timestamp = now_us();
    trajectory_setpoint_pub_->publish(msg);
  }

  /* ---------------- Offboard control ---------------- */

  void publish_offboard_control_mode()
  {
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = now_us();
    offboard_control_mode_pub_->publish(msg);
  }

  /* ---------------- Vehicle commands ---------------- */

  void arm()
  {
    send_vehicle_command(
        VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        1.0);
  }

  void set_offboard_mode()
  {
    send_vehicle_command(
        VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        1, 6);
  }

  void send_vehicle_command(
      uint16_t command,
      float p1 = 0.0,
      float p2 = 0.0)
  {
    VehicleCommand msg{};
    msg.command = command;
    msg.param1 = p1;
    msg.param2 = p2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = now_us();
    vehicle_command_pub_->publish(msg);
  }

  uint64_t now_us()
  {
    return this->get_clock()->now().nanoseconds() / 1000;
  }
};

/* ---------------- main ---------------- */

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HoverTestNode>());
  rclcpp::shutdown();
  return 0;
}
