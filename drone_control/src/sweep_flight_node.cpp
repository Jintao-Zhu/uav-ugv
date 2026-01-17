#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;

/* ---------------- Waypoint struct ---------------- */

struct Waypoint
{
  float x;
  float y;
  float z;
};

/* ---------------- Offboard Node ---------------- */

class OffboardControl : public rclcpp::Node
{
public:
  OffboardControl() : Node("sweep_flight_node")
  {
    offboard_control_mode_pub_ =
        create_publisher<OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

    trajectory_setpoint_pub_ =
        create_publisher<TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

    vehicle_command_pub_ =
        create_publisher<VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

    /* parameters */
    declare_parameter<std::string>(
        "waypoint_csv",
        "/home/suda/drone_ugv_ws/src/rmf_coverage_planner/data/waypoints.csv");

    declare_parameter<double>("flight_height", -5.0);

    flight_height_ =
        get_parameter("flight_height").as_double();

    load_waypoints(
        get_parameter("waypoint_csv").as_string());

    timer_ = create_wall_timer(
        100ms,
        std::bind(&OffboardControl::timer_callback, this));
  }

private:
  /* ---------------- ROS ---------------- */

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr
      offboard_control_mode_pub_;

  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr
      trajectory_setpoint_pub_;

  rclcpp::Publisher<VehicleCommand>::SharedPtr
      vehicle_command_pub_;

  uint64_t offboard_counter_{0};

  /* ---------------- Waypoints ---------------- */

  std::vector<Waypoint> waypoints_;
  size_t wp_idx_{0};

  int hold_count_{0};

  /* ---------------- Flight state ---------------- */

  enum FlightPhase
  {
    TAKEOFF,
    HOVER,
    WAYPOINT_FOLLOW,
    LAND,
    LANDED
  };

  FlightPhase phase_{TAKEOFF};

  int phase_timer_{0};
  float land_z_{0.0f};
  bool land_cmd_sent_{false};

  double flight_height_;

  /* ---------------- Core logic ---------------- */

  void timer_callback()
  {
    if (offboard_counter_ == 10)
    {
      set_offboard_mode();
      arm();
    }

    publish_offboard_control_mode();
    publish_trajectory_setpoint();

    if (offboard_counter_ < 11)
      offboard_counter_++;
  }

  /* ---------------- Waypoint load ---------------- */

  void load_waypoints(const std::string &path)
  {
    std::ifstream file(path);
    if (!file.is_open())
    {
      RCLCPP_FATAL(get_logger(),
                   "Cannot open waypoint csv: %s",
                   path.c_str());
      rclcpp::shutdown();
      return;
    }

    std::string line;
    std::getline(file, line); // header

    while (std::getline(file, line))
    {
      std::stringstream ss(line);
      std::string token;

      Waypoint wp;
      std::getline(ss, token, ',');
      wp.x = std::stof(token);
      std::getline(ss, token, ',');
      wp.y = std::stof(token);
      std::getline(ss, token, ',');
      wp.z = std::stof(token);

      waypoints_.push_back(wp);
    }

    RCLCPP_INFO(get_logger(),
                "Loaded %zu waypoints",
                waypoints_.size());
  }

  /* ---------------- Publishers ---------------- */

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

  void publish_trajectory_setpoint()
  {
    TrajectorySetpoint msg{};

    switch (phase_)
    {
    case TAKEOFF:
      msg.position = {0.0f, 0.0f, (float)flight_height_};
      msg.yaw = 0.0f;

      if (++phase_timer_ > 50)
      {
        phase_ = HOVER;
        phase_timer_ = 0;
        RCLCPP_INFO(get_logger(), "Takeoff complete");
      }
      break;

    case HOVER:
      msg.position = {0.0f, 0.0f, (float)flight_height_};
      msg.yaw = 0.0f;

      if (++phase_timer_ > 20)
      {
        phase_ = WAYPOINT_FOLLOW;
        wp_idx_ = 0;
        RCLCPP_INFO(get_logger(), "Start waypoint flight");
      }
      break;

    case WAYPOINT_FOLLOW:
    {
      if (wp_idx_ >= waypoints_.size())
      {
        phase_ = LAND;
        land_z_ = flight_height_;
        RCLCPP_INFO(get_logger(), "Waypoints done");
        break;
      }

      const auto &wp = waypoints_[wp_idx_];
      msg.position = {wp.x, wp.y, wp.z};

      if (wp_idx_ + 1 < waypoints_.size())
      {
        const auto &next = waypoints_[wp_idx_ + 1];
        msg.yaw = std::atan2(
            next.y - wp.y,
            next.x - wp.x);
      }

      if (++hold_count_ > 5)
      {
        wp_idx_++;
        hold_count_ = 0;
      }
      break;
    }

    case LAND:
      land_z_ += 0.03f;
      msg.position = {0.0f, 0.0f, land_z_};

      if (land_z_ >= -0.3f && !land_cmd_sent_)
      {
        land();
        land_cmd_sent_ = true;
      }

      if (land_z_ >= 0.2f)
      {
        phase_ = LANDED;
        phase_timer_ = 0;
        RCLCPP_INFO(get_logger(), "Landed");
      }
      break;

    case LANDED:
      msg.position = {0.0f, 0.0f, 0.2f};

      if (++phase_timer_ > 20)
      {
        disarm();
      }
      break;
    }

    msg.timestamp = now_us();
    trajectory_setpoint_pub_->publish(msg);
  }

  /* ---------------- Vehicle commands ---------------- */

  void arm()
  {
    send_vehicle_command(
        VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        1.0);
  }

  void disarm()
  {
    send_vehicle_command(
        VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        0.0);
  }

  void land()
  {
    send_vehicle_command(
        VehicleCommand::VEHICLE_CMD_NAV_LAND);
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
  rclcpp::spin(std::make_shared<OffboardControl>());
  rclcpp::shutdown();
  return 0;
}
