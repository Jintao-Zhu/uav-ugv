#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

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
using px4_msgs::msg::VehicleLocalPosition;

/* ---------------- Waypoint ---------------- */

struct Waypoint
{
  float x;
  float y;
  float z;
};

/* ================================================================
 * Offboard Sweep Flight Node (Position Interpolation Version)
 * ================================================================ */

class OffboardControl : public rclcpp::Node
{
public:
  OffboardControl() : Node("sweep_flight_node")
  {
    /* -------- Publishers -------- */

    offboard_control_mode_pub_ =
        create_publisher<OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

    trajectory_setpoint_pub_ =
        create_publisher<TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

    vehicle_command_pub_ =
        create_publisher<VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

    /* -------- Subscriber -------- */

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.durability_volatile();

    local_pos_sub_ =
        create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position",
            qos,
            std::bind(&OffboardControl::local_pos_cb, this, std::placeholders::_1));

    /* -------- Parameters -------- */

    declare_parameter<std::string>(
        "waypoint_csv",
        "/home/suda/drone_ugv_ws/src/rmf_coverage_planner/data/waypoints.csv");

    declare_parameter<double>("flight_height", -5.0);

    flight_height_ = get_parameter("flight_height").as_double();

    load_waypoints(get_parameter("waypoint_csv").as_string());

    /* -------- Timer -------- */

    timer_ = create_wall_timer(
        100ms,
        std::bind(&OffboardControl::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Sweep flight node started");
  }

private:
  /* ---------------- ROS ---------------- */

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_pos_sub_;

  uint64_t offboard_counter_{0};

  /* ---------------- State ---------------- */

  float cur_x_{0}, cur_y_{0}, cur_z_{0};
  bool pos_valid_{false};

  /* ---------------- Waypoints ---------------- */

  std::vector<Waypoint> waypoints_;
  size_t wp_idx_{0};

  /* ---------------- Flight Phases ---------------- */

  enum Phase
  {
    TAKEOFF,
    HOVER,
    WAYPOINT_MOVE,
    WAYPOINT_HOLD,
    LAND
  };

  Phase phase_{TAKEOFF};

  int hold_ticks_{0};

  /* ---------------- Target setpoint (interpolated) ---------------- */

  float sp_x_{0.0f};
  float sp_y_{0.0f};
  float sp_z_{0.0f};

  /* ---------------- Parameters ---------------- */

  double flight_height_;

  /* ---------------- Constants ---------------- */

  const float STEP_XY_ = 0.5f;     // m per 100ms  (~1.5 m/s)
  const float STEP_Z_  = 0.10f;     // m per 100ms
  const float POS_TOL_ = 0.25f;     // arrival tolerance
  const int   HOLD_TICKS_ = 5;      // 0.5 s

  /* ============================================================= */

  void local_pos_cb(const VehicleLocalPosition::SharedPtr msg)
  {
    if (!msg->xy_valid || !msg->z_valid)
      return;

    cur_x_ = msg->x;
    cur_y_ = msg->y;
    cur_z_ = msg->z;
    pos_valid_ = true;
  }

  void timer_callback()
  {
    if (offboard_counter_ == 10)
    {
      set_offboard_mode();
      arm();
      RCLCPP_INFO(get_logger(), "Offboard enabled, armed");
    }

    publish_offboard_control_mode();
    publish_trajectory_setpoint();

    if (offboard_counter_ < 11)
      offboard_counter_++;
  }

  /* ---------------- Waypoints ---------------- */

  void load_waypoints(const std::string &path)
  {
    std::ifstream file(path);
    std::string line;
    std::getline(file, line); // header

    while (std::getline(file, line))
    {
      Waypoint wp;
      sscanf(line.c_str(), "%f,%f,%f", &wp.x, &wp.y, &wp.z);
      waypoints_.push_back(wp);
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu waypoints", waypoints_.size());
  }

  /* ---------------- Control ---------------- */

  void publish_offboard_control_mode()
  {
    OffboardControlMode msg{};
    msg.position = true;
    msg.timestamp = now_us();
    offboard_control_mode_pub_->publish(msg);
  }

  void publish_trajectory_setpoint()
  {
    TrajectorySetpoint msg{};

    if (!pos_valid_)
      return;

    switch (phase_)
    {
    case TAKEOFF:
      sp_x_ = 0.0f;
      sp_y_ = 0.0f;
      sp_z_ = flight_height_;

      if (std::fabs(cur_z_ - flight_height_) < 0.3f)
      {
        phase_ = HOVER;
        hold_ticks_ = 0;
        RCLCPP_INFO(get_logger(), "Takeoff complete");
      }
      break;

    case HOVER:
      sp_x_ = 0.0f;
      sp_y_ = 0.0f;
      sp_z_ = flight_height_;

      if (++hold_ticks_ > 20)
      {
        phase_ = WAYPOINT_MOVE;
        wp_idx_ = 0;
        RCLCPP_INFO(get_logger(), "Start waypoint flight");
      }
      break;

    case WAYPOINT_MOVE:
    {
      if (wp_idx_ >= waypoints_.size())
      {
        phase_ = LAND;
        RCLCPP_INFO(get_logger(), "Waypoints finished");
        break;
      }

      const auto &wp = waypoints_[wp_idx_];

      move_towards(sp_x_, wp.x, STEP_XY_);
      move_towards(sp_y_, wp.y, STEP_XY_);
      move_towards(sp_z_, wp.z, STEP_Z_);

      float dist = std::hypot(
          std::hypot(cur_x_ - wp.x, cur_y_ - wp.y),
          cur_z_ - wp.z);

      if (dist < POS_TOL_)
      {
        phase_ = WAYPOINT_HOLD;
        hold_ticks_ = 0;
      }
      break;
    }

    case WAYPOINT_HOLD:
      if (++hold_ticks_ >= HOLD_TICKS_)
      {
        wp_idx_++;
        phase_ = WAYPOINT_MOVE;
      }
      break;

    case LAND:
      sp_x_ = 0.0f;
      sp_y_ = 0.0f;
      sp_z_ += 0.05f;

      if (sp_z_ >= -0.2f)
      {
        land();
      }
      break;
    }

    msg.position = {sp_x_, sp_y_, sp_z_};
    msg.yaw = 0.0f;
    msg.timestamp = now_us();
    trajectory_setpoint_pub_->publish(msg);
  }

  /* ---------------- Utils ---------------- */

  void move_towards(float &cur, float target, float step)
  {
    float d = target - cur;
    if (std::fabs(d) < step)
      cur = target;
    else
      cur += step * (d > 0 ? 1.0f : -1.0f);
  }

  void arm()
  {
    send_vehicle_command(
        VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
  }

  void land()
  {
    send_vehicle_command(
        VehicleCommand::VEHICLE_CMD_NAV_LAND);
  }

  void set_offboard_mode()
  {
    send_vehicle_command(
        VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
  }

  void send_vehicle_command(uint16_t cmd, float p1 = 0, float p2 = 0)
  {
    VehicleCommand msg{};
    msg.command = cmd;
    msg.param1 = p1;
    msg.param2 = p2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.from_external = true;
    msg.timestamp = now_us();
    vehicle_command_pub_->publish(msg);
  }

  uint64_t now_us()
  {
    return get_clock()->now().nanoseconds() / 1000;
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
