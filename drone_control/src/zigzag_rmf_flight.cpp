// 文件：zigzag_rmf_full.cpp
// 描述：从 RMF building.yaml 读取 vertices (格式如 [x,y,z,""])，
// 生成按行 Zigzag 覆盖航路（每行只取起止两点，避免大量短跳点），
// 将 ENU -> PX4 NED 转换并通过 /fmu/in/trajectory_setpoint 控制 PX4（Offboard）。
// 依赖：rclcpp, px4_msgs, yaml-cpp
// 编译：在 package.xml 与 CMakeLists 中添加依赖 rclcpp, px4_msgs, yaml-cpp

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <yaml-cpp/yaml.h>

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;

class ZigzagRMFFull : public rclcpp::Node
{
public:
  ZigzagRMFFull(): Node("zigzag_rmf_full")
  {
    // 参数（可通过 launch/ros2 param 覆盖）
    declare_parameter<std::string>("rmf_yaml_path", "/home/suda/rmf_ws_2/src/rmf_demos/rmf_demos_maps/maps/airport_terminal/airport_terminal.building.yaml");
    declare_parameter<double>("line_spacing_m", 3.0);              // Zigzag 行间距（米）
    declare_parameter<double>("flight_height_m", 10.0);             // 期望巡航高度（米，ENU 正数）
    declare_parameter<double>("origin_x_m", 40.0);                  // 在 Gazebo 中对应 RMF(0,0) 的世界坐标 X (m)
    declare_parameter<double>("origin_y_m", -35.0);                  // 在 Gazebo 中对应 RMF(0,0) 的世界坐标 Y (m)
    declare_parameter<double>("takeoff_origin_x_m", 40.0);          // 起飞点 ENU x（米）
    declare_parameter<double>("takeoff_origin_y_m", -40.0);          // 起飞点 ENU y（米）
    declare_parameter<double>("cruise_speed_m_s", 2.0);           // 估算速度（m/s）
    declare_parameter<int>("offboard_setpoint_count", 12);        // 进入 Offboard 之前连续发送的 setpoint 数
    declare_parameter<double>("min_hold_time_s", 0.8);            // 每个航段最低保持时间（s）

    // 读取参数
    rmf_yaml_path_ = get_parameter("rmf_yaml_path").as_string();
    spacing_m_ = get_parameter("line_spacing_m").as_double();
    flight_height_m_ = get_parameter("flight_height_m").as_double();
    origin_x_m_ = get_parameter("origin_x_m").as_double();
    origin_y_m_ = get_parameter("origin_y_m").as_double();
    takeoff_origin_x_m_ = get_parameter("takeoff_origin_x_m").as_double();
    takeoff_origin_y_m_ = get_parameter("takeoff_origin_y_m").as_double();
    cruise_speed_m_s_ = get_parameter("cruise_speed_m_s").as_double();
    required_setpoints_ = get_parameter("offboard_setpoint_count").as_int();
    min_hold_time_s_ = get_parameter("min_hold_time_s").as_double();

    // PX4 话题 publisher
    offboard_control_mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    // 载入 RMF 顶点（支持 [x,y,z,""] 等形式）
    load_rmf_vertices();

    // 生成 Zigzag 航路 —— 只取每行起点和终点（两点），rows = ceil(height/spacing)
    generate_zigzag_waypoints();

    // state init
    flight_phase_ = FlightPhase::WARMUP;
    offboard_counter_ = 0;
    waypoint_idx_ = 0;
    hold_ticks_remaining_ = 0;
    landed_disarmed_ = false;

    // 主循环 10 Hz
    timer_ = this->create_wall_timer(100ms, std::bind(&ZigzagRMFFull::main_loop, this));

    RCLCPP_INFO(get_logger(), "Node started. Waypoints: %zu", enu_waypoints_.size());
  }

private:
  enum class FlightPhase { WARMUP, TAKEOFF_HOLD, ZIGZAG_CRUISE, RETURN_HOME, LAND, LANDED };

  // 参数
  std::string rmf_yaml_path_;
  double spacing_m_;
  double flight_height_m_;     // ENU 正数
  double origin_x_m_, origin_y_m_; // Gazebo 世界中对应 RMF 原点的位置 (m)
  double takeoff_origin_x_m_, takeoff_origin_y_m_;
  double cruise_speed_m_s_;
  int required_setpoints_;
  double min_hold_time_s_;

  // RMF 顶点（原始 mm 单位或大数）被解析并保存在 rmf_vertices_mm_
  // each entry: [x_mm, y_mm, z_mm?]
  std::vector<std::array<double,3>> rmf_vertices_mm_;

  // 生成的 ENU waypoints (meters): each [x_enu, y_enu, z_enu]
  std::vector<std::array<double,3>> enu_waypoints_;

  // PX4 publishers
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;

  // timer / state
  rclcpp::TimerBase::SharedPtr timer_;
  FlightPhase flight_phase_;
  int offboard_counter_;
  size_t waypoint_idx_;
  int hold_ticks_remaining_;
  bool landed_disarmed_;

  // =========== 辅助函数 ===========

  void publish_vehicle_command(uint16_t cmd, float p1=0.0f, float p2=0.0f)
  {
    VehicleCommand m;
    m.command = cmd;
    m.param1 = p1;
    m.param2 = p2;
    m.target_system = 1;
    m.target_component = 1;
    m.source_system = 1;
    m.source_component = 1;
    m.from_external = true;
    m.timestamp = now().nanoseconds() / 1000;
    vehicle_command_pub_->publish(m);
  }

  void publish_offboard_control_mode()
  {
    OffboardControlMode m;
    m.position = true;
    m.velocity = false;
    m.acceleration = false;
    m.attitude = false;
    m.body_rate = false;
    m.timestamp = now().nanoseconds() / 1000;
    offboard_control_mode_pub_->publish(m);
  }

  // ENU -> PX4 NED relative to takeoff origin, and publish TrajectorySetpoint
  void publish_setpoint_enu(double x_enu, double y_enu, double z_enu)
  {
    // offset relative to takeoff origin (ENU)
    double dx = x_enu - takeoff_origin_x_m_;
    double dy = y_enu - takeoff_origin_y_m_;
    // PX4 NED mapping:
    //   px4_north = ENU y
    //   px4_east  = ENU x
    //   px4_down  = -ENU z
    double px4_north = dy;
    double px4_east  = dx;
    double px4_down  = -z_enu; // flight_height_m_ positive -> px4_down negative

    TrajectorySetpoint m;
    m.position[0] = static_cast<float>(px4_north);
    m.position[1] = static_cast<float>(px4_east);
    m.position[2] = static_cast<float>(px4_down);

    m.velocity[0] = NAN;
    m.velocity[1] = NAN;
    m.velocity[2] = NAN;
    m.yaw = NAN;
    m.yawspeed = NAN;

    m.timestamp = now().nanoseconds()/1000;
    trajectory_setpoint_pub_->publish(m);
  }

  // main loop 10Hz
  void main_loop()
  {
    // always publish OffboardControlMode continuously
    publish_offboard_control_mode();

    // Warmup: keep publishing hover setpoint at takeoff origin until required_setpoints_ reached
    if (offboard_counter_ < required_setpoints_) {
      publish_setpoint_enu(takeoff_origin_x_m_, takeoff_origin_y_m_, flight_height_m_);
      offboard_counter_++;
      if (offboard_counter_ == required_setpoints_) {
        // request offboard and arm
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(get_logger(), "Requested OFFBOARD & ARM.");
      }
      return;
    }

    // After warmup, handle phases
    switch (flight_phase_) {
      case FlightPhase::WARMUP:
        // after we already warmed up, transition to TAKEOFF_HOLD
        flight_phase_ = FlightPhase::TAKEOFF_HOLD;
        RCLCPP_INFO(get_logger(), "Warmup done -> TAKEOFF_HOLD");
        return;

      case FlightPhase::TAKEOFF_HOLD:
        publish_setpoint_enu(takeoff_origin_x_m_, takeoff_origin_y_m_, flight_height_m_);
        // hold a few seconds for stable hover
        static int hold_ticks = 0;
        hold_ticks++;
        if (hold_ticks >= static_cast<int>(std::round(2.5 * 10.0))) { // 2.5s
          hold_ticks = 0;
          flight_phase_ = FlightPhase::ZIGZAG_CRUISE;
          waypoint_idx_ = 0;
          hold_ticks_remaining_ = 0;
          RCLCPP_INFO(get_logger(), "Takeoff hold done -> ZIGZAG_CRUISE (waypoints: %zu)", enu_waypoints_.size());
        }
        return;

      case FlightPhase::ZIGZAG_CRUISE:
        if (enu_waypoints_.empty()) {
          RCLCPP_WARN(get_logger(), "No waypoints -> go return home");
          flight_phase_ = FlightPhase::RETURN_HOME;
          return;
        }

        // If still holding on current waypoint
        if (hold_ticks_remaining_ > 0) {
          auto &wp = enu_waypoints_[waypoint_idx_];
          publish_setpoint_enu(wp[0], wp[1], wp[2]);
          hold_ticks_remaining_--;
          return;
        }

        // publish current waypoint as setpoint
        {
          auto &wp = enu_waypoints_[waypoint_idx_];
          publish_setpoint_enu(wp[0], wp[1], wp[2]);

          // estimate hold duration toward next waypoint (distance / speed), ensure min hold
          size_t next_idx = (waypoint_idx_ + 1) % enu_waypoints_.size();
          double dx = enu_waypoints_[next_idx][0] - wp[0];
          double dy = enu_waypoints_[next_idx][1] - wp[1];
          double dist = std::hypot(dx, dy);
          double tsec = std::max(min_hold_time_s_, dist / std::max(0.1, cruise_speed_m_s_));
          hold_ticks_remaining_ = static_cast<int>(std::round(tsec * 10.0)); // 10 Hz ticks

          // advance to next
          waypoint_idx_ = next_idx;
          return;
        }

      case FlightPhase::RETURN_HOME:
        publish_setpoint_enu(takeoff_origin_x_m_, takeoff_origin_y_m_, flight_height_m_);
        // hold some seconds then land
        static int ret_hold_ticks = 0;
        ret_hold_ticks++;
        if (ret_hold_ticks >= static_cast<int>(std::round(2.5 * 10.0))) {
          ret_hold_ticks = 0;
          flight_phase_ = FlightPhase::LAND;
          RCLCPP_INFO(get_logger(), "Return home done -> LAND");
        }
        return;

      case FlightPhase::LAND:
        {
          // approach ground by incrementally raising ENU z toward zero
          static double current_z = flight_height_m_;
          current_z -= 0.1; // 0.1 m per tick -> ~1 m/s
          if (current_z < 0.2) current_z = 0.2;
          publish_setpoint_enu(takeoff_origin_x_m_, takeoff_origin_y_m_, current_z);

          // send PX4 land command once
          static bool land_sent = false;
          if (!land_sent) {
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
            land_sent = true;
            RCLCPP_INFO(get_logger(), "Sent LAND command.");
          }

          if (current_z <= 0.25) {
            flight_phase_ = FlightPhase::LANDED;
            RCLCPP_INFO(get_logger(), "Landed (sim). Disarming soon.");
          }
        }
        return;

      case FlightPhase::LANDED:
        if (!landed_disarmed_) {
          publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
          landed_disarmed_ = true;
          RCLCPP_INFO(get_logger(), "Disarm command published.");
        }
        return;
    }
  }

  // =========== RMF vertices 读取（支持 [x,y,z,""] 格式） ===========
  void load_rmf_vertices()
  {
    rmf_vertices_mm_.clear();
  
    try {
      YAML::Node root = YAML::LoadFile(rmf_yaml_path_);
    
      if (!root["levels"]) {
        RCLCPP_WARN(get_logger(), "RMF YAML has no 'levels'");
        return;
      }
    
      YAML::Node levels = root["levels"];
      if (!levels || levels.size() == 0) {
        RCLCPP_WARN(get_logger(), "levels is empty");
        return;
      }
    
      // ✅ 取第一个楼层（L1）
      auto it = levels.begin();
      YAML::Node first_level = it->second;
    
      YAML::Node vertices_node;
    
      // ✅✅✅ 第一优先级：你当前 YAML 用的就是这个结构
      // levels:
      //   L1:
      //     vertices: [ [x,y,z,""], ... ]
      if (first_level["vertices"]) {
        vertices_node = first_level["vertices"];
        RCLCPP_INFO(get_logger(), "Loaded vertices from: levels -> L1 -> vertices");
      }
    
      // ✅ 第二兼容：少量 RMF 版本放在 floors[0].vertices
      else if (first_level["floors"]
            && first_level["floors"].IsSequence()
            && first_level["floors"].size() > 0
            && first_level["floors"][0]["vertices"]) {
        vertices_node = first_level["floors"][0]["vertices"];
        RCLCPP_INFO(get_logger(), "Loaded vertices from: levels -> L1 -> floors[0] -> vertices");
      }
    
      if (!vertices_node) {
        RCLCPP_WARN(get_logger(), "No vertices node found in RMF YAML");
        return;
      }
    
      // ✅✅✅ 解析你现在用的标准格式： [x, y, z, ""]
      if (vertices_node.IsSequence()) {
        for (auto v : vertices_node) {
          if (v.IsSequence() && v.size() >= 2) {
            double x = v[0].as<double>();
            double y = v[1].as<double>();
            double z = (v.size() >= 3) ? v[2].as<double>() : 0.0;
            rmf_vertices_mm_.push_back({x, y, z});
          }
        }
      }
    
      RCLCPP_INFO(
        get_logger(),
        "Loaded %zu RMF vertices (raw units)",
        rmf_vertices_mm_.size()
      );
    
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(get_logger(), "YAML parse error: %s", e.what());
    }
  }


  // =========== 由 rmf_vertices_mm_ -> enu_waypoints_ (m) 使用 bounding-box + zigzag rows ===========
  void generate_zigzag_waypoints()
  {
    enu_waypoints_.clear();
    if (rmf_vertices_mm_.empty()) {
      RCLCPP_WARN(get_logger(), "No rmf vertices -> no waypoints.");
      return;
    }

    // find bbox in RMF units (assume mm-like or large units) -> we convert to meters by /1000
    double min_x = rmf_vertices_mm_[0][0], max_x = min_x;
    double min_y = rmf_vertices_mm_[0][1], max_y = min_y;
    for (auto &v : rmf_vertices_mm_) {
      min_x = std::min(min_x, v[0]);
      max_x = std::max(max_x, v[0]);
      min_y = std::min(min_y, v[1]);
      max_y = std::max(max_y, v[1]);
    }

    // convert bbox mm -> meters
    double xmin_m = min_x/100;
    double xmax_m = max_x/100;
    double ymin_m = min_y/100;
    double ymax_m = max_y/100;

    double width_m = xmax_m - xmin_m;
    double height_m = ymax_m - ymin_m;
    if (width_m <= 0 || height_m <= 0) {
      RCLCPP_WARN(get_logger(), "Degenerate bbox -> no waypoints.");
      return;
    }

    // compute number of rows
    int rows = std::max(1, static_cast<int>(std::ceil(height_m / spacing_m_)));
    bool reverse = false;

    for (int r = 0; r <= rows; ++r) {
      double y_m = ymin_m + r * spacing_m_;
      if (y_m > ymax_m) y_m = ymax_m;
      // RMF -> Gazebo transform:
      //   gazebo_x = origin_x_m_ + x_m
      //   gazebo_y = origin_y_m_ - y_m   (Y flip)
      double gx1 = origin_x_m_ + xmin_m;
      double gx2 = origin_x_m_ + xmax_m;
      double gy  = origin_y_m_ - y_m;

      if (!reverse) {
        enu_waypoints_.push_back({gx1, gy, flight_height_m_});
        enu_waypoints_.push_back({gx2, gy, flight_height_m_});
      } else {
        enu_waypoints_.push_back({gx2, gy, flight_height_m_});
        enu_waypoints_.push_back({gx1, gy, flight_height_m_});
      }
      reverse = !reverse;
    }

    RCLCPP_INFO(get_logger(), "Generated zigzag: %zu waypoints (bbox %.2fm x %.2fm, rows %d)",
                enu_waypoints_.size(), width_m, height_m, rows+1);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ZigzagRMFFull>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
