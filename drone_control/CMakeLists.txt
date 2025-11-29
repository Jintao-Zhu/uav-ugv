cmake_minimum_required(VERSION 3.8)
project(drone_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 手动指定 px4_msgs 的路径
set(px4_msgs_DIR /home/suda/drone_ugv_ws/install/px4_msgs/share/px4_msgs/cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_msgs REQUIRED) 
find_package(rclcpp_action REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(takeoff_node src/takeoff_node.cpp)
ament_target_dependencies(takeoff_node rclcpp px4_msgs)

add_executable(circle_node src/circle_node.cpp)
ament_target_dependencies(circle_node rclcpp px4_msgs)

add_executable(lower_circle_node src/lower_circle_node.cpp)
ament_target_dependencies(lower_circle_node rclcpp px4_msgs)

add_executable(forward_flight_node src/forward_flight_node.cpp)
ament_target_dependencies(forward_flight_node rclcpp px4_msgs)

add_executable(drone_goal_publisher src/drone_goal_publisher.cpp)
ament_target_dependencies(drone_goal_publisher
  rclcpp 
  rclcpp_action
  px4_msgs 
  geometry_msgs  # 这里添加 geometry_msgs 依赖
  tf2  # 新增：添加 tf2 依赖
  nav_msgs
  nav2_msgs
  tf2_geometry_msgs
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS 
  takeoff_node 
  forward_flight_node
  circle_node
  drone_goal_publisher
  lower_circle_node
DESTINATION lib/${PROJECT_NAME}
)


ament_package()
