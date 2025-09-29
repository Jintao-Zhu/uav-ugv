cmake_minimum_required(VERSION 3.8)
project(ugv_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

#new
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

#new
add_executable(obstacle_avoidance_node src/obstacle_avoidance_node.cpp)
ament_target_dependencies(obstacle_avoidance_node 
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  std_msgs
  visualization_msgs
)

add_executable(move_forward_control_node src/move_forward_control_node.cpp)
ament_target_dependencies(move_forward_control_node rclcpp geometry_msgs)

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
  obstacle_avoidance_node
  move_forward_control_node
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
