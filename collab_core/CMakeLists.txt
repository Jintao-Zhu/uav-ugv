cmake_minimum_required(VERSION 3.8)
project(collab_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 手动指定 px4_msgs 的路径
set(px4_msgs_DIR /home/suda/drone_ugv_ws/install/px4_msgs/share/px4_msgs/cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(coordination_node src/coordination_node.cpp)
ament_target_dependencies(coordination_node rclcpp geometry_msgs px4_msgs nav_msgs tf2 tf2_geometry_msgs tf2_ros)


add_executable(yahboomcar_coordination_node src/yahboomcar_coordination_node.cpp)  
ament_target_dependencies(yahboomcar_coordination_node
  rclcpp
  geometry_msgs
  nav_msgs
  px4_msgs        # 无人机位置消息依赖
  tf2
  sensor_msgs
  tf2_geometry_msgs
  std_msgs
)

add_executable(navigation_controller src/navigation_controller.cpp)
ament_target_dependencies(navigation_controller
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(navigation_to_red_cube src/navigation_to_red_cube.cpp)
ament_target_dependencies(navigation_to_red_cube
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS 
  coordination_node 
  yahboomcar_coordination_node
  navigation_controller
  navigation_to_red_cube
DESTINATION lib/${PROJECT_NAME}
)


ament_package()