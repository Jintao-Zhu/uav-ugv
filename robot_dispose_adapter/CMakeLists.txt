cmake_minimum_required(VERSION 3.8)
project(robot_dispose_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_task_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)


#  编译task_monitor_node节点（关键：统一用ament_target_dependencies，移除手动target_link_libraries）
add_executable(task_monitor_node src/task_monitor.cpp)
ament_target_dependencies(task_monitor_node
  rclcpp
  rmf_fleet_msgs
  rmf_task_msgs
  std_msgs
  geometry_msgs
  gazebo_msgs
  nlohmann_json  # 所有依赖统一通过ament_target_dependencies声明
)

# 安装可执行文件
install(TARGETS
  task_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
