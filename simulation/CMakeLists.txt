cmake_minimum_required(VERSION 3.8)
project(simulation)

# 避免在某些情况下跟随符号链接引起的 GLOB 警告
if(POLICY CMP0009)
  cmake_policy(SET CMP0009 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ✅ 编译 C++ 节点
add_executable(pose_to_tf src/pose_to_tf.cpp)
ament_target_dependencies(pose_to_tf rclcpp geometry_msgs tf2_ros)

add_executable(sub_test src/sub_test.cpp)
ament_target_dependencies(sub_test rclcpp geometry_msgs tf2_ros)

# 安装可执行文件
install(TARGETS
  pose_to_tf
  sub_test
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装 worlds 文件
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# 安装 models 文件，排除异常文件
install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
  PATTERN "* copy.*" EXCLUDE
  PATTERN "iris_copy.sdf" EXCLUDE
)

# 安装 config 和 maps 文件
install(DIRECTORY
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
