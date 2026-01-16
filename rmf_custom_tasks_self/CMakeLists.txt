cmake_minimum_required(VERSION 3.8)
# ========== 核心修改1：项目名改为rmf_custom_tasks_self ==========
project(rmf_custom_tasks_self)

# ========== 手动指定 NumPy 头文件路径（解决编译找不到的问题） ==========
# 替换成你查到的 NumPy 头文件路径
set(NUMPY_INCLUDE_DIR "/usr/local/lib/python3.10/dist-packages/numpy/core/include")
# 打印路径（方便调试，确认路径正确）
message(STATUS "NumPy include directory: ${NUMPY_INCLUDE_DIR}")
# 将路径添加到编译包含目录
include_directories(${NUMPY_INCLUDE_DIR})
# ========== 配置结束 ==========

# 消除CMake警告（可选）
cmake_policy(SET CMP0148 OLD)
add_compile_options(-Wno-dev)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 仅保留核心依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_task_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nlohmann_json REQUIRED)
# 全局队列需要的依赖
find_package(std_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 生成自定义服务（SingleNavTask.srv）
# ========== 此处PROJECT_NAME已自动变为rmf_custom_tasks_self，无需改 ==========
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SingleNavTask.srv"
  DEPENDENCIES rmf_task_msgs
)

# ==================== 兼容Humble接口链接方式 ====================
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 1. 编译原自定义任务节点（确认存在：single_point_task_publisher.cpp）
add_executable(single_point_task_publisher 
  src/single_point_task_publisher.cpp
)
ament_target_dependencies(single_point_task_publisher
  rclcpp
  rmf_task_msgs
  nlohmann_json
  std_msgs
)
# 替换废弃的rosidl_target_interfaces
target_link_libraries(single_point_task_publisher "${cpp_typesupport_target}")

# # 2. 编译全局队列节点（确认存在：global_task_queue.cpp）
# add_executable(global_task_queue_node
#   src/global_task_queue.cpp
# )
# ament_target_dependencies(global_task_queue_node
#   rclcpp
#   std_msgs
#   rmf_fleet_msgs
#   geometry_msgs
#   rmf_task_msgs
# )
# 链接自定义服务接口
# target_link_libraries(global_task_queue_node "${cpp_typesupport_target}")

# 仅安装两个核心可执行文件
# ========== 安装路径自动使用新PROJECT_NAME，无需改 ==========
install(TARGETS
  single_point_task_publisher
  #global_task_queue_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

