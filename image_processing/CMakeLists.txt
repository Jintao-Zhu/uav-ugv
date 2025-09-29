cmake_minimum_required(VERSION 3.8)
project(image_processing)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------- 1. 查找依赖包 --------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core dnn imgproc highgui)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)   # ✅ 补上

# 自定义消息
find_package(rosidl_default_generators REQUIRED)
set(msg_files
  "msg/RedCubeDetection.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

# -------------------------- 2. ONNX Runtime --------------------------
include_directories(/usr/local/onnxruntime/include)
link_directories(/usr/local/onnxruntime/lib)

# -------------------------- 3. 可执行文件 --------------------------
add_executable(red_cube_capture_node src/red_cube_capture_node.cpp)
add_executable(red_cube_detector_node src/red_cube_detector_node.cpp)
add_executable(image_capture_node src/image_capture_node.cpp)
add_executable(model_test src/model_test.cpp)

# -------------------------- 4. 通用依赖 --------------------------
set(COMMON_DEPENDENCIES
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  ament_index_cpp
  std_msgs
  geometry_msgs
  message_filters
  px4_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs   # ✅ 新增
)

# 获取消息支持
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# 逐个节点链接
ament_target_dependencies(red_cube_capture_node ${COMMON_DEPENDENCIES})
target_link_libraries(red_cube_capture_node ${OpenCV_LIBRARIES} onnxruntime "${cpp_typesupport_target}")
target_include_directories(red_cube_capture_node PRIVATE ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(red_cube_detector_node ${COMMON_DEPENDENCIES})
target_link_libraries(red_cube_detector_node ${OpenCV_LIBRARIES} onnxruntime "${cpp_typesupport_target}")
target_include_directories(red_cube_detector_node PRIVATE ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(image_capture_node ${COMMON_DEPENDENCIES})
target_link_libraries(image_capture_node ${OpenCV_LIBRARIES})

ament_target_dependencies(model_test ${COMMON_DEPENDENCIES})
target_link_libraries(model_test ${OpenCV_LIBRARIES} onnxruntime "${cpp_typesupport_target}")
target_include_directories(model_test PRIVATE ${EIGEN3_INCLUDE_DIRS})

# -------------------------- 5. 安装 --------------------------
install(TARGETS
  red_cube_capture_node
  red_cube_detector_node
  image_capture_node
  model_test
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
