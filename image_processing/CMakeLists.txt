cmake_minimum_required(VERSION 3.8)
project(image_processing)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --------------------------------------------------------------------------
# 修复 Anaconda / 系统库冲突
# --------------------------------------------------------------------------
set(CMAKE_INSTALL_RPATH "/usr/lib/x86_64-linux-gnu")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------------------
# 1. 依赖包
# --------------------------------------------------------------------------
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
find_package(visualization_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(TIFF REQUIRED)

# --------------------------------------------------------------------------
# 2. 自定义消息生成
# --------------------------------------------------------------------------
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/RedCubeDetection.msg"
)

# 生成接口并声明依赖
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

# 导出运行时依赖（关键！否则 ros2 topic echo 找不到 Python 类型支持）
ament_export_dependencies(rosidl_default_runtime)

# --------------------------------------------------------------------------
# 3. ONNX Runtime 路径
# --------------------------------------------------------------------------
include_directories(/usr/local/onnxruntime/include)
link_directories(/usr/local/onnxruntime/lib)

# --------------------------------------------------------------------------
# 4. 通用依赖
# --------------------------------------------------------------------------
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
  visualization_msgs
  CURL
  TIFF
)

# --------------------------------------------------------------------------
# 5. 获取类型支持目标（代替旧的 rosidl_target_interfaces）
# --------------------------------------------------------------------------
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# --------------------------------------------------------------------------
# 6. 可执行文件定义
# --------------------------------------------------------------------------
add_executable(red_cube_capture_node src/red_cube_capture_node.cpp)
add_executable(red_cube_detector_node src/red_cube_detector_node.cpp)
add_executable(image_capture_node src/image_capture_node.cpp)
add_executable(red_cube_tracker_node src/red_cube_tracker_node.cpp)

# --------------------------------------------------------------------------
# 7. 为各节点添加依赖和链接
# --------------------------------------------------------------------------
ament_target_dependencies(red_cube_capture_node ${COMMON_DEPENDENCIES})
target_link_libraries(red_cube_capture_node
  ${OpenCV_LIBRARIES}
  onnxruntime
  ${CURL_LIBRARIES}
  ${TIFF_LIBRARIES}
  "${cpp_typesupport_target}"
)
target_include_directories(red_cube_capture_node PRIVATE ${EIGEN3_INCLUDE_DIR})

ament_target_dependencies(red_cube_detector_node ${COMMON_DEPENDENCIES})
target_link_libraries(red_cube_detector_node
  ${OpenCV_LIBRARIES}
  onnxruntime
  ${CURL_LIBRARIES}
  ${TIFF_LIBRARIES}
  "${cpp_typesupport_target}"
)
target_include_directories(red_cube_detector_node PRIVATE ${EIGEN3_INCLUDE_DIR})

ament_target_dependencies(image_capture_node ${COMMON_DEPENDENCIES})
target_link_libraries(image_capture_node
  ${OpenCV_LIBRARIES}
  ${CURL_LIBRARIES}
  ${TIFF_LIBRARIES}
  "${cpp_typesupport_target}"
)


ament_target_dependencies(red_cube_tracker_node ${COMMON_DEPENDENCIES})
target_link_libraries(red_cube_tracker_node
  ${OpenCV_LIBRARIES}
  "${cpp_typesupport_target}"
)
target_include_directories(red_cube_tracker_node PRIVATE ${EIGEN3_INCLUDE_DIR})

# --------------------------------------------------------------------------
# 8. 安装节点和模型
# --------------------------------------------------------------------------
install(TARGETS
  red_cube_capture_node
  red_cube_detector_node
  image_capture_node
  red_cube_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# --------------------------------------------------------------------------
# 9. 确保 Python 类型支持安装（关键！）
# --------------------------------------------------------------------------
# 这一步让 rosidl_default_generators 输出的 Python 模块能随包一起被 install。
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}
  DESTINATION lib/python3.10/site-packages
)

# --------------------------------------------------------------------------
# 10. ament_package
# --------------------------------------------------------------------------
ament_package()
