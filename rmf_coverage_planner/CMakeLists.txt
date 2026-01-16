cmake_minimum_required(VERSION 3.8)
project(rmf_coverage_planner)

set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(/home/suda/CoveragePlanner CoveragePlanner)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

# =============== node 1 =====================
add_executable(rmf_yaml_vertices_to_pwh
  src/rmf_yaml_vertices_to_pwh.cpp
)

target_include_directories(rmf_yaml_vertices_to_pwh PRIVATE
  /home/suda/CoveragePlanner/include
)

target_link_libraries(rmf_yaml_vertices_to_pwh
  yaml-cpp
  coverage_planner
)

ament_target_dependencies(rmf_yaml_vertices_to_pwh
  rclcpp
  nav_msgs
  geometry_msgs
)

# =============== node 2 ==================
add_executable(rmf_cell_to_sweep
  src/rmf_cell_to_sweep.cpp
)

target_include_directories(rmf_cell_to_sweep PRIVATE
  /home/suda/CoveragePlanner/include
)

target_link_libraries(rmf_cell_to_sweep
  coverage_planner
)

ament_target_dependencies(rmf_cell_to_sweep
  rclcpp
  nav_msgs
  geometry_msgs
)

install(TARGETS 
  rmf_yaml_vertices_to_pwh
  rmf_cell_to_sweep
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
