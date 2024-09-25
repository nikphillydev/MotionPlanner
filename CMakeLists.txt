cmake_minimum_required(VERSION 3.8)
project(at_motion_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(at_messages REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_action
  tf2_ros
  tf2_geometry_msgs
  at_messages
  nav_msgs
  geometry_msgs
  Eigen3
)

# === Targets ===

add_executable(motion_planner_node src/motion_planner_node.cpp src/navigator.cpp src/utils.cpp)
ament_target_dependencies(motion_planner_node ${dependencies})

add_executable(test_server_node src/test_server_node.cpp src/utils.cpp)
ament_target_dependencies(test_server_node ${dependencies})

# === Install ===

install(TARGETS
  motion_planner_node
  test_server_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
