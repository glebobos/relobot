cmake_minimum_required(VERSION 3.8)
project(robot_pose_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# Shared library / Composable Node Component
add_library(robot_pose_publisher_component SHARED
  src/robot_pose_publisher.cpp
)
target_include_directories(robot_pose_publisher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robot_pose_publisher_component
  rclcpp
  rclcpp_components
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)
rclcpp_components_register_nodes(robot_pose_publisher_component
  "robot_pose_publisher::RobotPosePublisher"
)

# Standalone Executable
add_executable(robot_pose_publisher_node
  src/main.cpp
)
target_link_libraries(robot_pose_publisher_node
  robot_pose_publisher_component
)
ament_target_dependencies(robot_pose_publisher_node
  rclcpp
  rclcpp_components
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  robot_pose_publisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  robot_pose_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(robot_pose_publisher_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

ament_package()
