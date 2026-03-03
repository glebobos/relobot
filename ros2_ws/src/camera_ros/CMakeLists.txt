cmake_minimum_required(VERSION 3.18)
project(camera_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# dock_pose_publisher: C++ node replacing the old Python script
add_executable(dock_pose_publisher
  src/dock_pose_publisher.cpp
)
ament_target_dependencies(dock_pose_publisher
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  dock_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# This package also installs custom launch files and configs.
# The camera_ros node itself is built from upstream in /opt/camera_ws.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
