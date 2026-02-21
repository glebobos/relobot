cmake_minimum_required(VERSION 3.18)
project(camera_ros)

find_package(ament_cmake REQUIRED)

# This package only installs custom launch files, configs, and scripts.
# The camera_ros node itself is built from upstream in /opt/camera_ws.

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/dock_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
