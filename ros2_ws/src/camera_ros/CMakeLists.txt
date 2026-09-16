cmake_minimum_required(VERSION 3.18)
project(camera_ros)

find_package(ament_cmake REQUIRED)

# Install custom launch files and calibration
# The camera_ros node itself is built from upstream in /opt/camera_ws.
install(DIRECTORY
  launch
  calibration
  DESTINATION share/${PROJECT_NAME}/
)

# Python scripts (system_monitor)
install(PROGRAMS
  scripts/system_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME system_monitor
)

ament_package()
