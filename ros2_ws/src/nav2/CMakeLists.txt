cmake_minimum_required(VERSION 3.8)
project(nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Install Python modules
ament_python_install_package(frontier_explorer)
ament_python_install_package(docking)

# Build C++ dock_pose_publisher node
add_executable(dock_pose_publisher src/dock_pose_publisher.cpp)
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

# Install launch files, config, behavior trees
install(DIRECTORY
  launch
  config
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# Install python executable scripts into lib/nav2
install(PROGRAMS
  frontier_explorer/coverage_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME coverage_manager
)

install(PROGRAMS
  frontier_explorer/coverage_preview_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME coverage_preview_test
)

install(PROGRAMS
  docking/apriltag_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME apriltag_manager
)

ament_package()
