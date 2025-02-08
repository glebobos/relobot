cmake_minimum_required(VERSION 3.8)
project(diff_drive_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # Ensure this is included
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(controller_manager REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(asio_cmake_module REQUIRED)
find_package(io_context REQUIRED)
find_package(serial_driver REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Add serial library
find_library(SERIAL_LIBRARY serial)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/diff_drive_hardware.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

target_link_libraries(${PROJECT_NAME}
  ${SERIAL_LIBRARY}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ros2_control
  ros2_controllers
  controller_manager
  realtime_tools
  asio_cmake_module
  io_context
  serial_driver
)

pluginlib_export_plugin_description_file(hardware_interface diff_drive_hardware.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  ros2_control
  ros2_controllers
  controller_manager
  realtime_tools
  asio_cmake_module
  io_context
  serial_driver
)

ament_package()