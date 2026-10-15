cmake_minimum_required(VERSION 3.16)
project(coverage_geometry)
find_package(ament_cmake REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
add_library(ordered_goal_checker SHARED src/ordered_goal_checker.cpp)
target_compile_features(ordered_goal_checker PRIVATE cxx_std_20)
target_link_libraries(ordered_goal_checker
	${nav2_controller_TARGETS}
	nav2_ros_common::nav2_ros_common
	${nav_msgs_TARGETS}
	pluginlib::pluginlib
	tf2_geometry_msgs::tf2_geometry_msgs)
pluginlib_export_plugin_description_file(nav2_core plugins.xml)
install(TARGETS ordered_goal_checker LIBRARY DESTINATION lib)
ament_package()