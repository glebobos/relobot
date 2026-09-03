#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "robot_pose_publisher/robot_pose_publisher.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_pose_publisher::RobotPosePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
