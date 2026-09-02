#ifndef ROBOT_POSE_PUBLISHER__ROBOT_POSE_PUBLISHER_HPP_
#define ROBOT_POSE_PUBLISHER__ROBOT_POSE_PUBLISHER_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace robot_pose_publisher
{

class RobotPosePublisher : public rclcpp::Node
{
public:
  explicit RobotPosePublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~RobotPosePublisher() = default;

private:
  void tick();

  std::string map_frame_;
  std::string base_frame_;
  std::string publish_topic_;
  double publish_rate_hz_;
  double max_tf_age_sec_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_pose_publisher

#endif  // ROBOT_POSE_PUBLISHER__ROBOT_POSE_PUBLISHER_HPP_
