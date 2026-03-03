/**
 * @file dock_pose_publisher.cpp
 * @brief C++ replacement for dock_pose_publisher.py
 *
 * Looks up the AprilTag TF transform and publishes it as a PoseStamped
 * on /detected_dock_pose for the opennav_docking server.
 *
 * Parameters:
 *   dock_tag_frame (string, default: "tag25h9:0") - AprilTag TF frame
 *   base_frame     (string, default: "odom")       - Reference frame
 *   publish_rate   (double, default: 10.0)         - Hz
 */

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class DockPosePublisher : public rclcpp::Node
{
public:
  DockPosePublisher()
  : Node("dock_pose_publisher")
  {
    this->declare_parameter("dock_tag_frame", "tag25h9:0");
    this->declare_parameter("base_frame", "odom");
    this->declare_parameter("publish_rate", 10.0);

    tag_frame_   = this->get_parameter("dock_tag_frame").as_string();
    base_frame_  = this->get_parameter("base_frame").as_string();
    double rate  = this->get_parameter("publish_rate").as_double();

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/detected_dock_pose", 10);

    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate));
    timer_ = this->create_wall_timer(
      period, std::bind(&DockPosePublisher::tick, this));

    RCLCPP_INFO(this->get_logger(),
      "Dock pose publisher started: %s -> %s @ %.1f Hz",
      base_frame_.c_str(), tag_frame_.c_str(), rate);
  }

private:
  void tick()
  {
    try {
      auto tf = tf_buffer_->lookupTransform(
        base_frame_, tag_frame_, tf2::TimePointZero);

      // Update cached pose on successful detection
      last_pose_.header.frame_id  = base_frame_;
      last_pose_.pose.position.x  = tf.transform.translation.x;
      last_pose_.pose.position.y  = tf.transform.translation.y;
      last_pose_.pose.position.z  = tf.transform.translation.z;
      last_pose_.pose.orientation = tf.transform.rotation;
      has_pose_ = true;
    } catch (const tf2::TransformException & ex) {
      // Tag left FOV during close approach — keep publishing last known pose
      RCLCPP_DEBUG(this->get_logger(), "TF lookup failed: %s", ex.what());
    }

    // Always publish last known pose with current timestamp (mirrors Python behaviour)
    if (has_pose_) {
      last_pose_.header.stamp = this->get_clock()->now();
      pub_->publish(last_pose_);
    }
  }

  std::string tag_frame_;
  std::string base_frame_;
  bool        has_pose_{false};

  geometry_msgs::msg::PoseStamped                          last_pose_;
  std::unique_ptr<tf2_ros::Buffer>                         tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>              tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr                             timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
