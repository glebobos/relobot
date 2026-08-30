#include "robot_pose_publisher/robot_pose_publisher.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robot_pose_publisher
{

RobotPosePublisher::RobotPosePublisher(const rclcpp::NodeOptions & options)
: Node("robot_pose_publisher", options)
{
  map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  publish_topic_ = this->declare_parameter<std::string>("publish_topic", "/robot_pose");
  publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 30.0);
  max_tf_age_sec_ = this->declare_parameter<double>("max_tf_age_sec", 1.0);

  if (map_frame_.empty() || base_frame_.empty() || publish_topic_.empty()) {
    RCLCPP_ERROR(get_logger(), "Parameters 'map_frame', 'base_frame', and 'publish_topic' must not be empty.");
    throw std::invalid_argument("Empty parameter");
  }
  if (publish_rate_hz_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "Parameter 'publish_rate_hz' must be > 0.");
    throw std::invalid_argument("Invalid publish_rate_hz");
  }
  if (max_tf_age_sec_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "Parameter 'max_tf_age_sec' must be > 0.");
    throw std::invalid_argument("Invalid max_tf_age_sec");
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(publish_topic_, qos);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&RobotPosePublisher::tick, this));

  RCLCPP_INFO(
    get_logger(),
    "RobotPosePublisher active (%s -> %s @ %.1f Hz -> %s, max_tf_age=%.2fs)",
    map_frame_.c_str(),
    base_frame_.c_str(),
    publish_rate_hz_,
    publish_topic_.c_str(),
    max_tf_age_sec_);
}

void RobotPosePublisher::tick()
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      map_frame_,
      base_frame_,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Could not look up transform from %s to %s: %s",
      map_frame_.c_str(),
      base_frame_.c_str(),
      ex.what());
    return;
  }

  rclcpp::Time stamp(transform.header.stamp);
  double age_sec = (this->now() - stamp).seconds();
  if (age_sec > max_tf_age_sec_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Transform from %s to %s is %.2fs old (> %.2fs threshold); skipping publish.",
      map_frame_.c_str(),
      base_frame_.c_str(),
      age_sec,
      max_tf_age_sec_);
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = transform.header.stamp;
  pose.header.frame_id = map_frame_;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;

  pose_pub_->publish(pose);
}

}  // namespace robot_pose_publisher

RCLCPP_COMPONENTS_REGISTER_NODE(robot_pose_publisher::RobotPosePublisher)
