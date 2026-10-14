#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace relobot
{
class OrderedGoalChecker : public nav2_controller::SimpleGoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    SimpleGoalChecker::initialize(parent, name, costmap);
    costmap_ = costmap;
    auto node = parent.lock();
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".path_topic", rclcpp::ParameterValue(std::string("/coverage/execution_path")));
    std::string path_topic;
    node->get_parameter(name + ".path_topic", path_topic);
    subscription_ = parent.lock()->create_subscription<nav_msgs::msg::Path>(
      path_topic, rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::Path::ConstSharedPtr path) {
        std::lock_guard<std::mutex> lock(mutex_);
        path_ = *path;
        cumulative_.assign(path_.poses.size(), 0.0);
        for (size_t index = 1; index < path_.poses.size(); ++index) {
          const auto & start = path_.poses[index - 1].pose.position;
          const auto & end = path_.poses[index].pose.position;
          cumulative_[index] = cumulative_[index - 1] + std::hypot(end.x - start.x, end.y - start.y);
        }
        progress_ = 0.0;
        cursor_ = 0;
      });
  }

  void reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    SimpleGoalChecker::reset();
    progress_ = 0.0;
    cursor_ = 0;
  }

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query, const geometry_msgs::msg::Pose & goal,
    const geometry_msgs::msg::Twist & velocity) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (path_.poses.size() < 2 || path_.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        costmap_->get_logger(), *costmap_->get_clock(), 2000,
        "Coverage goal rejected: execution path unavailable (poses=%zu, frame='%s')",
        path_.poses.size(), path_.header.frame_id.c_str());
      return false;
    }
    geometry_msgs::msg::PoseStamped robot, endpoint;
    robot.header.frame_id = endpoint.header.frame_id = costmap_->getGlobalFrameID();
    robot.pose = query;
    endpoint.pose = goal;
    try {
      robot = costmap_->getTfBuffer()->transform(robot, path_.header.frame_id);
      endpoint = costmap_->getTfBuffer()->transform(endpoint, path_.header.frame_id);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        costmap_->get_logger(), *costmap_->get_clock(), 2000,
        "Coverage goal rejected: transform to '%s' failed: %s",
        path_.header.frame_id.c_str(), exception.what());
      return false;
    }
    const auto & target = path_.poses.back().pose.position;
    const double endpoint_mismatch = std::hypot(
      endpoint.pose.position.x - target.x, endpoint.pose.position.y - target.y);
    if (endpoint_mismatch > 0.05) {
      RCLCPP_WARN_THROTTLE(
        costmap_->get_logger(), *costmap_->get_clock(), 2000,
        "Coverage goal rejected: action endpoint differs from work path by %.3f m (limit 0.050 m)",
        endpoint_mismatch);
      return false;
    }
    double closest = std::numeric_limits<double>::infinity();
    double best_progress = progress_;
    size_t best_index = cursor_;
    for (size_t index = cursor_ > 0 ? cursor_ - 1 : 0; index + 1 < path_.poses.size(); ++index) {
      if (cumulative_[index] > progress_ + 0.6) {break;}
      const auto & start = path_.poses[index].pose.position;
      const auto & end = path_.poses[index + 1].pose.position;
      const double length = cumulative_[index + 1] - cumulative_[index];
      if (length < 1e-6 || cumulative_[index + 1] < progress_ - 0.1) {continue;}
      const double minimum = std::max(0.0, (progress_ - 0.1 - cumulative_[index]) / length);
      const double maximum = std::min(1.0, (progress_ + 0.6 - cumulative_[index]) / length);
      const double fraction = std::clamp(
        ((robot.pose.position.x - start.x) * (end.x - start.x) +
        (robot.pose.position.y - start.y) * (end.y - start.y)) / (length * length), minimum, maximum);
      const double distance = std::hypot(robot.pose.position.x - start.x - fraction * (end.x - start.x),
          robot.pose.position.y - start.y - fraction * (end.y - start.y));
      if (distance < closest) {
        closest = distance;
        best_progress = cumulative_[index] + fraction * length;
        best_index = index;
      }
    }
    if (closest > 0.50) {
      RCLCPP_WARN_THROTTLE(
        costmap_->get_logger(), *costmap_->get_clock(), 2000,
        "Coverage goal rejected: ordered tracking distance %.3f m exceeds 0.500 m",
        closest);
      return false;
    }
    progress_ = std::max(progress_, best_progress);
    cursor_ = std::max(cursor_, best_index);
    const double remaining = cumulative_.back() - progress_;
    const bool reached = remaining <= std::max(0.12, xy_goal_tolerance_) &&
      SimpleGoalChecker::isGoalReached(query, goal, velocity);
    const double path_endpoint_distance = std::hypot(
      robot.pose.position.x - target.x, robot.pose.position.y - target.y);
    if (!reached && path_endpoint_distance <= 0.15) {
      RCLCPP_WARN_THROTTLE(
        costmap_->get_logger(), *costmap_->get_clock(), 2000,
        "Coverage endpoint pending: path distance %.3f m, action distance %.3f m, "
        "ordered remaining %.3f m, xy tolerance %.3f m",
        path_endpoint_distance,
        std::hypot(query.position.x - goal.position.x, query.position.y - goal.position.y),
        remaining, xy_goal_tolerance_);
    }
    return reached;
  }

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;
  nav_msgs::msg::Path path_;
  std::vector<double> cumulative_;
  double progress_{0.0};
  size_t cursor_{0};
  std::mutex mutex_;
};
}
PLUGINLIB_EXPORT_CLASS(relobot::OrderedGoalChecker, nav2_core::GoalChecker)