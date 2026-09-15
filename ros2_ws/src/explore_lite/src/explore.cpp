/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>
#include <explore/costmap_tools.h>

#include <thread>

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two,
                              double tolerance = 0.75)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < tolerance;
}

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("start_paused", true);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  progress_timeout_ = timeout;
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // Publisher for exploration status
  rclcpp::QoS status_qos(10);
  status_qos.transient_local();
  status_pub_ = this->create_publisher<explore_lite_msgs::msg::ExploreStatus>("explore/status", status_qos);

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  bool start_paused;
  this->get_parameter("start_paused", start_paused);

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });

  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  if (start_paused) {
    exploring_timer_->cancel();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
    RCLCPP_INFO(logger_, "Exploration ready but paused. Publish true to explore/resume to start.");
  } else {
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
    makePlan();
  }
  status_pub_->publish(status_msg);
}

Explore::~Explore()
{
  stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::msg::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::msg::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::msg::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
#ifdef ELOQUENT
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#elif DASHING
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#else
  m.lifetime = rclcpp::Duration::from_seconds(0);  // foxy onwards
#endif
  // m.lifetime = rclcpp::Duration::from_nanoseconds(0); // suggested in
  // galactic
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    // m.pose.position = {}; // compile warning
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  if (is_navigating_) {
    // Actively navigating to a goal in Nav2. Monitor progress timeout watchdog only.
    auto pose = costmap_client_.getRobotPose();
    double dist_to_goal = std::hypot(current_target_.x - pose.position.x,
                                     current_target_.y - pose.position.y);
    if (prev_distance_ > dist_to_goal + 0.1) {
      last_progress_ = this->now();
      prev_distance_ = dist_to_goal;
    }
    if (this->now() - last_progress_ > tf2::durationFromSec(progress_timeout_)) {
      RCLCPP_WARN(logger_, "Frontier goal (%.2f, %.2f) timed out (no progress for %.0fs). Cancelling & blacklisting.",
                  current_target_.x, current_target_.y, progress_timeout_);
      frontier_blacklist_.push_back(current_target_);
      is_navigating_ = false;
      if (navigation_goal_handle_) {
        move_base_client_->async_cancel_goal(navigation_goal_handle_);
        navigation_goal_handle_ = nullptr;
      }
      makePlan();
    }
    return;
  }

  // Not navigating: Search for new frontiers
  auto pose = costmap_client_.getRobotPose();
  auto frontiers = search_.searchFrom(pose.position);
  RCLCPP_INFO(logger_, "Robot pose: (%.2f, %.2f), found %lu frontiers",
              pose.position.x, pose.position.y, frontiers.size());

  if (frontiers.empty()) {
    if (++no_frontier_retry_count_ < 5) {
      RCLCPP_INFO(logger_, "No frontiers in current snapshot, waiting for map updates (attempt %d/5)...", no_frontier_retry_count_);
      return;
    }
    RCLCPP_WARN(logger_, "No frontiers found after retries, stopping.");
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }

  // Publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  geometry_msgs::msg::Point target_position;
  bool found_valid_frontier = false;

  for (const auto& frontier : frontiers) {
    const auto& candidate = frontier.middle;
    if (goalOnBlacklist(candidate) || goalOnBlacklist(frontier.centroid)) {
      continue;
    }

    double dist_to_robot = std::hypot(candidate.x - pose.position.x,
                                      candidate.y - pose.position.y);
    if (dist_to_robot < 0.30) {
      // Robot is already standing at this frontier
      frontier_blacklist_.push_back(candidate);
      continue;
    }

    target_position = candidate;
    found_valid_frontier = true;
    break;
  }

  if (!found_valid_frontier) {
    if (++no_frontier_retry_count_ < 5) {
      RCLCPP_INFO(logger_, "All %lu detected frontiers blacklisted, waiting for map expansion (attempt %d/5)...",
                  frontiers.size(), no_frontier_retry_count_);
      return;
    }
    RCLCPP_WARN(logger_, "All %lu frontiers traversed or blacklisted, stopping.", frontiers.size());
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }

  no_frontier_retry_count_ = 0;

  // Mark navigating state BEFORE dispatching async goal
  is_navigating_ = true;
  current_target_ = target_position;
  last_progress_ = this->now();
  prev_distance_ = std::hypot(target_position.x - pose.position.x,
                              target_position.y - pose.position.y);

  RCLCPP_INFO(logger_, "Navigating to frontier at (%.2f, %.2f), distance: %.2fm",
              target_position.x, target_position.y, prev_distance_);

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;
  goal.pose.pose.orientation.w = 1.0;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
      [this, target_position](const NavigationGoalHandle::SharedPtr& goal_handle) {
        if (!goal_handle) {
          RCLCPP_WARN(logger_, "Goal to (%.2f, %.2f) was rejected by Nav2.",
                      target_position.x, target_position.y);
          frontier_blacklist_.push_back(target_position);
          is_navigating_ = false;
          navigation_goal_handle_ = nullptr;
          makePlan();
        } else {
          navigation_goal_handle_ = goal_handle;
        }
      };

  send_goal_options.result_callback =
      [this, target_position](const NavigationGoalHandle::WrappedResult& result) {
        is_navigating_ = false;
        navigation_goal_handle_ = nullptr;
        reachedGoal(result, target_position);
      };

  move_base_client_->async_send_goal(goal, send_goal_options);
}

bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  constexpr static double tolerance = 0.35; // 35 cm
  for (const auto& frontier_goal : frontier_blacklist_) {
    double dist = std::hypot(goal.x - frontier_goal.x, goal.y - frontier_goal.y);
    if (dist < tolerance)
      return true;
  }
  return false;
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal)
{
  is_navigating_ = false;
  navigation_goal_handle_ = nullptr;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger_, "Frontier goal (%.2f, %.2f) reached.", frontier_goal.x, frontier_goal.y);
      frontier_blacklist_.push_back(frontier_goal);
      makePlan();
      break;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(logger_, "Frontier goal (%.2f, %.2f) aborted by Nav2. Blacklisting.", frontier_goal.x, frontier_goal.y);
      frontier_blacklist_.push_back(frontier_goal);
      makePlan();
      break;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(logger_, "Frontier goal (%.2f, %.2f) canceled.", frontier_goal.x, frontier_goal.y);
      // Do not blacklist canceled goals and do not auto-restart
      break;

    default:
      RCLCPP_WARN(logger_, "Unknown result code from Nav2: %d", static_cast<int>(result.code));
      break;
  }
}

void Explore::start()
{
  frontier_blacklist_.clear();
  no_frontier_retry_count_ = 0;
  prev_distance_ = 0;
  last_progress_ = this->now();
  navigation_goal_handle_ = nullptr;
  is_navigating_ = false;
  RCLCPP_INFO(logger_, "Exploration started.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
  makePlan();
}

void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "Exploration stopped.");
  
  if (!finished_exploring) {
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
    status_pub_->publish(status_msg);
  }
  
  is_navigating_ = false;
  if (navigation_goal_handle_) {
    move_base_client_->async_cancel_goal(navigation_goal_handle_);
    navigation_goal_handle_ = nullptr;
  } else {
    move_base_client_->async_cancel_all_goals();
  }
  exploring_timer_->cancel();
}

void Explore::resume()
{
  frontier_blacklist_.clear();
  no_frontier_retry_count_ = 0;
  prev_distance_ = 0;
  last_progress_ = this->now();
  navigation_goal_handle_ = nullptr;
  is_navigating_ = false;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS;
  status_pub_->publish(status_msg);
  exploring_timer_->reset();
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
