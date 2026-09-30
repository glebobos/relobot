#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/optimizer_settings.hpp"
#include "tf2/utils.h"

namespace mppi
{
inline void validateForwardSequence(
  const models::ControlSequence & sequence, const models::OptimizerSettings & settings,
  const geometry_msgs::msg::PoseStamped & pose,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  auto * costmap = costmap_ros->getCostmap();
  const auto footprint = costmap_ros->getRobotFootprint();
  const double resolution = costmap->getResolution();
  double body_radius = 0.0;
  for (const auto & corner : footprint) {
    body_radius = std::max(body_radius, std::hypot(corner.x, corner.y));
  }
  double position_x = pose.pose.position.x;
  double position_y = pose.pose.position.y;
  double yaw = tf2::getYaw(pose.pose.orientation);
  const auto check_pose = [&]() {
      std::vector<geometry_msgs::msg::Point> polygon;
      nav2_costmap_2d::transformFootprint(position_x, position_y, yaw, footprint, polygon);
      unsigned int minimum_x = costmap->getSizeInCellsX(), minimum_y = costmap->getSizeInCellsY();
      unsigned int maximum_x = 0, maximum_y = 0;
      for (const auto & corner : polygon) {
        unsigned int cell_x, cell_y;
        if (!costmap->worldToMap(corner.x, corner.y, cell_x, cell_y)) {
          throw std::runtime_error("Constrained MPPI trajectory leaves the local map");
        }
        minimum_x = std::min(minimum_x, cell_x);
        maximum_x = std::max(maximum_x, cell_x);
        minimum_y = std::min(minimum_y, cell_y);
        maximum_y = std::max(maximum_y, cell_y);
      }
      for (unsigned int cell_y = minimum_y; cell_y <= maximum_y; ++cell_y) {
        for (unsigned int cell_x = minimum_x; cell_x <= maximum_x; ++cell_x) {
          if (costmap->getCost(cell_x, cell_y) < nav2_costmap_2d::LETHAL_OBSTACLE) {continue;}
          double world_x, world_y;
          costmap->mapToWorld(cell_x, cell_y, world_x, world_y);
          bool positive = false, negative = false;
          for (size_t index = 0; index < polygon.size(); ++index) {
            const auto & start = polygon[index];
            const auto & end = polygon[(index + 1) % polygon.size()];
            const double cross = (end.x - start.x) * (world_y - start.y) -
              (end.y - start.y) * (world_x - start.x);
            const double padding = resolution * std::sqrt(0.5) *
              std::hypot(end.x - start.x, end.y - start.y);
            positive = positive || cross > padding;
            negative = negative || cross < -padding;
          }
          if (!(positive && negative)) {
            throw std::runtime_error("Constrained MPPI trajectory intersects an obstacle or unknown cell");
          }
        }
      }
    };
  check_pose();
  for (size_t index = settings.shift_control_sequence ? 1 : 0;
    index < sequence.vx.size(); ++index)
  {
    const double linear = sequence.vx(index), angular = sequence.wz(index);
    if (!std::isfinite(linear) || !std::isfinite(angular) || linear < 0.0 ||
      linear > settings.constraints.vx_max + 1e-6 || std::abs(sequence.vy(index)) > 1e-6)
    {
      throw std::runtime_error("Non-finite or invalid forward MPPI control sequence");
    }
    const unsigned int steps = std::max(1.0, std::ceil(
        (linear + body_radius * std::abs(angular)) * settings.model_dt / (resolution * 0.25)));
    const double duration = settings.model_dt / steps;
    for (unsigned int step = 0; step < steps; ++step) {
      const double next_yaw = yaw + angular * duration;
      if (std::abs(angular) > 1e-8) {
        position_x += linear / angular * (std::sin(next_yaw) - std::sin(yaw));
        position_y += linear / angular * (std::cos(yaw) - std::cos(next_yaw));
      } else {
        position_x += linear * duration * std::cos(yaw);
        position_y += linear * duration * std::sin(yaw);
      }
      yaw = next_yaw;
      check_pose();
    }
  }
}
}