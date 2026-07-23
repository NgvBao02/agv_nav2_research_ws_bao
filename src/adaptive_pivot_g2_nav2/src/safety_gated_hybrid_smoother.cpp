// Copyright 2026 Adaptive Pivot-G2 Research Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "adaptive_pivot_g2_nav2/safety_gated_hybrid_smoother.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "adaptive_pivot_g2/hybrid_selection.hpp"
#include "nav2_core/smoother_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace adaptive_pivot_g2_nav2
{
namespace
{

using Costmap = nav2_costmap_2d::Costmap2D;
using CollisionChecker =
  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<Costmap>>;

constexpr double kEpsilon = 1.0e-10;

template<typename T>
T declare_and_get(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,
  const T & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
}

double normalized_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

struct PathEvaluation
{
  bool safe{false};
  double maximum_proximity_cost{std::numeric_limits<double>::infinity()};
  double curvature_energy{std::numeric_limits<double>::infinity()};
  std::size_t pivot_count{0};
};

void append_json_number(std::ostringstream & stream, double value)
{
  if (std::isfinite(value)) {
    stream << value;
  } else {
    stream << "null";
  }
}

bool evaluate_pose(
  double x,
  double y,
  double yaw,
  const std::shared_ptr<Costmap> & costmap,
  CollisionChecker & checker,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double & maximum_proximity_cost)
{
  unsigned int map_x = 0;
  unsigned int map_y = 0;
  if (!costmap->worldToMap(x, y, map_x, map_y)) {
    return false;
  }
  const double center_cost = static_cast<double>(costmap->getCost(map_x, map_y));
  const double footprint_cost = checker.footprintCostAtPose(x, y, yaw, footprint);
  if (!std::isfinite(footprint_cost) || footprint_cost < 0.0 ||
    footprint_cost >= nav2_costmap_2d::LETHAL_OBSTACLE)
  {
    return false;
  }
  maximum_proximity_cost = std::max(
    maximum_proximity_cost, std::max(center_cost, footprint_cost));
  return true;
}

std::vector<std::vector<std::pair<double, double>>> translation_segments(
  const nav_msgs::msg::Path & path,
  std::size_t & pivot_count)
{
  std::vector<std::vector<std::pair<double, double>>> segments;
  pivot_count = 0;
  if (path.poses.empty()) {
    return segments;
  }
  segments.push_back(
    {{path.poses.front().pose.position.x, path.poses.front().pose.position.y}});
  for (std::size_t index = 1; index < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1].pose;
    const auto & current = path.poses[index].pose;
    const double distance = std::hypot(
      current.position.x - previous.position.x,
      current.position.y - previous.position.y);
    const double yaw_change = std::abs(normalized_angle(
        tf2::getYaw(current.orientation) - tf2::getYaw(previous.orientation)));
    if (distance <= 1.0e-4 && yaw_change >= 0.0872664626) {
      ++pivot_count;
      segments.push_back({{current.position.x, current.position.y}});
    } else if (distance > 1.0e-4) {
      segments.back().emplace_back(current.position.x, current.position.y);
    }
  }
  return segments;
}

std::vector<std::pair<double, double>> resample(
  const std::vector<std::pair<double, double>> & points,
  double spacing)
{
  if (points.size() < 2) {
    return points;
  }
  std::vector<std::pair<double, double>> result{points.front()};
  double traversed = 0.0;
  double next_distance = spacing;
  for (std::size_t index = 1; index < points.size(); ++index) {
    const auto & first = points[index - 1];
    const auto & last = points[index];
    const double dx = last.first - first.first;
    const double dy = last.second - first.second;
    const double length = std::hypot(dx, dy);
    if (length <= kEpsilon) {
      continue;
    }
    const double segment_end = traversed + length;
    while (next_distance <= segment_end + kEpsilon) {
      const double fraction = (next_distance - traversed) / length;
      result.emplace_back(first.first + fraction * dx, first.second + fraction * dy);
      next_distance += spacing;
    }
    traversed = segment_end;
  }
  if (std::hypot(
      result.back().first - points.back().first,
      result.back().second - points.back().second) > kEpsilon)
  {
    result.push_back(points.back());
  }
  return result;
}

double curvature_energy(
  const nav_msgs::msg::Path & path,
  double spacing,
  std::size_t & pivot_count)
{
  double energy = 0.0;
  for (const auto & segment : translation_segments(path, pivot_count)) {
    const auto sampled = resample(segment, spacing);
    for (std::size_t index = 1; index + 1 < sampled.size(); ++index) {
      const auto & first = sampled[index - 1];
      const auto & middle = sampled[index];
      const auto & last = sampled[index + 1];
      const double side_a = std::hypot(middle.first - first.first, middle.second - first.second);
      const double side_b = std::hypot(last.first - middle.first, last.second - middle.second);
      const double chord = std::hypot(last.first - first.first, last.second - first.second);
      const double denominator = side_a * side_b * chord;
      if (denominator <= kEpsilon) {
        continue;
      }
      const double cross =
        (middle.first - first.first) * (last.second - first.second) -
        (middle.second - first.second) * (last.first - first.first);
      const double curvature = 2.0 * cross / denominator;
      energy += curvature * curvature * 0.5 * (side_a + side_b);
    }
  }
  return energy;
}

PathEvaluation evaluate_path(
  const nav_msgs::msg::Path & path,
  const std::shared_ptr<Costmap> & costmap,
  CollisionChecker & checker,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double spacing)
{
  PathEvaluation evaluation;
  if (path.poses.empty()) {
    return evaluation;
  }
  evaluation.maximum_proximity_cost = 0.0;
  for (std::size_t index = 0; index < path.poses.size(); ++index) {
    const auto & current = path.poses[index].pose;
    if (index == 0) {
      if (!evaluate_pose(
          current.position.x, current.position.y, tf2::getYaw(current.orientation),
          costmap, checker, footprint, evaluation.maximum_proximity_cost))
      {
        return evaluation;
      }
      continue;
    }
    const auto & previous = path.poses[index - 1].pose;
    const double dx = current.position.x - previous.position.x;
    const double dy = current.position.y - previous.position.y;
    const double distance = std::hypot(dx, dy);
    const double previous_yaw = tf2::getYaw(previous.orientation);
    const double yaw_change = normalized_angle(
      tf2::getYaw(current.orientation) - previous_yaw);
    double radius = 0.01;
    for (const auto & point : footprint) {
      radius = std::max(radius, std::hypot(point.x, point.y));
    }
    const double angular_spacing = spacing / radius;
    const int steps = std::max({
          1,
          static_cast<int>(std::ceil(distance / spacing)),
          static_cast<int>(std::ceil(std::abs(yaw_change) / angular_spacing))});
    for (int step = 1; step <= steps; ++step) {
      const double fraction = static_cast<double>(step) / static_cast<double>(steps);
      if (!evaluate_pose(
          previous.position.x + fraction * dx,
          previous.position.y + fraction * dy,
          previous_yaw + fraction * yaw_change,
          costmap, checker, footprint, evaluation.maximum_proximity_cost))
      {
        return evaluation;
      }
    }
  }
  evaluation.curvature_energy = curvature_energy(path, 0.05, evaluation.pivot_count);
  evaluation.safe = std::isfinite(evaluation.curvature_energy);
  return evaluation;
}

}  // namespace

void SafetyGatedHybridSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_subscriber)
{
  const auto node = parent.lock();
  if (!node) {
    throw nav2_core::FailedToSmoothPath("Hybrid smoother lifecycle node expired");
  }
  plugin_name_ = std::move(name);
  node_ = parent;
  logger_ = node->get_logger();
  costmap_subscriber_ = costmap_subscriber;
  footprint_subscriber_ = footprint_subscriber;
  const std::string prefix = plugin_name_ + ".";
  minimum_cost_improvement_ = declare_and_get<double>(
    node, prefix + "minimum_cost_improvement", 20.0);
  maximum_curvature_energy_ratio_ = declare_and_get<double>(
    node, prefix + "maximum_curvature_energy_ratio", 2.0);
  curvature_energy_floor_ = declare_and_get<double>(
    node, prefix + "curvature_energy_floor", 0.25);
  evaluation_spacing_ = declare_and_get<double>(
    node, prefix + "evaluation_spacing", 0.025);
  const double footprint_length = declare_and_get<double>(
    node, prefix + "fallback_footprint_length", 0.44);
  const double footprint_width = declare_and_get<double>(
    node, prefix + "fallback_footprint_width", 0.34);
  if (!std::isfinite(minimum_cost_improvement_) || minimum_cost_improvement_ < 0.0 ||
    !std::isfinite(maximum_curvature_energy_ratio_) ||
    maximum_curvature_energy_ratio_ < 1.0 ||
    !std::isfinite(curvature_energy_floor_) || curvature_energy_floor_ < 0.0 ||
    !std::isfinite(evaluation_spacing_) || evaluation_spacing_ <= 0.0 ||
    !std::isfinite(footprint_length) || footprint_length <= 0.0 ||
    !std::isfinite(footprint_width) || footprint_width <= 0.0)
  {
    throw nav2_core::FailedToSmoothPath("Hybrid selection parameters are invalid");
  }
  const double half_length = 0.5 * footprint_length;
  const double half_width = 0.5 * footprint_width;
  fallback_footprint_.resize(4);
  fallback_footprint_[0].x = half_length;
  fallback_footprint_[0].y = half_width;
  fallback_footprint_[1].x = half_length;
  fallback_footprint_[1].y = -half_width;
  fallback_footprint_[2].x = -half_length;
  fallback_footprint_[2].y = -half_width;
  fallback_footprint_[3].x = -half_length;
  fallback_footprint_[3].y = half_width;

  simple_smoother_ = std::make_unique<nav2_smoother::SimpleSmoother>();
  pivot_smoother_ = std::make_unique<AdaptivePivotG2Smoother>();
  simple_smoother_->configure(
    parent, plugin_name_ + ".simple", tf, costmap_subscriber, footprint_subscriber);
  pivot_smoother_->configure(
    parent, plugin_name_ + ".pivot", tf, costmap_subscriber, footprint_subscriber);
  diagnostics_publisher_ = node->create_publisher<std_msgs::msg::String>(
    "/research/adaptive_hybrid/diagnostics", rclcpp::QoS(10));
  RCLCPP_INFO(logger_, "Configured safety-gated hybrid smoother as '%s'", plugin_name_.c_str());
}

void SafetyGatedHybridSmoother::cleanup()
{
  if (simple_smoother_) {
    simple_smoother_->cleanup();
  }
  if (pivot_smoother_) {
    pivot_smoother_->cleanup();
  }
  simple_smoother_.reset();
  pivot_smoother_.reset();
  diagnostics_publisher_.reset();
  footprint_subscriber_.reset();
  costmap_subscriber_.reset();
}

void SafetyGatedHybridSmoother::activate()
{
  simple_smoother_->activate();
  pivot_smoother_->activate();
  diagnostics_publisher_->on_activate();
}

void SafetyGatedHybridSmoother::deactivate()
{
  diagnostics_publisher_->on_deactivate();
  pivot_smoother_->deactivate();
  simple_smoother_->deactivate();
}

bool SafetyGatedHybridSmoother::smooth(
  nav_msgs::msg::Path & path,
  const rclcpp::Duration & max_time)
{
  if (!simple_smoother_ || !pivot_smoother_ || !costmap_subscriber_) {
    throw nav2_core::FailedToSmoothPath("Hybrid smoother is not configured");
  }
  const auto costmap = costmap_subscriber_->getCostmap();
  if (!costmap) {
    throw nav2_core::FailedToSmoothPath("Hybrid smoother has not received a costmap");
  }
  std::vector<geometry_msgs::msg::Point> footprint;
  std_msgs::msg::Header footprint_header;
  if (!footprint_subscriber_ ||
    !footprint_subscriber_->getFootprintInRobotFrame(footprint, footprint_header) ||
    footprint.size() < 3)
  {
    footprint = fallback_footprint_;
  }
  CollisionChecker checker(costmap);
  const auto started = std::chrono::steady_clock::now();
  nav_msgs::msg::Path simple_path = path;
  nav_msgs::msg::Path pivot_path = path;
  bool simple_completed = false;
  bool pivot_completed = false;
  try {
    simple_completed = simple_smoother_->smooth(simple_path, max_time);
  } catch (const std::exception & error) {
    RCLCPP_WARN(logger_, "Embedded Simple candidate failed: %s", error.what());
  }
  const std::chrono::duration<double> first_elapsed =
    std::chrono::steady_clock::now() - started;
  const double remaining_seconds = max_time.seconds() - first_elapsed.count();
  if (remaining_seconds > 0.0) {
    try {
      pivot_completed = pivot_smoother_->smooth(
        pivot_path, rclcpp::Duration::from_seconds(remaining_seconds));
    } catch (const std::exception & error) {
      RCLCPP_WARN(logger_, "Embedded Pivot-G2 candidate failed: %s", error.what());
    }
  }

  const PathEvaluation simple_evaluation = simple_completed ?
    evaluate_path(simple_path, costmap, checker, footprint, evaluation_spacing_) :
    PathEvaluation{};
  const PathEvaluation pivot_evaluation = pivot_completed ?
    evaluate_path(pivot_path, costmap, checker, footprint, evaluation_spacing_) :
    PathEvaluation{};
  const PathEvaluation raw_evaluation =
    evaluate_path(path, costmap, checker, footprint, evaluation_spacing_);
  const auto selection = adaptive_pivot_g2::select_hybrid_candidate(
    {simple_evaluation.safe, simple_evaluation.maximum_proximity_cost,
      simple_evaluation.curvature_energy},
    {pivot_evaluation.safe, pivot_evaluation.maximum_proximity_cost,
      pivot_evaluation.curvature_energy},
    minimum_cost_improvement_, maximum_curvature_energy_ratio_,
    curvature_energy_floor_);
  const bool choose_raw = !selection.valid && raw_evaluation.safe;
  if (!selection.valid && !choose_raw) {
    throw nav2_core::FailedToSmoothPath(
            "Hybrid smoother found no swept-footprint-safe candidate or raw fallback");
  }
  const bool choose_pivot = selection.valid && selection.use_pivot;
  if (choose_pivot) {
    path = std::move(pivot_path);
  } else if (!choose_raw) {
    path = std::move(simple_path);
  }
  const std::string selected = choose_raw ? "raw" :
    (choose_pivot ? "pivot_g2" : "simple");
  const std::string reason = choose_raw ?
    "smoothed_candidates_unsafe_raw_fallback" : selection.reason;

  std::ostringstream diagnostics;
  diagnostics << "{\"method\":\"adaptive_hybrid\",\"selected\":\""
              << selected
              << "\",\"reason\":\"" << reason
              << "\",\"raw_safe\":" << (raw_evaluation.safe ? "true" : "false")
              << ",\"simple_safe\":" << (simple_evaluation.safe ? "true" : "false")
              << ",\"pivot_safe\":" << (pivot_evaluation.safe ? "true" : "false")
              << ",\"raw_max_cost\":";
  append_json_number(diagnostics, raw_evaluation.maximum_proximity_cost);
  diagnostics << ",\"simple_max_cost\":";
  append_json_number(diagnostics, simple_evaluation.maximum_proximity_cost);
  diagnostics << ",\"pivot_max_cost\":";
  append_json_number(diagnostics, pivot_evaluation.maximum_proximity_cost);
  diagnostics << ",\"raw_energy\":";
  append_json_number(diagnostics, raw_evaluation.curvature_energy);
  diagnostics << ",\"simple_energy\":";
  append_json_number(diagnostics, simple_evaluation.curvature_energy);
  diagnostics << ",\"pivot_energy\":";
  append_json_number(diagnostics, pivot_evaluation.curvature_energy);
  diagnostics << ",\"pivot_markers\":" << pivot_evaluation.pivot_count << "}";
  if (diagnostics_publisher_ && diagnostics_publisher_->is_activated()) {
    std_msgs::msg::String message;
    message.data = diagnostics.str();
    diagnostics_publisher_->publish(message);
  }
  RCLCPP_INFO(logger_, "%s", diagnostics.str().c_str());
  return true;
}

}  // namespace adaptive_pivot_g2_nav2

PLUGINLIB_EXPORT_CLASS(
  adaptive_pivot_g2_nav2::SafetyGatedHybridSmoother,
  nav2_core::Smoother)
