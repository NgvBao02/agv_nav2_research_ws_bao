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
#include <cmath>
#include <cstddef>
#include <cstdint>
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
  double translational_curvature_energy{std::numeric_limits<double>::infinity()};
  double pivot_rotation{std::numeric_limits<double>::infinity()};
  double maneuver_effort{std::numeric_limits<double>::infinity()};
  double path_length{std::numeric_limits<double>::infinity()};
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
  unsigned char maximum_center_cost,
  double & maximum_proximity_cost)
{
  unsigned int map_x = 0;
  unsigned int map_y = 0;
  if (!costmap->worldToMap(x, y, map_x, map_y)) {
    return false;
  }
  const double center_cost = static_cast<double>(costmap->getCost(map_x, map_y));
  const double footprint_cost = checker.footprintCostAtPose(x, y, yaw, footprint);
  if (center_cost > static_cast<double>(maximum_center_cost) ||
    !std::isfinite(footprint_cost) || footprint_cost < 0.0 ||
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
  std::size_t & pivot_count,
  double & pivot_rotation,
  double duplicate_position_tolerance,
  double minimum_pivot_angle)
{
  std::vector<std::vector<std::pair<double, double>>> segments;
  pivot_count = 0;
  pivot_rotation = 0.0;
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
    if (distance <= duplicate_position_tolerance &&
      yaw_change >= minimum_pivot_angle)
    {
      ++pivot_count;
      pivot_rotation += yaw_change;
      segments.push_back({{current.position.x, current.position.y}});
    } else if (distance > duplicate_position_tolerance) {
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
  std::size_t & pivot_count,
  double & pivot_rotation,
  double duplicate_position_tolerance,
  double minimum_pivot_angle)
{
  double energy = 0.0;
  for (const auto & segment : translation_segments(
      path, pivot_count, pivot_rotation,
      duplicate_position_tolerance, minimum_pivot_angle))
  {
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

double path_length(const nav_msgs::msg::Path & path)
{
  double length = 0.0;
  for (std::size_t index = 1; index < path.poses.size(); ++index) {
    length += std::hypot(
      path.poses[index].pose.position.x - path.poses[index - 1].pose.position.x,
      path.poses[index].pose.position.y - path.poses[index - 1].pose.position.y);
  }
  return length;
}

PathEvaluation evaluate_path(
  const nav_msgs::msg::Path & path,
  const std::shared_ptr<Costmap> & costmap,
  CollisionChecker & checker,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double spacing,
  double duplicate_position_tolerance,
  double minimum_pivot_angle,
  double pivot_rotation_characteristic_length,
  unsigned char maximum_center_cost)
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
          costmap, checker, footprint, maximum_center_cost,
          evaluation.maximum_proximity_cost))
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
          costmap, checker, footprint, maximum_center_cost,
          evaluation.maximum_proximity_cost))
      {
        return evaluation;
      }
    }
  }
  evaluation.translational_curvature_energy = curvature_energy(
    path, 0.05, evaluation.pivot_count, evaluation.pivot_rotation,
    duplicate_position_tolerance, minimum_pivot_angle);
  evaluation.path_length = path_length(path);
  evaluation.maneuver_effort =
    evaluation.translational_curvature_energy +
    evaluation.pivot_rotation / pivot_rotation_characteristic_length;
  evaluation.safe =
    std::isfinite(evaluation.translational_curvature_energy) &&
    std::isfinite(evaluation.pivot_rotation) &&
    std::isfinite(evaluation.maneuver_effort) &&
    std::isfinite(evaluation.path_length);
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
  peak_cost_deadband_ = declare_and_get<double>(
    node, prefix + "peak_cost_deadband", 20.0);
  relative_effort_deadband_ = declare_and_get<double>(
    node, prefix + "relative_effort_deadband", 0.05);
  effort_floor_ = declare_and_get<double>(
    node, prefix + "effort_floor", 0.25);
  path_length_tolerance_ = declare_and_get<double>(
    node, prefix + "path_length_tolerance", 1.0e-6);
  pivot_rotation_characteristic_length_ = declare_and_get<double>(
    node, prefix + "pivot_rotation_characteristic_length", 0.2548);
  const int64_t maximum_center_cost = declare_and_get<int64_t>(
    node, prefix + "maximum_center_cost", 252);
  maximum_center_cost_ = static_cast<unsigned char>(
    std::clamp<int64_t>(maximum_center_cost, 0, 252));
  evaluation_spacing_ = declare_and_get<double>(
    node, prefix + "evaluation_spacing", 0.025);
  pivot_duplicate_position_tolerance_ = declare_and_get<double>(
    node, prefix + "pivot_duplicate_position_tolerance", 1.0e-4);
  minimum_pivot_angle_ = declare_and_get<double>(
    node, prefix + "minimum_pivot_angle", 0.0872664626);
  const double footprint_length = declare_and_get<double>(
    node, prefix + "fallback_footprint_length", 0.44);
  const double footprint_width = declare_and_get<double>(
    node, prefix + "fallback_footprint_width", 0.34);
  if (!std::isfinite(peak_cost_deadband_) || peak_cost_deadband_ < 0.0 ||
    !std::isfinite(relative_effort_deadband_) ||
    relative_effort_deadband_ < 0.0 || relative_effort_deadband_ > 1.0 ||
    !std::isfinite(effort_floor_) || effort_floor_ <= 0.0 ||
    !std::isfinite(path_length_tolerance_) || path_length_tolerance_ < 0.0 ||
    !std::isfinite(pivot_rotation_characteristic_length_) ||
    pivot_rotation_characteristic_length_ <= 0.0 ||
    !std::isfinite(evaluation_spacing_) || evaluation_spacing_ <= 0.0 ||
    !std::isfinite(pivot_duplicate_position_tolerance_) ||
    pivot_duplicate_position_tolerance_ < 0.0 ||
    !std::isfinite(minimum_pivot_angle_) || minimum_pivot_angle_ <= 0.0 ||
    minimum_pivot_angle_ > std::acos(-1.0) ||
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
  pivot_smoother_ = std::make_unique<AdaptivePivotG2Smoother>(
    AdaptivePivotG2Smoother::PreprocessingMode::kConditionOnly,
    AdaptivePivotG2Smoother::CandidateSearchMode::kLegacyJointDq);
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
  nav_msgs::msg::Path simple_path = path;
  nav_msgs::msg::Path pivot_path = path;
  bool simple_completed = false;
  bool pivot_completed = false;
  const double candidate_seconds = 0.5 * max_time.seconds();
  if (candidate_seconds > 0.0) {
    try {
      simple_completed = simple_smoother_->smooth(
        simple_path, rclcpp::Duration::from_seconds(candidate_seconds));
    } catch (const std::exception & error) {
      RCLCPP_WARN(logger_, "Embedded Simple candidate failed: %s", error.what());
    }
    try {
      pivot_completed = pivot_smoother_->smooth(
        pivot_path, rclcpp::Duration::from_seconds(candidate_seconds));
    } catch (const std::exception & error) {
      RCLCPP_WARN(logger_, "Embedded Pivot-G2 candidate failed: %s", error.what());
    }
  }

  const PathEvaluation simple_evaluation = simple_completed ?
    evaluate_path(
    simple_path, costmap, checker, footprint, evaluation_spacing_,
    pivot_duplicate_position_tolerance_, minimum_pivot_angle_,
    pivot_rotation_characteristic_length_, maximum_center_cost_) :
    PathEvaluation{};
  const PathEvaluation pivot_evaluation = pivot_completed ?
    evaluate_path(
    pivot_path, costmap, checker, footprint, evaluation_spacing_,
    pivot_duplicate_position_tolerance_, minimum_pivot_angle_,
    pivot_rotation_characteristic_length_, maximum_center_cost_) :
    PathEvaluation{};
  const PathEvaluation raw_evaluation =
    evaluate_path(
    path, costmap, checker, footprint, evaluation_spacing_,
    pivot_duplicate_position_tolerance_, minimum_pivot_angle_,
    pivot_rotation_characteristic_length_, maximum_center_cost_);
  const auto selection =
    adaptive_pivot_g2::select_hybrid_candidate_with_raw_fallback(
    {raw_evaluation.safe, raw_evaluation.maximum_proximity_cost,
      raw_evaluation.maneuver_effort, raw_evaluation.path_length},
    {simple_evaluation.safe, simple_evaluation.maximum_proximity_cost,
      simple_evaluation.maneuver_effort, simple_evaluation.path_length},
    {pivot_evaluation.safe, pivot_evaluation.maximum_proximity_cost,
      pivot_evaluation.maneuver_effort, pivot_evaluation.path_length},
    {peak_cost_deadband_, relative_effort_deadband_, effort_floor_,
      path_length_tolerance_});
  if (!selection.valid) {
    throw nav2_core::FailedToSmoothPath(
            "Hybrid smoother found no swept-footprint-safe candidate or raw fallback");
  }
  const bool choose_raw = selection.use_raw;
  const bool choose_pivot = selection.valid && selection.use_pivot;
  if (choose_pivot) {
    path = std::move(pivot_path);
  } else if (!choose_raw) {
    path = std::move(simple_path);
  }
  const std::string selected = choose_raw ? "raw" :
    (choose_pivot ? "pivot_g2" : "simple");
  const std::string reason = selection.reason;

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
  append_json_number(diagnostics, raw_evaluation.translational_curvature_energy);
  diagnostics << ",\"simple_energy\":";
  append_json_number(diagnostics, simple_evaluation.translational_curvature_energy);
  diagnostics << ",\"pivot_energy\":";
  append_json_number(diagnostics, pivot_evaluation.translational_curvature_energy);
  diagnostics << ",\"raw_pivot_rotation_rad\":";
  append_json_number(diagnostics, raw_evaluation.pivot_rotation);
  diagnostics << ",\"simple_pivot_rotation_rad\":";
  append_json_number(diagnostics, simple_evaluation.pivot_rotation);
  diagnostics << ",\"pivot_pivot_rotation_rad\":";
  append_json_number(diagnostics, pivot_evaluation.pivot_rotation);
  diagnostics << ",\"raw_maneuver_effort\":";
  append_json_number(diagnostics, raw_evaluation.maneuver_effort);
  diagnostics << ",\"simple_maneuver_effort\":";
  append_json_number(diagnostics, simple_evaluation.maneuver_effort);
  diagnostics << ",\"pivot_maneuver_effort\":";
  append_json_number(diagnostics, pivot_evaluation.maneuver_effort);
  diagnostics << ",\"raw_path_length_m\":";
  append_json_number(diagnostics, raw_evaluation.path_length);
  diagnostics << ",\"simple_path_length_m\":";
  append_json_number(diagnostics, simple_evaluation.path_length);
  diagnostics << ",\"pivot_path_length_m\":";
  append_json_number(diagnostics, pivot_evaluation.path_length);
  diagnostics << ",\"candidate_time_budget_s\":" << candidate_seconds
              << ",\"maximum_center_cost\":"
              << static_cast<int>(maximum_center_cost_)
              << ",\"pivot_markers\":" << pivot_evaluation.pivot_count << "}";
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
