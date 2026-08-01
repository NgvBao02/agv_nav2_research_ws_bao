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

#include "adaptive_pivot_g2_nav2/adaptive_pivot_g2_smoother.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "adaptive_pivot_g2/candidate_selection.hpp"
#include "adaptive_pivot_g2/path_conditioning.hpp"
#include "adaptive_pivot_g2/path_optimization.hpp"
#include "adaptive_pivot_g2/quintic_transition.hpp"
#include "adaptive_pivot_g2/time_parameterization.hpp"
#include "nav2_core/smoother_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace adaptive_pivot_g2_nav2
{
namespace
{

using adaptive_pivot_g2::CornerInput;
using adaptive_pivot_g2::TransitionCandidate;
using adaptive_pivot_g2::Vec2;
using Costmap = nav2_costmap_2d::Costmap2D;
using CollisionChecker =
  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<Costmap>>;

constexpr double kEpsilon = 1.0e-10;

double maximum_straight_speed(const adaptive_pivot_g2::RobotLimits & limits)
{
  return std::min(limits.max_linear_speed, limits.max_wheel_speed);
}

void append_json_string(std::ostringstream & stream, const std::string & value)
{
  stream << '"';
  for (const char character : value) {
    if (character == '"' || character == '\\') {
      stream << '\\';
    }
    stream << character;
  }
  stream << '"';
}

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

Vec2 position_of(const geometry_msgs::msg::PoseStamped & pose)
{
  return {pose.pose.position.x, pose.pose.position.y};
}

Vec2 normalized(const Vec2 & value)
{
  const double length = adaptive_pivot_g2::norm(value);
  if (length <= kEpsilon) {
    return {};
  }
  return value / length;
}

double signed_angle(const Vec2 & incoming, const Vec2 & outgoing)
{
  return std::atan2(
    adaptive_pivot_g2::cross(incoming, outgoing),
    adaptive_pivot_g2::dot(incoming, outgoing));
}

geometry_msgs::msg::PoseStamped make_pose(
  const Vec2 & point,
  double heading,
  const std_msgs::msg::Header & header)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;
  pose.pose.position.x = point.x;
  pose.pose.position.y = point.y;
  pose.pose.orientation.z = std::sin(0.5 * heading);
  pose.pose.orientation.w = std::cos(0.5 * heading);
  return pose;
}

bool pose_is_safe(
  const Vec2 & point,
  double heading,
  const std::shared_ptr<Costmap> & costmap,
  CollisionChecker & checker,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  unsigned char max_footprint_cost,
  double * proximity_cost = nullptr)
{
  unsigned int map_x = 0;
  unsigned int map_y = 0;
  if (!costmap->worldToMap(point.x, point.y, map_x, map_y)) {
    return false;
  }
  const unsigned char center_cost = costmap->getCost(map_x, map_y);
  if (center_cost > max_footprint_cost) {
    return false;
  }
  const double footprint_cost = checker.footprintCostAtPose(
    point.x, point.y, heading, footprint);
  // Inflation costs are defined for the robot center and already encode the
  // inscribed radius. Applying the same threshold to every footprint point
  // would inflate the robot twice. The center enforces the clearance gate;
  // the swept footprint below only rejects actual lethal / unknown contact.
  const bool safe = std::isfinite(footprint_cost) && footprint_cost >= 0.0 &&
    footprint_cost < nav2_costmap_2d::LETHAL_OBSTACLE;
  if (safe && proximity_cost != nullptr) {
    *proximity_cost = std::max(static_cast<double>(center_cost), footprint_cost);
  }
  return safe;
}

bool line_is_safe(
  const Vec2 & start,
  const Vec2 & finish,
  const std::shared_ptr<Costmap> & costmap,
  CollisionChecker & checker,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  unsigned char max_footprint_cost)
{
  const Vec2 delta = finish - start;
  const double length = adaptive_pivot_g2::norm(delta);
  if (length <= kEpsilon) {
    return pose_is_safe(
      start, 0.0, costmap, checker, footprint, max_footprint_cost);
  }
  const double heading = std::atan2(delta.y, delta.x);
  const double spacing = std::max(0.005, 0.5 * costmap->getResolution());
  const int segments = std::max(1, static_cast<int>(std::ceil(length / spacing)));
  for (int index = 0; index <= segments; ++index) {
    const double fraction = static_cast<double>(index) / static_cast<double>(segments);
    if (!pose_is_safe(
        start + delta * fraction, heading, costmap, checker, footprint,
        max_footprint_cost))
    {
      return false;
    }
  }
  return true;
}

bool transition_is_safe(
  const TransitionCandidate & candidate,
  const std::shared_ptr<Costmap> & costmap,
  CollisionChecker & checker,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  unsigned char max_footprint_cost,
  double & maximum_proximity_cost)
{
  maximum_proximity_cost = 0.0;
  if (candidate.samples.empty()) {
    return false;
  }
  double footprint_radius = 0.01;
  for (const auto & point : footprint) {
    footprint_radius = std::max(footprint_radius, std::hypot(point.x, point.y));
  }
  const double linear_spacing = std::max(0.005, 0.5 * costmap->getResolution());
  const double angular_spacing = linear_spacing / footprint_radius;
  double proximity_cost = 0.0;
  if (!pose_is_safe(
      candidate.samples.front().position, candidate.samples.front().heading,
      costmap, checker, footprint, max_footprint_cost, &proximity_cost))
  {
    return false;
  }
  maximum_proximity_cost = proximity_cost;
  for (std::size_t index = 1U; index < candidate.samples.size(); ++index) {
    const auto & previous = candidate.samples[index - 1U];
    const auto & current = candidate.samples[index];
    const Vec2 delta = current.position - previous.position;
    const double translation = adaptive_pivot_g2::norm(delta);
    const double heading_change = std::atan2(
      std::sin(current.heading - previous.heading),
      std::cos(current.heading - previous.heading));
    const int steps = std::max({
          1,
          static_cast<int>(std::ceil(translation / linear_spacing)),
          static_cast<int>(std::ceil(std::abs(heading_change) / angular_spacing))});
    for (int step = 1; step <= steps; ++step) {
      const double fraction = static_cast<double>(step) / static_cast<double>(steps);
      proximity_cost = 0.0;
      if (!pose_is_safe(
          previous.position + delta * fraction,
          previous.heading + heading_change * fraction,
          costmap, checker, footprint, max_footprint_cost, &proximity_cost))
      {
        return false;
      }
      maximum_proximity_cost = std::max(maximum_proximity_cost, proximity_cost);
    }
  }
  return true;
}

bool pivot_is_safe(
  const Vec2 & vertex,
  double incoming_heading,
  double turn_angle,
  const std::shared_ptr<Costmap> & costmap,
  CollisionChecker & checker,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  unsigned char max_footprint_cost)
{
  double footprint_radius = 0.0;
  for (const auto & point : footprint) {
    footprint_radius = std::max(footprint_radius, std::hypot(point.x, point.y));
  }
  const double linear_sweep_spacing = std::max(0.005, 0.5 * costmap->getResolution());
  const double angular_spacing = linear_sweep_spacing / std::max(0.01, footprint_radius);
  const int steps = std::max(
    1, static_cast<int>(std::ceil(std::abs(turn_angle) / angular_spacing)));
  for (int index = 0; index <= steps; ++index) {
    const double fraction = static_cast<double>(index) / static_cast<double>(steps);
    if (!pose_is_safe(
        vertex, incoming_heading + fraction * turn_angle, costmap, checker, footprint,
        max_footprint_cost))
    {
      return false;
    }
  }
  return true;
}

bool terminal_rotation_is_safe(
  const nav_msgs::msg::Path & path,
  const std::shared_ptr<Costmap> & costmap,
  CollisionChecker & checker,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  unsigned char max_footprint_cost)
{
  if (path.poses.size() < 2U) {
    return false;
  }
  const Vec2 before = position_of(path.poses[path.poses.size() - 2U]);
  const Vec2 finish = position_of(path.poses.back());
  const Vec2 delta = finish - before;
  if (adaptive_pivot_g2::norm(delta) <= kEpsilon) {
    return true;
  }
  const double path_heading = std::atan2(delta.y, delta.x);
  const auto & orientation = path.poses.back().pose.orientation;
  const double goal_heading = 2.0 * std::atan2(
    orientation.z, orientation.w);
  const double heading_change = std::atan2(
    std::sin(goal_heading - path_heading),
    std::cos(goal_heading - path_heading));
  return pivot_is_safe(
    finish, path_heading, heading_change, costmap, checker, footprint,
    max_footprint_cost);
}

std::vector<Vec2> remove_duplicate_points(const nav_msgs::msg::Path & path)
{
  std::vector<Vec2> points;
  points.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    const Vec2 point = position_of(pose);
    if (points.empty() || adaptive_pivot_g2::distance(points.back(), point) > 1.0e-6) {
      points.push_back(point);
    }
  }
  return points;
}

std::vector<Vec2> prune_line_of_sight(
  const std::vector<Vec2> & input,
  const std::shared_ptr<Costmap> & costmap,
  CollisionChecker & checker,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  unsigned char max_footprint_cost)
{
  if (input.size() < 3) {
    return input;
  }
  std::vector<Vec2> output;
  output.reserve(input.size());
  std::size_t anchor = 0;
  output.push_back(input.front());
  while (anchor + 1 < input.size()) {
    std::size_t candidate = input.size() - 1;
    while (candidate > anchor + 1 &&
      !line_is_safe(
        input[anchor], input[candidate], costmap, checker, footprint,
        max_footprint_cost))
    {
      --candidate;
    }
    output.push_back(input[candidate]);
    anchor = candidate;
  }
  return output;
}

struct ConditioningRun
{
  adaptive_pivot_g2::PathConditioningResult result;
  double maximum_deviation{0.0};
  double oscillation_maximum_deviation{0.0};
};

ConditioningRun condition_planner_path(
  const std::vector<Vec2> & input,
  double maximum_deviation_override,
  double deviation_resolution_ratio,
  double oscillation_maximum_span,
  double oscillation_maximum_deviation_override,
  double oscillation_deviation_resolution_ratio,
  double oscillation_minimum_turn_angle,
  std::size_t oscillation_minimum_sign_changes,
  const std::shared_ptr<Costmap> & costmap,
  CollisionChecker & checker,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  unsigned char max_footprint_cost)
{
  ConditioningRun run;
  run.maximum_deviation = maximum_deviation_override > 0.0 ?
    maximum_deviation_override :
    deviation_resolution_ratio * costmap->getResolution();
  run.oscillation_maximum_deviation =
    oscillation_maximum_deviation_override > 0.0 ?
    oscillation_maximum_deviation_override :
    oscillation_deviation_resolution_ratio * costmap->getResolution();
  adaptive_pivot_g2::PathConditioningOptions options;
  options.maximum_deviation = run.maximum_deviation;
  options.oscillation_maximum_span = oscillation_maximum_span;
  options.oscillation_maximum_deviation =
    run.oscillation_maximum_deviation;
  options.oscillation_minimum_turn_angle = oscillation_minimum_turn_angle;
  options.oscillation_minimum_sign_changes = oscillation_minimum_sign_changes;
  run.result = adaptive_pivot_g2::condition_polyline(
    input, options,
    [&](const Vec2 & start, const Vec2 & finish) {
      return line_is_safe(
        start, finish, costmap, checker, footprint, max_footprint_cost);
    });
  return run;
}

std::vector<Vec2> validated_conditioned_points(
  const ConditioningRun & conditioning)
{
  if (!conditioning.result.valid) {
    throw nav2_core::FailedToSmoothPath(
            "Pivot-G2 path conditioning failed: " +
            conditioning.result.rejection_reason);
  }
  return conditioning.result.points;
}

std::string resolve_stitch_rejection(
  bool endpoints_valid,
  bool timing_valid,
  const std::string & timing_rejection,
  const std::string & sweep_rejection)
{
  if (!endpoints_valid) {
    return "endpoints_not_preserved";
  }
  if (!timing_valid) {
    return timing_rejection;
  }
  return sweep_rejection;
}

void append_line(
  std::vector<geometry_msgs::msg::PoseStamped> & output,
  const Vec2 & finish,
  double spacing,
  const std_msgs::msg::Header & header)
{
  const Vec2 start = position_of(output.back());
  const Vec2 delta = finish - start;
  const double length = adaptive_pivot_g2::norm(delta);
  if (length <= 1.0e-8) {
    return;
  }
  const double heading = std::atan2(delta.y, delta.x);
  const int segments = std::max(1, static_cast<int>(std::ceil(length / spacing)));
  output.back().pose.orientation.z = std::sin(0.5 * heading);
  output.back().pose.orientation.w = std::cos(0.5 * heading);
  for (int index = 1; index <= segments; ++index) {
    const double fraction = static_cast<double>(index) / static_cast<double>(segments);
    output.push_back(make_pose(start + delta * fraction, heading, header));
  }
}

bool stitched_path_is_safe(
  const nav_msgs::msg::Path & path,
  const std::shared_ptr<Costmap> & costmap,
  CollisionChecker & checker,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  unsigned char max_footprint_cost,
  double minimum_pivot_angle,
  std::string * rejection_reason = nullptr)
{
  const auto reject = [rejection_reason](const std::string & reason) {
      if (rejection_reason != nullptr) {
        *rejection_reason = reason;
      }
      return false;
    };
  if (path.poses.size() < 2U) {
    return reject("fewer_than_two_poses");
  }
  double footprint_radius = 0.01;
  for (const auto & point : footprint) {
    footprint_radius = std::max(footprint_radius, std::hypot(point.x, point.y));
  }
  const double spacing = std::max(0.005, 0.5 * costmap->getResolution());
  for (std::size_t index = 0; index < path.poses.size(); ++index) {
    const auto & current = path.poses[index].pose;
    const double current_yaw = 2.0 * std::atan2(
      current.orientation.z, current.orientation.w);
    if (!std::isfinite(current.position.x) || !std::isfinite(current.position.y) ||
      !std::isfinite(current_yaw))
    {
      return reject("non_finite_pose_" + std::to_string(index));
    }
    if (index == 0U) {
      if (!pose_is_safe(
          {current.position.x, current.position.y}, current_yaw, costmap, checker,
          footprint, max_footprint_cost))
      {
        return reject("unsafe_initial_pose");
      }
      continue;
    }

    const auto & previous = path.poses[index - 1U].pose;
    const Vec2 start{previous.position.x, previous.position.y};
    const Vec2 finish{current.position.x, current.position.y};
    const double translation = adaptive_pivot_g2::distance(start, finish);
    const double previous_yaw = 2.0 * std::atan2(
      previous.orientation.z, previous.orientation.w);
    const double yaw_change = std::atan2(
      std::sin(current_yaw - previous_yaw),
      std::cos(current_yaw - previous_yaw));
    if (translation <= 1.0e-8 && std::abs(yaw_change) < minimum_pivot_angle) {
      return reject("non_intentional_duplicate_" + std::to_string(index));
    }
    const int steps = std::max({
          1,
          static_cast<int>(std::ceil(translation / spacing)),
          static_cast<int>(std::ceil(
          std::abs(yaw_change) / (spacing / footprint_radius)))});
    for (int step = 1; step <= steps; ++step) {
      const double fraction = static_cast<double>(step) / static_cast<double>(steps);
      if (!pose_is_safe(
          start + (finish - start) * fraction,
          previous_yaw + fraction * yaw_change, costmap, checker, footprint,
          max_footprint_cost))
      {
        return reject(
          "unsafe_sweep_" + std::to_string(index - 1U) + "_" +
          std::to_string(index));
      }
    }
  }
  return true;
}

}  // namespace

void AdaptivePivotG2Smoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_subscriber)
{
  const auto node = parent.lock();
  if (!node) {
    throw nav2_core::FailedToSmoothPath("Pivot-G2 lifecycle node expired during configure");
  }
  node_ = parent;
  plugin_name_ = std::move(name);
  logger_ = node->get_logger();
  costmap_subscriber_ = std::move(costmap_subscriber);
  footprint_subscriber_ = std::move(footprint_subscriber);

  const std::string prefix = plugin_name_ + ".";
  adaptive_search_options_.minimum_radius = declare_and_get<double>(
    node, prefix + "minimum_radius", 0.10);
  adaptive_search_options_.maximum_radius = declare_and_get<double>(
    node, prefix + "maximum_physical_radius", 1.50);
  const int64_t initial_search_samples = declare_and_get<int64_t>(
    node, prefix + "initial_search_samples", 6);
  const int64_t maximum_evaluations = declare_and_get<int64_t>(
    node, prefix + "maximum_evaluations_per_corner", 20);
  adaptive_search_options_.radius_tolerance = declare_and_get<double>(
    node, prefix + "radius_tolerance", 0.01);
  adaptive_search_options_.objective_tolerance = declare_and_get<double>(
    node, prefix + "objective_tolerance", 0.01);
  segment_margin_override_ = declare_and_get<double>(
    node, prefix + "segment_margin", 0.0);
  const int64_t retained_candidates = declare_and_get<int64_t>(
    node, prefix + "retained_candidates_per_corner", 5);
  curvature_energy_scale_ = declare_and_get<double>(
    node, prefix + "curvature_energy_scale", 1.0);
  delta_time_selection_ = declare_and_get<double>(
    node, prefix + "delta_time_selection", 0.15);
  time_competitive_slack_ = declare_and_get<double>(
    node, prefix + "time_competitive_slack", 10.0);
  selection_weights_.clearance = declare_and_get<double>(
    node, prefix + "clearance_weight", 0.15);
  selection_weights_.angular_speed = declare_and_get<double>(
    node, prefix + "angular_speed_weight", 0.10);
  selection_weights_.curvature_energy = declare_and_get<double>(
    node, prefix + "curvature_energy_weight", 0.75);
  corner_angle_threshold_ = declare_and_get<double>(
    node, prefix + "corner_angle_threshold", 0.0872664626);
  path_conditioning_max_deviation_ = declare_and_get<double>(
    node, prefix + "path_conditioning_max_deviation", 0.0);
  path_conditioning_resolution_ratio_ = declare_and_get<double>(
    node, prefix + "path_conditioning_resolution_ratio", 1.5);
  oscillation_maximum_span_ = declare_and_get<double>(
    node, prefix + "oscillation_maximum_span", 2.0);
  oscillation_maximum_deviation_ = declare_and_get<double>(
    node, prefix + "oscillation_maximum_deviation", 0.0);
  oscillation_deviation_resolution_ratio_ = declare_and_get<double>(
    node, prefix + "oscillation_deviation_resolution_ratio", 3.0);
  oscillation_minimum_turn_angle_ = declare_and_get<double>(
    node, prefix + "oscillation_minimum_turn_angle", 0.20);
  const int64_t oscillation_minimum_sign_changes = declare_and_get<int64_t>(
    node, prefix + "oscillation_minimum_sign_changes", 2);
  output_spacing_ = declare_and_get<double>(node, prefix + "output_spacing", 0.05);
  const int64_t max_footprint_cost = declare_and_get<int64_t>(
    node, prefix + "max_footprint_cost", 252);
  max_footprint_cost_ = static_cast<unsigned char>(
    std::clamp<int64_t>(max_footprint_cost, 0, 252));
  line_of_sight_pruning_ = declare_and_get<bool>(
    node, prefix + "line_of_sight_pruning", false);

  const double selection_weight_sum = selection_weights_.clearance +
    selection_weights_.angular_speed + selection_weights_.curvature_energy;
  if (!std::isfinite(delta_time_selection_) || delta_time_selection_ < 0.0 ||
    !std::isfinite(time_competitive_slack_) || time_competitive_slack_ < 0.0 ||
    !std::isfinite(selection_weights_.clearance) || selection_weights_.clearance < 0.0 ||
    !std::isfinite(selection_weights_.angular_speed) ||
    selection_weights_.angular_speed < 0.0 ||
    !std::isfinite(selection_weights_.curvature_energy) ||
    selection_weights_.curvature_energy < 0.0 ||
    !std::isfinite(selection_weight_sum) || selection_weight_sum <= kEpsilon ||
    !std::isfinite(adaptive_search_options_.minimum_radius) ||
    adaptive_search_options_.minimum_radius <= 0.0 ||
    !std::isfinite(adaptive_search_options_.maximum_radius) ||
    adaptive_search_options_.maximum_radius <
    adaptive_search_options_.minimum_radius ||
    initial_search_samples < 2 || maximum_evaluations < initial_search_samples ||
    !std::isfinite(adaptive_search_options_.radius_tolerance) ||
    adaptive_search_options_.radius_tolerance <= 0.0 ||
    !std::isfinite(adaptive_search_options_.objective_tolerance) ||
    adaptive_search_options_.objective_tolerance < 0.0 ||
    !std::isfinite(segment_margin_override_) || segment_margin_override_ < 0.0 ||
    !std::isfinite(path_conditioning_max_deviation_) ||
    path_conditioning_max_deviation_ < 0.0 ||
    !std::isfinite(path_conditioning_resolution_ratio_) ||
    path_conditioning_resolution_ratio_ <= 0.0 ||
    !std::isfinite(oscillation_maximum_span_) ||
    oscillation_maximum_span_ < 0.0 ||
    !std::isfinite(oscillation_maximum_deviation_) ||
    oscillation_maximum_deviation_ < 0.0 ||
    !std::isfinite(oscillation_deviation_resolution_ratio_) ||
    oscillation_deviation_resolution_ratio_ <= 0.0 ||
    !std::isfinite(oscillation_minimum_turn_angle_) ||
    !(oscillation_minimum_turn_angle_ >= 0.0 &&
    oscillation_minimum_turn_angle_ <= std::acos(-1.0)) ||
    oscillation_minimum_sign_changes < 1 ||
    retained_candidates < 1 ||
    !std::isfinite(curvature_energy_scale_) || curvature_energy_scale_ <= 0.0)
  {
    throw nav2_core::FailedToSmoothPath("Pivot-G2 candidate selection parameters are invalid");
  }
  adaptive_search_options_.initial_samples =
    static_cast<std::size_t>(initial_search_samples);
  adaptive_search_options_.maximum_evaluations =
    static_cast<std::size_t>(maximum_evaluations);
  retained_candidates_per_corner_ =
    static_cast<std::size_t>(retained_candidates);
  oscillation_minimum_sign_changes_ =
    static_cast<std::size_t>(oscillation_minimum_sign_changes);

  transition_options_.control_fraction = declare_and_get<double>(
    node, prefix + "bezier_control_fraction", 0.35);
  transition_options_.max_trim_fraction = declare_and_get<double>(
    node, prefix + "max_trim_fraction", 0.45);
  transition_options_.sample_spacing = declare_and_get<double>(
    node, prefix + "sample_spacing", 0.02);
  limits_.wheel_separation = declare_and_get<double>(
    node, prefix + "wheel_separation", 0.2548);
  limits_.max_linear_speed = declare_and_get<double>(
    node, prefix + "max_linear_speed", 0.30);
  limits_.max_angular_speed = declare_and_get<double>(
    node, prefix + "max_angular_speed", 0.80);
  limits_.max_wheel_speed = declare_and_get<double>(
    node, prefix + "max_wheel_speed", 0.36);
  limits_.max_lateral_acceleration = declare_and_get<double>(
    node, prefix + "max_lateral_acceleration", 0.18);
  limits_.max_linear_acceleration = declare_and_get<double>(
    node, prefix + "max_linear_acceleration", 0.35);
  limits_.max_linear_deceleration = declare_and_get<double>(
    node, prefix + "max_linear_deceleration", 0.45);
  limits_.max_angular_acceleration = declare_and_get<double>(
    node, prefix + "max_angular_acceleration", 1.20);

  const double footprint_length = declare_and_get<double>(
    node, prefix + "fallback_footprint_length", 0.44);
  const double footprint_width = declare_and_get<double>(
    node, prefix + "fallback_footprint_width", 0.34);
  const double positive_geometry_and_limits[]{
    output_spacing_,
    transition_options_.control_fraction,
    transition_options_.max_trim_fraction,
    transition_options_.sample_spacing,
    limits_.wheel_separation,
    limits_.max_linear_speed,
    limits_.max_angular_speed,
    limits_.max_wheel_speed,
    limits_.max_lateral_acceleration,
    limits_.max_linear_acceleration,
    limits_.max_linear_deceleration,
    limits_.max_angular_acceleration,
    footprint_length,
    footprint_width};
  if (std::any_of(
      std::begin(positive_geometry_and_limits),
      std::end(positive_geometry_and_limits),
      [](double value) {return !std::isfinite(value) || value <= 0.0;}) ||
    transition_options_.control_fraction >= 0.5 ||
    transition_options_.max_trim_fraction >= 0.5 ||
    !std::isfinite(corner_angle_threshold_) ||
    corner_angle_threshold_ <= 0.0 ||
    corner_angle_threshold_ > std::acos(-1.0))
  {
    throw nav2_core::FailedToSmoothPath(
            "Pivot-G2 geometry, motion limits, or footprint parameters are invalid");
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

  diagnostics_publisher_ = node->create_publisher<std_msgs::msg::String>(
    "/research/pivot_g2/diagnostics", rclcpp::QoS(10));
  RCLCPP_INFO(
    logger_, "Configured Pivot-G2 smoother '%s' with adaptive radius search",
    plugin_name_.c_str());
}

void AdaptivePivotG2Smoother::cleanup()
{
  diagnostics_publisher_.reset();
  footprint_subscriber_.reset();
  costmap_subscriber_.reset();
}

void AdaptivePivotG2Smoother::activate()
{
  if (diagnostics_publisher_) {
    diagnostics_publisher_->on_activate();
  }
}

void AdaptivePivotG2Smoother::deactivate()
{
  if (diagnostics_publisher_) {
    diagnostics_publisher_->on_deactivate();
  }
}

void AdaptivePivotG2Smoother::publish_diagnostics(
  const std::vector<CornerDecision> & decisions,
  const adaptive_pivot_g2::PathOptimizationResult & path_optimization,
  const std::map<std::string, std::size_t> & rejection_counts,
  const adaptive_pivot_g2::PathConditioningResult & conditioning,
  std::size_t input_point_count,
  double effective_conditioning_deviation,
  double effective_oscillation_deviation,
  double effective_segment_margin,
  const std::string & fallback_status,
  const std::string & selected_stitch_rejection,
  std::size_t output_point_count,
  double runtime_seconds)
{
  std::size_t transition_count = 0;
  std::size_t pass_through_count = 0;
  std::size_t candidate_count = 0;
  std::size_t competitive_count = 0;
  std::size_t evaluation_count = 0;
  double score_sum = 0.0;
  for (const auto & decision : decisions) {
    transition_count += decision.use_transition ? 1U : 0U;
    pass_through_count += decision.pass_through ? 1U : 0U;
    candidate_count += decision.candidate_count;
    competitive_count += decision.competitive_count;
    evaluation_count += decision.evaluation_count;
    score_sum += decision.use_transition ? decision.selection_score : 0.0;
  }
  const std::size_t pivot_count =
    decisions.size() - transition_count - pass_through_count;
  std::ostringstream diagnostics;
  diagnostics << "{\"method\":\"pivot_g2\",\"search_mode\":\"adaptive\""
              << ",\"corners\":" << decisions.size()
              << ",\"g2_transitions\":" << transition_count
              << ",\"pivots\":" << pivot_count
              << ",\"pass_through_corners\":" << pass_through_count
              << ",\"evaluations\":" << evaluation_count
              << ",\"valid_candidates\":" << candidate_count
              << ",\"competitive_candidates\":" << competitive_count
              << ",\"dp_states\":" << path_optimization.state_count
              << ",\"compatible_edges\":"
              << path_optimization.compatible_edge_count
              << ",\"conditioning_input_points\":" << input_point_count
              << ",\"conditioning_output_points\":" << conditioning.points.size()
              << ",\"conditioning_max_deviation\":"
              << effective_conditioning_deviation
              << ",\"conditioning_observed_deviation\":"
              << conditioning.maximum_removed_deviation
              << ",\"conditioning_shortcuts\":"
              << conditioning.accepted_shortcuts
              << ",\"conditioning_oscillation_shortcuts\":"
              << conditioning.accepted_oscillation_shortcuts
              << ",\"conditioning_oscillation_deviation\":"
              << effective_oscillation_deviation
              << ",\"conditioning_safety_rejections\":"
              << conditioning.safety_rejected_shortcuts
              << ",\"segment_margin\":" << effective_segment_margin
              << ",\"fallback\":\"" << fallback_status << "\""
              << ",\"selected_stitch_rejection\":";
  append_json_string(diagnostics, selected_stitch_rejection);
  diagnostics
              << ",\"mean_transition_score\":"
              << (transition_count > 0U ?
  score_sum / static_cast<double>(transition_count) : 0.0)
              << ",\"max_footprint_cost\":"
              << static_cast<int>(max_footprint_cost_)
              << ",\"output_points\":" << output_point_count
              << ",\"corner_search\":[";
  for (std::size_t index = 0; index < decisions.size(); ++index) {
    const auto & decision = decisions[index];
    if (index > 0U) {
      diagnostics << ',';
    }
    diagnostics << "{\"index\":" << index
                << ",\"x\":" << decision.vertex.x
                << ",\"y\":" << decision.vertex.y
                << ",\"turn_angle\":" << decision.turn_angle
                << ",\"incoming_length\":"
                << adaptive_pivot_g2::distance(
      decision.vertex, index == 0U ?
      conditioning.points.front() : decisions[index - 1U].vertex)
                << ",\"outgoing_length\":"
                << adaptive_pivot_g2::distance(
      decision.vertex, index + 1U == decisions.size() ?
      conditioning.points.back() : decisions[index + 1U].vertex)
                << ",\"pivot_safe\":" << (decision.pivot_safe ? "true" : "false")
                << ",\"pass_through\":"
                << (decision.pass_through ? "true" : "false")
                << ",\"radius_min\":" << decision.minimum_search_radius
                << ",\"radius_max\":" << decision.maximum_search_radius
                << ",\"trim_min\":" << decision.minimum_search_trim
                << ",\"trim_max\":" << decision.maximum_search_trim
                << ",\"evaluations\":" << decision.evaluation_count
                << ",\"safe_feasible\":" << decision.feasible_count
                << ",\"states\":" << decision.optimization_states.size()
                << ",\"selected_radius\":"
                << (decision.use_transition ?
    decision.transition.design_radius : 0.0)
                << ",\"selected_trim\":"
                << (decision.use_transition ?
    decision.transition.trim_distance : 0.0)
                << ",\"selected_score\":"
                << (decision.use_transition ? decision.selection_score : 0.0)
                << '}';
  }
  diagnostics << "],\"conditioned_polyline\":[";
  for (std::size_t index = 0; index < conditioning.points.size(); ++index) {
    if (index > 0U) {
      diagnostics << ',';
    }
    diagnostics << '[' << conditioning.points[index].x << ','
                << conditioning.points[index].y << ']';
  }
  diagnostics << "],\"rejections\":{";
  bool first_rejection = true;
  for (const auto & rejection : rejection_counts) {
    if (!first_rejection) {
      diagnostics << ',';
    }
    append_json_string(diagnostics, rejection.first);
    diagnostics << ':' << rejection.second;
    first_rejection = false;
  }
  diagnostics << "},\"runtime_s\":" << runtime_seconds << "}";
  if (diagnostics_publisher_ && diagnostics_publisher_->is_activated()) {
    std_msgs::msg::String message;
    message.data = diagnostics.str();
    diagnostics_publisher_->publish(message);
  }
  RCLCPP_DEBUG(logger_, "%s", diagnostics.str().c_str());
  RCLCPP_INFO(
    logger_,
    "Pivot-G2 adaptive: %zu corners, %zu G2, %zu pivots, %zu pass-through, "
    "%zu evaluations, %.6f s",
    decisions.size(), transition_count, pivot_count, pass_through_count,
    evaluation_count, runtime_seconds);
}

std::vector<adaptive_pivot_g2::CandidateObjective>
AdaptivePivotG2Smoother::parameterize_common_window(
  std::vector<TransitionState> & evaluations,
  double common_trim,
  std::map<std::string, std::size_t> & rejection_counts,
  double & fastest_time,
  std::size_t & feasible_count,
  double & common_entry_speed,
  double & common_exit_speed) const
{
  std::vector<adaptive_pivot_g2::CandidateObjective> objectives;
  objectives.reserve(evaluations.size());
  fastest_time = std::numeric_limits<double>::infinity();
  feasible_count = 0U;
  common_entry_speed = maximum_straight_speed(limits_);
  common_exit_speed = maximum_straight_speed(limits_);

  // Every local alternative must be timed from the same attainable boundary
  // state.  Solve that state as the fixed point of all candidate profiles;
  // requiring cruise speed at a short corner window incorrectly rejects
  // otherwise feasible low-curvature transitions.
  for (int iteration = 0; iteration < 40; ++iteration) {
    double next_entry_speed = common_entry_speed;
    double next_exit_speed = common_exit_speed;
    bool any_valid = false;
    for (std::size_t index = 0; index < evaluations.size(); ++index) {
      const auto profile = adaptive_pivot_g2::parameterize_transition_window(
        evaluations[index].candidate, common_trim, limits_,
        common_entry_speed, common_exit_speed,
        transition_options_.sample_spacing);
      if (!profile.valid) {
        continue;
      }
      any_valid = true;
      next_entry_speed = std::min(
        next_entry_speed, profile.linear_speed.front());
      next_exit_speed = std::min(
        next_exit_speed, profile.linear_speed.back());
    }
    if (!any_valid) {
      break;
    }
    const bool converged =
      std::abs(next_entry_speed - common_entry_speed) <= 1.0e-8 &&
      std::abs(next_exit_speed - common_exit_speed) <= 1.0e-8;
    common_entry_speed = next_entry_speed;
    common_exit_speed = next_exit_speed;
    if (converged) {
      break;
    }
  }

  // Recompute once at the final shared caps so the stored time and angular
  // metrics all correspond to the exact same boundary conditions.
  for (std::size_t index = 0; index < evaluations.size(); ++index) {
    auto & evaluation = evaluations[index];
    const auto profile = adaptive_pivot_g2::parameterize_transition_window(
      evaluation.candidate, common_trim, limits_,
      common_entry_speed, common_exit_speed,
      transition_options_.sample_spacing);
    if (!profile.valid) {
      evaluation.common_window_time = std::numeric_limits<double>::infinity();
      ++rejection_counts[profile.rejection_reason];
      continue;
    }
    if (std::abs(profile.linear_speed.front() - common_entry_speed) > 1.0e-6 ||
      std::abs(profile.linear_speed.back() - common_exit_speed) > 1.0e-6)
    {
      evaluation.common_window_time = std::numeric_limits<double>::infinity();
      ++rejection_counts["common_window_boundary_state_not_converged"];
      continue;
    }
    ++feasible_count;
    evaluation.common_window_time = profile.total_time;
    evaluation.max_abs_angular_speed = 0.0;
    for (const double angular_speed : profile.angular_speed) {
      evaluation.max_abs_angular_speed = std::max(
        evaluation.max_abs_angular_speed, std::abs(angular_speed));
    }
    evaluation.objective_cost = adaptive_pivot_g2::stable_candidate_cost(
      evaluation.peak_cost, evaluation.max_abs_angular_speed,
      evaluation.candidate.curvature_energy, limits_.max_angular_speed,
      curvature_energy_scale_, selection_weights_);
    fastest_time = std::min(fastest_time, evaluation.common_window_time);
    const double clearance_proxy = 1.0 - std::clamp(
      evaluation.peak_cost /
      static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE),
      0.0, 1.0);
    objectives.push_back(
      {index, evaluation.common_window_time, clearance_proxy,
        evaluation.max_abs_angular_speed,
        evaluation.candidate.curvature_energy});
  }
  return objectives;
}

nav_msgs::msg::Path AdaptivePivotG2Smoother::build_output_path(
  const std::vector<Vec2> & points,
  const std::vector<CornerDecision> & decisions,
  const std_msgs::msg::Header & header,
  const geometry_msgs::msg::Quaternion & goal_orientation,
  bool force_pivot) const
{
  nav_msgs::msg::Path output;
  output.header = header;
  const double first_heading = std::atan2(
    points[1].y - points[0].y, points[1].x - points[0].x);
  output.poses.push_back(make_pose(points.front(), first_heading, header));
  for (const auto & decision : decisions) {
    if (decision.pass_through) {
      append_line(output.poses, decision.vertex, output_spacing_, header);
    } else if (decision.use_transition && !force_pivot) {
      append_line(
        output.poses, decision.transition.samples.front().position,
        output_spacing_, header);
      for (std::size_t index = 1; index < decision.transition.samples.size(); ++index) {
        const auto & sample = decision.transition.samples[index];
        output.poses.push_back(make_pose(sample.position, sample.heading, header));
      }
    } else {
      const double incoming_heading = std::atan2(
        decision.incoming.y, decision.incoming.x);
      const double outgoing_heading = std::atan2(
        decision.outgoing.y, decision.outgoing.x);
      append_line(output.poses, decision.vertex, output_spacing_, header);
      output.poses.back() = make_pose(
        decision.vertex, incoming_heading, header);
      output.poses.push_back(make_pose(
          decision.vertex, outgoing_heading, header));
    }
  }
  append_line(output.poses, points.back(), output_spacing_, header);
  output.poses.back().pose.orientation = goal_orientation;
  return output;
}

bool AdaptivePivotG2Smoother::stitched_timing_is_valid(
  const std::vector<CornerDecision> & decisions,
  const nav_msgs::msg::Path & candidate_path,
  bool force_pivot,
  double effective_segment_margin,
  std::string * rejection_reason) const
{
  const auto reject = [rejection_reason](const std::string & reason) {
      if (rejection_reason != nullptr) {
        *rejection_reason = reason;
      }
      return false;
    };
  for (std::size_t index = 0; index < decisions.size(); ++index) {
    const auto & decision = decisions[index];
    if (decision.pass_through) {
      continue;
    } else if (decision.use_transition && !force_pivot) {
      // Every selected transition reached this point through one continuous
      // common-window profile, including any straight context needed to make
      // candidate trim distances directly comparable.
      if (!std::isfinite(decision.transition_time) ||
        decision.transition_time < 0.0 ||
        decision.transition.samples.size() < 2U)
      {
        return reject("transition_profile_" + std::to_string(index));
      }
    } else {
      const double rotation_time =
        adaptive_pivot_g2::minimum_rotation_time(
        decision.turn_angle, limits_.max_angular_speed,
        limits_.max_angular_acceleration);
      if (!std::isfinite(rotation_time)) {
        return reject("pivot_rotation_" + std::to_string(index));
      }
    }
  }
  for (std::size_t index = 1; index < decisions.size(); ++index) {
    const double left_trim =
      decisions[index - 1U].use_transition && !force_pivot ?
      decisions[index - 1U].transition.trim_distance : 0.0;
    const double right_trim =
      decisions[index].use_transition && !force_pivot ?
      decisions[index].transition.trim_distance : 0.0;
    const double available = adaptive_pivot_g2::distance(
      decisions[index - 1U].vertex, decisions[index].vertex) -
      left_trim - right_trim;
    if (available + kEpsilon < effective_segment_margin ||
      !std::isfinite(adaptive_pivot_g2::minimum_translation_time(
        available, 0.0, 0.0, maximum_straight_speed(limits_),
        limits_.max_linear_acceleration,
        limits_.max_linear_deceleration)))
    {
      return reject("shared_segment_" + std::to_string(index - 1U));
    }
  }
  for (std::size_t index = 1U; index < candidate_path.poses.size(); ++index) {
    const Vec2 previous = position_of(candidate_path.poses[index - 1U]);
    const Vec2 current = position_of(candidate_path.poses[index]);
    const double translation =
      adaptive_pivot_g2::distance(previous, current);
    if (translation > kEpsilon) {
      const double translation_time =
        adaptive_pivot_g2::minimum_translation_time(
        translation, 0.0, 0.0, maximum_straight_speed(limits_),
        limits_.max_linear_acceleration,
        limits_.max_linear_deceleration);
      if (!std::isfinite(translation_time)) {
        return reject("translation_segment_" + std::to_string(index - 1U));
      }
    } else {
      const auto & previous_orientation =
        candidate_path.poses[index - 1U].pose.orientation;
      const auto & current_orientation =
        candidate_path.poses[index].pose.orientation;
      const double previous_yaw = 2.0 * std::atan2(
        previous_orientation.z, previous_orientation.w);
      const double current_yaw = 2.0 * std::atan2(
        current_orientation.z, current_orientation.w);
      const double yaw_change = std::atan2(
        std::sin(current_yaw - previous_yaw),
        std::cos(current_yaw - previous_yaw));
      const double rotation_time =
        adaptive_pivot_g2::minimum_rotation_time(
        yaw_change, limits_.max_angular_speed,
        limits_.max_angular_acceleration);
      if (std::abs(yaw_change) <= kEpsilon ||
        !std::isfinite(rotation_time))
      {
        return reject("duplicate_segment_" + std::to_string(index - 1U));
      }
    }
  }
  if (rejection_reason != nullptr) {
    rejection_reason->clear();
  }
  return true;
}

bool AdaptivePivotG2Smoother::smooth(
  nav_msgs::msg::Path & path,
  const rclcpp::Duration & max_time)
{
  if (path.poses.size() < 2) {
    throw nav2_core::InvalidPath("Pivot-G2 needs at least two path poses");
  }
  if (!costmap_subscriber_) {
    throw nav2_core::FailedToSmoothPath("Pivot-G2 has no costmap subscriber");
  }
  const auto costmap = costmap_subscriber_->getCostmap();
  if (!costmap) {
    throw nav2_core::FailedToSmoothPath("Pivot-G2 has not received a costmap");
  }
  const auto started = std::chrono::steady_clock::now();
  const auto timed_out = [&]() {
      const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - started;
      return elapsed.count() > max_time.seconds();
    };

  std::vector<geometry_msgs::msg::Point> footprint;
  std_msgs::msg::Header footprint_header;
  if (!footprint_subscriber_ ||
    !footprint_subscriber_->getFootprintInRobotFrame(footprint, footprint_header) ||
    footprint.size() < 3)
  {
    footprint = fallback_footprint_;
    RCLCPP_WARN(logger_, "Using configured fallback footprint for Pivot-G2 collision checks");
  }
  CollisionChecker collision_checker(costmap);

  std::vector<Vec2> points = remove_duplicate_points(path);
  if (points.size() < 2) {
    throw nav2_core::InvalidPath("Pivot-G2 input collapses to fewer than two positions");
  }
  const std::size_t input_point_count = points.size();
  if (line_of_sight_pruning_) {
    points = prune_line_of_sight(
      points, costmap, collision_checker, footprint, max_footprint_cost_);
  }
  const auto conditioning = condition_planner_path(
    points, path_conditioning_max_deviation_,
    path_conditioning_resolution_ratio_, oscillation_maximum_span_,
    oscillation_maximum_deviation_, oscillation_deviation_resolution_ratio_,
    oscillation_minimum_turn_angle_, oscillation_minimum_sign_changes_,
    costmap, collision_checker, footprint, max_footprint_cost_);
  points = validated_conditioned_points(conditioning);
  if (timed_out()) {
    return false;
  }

  std::vector<CornerDecision> decisions;
  decisions.reserve(points.size() > 2 ? points.size() - 2 : 0);
  std::map<std::string, std::size_t> rejection_counts;
  adaptive_pivot_g2::PathOptimizationResult path_optimization;
  const double automatic_segment_margin = std::max({
      output_spacing_, 2.0 * transition_options_.sample_spacing,
      costmap->getResolution()});
  const double effective_segment_margin = segment_margin_override_ > 0.0 ?
    segment_margin_override_ : automatic_segment_margin;
  for (std::size_t index = 1; index + 1 < points.size(); ++index) {
    const Vec2 incoming_vector = points[index] - points[index - 1];
    const Vec2 outgoing_vector = points[index + 1] - points[index];
    const double incoming_length = adaptive_pivot_g2::norm(incoming_vector);
    const double outgoing_length = adaptive_pivot_g2::norm(outgoing_vector);
    const Vec2 incoming = normalized(incoming_vector);
    const Vec2 outgoing = normalized(outgoing_vector);
    const double turn_angle = signed_angle(incoming, outgoing);

    CornerDecision decision;
    decision.vertex = points[index];
    decision.incoming = incoming;
    decision.outgoing = outgoing;
    decision.turn_angle = turn_angle;
    const double incoming_heading = std::atan2(incoming.y, incoming.x);
    decision.pass_through = std::abs(turn_angle) < corner_angle_threshold_;
    if (decision.pass_through) {
      // A sub-threshold heading change is not an intentional stop-and-rotate
      // marker. Preserve the vertex as one translational pose so the controller
      // cannot silently discard a duplicate-position pair it does not classify
      // as a pivot.
      decision.pivot_safe = true;
      decision.optimization_states.push_back({true, 0.0, 0.0, 0U});
      decisions.push_back(std::move(decision));
      continue;
    }
    decision.pivot_safe = pivot_is_safe(
      decision.vertex, incoming_heading, turn_angle,
      costmap, collision_checker, footprint, max_footprint_cost_);

    std::vector<TransitionState> evaluations;
    double common_trim = 0.0;
    const CornerInput corner{
      points[index], incoming, outgoing, incoming_length, outgoing_length};
    const double meaningful_trim_resolution = 0.5 * std::min(
        transition_options_.sample_spacing, costmap->getResolution());
    const auto search = adaptive_pivot_g2::search_trim_distance(
        std::abs(turn_angle), std::min(incoming_length, outgoing_length),
        meaningful_trim_resolution, adaptive_search_options_,
      [&](double trim_distance) {
        if (timed_out()) {
          return adaptive_pivot_g2::SearchEvaluation{
          adaptive_pivot_g2::SearchSampleStatus::kInfeasible,
          std::numeric_limits<double>::infinity(), 0U, "time_budget"};
        }
        TransitionCandidate candidate =
        adaptive_pivot_g2::generate_quintic_transition_for_trim(
            corner, limits_, transition_options_, trim_distance);
        if (!candidate.valid) {
          return adaptive_pivot_g2::SearchEvaluation{
          adaptive_pivot_g2::SearchSampleStatus::kInfeasible,
          std::numeric_limits<double>::infinity(), 0U,
          candidate.rejection_reason};
        }
        double peak_cost = 0.0;
        if (!transition_is_safe(
              candidate, costmap, collision_checker, footprint,
              max_footprint_cost_, peak_cost))
        {
          return adaptive_pivot_g2::SearchEvaluation{
          adaptive_pivot_g2::SearchSampleStatus::kUnsafe,
          std::numeric_limits<double>::infinity(), 0U,
          "swept_footprint"};
        }
        const auto profile = adaptive_pivot_g2::parameterize_time(
            candidate.samples, limits_, maximum_straight_speed(limits_),
            maximum_straight_speed(limits_));
        if (!profile.valid) {
          return adaptive_pivot_g2::SearchEvaluation{
          adaptive_pivot_g2::SearchSampleStatus::kInfeasible,
          std::numeric_limits<double>::infinity(), 0U,
          profile.rejection_reason};
        }
        double max_abs_angular_speed = 0.0;
        for (const double angular_speed : profile.angular_speed) {
          max_abs_angular_speed = std::max(
              max_abs_angular_speed, std::abs(angular_speed));
        }
        const double objective_cost =
        adaptive_pivot_g2::stable_candidate_cost(
            peak_cost, max_abs_angular_speed, candidate.curvature_energy,
            limits_.max_angular_speed, curvature_energy_scale_,
            selection_weights_);
        const std::size_t payload_index = evaluations.size();
        evaluations.push_back(
          {std::move(candidate), profile.total_time,
            std::numeric_limits<double>::infinity(), peak_cost,
            max_abs_angular_speed, objective_cost});
        return adaptive_pivot_g2::SearchEvaluation{
        adaptive_pivot_g2::SearchSampleStatus::kFeasible,
        objective_cost, payload_index, ""};
        });
    decision.evaluation_count = search.samples.size();
    decision.feasible_count = search.feasible_count;
    if (search.valid_domain) {
      const double half_angle_tangent = std::tan(0.5 * std::abs(turn_angle));
      decision.minimum_search_trim = search.minimum_trim;
      decision.maximum_search_trim = search.maximum_trim;
      decision.minimum_search_radius =
        search.minimum_trim / half_angle_tangent;
      decision.maximum_search_radius =
        search.maximum_trim / half_angle_tangent;
    }
    for (const auto & sample : search.samples) {
      if (sample.status != adaptive_pivot_g2::SearchSampleStatus::kFeasible) {
        ++rejection_counts[
          sample.rejection_reason.empty() ? "unspecified" :
          sample.rejection_reason];
      }
    }
    if (timed_out()) {
      return false;
    }

    for (const auto & evaluation : evaluations) {
      common_trim = std::max(
        common_trim, evaluation.candidate.trim_distance);
    }
    double fastest_time = std::numeric_limits<double>::infinity();
    std::size_t window_feasible_count = 0U;
    double common_entry_speed = maximum_straight_speed(limits_);
    double common_exit_speed = maximum_straight_speed(limits_);
    parameterize_common_window(
      evaluations, common_trim, rejection_counts, fastest_time,
      window_feasible_count, common_entry_speed, common_exit_speed);
    decision.pivot_time = common_trim > kEpsilon ?
      adaptive_pivot_g2::estimate_pivot_window_time(
      common_trim, turn_angle, limits_,
      common_entry_speed, common_exit_speed) :
      adaptive_pivot_g2::minimum_rotation_time(
      turn_angle, limits_.max_angular_speed, limits_.max_angular_acceleration);

    const bool transition_branch_open = std::isfinite(fastest_time) &&
      (!decision.pivot_safe ||
      fastest_time + delta_time_selection_ < decision.pivot_time);
    double effective_slack = time_competitive_slack_;
    if (transition_branch_open && decision.pivot_safe) {
      effective_slack = std::min(
        effective_slack,
        std::max(
          0.0, decision.pivot_time - delta_time_selection_ - fastest_time));
    }
    decision.candidate_count = window_feasible_count;
    std::vector<std::size_t> competitive;
    if (transition_branch_open) {
      for (std::size_t evaluation_index = 0;
        evaluation_index < evaluations.size(); ++evaluation_index)
      {
        if (evaluations[evaluation_index].common_window_time <=
          fastest_time + effective_slack + kEpsilon)
        {
          competitive.push_back(evaluation_index);
        }
      }
    }
    decision.competitive_count = competitive.size();
    std::sort(
        competitive.begin(), competitive.end(),
      [&evaluations](std::size_t lhs, std::size_t rhs) {
        if (std::abs(
              evaluations[lhs].objective_cost -
              evaluations[rhs].objective_cost) > kEpsilon)
        {
          return evaluations[lhs].objective_cost <
                 evaluations[rhs].objective_cost;
        }
        return evaluations[lhs].candidate.trim_distance <
               evaluations[rhs].candidate.trim_distance;
        });

    std::vector<std::size_t> retained;
    const auto retain = [&](std::size_t evaluation_index) {
        if (retained.size() < retained_candidates_per_corner_ &&
          std::find(retained.begin(), retained.end(), evaluation_index) ==
          retained.end())
        {
          retained.push_back(evaluation_index);
        }
      };
    if (!competitive.empty()) {
      retain(competitive.front());
      const auto trim_order = std::minmax_element(
          competitive.begin(), competitive.end(),
        [&evaluations](std::size_t lhs, std::size_t rhs) {
          return evaluations[lhs].candidate.trim_distance <
                 evaluations[rhs].candidate.trim_distance;
          });
      retain(*trim_order.first);
      retain(*trim_order.second);
      for (const std::size_t evaluation_index : competitive) {
        retain(evaluation_index);
      }
    }
    for (const std::size_t evaluation_index : retained) {
      const auto & evaluation = evaluations[evaluation_index];
      decision.optimization_states.push_back(
        {false, evaluation.candidate.trim_distance,
          evaluation.objective_cost, evaluation_index});
    }
    if (decision.pivot_safe) {
      decision.optimization_states.push_back(
        {true, 0.0, transition_branch_open ? 1.0 : 0.0, 0U});
    }
    if (decision.optimization_states.empty()) {
      RCLCPP_WARN(
          logger_,
          "Adaptive Pivot-G2 corner %zu has no safe state "
          "(pivot_safe=%d, feasible=%zu, competitive=%zu)",
          index, decision.pivot_safe, decision.feasible_count,
          decision.competitive_count);
      throw nav2_core::FailedToSmoothPath(
                "Adaptive Pivot-G2 found no safe pivot or G2 state");
    }
    decision.transition_states = std::move(evaluations);
    decisions.push_back(std::move(decision));
  }

  std::vector<std::vector<adaptive_pivot_g2::CornerState>> corner_states;
  std::vector<double> shared_segment_lengths;
  std::vector<double> segment_margins;
  corner_states.reserve(decisions.size());
  for (std::size_t index = 0; index < decisions.size(); ++index) {
    corner_states.push_back(decisions[index].optimization_states);
    if (index > 0U) {
      shared_segment_lengths.push_back(adaptive_pivot_g2::distance(
            decisions[index - 1U].vertex, decisions[index].vertex));
      segment_margins.push_back(effective_segment_margin);
    }
  }
  path_optimization = adaptive_pivot_g2::optimize_corner_states(
      corner_states, shared_segment_lengths, segment_margins);
  if (!path_optimization.valid) {
    const double minimum_shared_segment = shared_segment_lengths.empty() ?
      0.0 : *std::min_element(
        shared_segment_lengths.begin(), shared_segment_lengths.end());
    const std::size_t segments_below_margin = static_cast<std::size_t>(
      std::count_if(
          shared_segment_lengths.begin(), shared_segment_lengths.end(),
        [effective_segment_margin](double length) {
          return length + kEpsilon < effective_segment_margin;
          }));
    RCLCPP_WARN(
        logger_,
        "Adaptive Pivot-G2 DP found no compatible sequence "
        "(corners=%zu, states=%zu, compatible_edges=%zu, "
        "min_segment=%.9f, margin=%.9f, segments_below_margin=%zu)",
        decisions.size(), path_optimization.state_count,
        path_optimization.compatible_edge_count, minimum_shared_segment,
        effective_segment_margin, segments_below_margin);
    throw nav2_core::FailedToSmoothPath(
              "Adaptive Pivot-G2 has no globally compatible corner-state sequence");
  }
  for (std::size_t index = 0; index < decisions.size(); ++index) {
    const auto & state = decisions[index].optimization_states[
      path_optimization.selected_state_indices[index]];
    if (!state.pivot) {
      const auto & evaluation =
        decisions[index].transition_states[state.payload_index];
      decisions[index].use_transition = true;
      decisions[index].transition = evaluation.candidate;
      decisions[index].transition_time = evaluation.common_window_time;
      decisions[index].selection_score = evaluation.objective_cost;
      decisions[index].clearance_proxy = 1.0 - std::clamp(
          evaluation.peak_cost /
          static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE),
          0.0, 1.0);
    }
  }

  const auto goal_orientation = path.poses.back().pose.orientation;
  nav_msgs::msg::Path output = build_output_path(
    points, decisions, path.header, goal_orientation, false);
  std::string fallback_status{"none"};
  const auto endpoints_are_preserved = [&points](const nav_msgs::msg::Path & candidate) {
      return !candidate.poses.empty() &&
             adaptive_pivot_g2::distance(
        position_of(candidate.poses.front()), points.front()) <= 1.0e-9 &&
             adaptive_pivot_g2::distance(
        position_of(candidate.poses.back()), points.back()) <= 1.0e-9;
    };
  std::string selected_stitch_rejection;
  std::string selected_timing_rejection;
  const bool selected_endpoints_valid = endpoints_are_preserved(output);
  const bool selected_timing_valid = stitched_timing_is_valid(
    decisions, output, false, effective_segment_margin,
    &selected_timing_rejection);
  bool selected_sweep_valid = stitched_path_is_safe(
    output, costmap, collision_checker, footprint, max_footprint_cost_,
    corner_angle_threshold_, &selected_stitch_rejection);
  if (selected_sweep_valid &&
    !terminal_rotation_is_safe(
      output, costmap, collision_checker, footprint, max_footprint_cost_))
  {
    selected_sweep_valid = false;
    selected_stitch_rejection = "unsafe_terminal_rotation";
  }
  selected_stitch_rejection = resolve_stitch_rejection(
    selected_endpoints_valid, selected_timing_valid,
    selected_timing_rejection, selected_stitch_rejection);
  const bool selected_stitch_valid =
    selected_endpoints_valid && selected_timing_valid && selected_sweep_valid;
  if (!selected_stitch_valid) {
    const bool all_pivots_safe = std::all_of(
      decisions.begin(), decisions.end(),
      [](const CornerDecision & decision) {return decision.pivot_safe;});
    nav_msgs::msg::Path pivot_fallback = build_output_path(
      points, decisions, path.header, goal_orientation, true);
    std::string fallback_stitch_rejection;
    std::string fallback_timing_rejection;
    const bool fallback_endpoints_valid = endpoints_are_preserved(pivot_fallback);
    const bool fallback_timing_valid = stitched_timing_is_valid(
      decisions, pivot_fallback, true, effective_segment_margin,
      &fallback_timing_rejection);
    bool fallback_sweep_valid = stitched_path_is_safe(
      pivot_fallback, costmap, collision_checker, footprint,
      max_footprint_cost_, corner_angle_threshold_, &fallback_stitch_rejection);
    if (fallback_sweep_valid &&
      !terminal_rotation_is_safe(
        pivot_fallback, costmap, collision_checker, footprint,
        max_footprint_cost_))
    {
      fallback_sweep_valid = false;
      fallback_stitch_rejection = "unsafe_terminal_rotation";
    }
    if (!all_pivots_safe || !fallback_endpoints_valid ||
      !fallback_timing_valid || !fallback_sweep_valid)
    {
      std::ostringstream reason;
      reason << "Adaptive Pivot-G2 stitched path and all-pivot fallback are unsafe"
             << " (selected: endpoints=" << selected_endpoints_valid
             << ", timing=" << selected_timing_valid
             << ", sweep=" << selected_sweep_valid
             << ", reason=" << selected_stitch_rejection
             << "; fallback: pivots=" << all_pivots_safe
             << ", endpoints=" << fallback_endpoints_valid
             << ", timing=" << fallback_timing_valid
             << ", timing_reason=" << fallback_timing_rejection
             << ", sweep=" << fallback_sweep_valid
             << ", reason=" << fallback_stitch_rejection << ')';
      RCLCPP_WARN(logger_, "%s", reason.str().c_str());
      throw nav2_core::FailedToSmoothPath(
              reason.str());
    }
    output = std::move(pivot_fallback);
    for (auto & decision : decisions) {
      decision.use_transition = false;
    }
    fallback_status = "all_pivot";
  }
  if (timed_out()) {
    return false;
  }
  path = std::move(output);

  const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - started;
  publish_diagnostics(
    decisions, path_optimization, rejection_counts, conditioning.result,
    input_point_count, conditioning.maximum_deviation,
    conditioning.oscillation_maximum_deviation,
    effective_segment_margin, fallback_status, selected_stitch_rejection,
    path.poses.size(), elapsed.count());
  return true;
}

}  // namespace adaptive_pivot_g2_nav2

PLUGINLIB_EXPORT_CLASS(
  adaptive_pivot_g2_nav2::AdaptivePivotG2Smoother,
  nav2_core::Smoother)
