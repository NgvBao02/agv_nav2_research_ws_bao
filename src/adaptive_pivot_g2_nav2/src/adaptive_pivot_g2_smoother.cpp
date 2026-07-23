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
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "adaptive_pivot_g2/candidate_selection.hpp"
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
  double & clearance_proxy)
{
  double maximum_proximity_cost = 0.0;
  for (const auto & sample : candidate.samples) {
    double proximity_cost = 0.0;
    if (!pose_is_safe(
        sample.position, sample.heading, costmap, checker, footprint,
        max_footprint_cost, &proximity_cost))
    {
      return false;
    }
    maximum_proximity_cost = std::max(maximum_proximity_cost, proximity_cost);
  }
  // Inflation and footprint costs are monotonic with obstacle proximity.  The
  // normalized inverse is used only to rank already collision-safe candidates;
  // it is intentionally named a proxy rather than a metric distance.
  clearance_proxy = 1.0 - std::clamp(
    maximum_proximity_cost / static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE),
    0.0, 1.0);
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

std::vector<Vec2> reduce_small_angles(
  const std::vector<Vec2> & input,
  double angle_threshold)
{
  if (input.size() < 3) {
    return input;
  }
  std::vector<Vec2> output;
  output.reserve(input.size());
  output.push_back(input.front());
  for (std::size_t index = 1; index + 1 < input.size(); ++index) {
    const Vec2 incoming = normalized(input[index] - output.back());
    const Vec2 outgoing = normalized(input[index + 1] - input[index]);
    if (adaptive_pivot_g2::norm(incoming) <= kEpsilon ||
      adaptive_pivot_g2::norm(outgoing) <= kEpsilon)
    {
      continue;
    }
    if (std::abs(signed_angle(incoming, outgoing)) >= angle_threshold) {
      output.push_back(input[index]);
    }
  }
  output.push_back(input.back());
  return output;
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
  radius_candidates_ = declare_and_get<std::vector<double>>(
    node, prefix + "radius_candidates",
    {0.20, 0.30, 0.40, 0.50, 0.60, 0.75, 1.00, 1.25, 1.50});
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
  output_spacing_ = declare_and_get<double>(node, prefix + "output_spacing", 0.05);
  const int64_t max_footprint_cost = declare_and_get<int64_t>(
    node, prefix + "max_footprint_cost", 200);
  max_footprint_cost_ = static_cast<unsigned char>(
    std::clamp<int64_t>(max_footprint_cost, 0, 252));
  line_of_sight_pruning_ = declare_and_get<bool>(
    node, prefix + "line_of_sight_pruning", false);

  const double selection_weight_sum = selection_weights_.clearance +
    selection_weights_.angular_speed + selection_weights_.curvature_energy;
  if (radius_candidates_.empty() ||
    std::any_of(
      radius_candidates_.begin(), radius_candidates_.end(),
      [](double radius) {return !std::isfinite(radius) || radius <= 0.0;}) ||
    !std::isfinite(delta_time_selection_) || delta_time_selection_ < 0.0 ||
    !std::isfinite(time_competitive_slack_) || time_competitive_slack_ < 0.0 ||
    !std::isfinite(selection_weights_.clearance) || selection_weights_.clearance < 0.0 ||
    !std::isfinite(selection_weights_.angular_speed) ||
    selection_weights_.angular_speed < 0.0 ||
    !std::isfinite(selection_weights_.curvature_energy) ||
    selection_weights_.curvature_energy < 0.0 ||
    !std::isfinite(selection_weight_sum) || selection_weight_sum <= kEpsilon)
  {
    throw nav2_core::FailedToSmoothPath("Pivot-G2 candidate selection parameters are invalid");
  }

  transition_options_.control_fraction = declare_and_get<double>(
    node, prefix + "bezier_control_fraction", 0.35);
  transition_options_.max_trim_fraction = declare_and_get<double>(
    node, prefix + "max_trim_fraction", 0.45);
  transition_options_.sample_spacing = declare_and_get<double>(
    node, prefix + "sample_spacing", 0.02);
  limits_.wheel_separation = declare_and_get<double>(
    node, prefix + "wheel_separation", 0.2548);
  limits_.max_linear_speed = declare_and_get<double>(
    node, prefix + "max_linear_speed", 0.08);
  limits_.max_angular_speed = declare_and_get<double>(
    node, prefix + "max_angular_speed", 0.425);
  limits_.max_wheel_speed = declare_and_get<double>(
    node, prefix + "max_wheel_speed", 0.17);
  limits_.max_linear_acceleration = declare_and_get<double>(
    node, prefix + "max_linear_acceleration", 0.20);
  limits_.max_linear_deceleration = declare_and_get<double>(
    node, prefix + "max_linear_deceleration", 0.25);
  limits_.max_angular_acceleration = declare_and_get<double>(
    node, prefix + "max_angular_acceleration", 0.80);

  const double footprint_length = declare_and_get<double>(
    node, prefix + "fallback_footprint_length", 0.44);
  const double footprint_width = declare_and_get<double>(
    node, prefix + "fallback_footprint_width", 0.34);
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
  RCLCPP_INFO(logger_, "Configured adaptive Pivot-G2 smoother as '%s'", plugin_name_.c_str());
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
  if (line_of_sight_pruning_) {
    points = prune_line_of_sight(
      points, costmap, collision_checker, footprint, max_footprint_cost_);
  }
  points = reduce_small_angles(points, corner_angle_threshold_);
  if (timed_out()) {
    return false;
  }

  std::vector<CornerDecision> decisions;
  decisions.reserve(points.size() > 2 ? points.size() - 2 : 0);
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
    const bool pivot_safe = pivot_is_safe(
      decision.vertex, incoming_heading, turn_angle,
      costmap, collision_checker, footprint, max_footprint_cost_);

    struct Evaluation
    {
      TransitionCandidate candidate;
      double curve_time{std::numeric_limits<double>::infinity()};
      double common_window_time{std::numeric_limits<double>::infinity()};
      double clearance_proxy{0.0};
      double max_abs_angular_speed{std::numeric_limits<double>::infinity()};
    };
    std::vector<Evaluation> evaluations;
    double common_trim = 0.0;
    for (const double radius : radius_candidates_) {
      transition_options_.design_radius = radius;
      TransitionCandidate candidate = adaptive_pivot_g2::generate_quintic_transition(
        CornerInput{
          points[index], incoming, outgoing, incoming_length, outgoing_length},
        limits_, transition_options_);
      double clearance_proxy = 0.0;
      if (!candidate.valid || !transition_is_safe(
          candidate, costmap, collision_checker, footprint, max_footprint_cost_,
          clearance_proxy))
      {
        continue;
      }
      const auto profile = adaptive_pivot_g2::parameterize_time(
        candidate.samples, limits_, limits_.max_linear_speed, limits_.max_linear_speed);
      if (!profile.valid) {
        continue;
      }
      common_trim = std::max(common_trim, candidate.trim_distance);
      double max_abs_angular_speed = 0.0;
      for (const double angular_speed : profile.angular_speed) {
        max_abs_angular_speed = std::max(max_abs_angular_speed, std::abs(angular_speed));
      }
      evaluations.push_back(
        {std::move(candidate), profile.total_time, std::numeric_limits<double>::infinity(),
          clearance_proxy, max_abs_angular_speed});
      if (timed_out()) {
        return false;
      }
    }

    decision.pivot_time = common_trim > kEpsilon ?
      adaptive_pivot_g2::estimate_pivot_window_time(
      common_trim, turn_angle, limits_,
      limits_.max_linear_speed, limits_.max_linear_speed) :
      adaptive_pivot_g2::minimum_rotation_time(
      turn_angle, limits_.max_angular_speed, limits_.max_angular_acceleration);

    std::vector<adaptive_pivot_g2::CandidateObjective> objectives;
    objectives.reserve(evaluations.size());
    for (std::size_t candidate_index = 0; candidate_index < evaluations.size();
      ++candidate_index)
    {
      auto & evaluation = evaluations[candidate_index];
      const double straight_compensation =
        2.0 * (common_trim - evaluation.candidate.trim_distance) /
        limits_.max_linear_speed;
      evaluation.common_window_time = evaluation.curve_time + straight_compensation;
      objectives.push_back(
        {candidate_index, evaluation.common_window_time, evaluation.clearance_proxy,
          evaluation.max_abs_angular_speed, evaluation.candidate.curvature_energy});
    }

    const auto fastest = adaptive_pivot_g2::select_competitive_candidate(
      objectives, 0.0, selection_weights_);
    const bool transition_branch_open = fastest.valid &&
      (!pivot_safe || fastest.fastest_time + delta_time_selection_ < decision.pivot_time);
    double effective_slack = time_competitive_slack_;
    if (transition_branch_open && pivot_safe) {
      effective_slack = std::min(
        effective_slack,
        std::max(0.0, decision.pivot_time - delta_time_selection_ - fastest.fastest_time));
    }
    const auto selected = transition_branch_open ?
      adaptive_pivot_g2::select_competitive_candidate(
      objectives, effective_slack, selection_weights_) :
      adaptive_pivot_g2::CandidateSelection{};
    decision.candidate_count = evaluations.size();
    decision.competitive_count = selected.competitive_count;
    decision.transition_time = selected.selected_time;
    decision.selection_score = selected.valid ? selected.selected_score : 0.0;
    if (selected.valid) {
      auto & evaluation = evaluations[selected.candidate_index];
      decision.use_transition = true;
      decision.clearance_proxy = evaluation.clearance_proxy;
      decision.transition = std::move(evaluation.candidate);
    } else if (!pivot_safe) {
      throw nav2_core::FailedToSmoothPath(
              "Pivot-G2 found neither a swept-footprint-safe pivot nor G2 transition");
    }
    decisions.push_back(std::move(decision));
  }

  nav_msgs::msg::Path output;
  output.header = path.header;
  const double first_heading = std::atan2(
    points[1].y - points[0].y, points[1].x - points[0].x);
  output.poses.push_back(make_pose(points.front(), first_heading, path.header));
  for (const auto & decision : decisions) {
    if (decision.use_transition) {
      append_line(
        output.poses, decision.transition.samples.front().position,
        output_spacing_, path.header);
      for (std::size_t index = 1; index < decision.transition.samples.size(); ++index) {
        const auto & sample = decision.transition.samples[index];
        output.poses.push_back(make_pose(sample.position, sample.heading, path.header));
      }
    } else {
      const double incoming_heading = std::atan2(
        decision.incoming.y, decision.incoming.x);
      const double outgoing_heading = std::atan2(
        decision.outgoing.y, decision.outgoing.x);
      append_line(output.poses, decision.vertex, output_spacing_, path.header);
      output.poses.back() = make_pose(decision.vertex, incoming_heading, path.header);
      output.poses.push_back(make_pose(decision.vertex, outgoing_heading, path.header));
    }
  }
  append_line(output.poses, points.back(), output_spacing_, path.header);
  output.poses.back().pose.orientation = path.poses.back().pose.orientation;

  if (timed_out()) {
    return false;
  }
  path = std::move(output);

  std::size_t transition_count = 0;
  std::size_t candidate_count = 0;
  std::size_t competitive_count = 0;
  double score_sum = 0.0;
  for (const auto & decision : decisions) {
    transition_count += decision.use_transition ? 1U : 0U;
    candidate_count += decision.candidate_count;
    competitive_count += decision.competitive_count;
    score_sum += decision.use_transition ? decision.selection_score : 0.0;
  }
  const std::size_t pivot_count = decisions.size() - transition_count;
  const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - started;
  std::ostringstream diagnostics;
  diagnostics << "{\"method\":\"pivot_g2\",\"corners\":" << decisions.size()
              << ",\"g2_transitions\":" << transition_count
              << ",\"pivots\":" << pivot_count
              << ",\"valid_candidates\":" << candidate_count
              << ",\"competitive_candidates\":" << competitive_count
              << ",\"mean_transition_score\":"
              << (transition_count > 0U ? score_sum / static_cast<double>(transition_count) : 0.0)
              << ",\"max_footprint_cost\":" << static_cast<int>(max_footprint_cost_)
              << ",\"output_points\":" << path.poses.size()
              << ",\"runtime_s\":" << elapsed.count() << "}";
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
  adaptive_pivot_g2_nav2::AdaptivePivotG2Smoother,
  nav2_core::Smoother)
