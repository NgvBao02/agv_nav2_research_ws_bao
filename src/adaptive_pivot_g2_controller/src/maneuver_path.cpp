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

#include "adaptive_pivot_g2_controller/maneuver_path.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <vector>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace adaptive_pivot_g2_controller
{
namespace
{

double normalized_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double planar_distance(
  const geometry_msgs::msg::PoseStamped & first,
  const geometry_msgs::msg::PoseStamped & last)
{
  return std::hypot(
    last.pose.position.x - first.pose.position.x,
    last.pose.position.y - first.pose.position.y);
}

}  // namespace

TerminalDriveGeometry shortest_terminal_drive(double bearing_error)
{
  if (!std::isfinite(bearing_error)) {
    throw std::invalid_argument("terminal bearing error must be finite");
  }
  TerminalDriveGeometry output;
  output.heading_error = normalized_angle(bearing_error);
  if (std::cos(output.heading_error) < 0.0) {
    output.direction = -1.0;
    output.heading_error = normalized_angle(
      output.heading_error -
      std::copysign(3.14159265358979323846, output.heading_error));
  }
  return output;
}

std::optional<double> preview_path_heading(
  const nav_msgs::msg::Path & path,
  double preview_distance)
{
  if (!std::isfinite(preview_distance) || preview_distance <= 0.0) {
    throw std::invalid_argument(
            "path-heading preview distance must be finite and positive");
  }
  if (path.poses.size() < 2U) {
    return std::nullopt;
  }
  const auto & origin = path.poses.front().pose.position;
  double accumulated_distance = 0.0;
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    const auto & first = path.poses[index - 1U].pose.position;
    const auto & last = path.poses[index].pose.position;
    const double delta_x = last.x - first.x;
    const double delta_y = last.y - first.y;
    const double segment_length = std::hypot(delta_x, delta_y);
    if (segment_length <= 1.0e-12) {
      continue;
    }
    const double ratio = std::clamp(
      (preview_distance - accumulated_distance) / segment_length,
      0.0, 1.0);
    const double target_x = first.x + ratio * delta_x;
    const double target_y = first.y + ratio * delta_y;
    if (accumulated_distance + segment_length >= preview_distance) {
      return std::atan2(target_y - origin.y, target_x - origin.x);
    }
    accumulated_distance += segment_length;
  }
  const auto & endpoint = path.poses.back().pose.position;
  if (std::hypot(endpoint.x - origin.x, endpoint.y - origin.y) <= 1.0e-12) {
    return std::nullopt;
  }
  return std::atan2(endpoint.y - origin.y, endpoint.x - origin.x);
}

double angular_braking_speed_limit(
  double heading_error,
  double yaw_tolerance,
  double effective_angular_deceleration,
  double maximum_angular_speed)
{
  const std::array<double, 4> values = {
    heading_error, yaw_tolerance, effective_angular_deceleration,
    maximum_angular_speed};
  if (!std::all_of(
      values.begin(), values.end(),
      [](double value) {return std::isfinite(value);}) ||
    yaw_tolerance < 0.0 ||
    effective_angular_deceleration <= 0.0 ||
    maximum_angular_speed <= 0.0)
  {
    throw std::invalid_argument("angular braking inputs are invalid");
  }
  const double available_angle = std::max(
    0.0, std::abs(heading_error) - yaw_tolerance);
  return std::min(
    maximum_angular_speed,
    std::sqrt(2.0 * effective_angular_deceleration * available_angle));
}

std::vector<ManeuverSegment> split_path_at_pivots(
  const nav_msgs::msg::Path & path,
  double duplicate_position_tolerance,
  double minimum_pivot_angle)
{
  if (path.poses.size() < 2) {
    throw std::invalid_argument("maneuver-aware controller needs at least two path poses");
  }
  if (!std::isfinite(duplicate_position_tolerance) ||
    duplicate_position_tolerance < 0.0 ||
    !std::isfinite(minimum_pivot_angle) || minimum_pivot_angle < 0.0)
  {
    throw std::invalid_argument("pivot marker tolerances must be finite and non-negative");
  }

  std::vector<ManeuverSegment> segments;
  ManeuverSegment current;
  current.path.header = path.header;
  current.path.poses.push_back(path.poses.front());
  for (std::size_t index = 1; index < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1];
    const auto & pose = path.poses[index];
    const bool duplicate = planar_distance(previous, pose) <= duplicate_position_tolerance;
    const double heading_change = std::abs(normalized_angle(
        tf2::getYaw(pose.pose.orientation) - tf2::getYaw(previous.pose.orientation)));
    if (duplicate && heading_change >= minimum_pivot_angle) {
      current.ends_with_pivot = true;
      current.pivot_target = pose;
      segments.push_back(current);
      current.path.poses.clear();
      current.ends_with_pivot = false;
      current.pivot_target = geometry_msgs::msg::PoseStamped();
      current.path.header = path.header;
      current.path.poses.push_back(pose);
    } else if (!duplicate) {
      current.path.poses.push_back(pose);
    }
  }
  if (current.path.poses.size() >= 2 || segments.empty()) {
    segments.push_back(current);
  } else if (!segments.empty()) {
    // A terminal duplicate without further translation is the final heading
    // target of the preceding pivot, not a standalone trackable segment.
    segments.back().pivot_target = current.path.poses.front();
  }
  return segments;
}

}  // namespace adaptive_pivot_g2_controller
