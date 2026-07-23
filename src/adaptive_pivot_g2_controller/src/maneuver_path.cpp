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

#include <cmath>
#include <stdexcept>
#include <vector>

#include "tf2/utils.hpp"

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
