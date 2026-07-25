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

#ifndef ADAPTIVE_PIVOT_G2_CONTROLLER__MANEUVER_PATH_HPP_
#define ADAPTIVE_PIVOT_G2_CONTROLLER__MANEUVER_PATH_HPP_

#include <optional>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace adaptive_pivot_g2_controller
{

struct ManeuverSegment
{
  nav_msgs::msg::Path path;
  bool ends_with_pivot{false};
  geometry_msgs::msg::PoseStamped pivot_target;
};

struct TerminalDriveGeometry
{
  double direction{1.0};
  double heading_error{0.0};
};

/// Split a path at duplicate-position poses whose headings encode a pivot.
std::vector<ManeuverSegment> split_path_at_pivots(
  const nav_msgs::msg::Path & path,
  double duplicate_position_tolerance,
  double minimum_pivot_angle);

/// Select forward or reverse motion requiring at most a 90 degree turn.
TerminalDriveGeometry shortest_terminal_drive(double bearing_error);

/// Heading from the first path point toward an arc-length preview point.
std::optional<double> preview_path_heading(
  const nav_msgs::msg::Path & path,
  double preview_distance);

/// Maximum angular speed that can stop inside the remaining yaw error.
double angular_braking_speed_limit(
  double heading_error,
  double yaw_tolerance,
  double effective_angular_deceleration,
  double maximum_angular_speed);

}  // namespace adaptive_pivot_g2_controller

#endif  // ADAPTIVE_PIVOT_G2_CONTROLLER__MANEUVER_PATH_HPP_
