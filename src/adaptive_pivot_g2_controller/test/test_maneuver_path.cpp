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

#include <cmath>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "adaptive_pivot_g2_controller/maneuver_path.hpp"

namespace adaptive_pivot_g2_controller
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped output;
  output.header.frame_id = "map";
  output.pose.position.x = x;
  output.pose.position.y = y;
  output.pose.orientation.z = std::sin(0.5 * yaw);
  output.pose.orientation.w = std::cos(0.5 * yaw);
  return output;
}

TEST(ManeuverPath, SplitsDuplicatePositionHeadingChange)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses = {
    pose(0.0, 0.0, 0.0), pose(1.0, 0.0, 0.0),
    pose(1.0, 0.0, 0.5 * kPi), pose(1.0, 1.0, 0.5 * kPi)};

  const auto segments = split_path_at_pivots(path, 1.0e-4, 0.05);

  ASSERT_EQ(segments.size(), 2U);
  EXPECT_TRUE(segments[0].ends_with_pivot);
  EXPECT_FALSE(segments[1].ends_with_pivot);
  EXPECT_EQ(segments[0].path.poses.size(), 2U);
  EXPECT_EQ(segments[1].path.poses.size(), 2U);
  EXPECT_NEAR(tf2::getYaw(segments[0].pivot_target.pose.orientation), 0.5 * kPi, 1.0e-12);
}

TEST(ManeuverPath, RemovesRedundantDuplicateWithoutPivot)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses = {
    pose(0.0, 0.0, 0.0), pose(1.0, 0.0, 0.0),
    pose(1.0, 0.0, 0.01), pose(2.0, 0.0, 0.0)};

  const auto segments = split_path_at_pivots(path, 1.0e-4, 0.05);

  ASSERT_EQ(segments.size(), 1U);
  EXPECT_FALSE(segments[0].ends_with_pivot);
  EXPECT_EQ(segments[0].path.poses.size(), 3U);
}

TEST(ManeuverPath, RejectsUnitPath)
{
  nav_msgs::msg::Path path;
  path.poses.push_back(pose(0.0, 0.0, 0.0));
  EXPECT_THROW(split_path_at_pivots(path, 1.0e-4, 0.05), std::invalid_argument);
}

TEST(ManeuverPath, TerminalDriveUsesShortestForwardOrReverseHeading)
{
  const auto forward = shortest_terminal_drive(0.25);
  EXPECT_DOUBLE_EQ(forward.direction, 1.0);
  EXPECT_NEAR(forward.heading_error, 0.25, 1.0e-12);

  const auto reverse_left = shortest_terminal_drive(kPi - 0.20);
  EXPECT_DOUBLE_EQ(reverse_left.direction, -1.0);
  EXPECT_NEAR(reverse_left.heading_error, -0.20, 1.0e-12);

  const auto reverse_right = shortest_terminal_drive(-kPi + 0.30);
  EXPECT_DOUBLE_EQ(reverse_right.direction, -1.0);
  EXPECT_NEAR(reverse_right.heading_error, 0.30, 1.0e-12);
}

TEST(ManeuverPath, TerminalDriveRejectsNonFiniteBearing)
{
  EXPECT_THROW(
    shortest_terminal_drive(std::numeric_limits<double>::infinity()),
    std::invalid_argument);
}

}  // namespace
}  // namespace adaptive_pivot_g2_controller
