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
#include <memory>
#include <vector>

#include "adaptive_pivot_g2_nav2/footprint_safety.hpp"
#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"

namespace
{

using adaptive_pivot_g2::Vec2;
using adaptive_pivot_g2_nav2::footprint_safety::CollisionChecker;
using adaptive_pivot_g2_nav2::footprint_safety::Costmap;
using adaptive_pivot_g2_nav2::footprint_safety::line_is_safe;
using adaptive_pivot_g2_nav2::footprint_safety::pivot_is_safe;
using adaptive_pivot_g2_nav2::footprint_safety::pose_is_safe;

constexpr unsigned char kMaximumCenterCost = 252U;

std::vector<geometry_msgs::msg::Point> rectangular_footprint()
{
  std::vector<geometry_msgs::msg::Point> footprint(4U);
  footprint[0].x = 0.22;
  footprint[0].y = 0.17;
  footprint[1].x = 0.22;
  footprint[1].y = -0.17;
  footprint[2].x = -0.22;
  footprint[2].y = -0.17;
  footprint[3].x = -0.22;
  footprint[3].y = 0.17;
  return footprint;
}

std::shared_ptr<Costmap> empty_costmap()
{
  return std::make_shared<Costmap>(
    800U, 800U, 0.005, -2.0, -2.0, nav2_costmap_2d::FREE_SPACE);
}

void set_world_cost(
  const std::shared_ptr<Costmap> & costmap,
  double world_x,
  double world_y,
  unsigned char cost)
{
  unsigned int map_x = 0U;
  unsigned int map_y = 0U;
  ASSERT_TRUE(costmap->worldToMap(world_x, world_y, map_x, map_y));
  costmap->setCost(map_x, map_y, cost);
}

TEST(FootprintSafety, ActualFootprintAcceptsChordRejectedByFifteenCentimeterPadding)
{
  auto costmap = empty_costmap();
  set_world_cost(costmap, 0.0, 0.27, nav2_costmap_2d::LETHAL_OBSTACLE);
  CollisionChecker checker(costmap);
  const auto footprint = rectangular_footprint();
  auto padded_footprint = footprint;
  nav2_costmap_2d::padFootprint(padded_footprint, 0.15);

  EXPECT_TRUE(line_is_safe(
      {-0.5, 0.0}, {0.5, 0.0}, costmap, checker, footprint,
      kMaximumCenterCost));
  EXPECT_FALSE(line_is_safe(
      {-0.5, 0.0}, {0.5, 0.0}, costmap, checker, padded_footprint,
      kMaximumCenterCost));
}

TEST(FootprintSafety, RotationChecksTheSweptRectangleBetweenSafeEndpoints)
{
  auto costmap = empty_costmap();
  set_world_cost(costmap, 0.18, 0.18, nav2_costmap_2d::LETHAL_OBSTACLE);
  CollisionChecker checker(costmap);
  const auto footprint = rectangular_footprint();
  const double half_pi = 0.5 * std::acos(-1.0);

  ASSERT_TRUE(pose_is_safe(
      {0.0, 0.0}, 0.0, costmap, checker, footprint, kMaximumCenterCost));
  ASSERT_TRUE(pose_is_safe(
      {0.0, 0.0}, half_pi, costmap, checker, footprint, kMaximumCenterCost));
  EXPECT_FALSE(pivot_is_safe(
      {0.0, 0.0}, 0.0, half_pi, costmap, checker, footprint,
      kMaximumCenterCost));
}

TEST(FootprintSafety, RejectsLethalAndUnknownCellsInsideThePhysicalBody)
{
  const auto footprint = rectangular_footprint();
  for (const unsigned char cost : {
      nav2_costmap_2d::LETHAL_OBSTACLE,
      nav2_costmap_2d::NO_INFORMATION})
  {
    auto costmap = empty_costmap();
    set_world_cost(costmap, 0.10, 0.0, cost);
    CollisionChecker checker(costmap);
    EXPECT_FALSE(pose_is_safe(
        {0.0, 0.0}, 0.0, costmap, checker, footprint,
        kMaximumCenterCost));
  }
}

TEST(FootprintSafety, RejectsPoseAndSweepOutsideTheCostmap)
{
  auto costmap = empty_costmap();
  CollisionChecker checker(costmap);
  const auto footprint = rectangular_footprint();

  EXPECT_FALSE(pose_is_safe(
      {1.90, 0.0}, 0.0, costmap, checker, footprint, kMaximumCenterCost));
  EXPECT_FALSE(line_is_safe(
      {0.0, 0.0}, {1.90, 0.0}, costmap, checker, footprint,
      kMaximumCenterCost));
}

}  // namespace
