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

#include "adaptive_pivot_g2_nav2/footprint_safety.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace adaptive_pivot_g2_nav2
{
namespace footprint_safety
{
namespace
{

constexpr double kEpsilon = 1.0e-10;

bool filled_footprint_has_hard_collision(
  const adaptive_pivot_g2::Vec2 & point,
  double heading,
  const std::shared_ptr<Costmap> & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint)
{
  const double cosine = std::cos(heading);
  const double sine = std::sin(heading);
  std::vector<nav2_costmap_2d::MapLocation> polygon;
  polygon.reserve(footprint.size());
  for (const auto & footprint_point : footprint) {
    const double world_x = point.x +
      cosine * footprint_point.x - sine * footprint_point.y;
    const double world_y = point.y +
      sine * footprint_point.x + cosine * footprint_point.y;
    unsigned int map_x = 0U;
    unsigned int map_y = 0U;
    if (!costmap->worldToMap(world_x, world_y, map_x, map_y)) {
      return true;
    }
    polygon.push_back({map_x, map_y});
  }

  std::vector<nav2_costmap_2d::MapLocation> filled_cells;
  costmap->convexFillCells(polygon, filled_cells);
  return std::any_of(
    filled_cells.begin(), filled_cells.end(),
    [&costmap](const nav2_costmap_2d::MapLocation & cell) {
      const unsigned char cost = costmap->getCost(cell.x, cell.y);
      return cost == nav2_costmap_2d::NO_INFORMATION ||
             cost >= nav2_costmap_2d::LETHAL_OBSTACLE;
    });
}

}  // namespace

bool pose_is_safe(
  const adaptive_pivot_g2::Vec2 & point,
  double heading,
  const std::shared_ptr<Costmap> & costmap,
  CollisionChecker & checker,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  unsigned char max_footprint_cost,
  double * proximity_cost)
{
  if (!costmap || footprint.size() < 3U ||
    !adaptive_pivot_g2::finite(point) || !std::isfinite(heading))
  {
    return false;
  }
  unsigned int map_x = 0U;
  unsigned int map_y = 0U;
  if (!costmap->worldToMap(point.x, point.y, map_x, map_y)) {
    return false;
  }
  const unsigned char center_cost = costmap->getCost(map_x, map_y);
  if (center_cost > max_footprint_cost ||
    filled_footprint_has_hard_collision(point, heading, costmap, footprint))
  {
    return false;
  }

  const double footprint_cost = checker.footprintCostAtPose(
    point.x, point.y, heading, footprint);
  // Inflation costs are a center-clearance policy and are not a second
  // footprint inflation. The filled polygon above rejects only actual lethal
  // or unknown contact anywhere under the physical body.
  const bool safe = std::isfinite(footprint_cost) && footprint_cost >= 0.0 &&
    footprint_cost < nav2_costmap_2d::LETHAL_OBSTACLE;
  if (safe && proximity_cost != nullptr) {
    *proximity_cost = std::max(static_cast<double>(center_cost), footprint_cost);
  }
  return safe;
}

bool line_is_safe(
  const adaptive_pivot_g2::Vec2 & start,
  const adaptive_pivot_g2::Vec2 & finish,
  const std::shared_ptr<Costmap> & costmap,
  CollisionChecker & checker,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  unsigned char max_footprint_cost)
{
  if (!costmap) {
    return false;
  }
  const adaptive_pivot_g2::Vec2 delta = finish - start;
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

bool pivot_is_safe(
  const adaptive_pivot_g2::Vec2 & vertex,
  double incoming_heading,
  double turn_angle,
  const std::shared_ptr<Costmap> & costmap,
  CollisionChecker & checker,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  unsigned char max_footprint_cost)
{
  if (!costmap) {
    return false;
  }
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
        vertex, incoming_heading + fraction * turn_angle, costmap, checker,
        footprint, max_footprint_cost))
    {
      return false;
    }
  }
  return true;
}

}  // namespace footprint_safety
}  // namespace adaptive_pivot_g2_nav2
