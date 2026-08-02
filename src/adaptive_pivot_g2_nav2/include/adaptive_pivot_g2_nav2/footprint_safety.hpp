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

#ifndef ADAPTIVE_PIVOT_G2_NAV2__FOOTPRINT_SAFETY_HPP_
#define ADAPTIVE_PIVOT_G2_NAV2__FOOTPRINT_SAFETY_HPP_

#include <memory>
#include <vector>

#include "adaptive_pivot_g2/types.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

namespace adaptive_pivot_g2_nav2
{
namespace footprint_safety
{

using Costmap = nav2_costmap_2d::Costmap2D;
using CollisionChecker =
  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<Costmap>>;

/// Check the complete, unpadded footprint at one pose.
bool pose_is_safe(
  const adaptive_pivot_g2::Vec2 & point,
  double heading,
  const std::shared_ptr<Costmap> & costmap,
  CollisionChecker & checker,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  unsigned char max_footprint_cost,
  double * proximity_cost = nullptr);

/// Check swept translation at the chord heading using the actual footprint.
bool line_is_safe(
  const adaptive_pivot_g2::Vec2 & start,
  const adaptive_pivot_g2::Vec2 & finish,
  const std::shared_ptr<Costmap> & costmap,
  CollisionChecker & checker,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  unsigned char max_footprint_cost);

/// Check the swept footprint for an in-place rotation.
bool pivot_is_safe(
  const adaptive_pivot_g2::Vec2 & vertex,
  double incoming_heading,
  double turn_angle,
  const std::shared_ptr<Costmap> & costmap,
  CollisionChecker & checker,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  unsigned char max_footprint_cost);

}  // namespace footprint_safety
}  // namespace adaptive_pivot_g2_nav2

#endif  // ADAPTIVE_PIVOT_G2_NAV2__FOOTPRINT_SAFETY_HPP_
