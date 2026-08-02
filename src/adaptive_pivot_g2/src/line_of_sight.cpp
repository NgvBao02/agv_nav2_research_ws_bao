// Copyright 2026 PSTMO Research Team
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

#include "adaptive_pivot_g2/line_of_sight.hpp"

#include <algorithm>
#include <string>

namespace adaptive_pivot_g2
{
LineOfSightPruningResult prune_line_of_sight(
  const std::vector<Vec2> & input,
  const LineOfSightSegmentPredicate & segment_is_safe,
  const LineOfSightJunctionPredicate & junction_is_safe,
  const LineOfSightEndpointPredicate & start_rotation_is_safe,
  const LineOfSightEndpointPredicate & goal_rotation_is_safe)
{
  LineOfSightPruningResult result;
  if (input.size() < 2U) {
    result.rejection_reason = "polyline needs at least two points";
    return result;
  }
  if (!segment_is_safe || !junction_is_safe ||
    !start_rotation_is_safe || !goal_rotation_is_safe)
  {
    result.rejection_reason = "LOS safety predicate is empty";
    return result;
  }
  if (std::any_of(input.begin(), input.end(), [](const Vec2 & point) {
      return !finite(point);
    }))
  {
    result.rejection_reason = "polyline contains a non-finite point";
    return result;
  }

  std::vector<Vec2> points;
  points.reserve(input.size());
  for (const auto & point : input) {
    if (points.empty() || distance(points.back(), point) > 1.0e-12) {
      points.push_back(point);
    }
  }
  if (points.size() < 2U) {
    result.rejection_reason = "polyline collapses to fewer than two distinct points";
    return result;
  }

  result.points.reserve(points.size());
  result.points.push_back(points.front());
  std::size_t anchor = 0U;
  while (anchor + 1U < points.size()) {
    std::size_t selected = points.size();
    std::string last_rejection{"no candidate evaluated"};
    for (std::size_t candidate = points.size() - 1U; candidate > anchor; --candidate) {
      const bool shortcut = candidate > anchor + 1U;
      if (shortcut) {
        ++result.attempted_shortcuts;
      }
      if (!segment_is_safe(points[anchor], points[candidate])) {
        ++result.safety_rejections;
        last_rejection = "unsafe swept translation to point index " +
          std::to_string(candidate);
        continue;
      }
      if (anchor == 0U &&
        !start_rotation_is_safe(points[anchor], points[candidate]))
      {
        ++result.safety_rejections;
        last_rejection = "unsafe start rotation to point index " +
          std::to_string(candidate);
        continue;
      }
      if (anchor > 0U &&
        !junction_is_safe(
          result.points[result.points.size() - 2U], points[anchor], points[candidate]))
      {
        ++result.safety_rejections;
        last_rejection = "unsafe junction rotation at point index " +
          std::to_string(anchor);
        continue;
      }
      if (candidate + 1U == points.size() &&
        !goal_rotation_is_safe(points[anchor], points[candidate]))
      {
        ++result.safety_rejections;
        last_rejection = "unsafe goal rotation from point index " +
          std::to_string(anchor);
        continue;
      }
      selected = candidate;
      if (shortcut) {
        ++result.accepted_shortcuts;
      }
      break;
    }
    if (selected == points.size()) {
      result.rejection_reason =
        "no footprint-safe outgoing edge at conditioned point index " +
        std::to_string(anchor) + ": " + last_rejection;
      return result;
    }
    result.points.push_back(points[selected]);
    anchor = selected;
  }
  result.valid = true;
  return result;
}

}  // namespace adaptive_pivot_g2
