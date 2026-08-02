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
#include <utility>

namespace adaptive_pivot_g2
{
namespace
{

LineOfSightPruningResult fallback_result(
  const std::vector<Vec2> & input,
  std::string reason,
  LineOfSightPruningResult result)
{
  result.valid = true;
  result.fallback_to_input = true;
  result.fallback_reason = std::move(reason);
  result.points = input;
  return result;
}

}  // namespace

LineOfSightPruningResult prune_line_of_sight(
  const std::vector<Vec2> & input,
  const LineOfSightSegmentPredicate & segment_is_safe,
  const LineOfSightJunctionPredicate & junction_is_safe)
{
  LineOfSightPruningResult result;
  if (input.size() < 2U) {
    result.fallback_reason = "polyline needs at least two points";
    return result;
  }
  if (!segment_is_safe || !junction_is_safe) {
    result.fallback_reason = "LOS safety predicate is empty";
    return result;
  }
  if (std::any_of(input.begin(), input.end(), [](const Vec2 & point) {
      return !finite(point);
    }))
  {
    result.fallback_reason = "polyline contains a non-finite point";
    return result;
  }

  result.points.reserve(input.size());
  result.points.push_back(input.front());
  std::size_t anchor = 0U;
  while (anchor + 1U < input.size()) {
    std::size_t selected = input.size();
    for (std::size_t candidate = input.size() - 1U; candidate > anchor; --candidate) {
      const bool shortcut = candidate > anchor + 1U;
      if (shortcut) {
        ++result.attempted_shortcuts;
      }
      if (!segment_is_safe(input[anchor], input[candidate])) {
        ++result.safety_rejections;
        continue;
      }
      if (result.points.size() >= 2U &&
        !junction_is_safe(
          result.points[result.points.size() - 2U], input[anchor], input[candidate]))
      {
        ++result.safety_rejections;
        continue;
      }
      selected = candidate;
      if (shortcut) {
        ++result.accepted_shortcuts;
      }
      break;
    }
    if (selected == input.size()) {
      return fallback_result(
        input, "no footprint-safe outgoing edge at input index " +
        std::to_string(anchor), std::move(result));
    }
    result.points.push_back(input[selected]);
    anchor = selected;
  }
  result.valid = true;
  return result;
}

}  // namespace adaptive_pivot_g2
