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

#ifndef ADAPTIVE_PIVOT_G2__LINE_OF_SIGHT_HPP_
#define ADAPTIVE_PIVOT_G2__LINE_OF_SIGHT_HPP_

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include "adaptive_pivot_g2/types.hpp"

namespace adaptive_pivot_g2
{

using LineOfSightSegmentPredicate =
  std::function<bool(const Vec2 &, const Vec2 &)>;
using LineOfSightJunctionPredicate =
  std::function<bool(const Vec2 &, const Vec2 &, const Vec2 &)>;

struct LineOfSightPruningResult
{
  bool valid{false};
  bool fallback_to_input{false};
  std::string fallback_reason;
  std::vector<Vec2> points;
  std::size_t attempted_shortcuts{0U};
  std::size_t accepted_shortcuts{0U};
  std::size_t safety_rejections{0U};
};

/// Greedily retain the farthest safe visible point from each anchor.
///
/// Unlike a geometric point-only LOS test, both predicates are mandatory:
/// `segment_is_safe` checks the complete swept motion on every candidate,
/// including the adjacent input edge, and `junction_is_safe` checks the
/// orientation sweep at every retained interior vertex. If no safe outgoing
/// edge exists, the original input is returned as an explicit fallback rather
/// than silently accepting an unchecked edge.
LineOfSightPruningResult prune_line_of_sight(
  const std::vector<Vec2> & input,
  const LineOfSightSegmentPredicate & segment_is_safe,
  const LineOfSightJunctionPredicate & junction_is_safe);

}  // namespace adaptive_pivot_g2

#endif  // ADAPTIVE_PIVOT_G2__LINE_OF_SIGHT_HPP_
