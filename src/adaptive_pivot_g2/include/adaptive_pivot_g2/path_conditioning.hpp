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

#ifndef ADAPTIVE_PIVOT_G2__PATH_CONDITIONING_HPP_
#define ADAPTIVE_PIVOT_G2__PATH_CONDITIONING_HPP_

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include "adaptive_pivot_g2/types.hpp"

namespace adaptive_pivot_g2
{

using SegmentSafetyPredicate = std::function<bool(const Vec2 &, const Vec2 &)>;

struct PathConditioningOptions
{
  double maximum_deviation{0.0};
  double oscillation_maximum_span{0.0};
  double oscillation_maximum_deviation{0.0};
  double oscillation_minimum_turn_angle{0.20};
  std::size_t oscillation_minimum_sign_changes{2U};
};

struct PathConditioningResult
{
  bool valid{false};
  std::string rejection_reason;
  std::vector<Vec2> points;
  std::vector<std::size_t> retained_input_indices;
  std::size_t accepted_shortcuts{0};
  std::size_t accepted_oscillation_shortcuts{0};
  std::size_t safety_rejected_shortcuts{0};
  double maximum_removed_deviation{0.0};
};

/// Remove grid-scale polyline oscillations without changing the safety corridor.
///
/// A shortcut is accepted only when every skipped input point lies within
/// `maximum_deviation` of its chord and `segment_is_safe` accepts the complete
/// chord. Endpoints and input order are preserved. A zero deviation disables
/// shortcutting.
PathConditioningResult condition_polyline(
  const std::vector<Vec2> & input,
  double maximum_deviation,
  const SegmentSafetyPredicate & segment_is_safe);

PathConditioningResult condition_polyline(
  const std::vector<Vec2> & input,
  const PathConditioningOptions & options,
  const SegmentSafetyPredicate & segment_is_safe);

}  // namespace adaptive_pivot_g2

#endif  // ADAPTIVE_PIVOT_G2__PATH_CONDITIONING_HPP_
