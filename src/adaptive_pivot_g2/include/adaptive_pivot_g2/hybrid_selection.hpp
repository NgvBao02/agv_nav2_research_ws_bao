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

#ifndef ADAPTIVE_PIVOT_G2__HYBRID_SELECTION_HPP_
#define ADAPTIVE_PIVOT_G2__HYBRID_SELECTION_HPP_

#include <string>

namespace adaptive_pivot_g2
{

struct HybridCandidate
{
  bool safe{false};
  double maximum_proximity_cost{0.0};
  double maneuver_effort{0.0};
  double path_length{0.0};
};

struct HybridSelectionPolicy
{
  /// A cost advantage at least this large takes priority over effort.
  double peak_cost_deadband{20.0};
  /// Inside the cost deadband, require this symmetric relative effort gap.
  double relative_effort_deadband{0.05};
  /// Stabilizes the relative comparison for nearly straight candidates.
  double effort_floor{0.25};
  /// Final deterministic geometric tie-break tolerance.
  double path_length_tolerance{1.0e-6};
};

struct HybridSelection
{
  bool valid{false};
  bool use_pivot{false};
  std::string reason{"no_safe_candidate"};
  bool use_raw{false};
};

/// Compare Simple and Pivot with the same safety, cost, effort and length rule.
HybridSelection select_hybrid_candidate(
  const HybridCandidate & simple,
  const HybridCandidate & pivot,
  const HybridSelectionPolicy & policy);

/// Apply the same symmetric rule and use raw only when both smoothed
/// candidates are unsafe.
HybridSelection select_hybrid_candidate_with_raw_fallback(
  const HybridCandidate & raw,
  const HybridCandidate & simple,
  const HybridCandidate & pivot,
  const HybridSelectionPolicy & policy);

}  // namespace adaptive_pivot_g2

#endif  // ADAPTIVE_PIVOT_G2__HYBRID_SELECTION_HPP_
