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
  double translational_curvature_energy{0.0};
};

struct HybridSelection
{
  bool valid{false};
  bool use_pivot{false};
  std::string reason{"no_safe_candidate"};
};

/// Apply the declared safety-gain and curvature-budget decision rule.
HybridSelection select_hybrid_candidate(
  const HybridCandidate & simple,
  const HybridCandidate & pivot,
  double minimum_cost_improvement,
  double maximum_curvature_energy_ratio,
  double curvature_energy_floor);

}  // namespace adaptive_pivot_g2

#endif  // ADAPTIVE_PIVOT_G2__HYBRID_SELECTION_HPP_
