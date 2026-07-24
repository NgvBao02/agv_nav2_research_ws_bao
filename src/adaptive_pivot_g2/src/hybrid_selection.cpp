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

#include "adaptive_pivot_g2/hybrid_selection.hpp"

#include <cmath>

namespace adaptive_pivot_g2
{

HybridSelection select_hybrid_candidate(
  const HybridCandidate & simple,
  const HybridCandidate & pivot,
  double minimum_cost_improvement,
  double maximum_curvature_energy_ratio,
  double curvature_energy_floor)
{
  const bool parameters_valid =
    std::isfinite(minimum_cost_improvement) && minimum_cost_improvement >= 0.0 &&
    std::isfinite(maximum_curvature_energy_ratio) &&
    maximum_curvature_energy_ratio >= 1.0 &&
    std::isfinite(curvature_energy_floor) && curvature_energy_floor >= 0.0;
  const auto candidate_valid = [](const HybridCandidate & candidate) {
      return !candidate.safe ||
             (std::isfinite(candidate.maximum_proximity_cost) &&
             candidate.maximum_proximity_cost >= 0.0 &&
             std::isfinite(candidate.translational_curvature_energy) &&
             candidate.translational_curvature_energy >= 0.0);
    };
  if (!parameters_valid || !candidate_valid(simple) || !candidate_valid(pivot)) {
    return {};
  }
  if (!simple.safe && !pivot.safe) {
    return {};
  }
  if (!simple.safe) {
    return {true, true, "simple_unsafe"};
  }
  if (!pivot.safe) {
    return {true, false, "pivot_unsafe"};
  }

  const double cost_improvement = simple.maximum_proximity_cost -
    pivot.maximum_proximity_cost;
  const double energy_budget = maximum_curvature_energy_ratio *
    (simple.translational_curvature_energy + curvature_energy_floor);
  if (cost_improvement >= minimum_cost_improvement &&
    pivot.translational_curvature_energy <= energy_budget)
  {
    return {true, true, "safety_gain_within_energy_budget"};
  }
  return {true, false, "simple_default"};
}

HybridSelection select_hybrid_candidate_with_raw_fallback(
  const HybridCandidate & raw,
  const HybridCandidate & simple,
  const HybridCandidate & pivot,
  double minimum_cost_improvement,
  double maximum_curvature_energy_ratio,
  double curvature_energy_floor)
{
  HybridSelection selection = select_hybrid_candidate(
    simple, pivot, minimum_cost_improvement, maximum_curvature_energy_ratio,
    curvature_energy_floor);
  if (selection.valid) {
    return selection;
  }
  if (raw.safe && std::isfinite(raw.maximum_proximity_cost) &&
    raw.maximum_proximity_cost >= 0.0 &&
    std::isfinite(raw.translational_curvature_energy) &&
    raw.translational_curvature_energy >= 0.0)
  {
    return {true, false, "smoothed_candidates_unsafe_raw_fallback", true};
  }
  return selection;
}

}  // namespace adaptive_pivot_g2
