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

#include <algorithm>
#include <cmath>

namespace adaptive_pivot_g2
{

HybridSelection select_hybrid_candidate(
  const HybridCandidate & simple,
  const HybridCandidate & pivot,
  const HybridSelectionPolicy & policy)
{
  const bool parameters_valid =
    std::isfinite(policy.peak_cost_deadband) && policy.peak_cost_deadband >= 0.0 &&
    std::isfinite(policy.relative_effort_deadband) &&
    policy.relative_effort_deadband >= 0.0 &&
    policy.relative_effort_deadband <= 1.0 &&
    std::isfinite(policy.effort_floor) && policy.effort_floor > 0.0 &&
    std::isfinite(policy.path_length_tolerance) &&
    policy.path_length_tolerance >= 0.0;
  const auto candidate_valid = [](const HybridCandidate & candidate) {
      return !candidate.safe ||
             (std::isfinite(candidate.maximum_proximity_cost) &&
             candidate.maximum_proximity_cost >= 0.0 &&
             std::isfinite(candidate.maneuver_effort) &&
             candidate.maneuver_effort >= 0.0 &&
             std::isfinite(candidate.path_length) &&
             candidate.path_length >= 0.0);
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

  const double cost_delta = simple.maximum_proximity_cost -
    pivot.maximum_proximity_cost;
  if (std::abs(cost_delta) >= policy.peak_cost_deadband && cost_delta > 0.0) {
    return {true, true, "pivot_lower_peak_cost"};
  }
  if (std::abs(cost_delta) >= policy.peak_cost_deadband && cost_delta < 0.0) {
    return {true, false, "simple_lower_peak_cost"};
  }

  const double effort_delta = simple.maneuver_effort - pivot.maneuver_effort;
  const double effort_scale = std::max(
    policy.effort_floor, std::max(simple.maneuver_effort, pivot.maneuver_effort));
  const double relative_effort_gap = std::abs(effort_delta) / effort_scale;
  if (relative_effort_gap >= policy.relative_effort_deadband && effort_delta > 0.0) {
    return {true, true, "pivot_lower_maneuver_effort"};
  }
  if (relative_effort_gap >= policy.relative_effort_deadband && effort_delta < 0.0) {
    return {true, false, "simple_lower_maneuver_effort"};
  }

  if (cost_delta > 0.0) {
    return {true, true, "pivot_lower_residual_cost"};
  }
  if (cost_delta < 0.0) {
    return {true, false, "simple_lower_residual_cost"};
  }

  const double length_delta = simple.path_length - pivot.path_length;
  if (length_delta > policy.path_length_tolerance) {
    return {true, true, "pivot_shorter_path"};
  }
  if (length_delta < -policy.path_length_tolerance) {
    return {true, false, "simple_shorter_path"};
  }

  // The candidates are indistinguishable under every declared metric.
  // Choosing a stable label here avoids output flicker; it has no utility bias.
  return {true, false, "metric_tie_stable_simple"};
}

HybridSelection select_hybrid_candidate_with_raw_fallback(
  const HybridCandidate & raw,
  const HybridCandidate & simple,
  const HybridCandidate & pivot,
  const HybridSelectionPolicy & policy)
{
  HybridSelection selection = select_hybrid_candidate(simple, pivot, policy);
  if (selection.valid) {
    return selection;
  }
  if (raw.safe && std::isfinite(raw.maximum_proximity_cost) &&
    raw.maximum_proximity_cost >= 0.0 &&
    std::isfinite(raw.maneuver_effort) &&
    raw.maneuver_effort >= 0.0 &&
    std::isfinite(raw.path_length) &&
    raw.path_length >= 0.0)
  {
    return {true, false, "smoothed_candidates_unsafe_raw_fallback", true};
  }
  return selection;
}

}  // namespace adaptive_pivot_g2
