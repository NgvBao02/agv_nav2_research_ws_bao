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

#include "adaptive_pivot_g2/candidate_selection.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace adaptive_pivot_g2
{
namespace
{

constexpr double kEpsilon = 1.0e-12;

bool finite_candidate(const CandidateObjective & candidate)
{
  return std::isfinite(candidate.common_window_time) &&
         candidate.common_window_time >= 0.0 &&
         std::isfinite(candidate.clearance) &&
         std::isfinite(candidate.max_abs_angular_speed) &&
         candidate.max_abs_angular_speed >= 0.0 &&
         std::isfinite(candidate.curvature_energy) &&
         candidate.curvature_energy >= 0.0;
}

double normalized_high(double value, double minimum, double maximum)
{
  return maximum - minimum <= kEpsilon ? 1.0 : (value - minimum) / (maximum - minimum);
}

double normalized_low(double value, double minimum, double maximum)
{
  return maximum - minimum <= kEpsilon ? 1.0 : (maximum - value) / (maximum - minimum);
}

}  // namespace

CandidateSelection select_competitive_candidate(
  const std::vector<CandidateObjective> & candidates,
  double time_slack,
  const SelectionWeights & weights)
{
  CandidateSelection result;
  const double weight_sum = weights.clearance + weights.angular_speed +
    weights.curvature_energy;
  if (!std::isfinite(time_slack) || time_slack < 0.0 ||
    !std::isfinite(weights.clearance) || weights.clearance < 0.0 ||
    !std::isfinite(weights.angular_speed) || weights.angular_speed < 0.0 ||
    !std::isfinite(weights.curvature_energy) || weights.curvature_energy < 0.0 ||
    !std::isfinite(weight_sum) || weight_sum <= kEpsilon)
  {
    return result;
  }

  for (const auto & candidate : candidates) {
    if (finite_candidate(candidate)) {
      result.fastest_time = std::min(result.fastest_time, candidate.common_window_time);
    }
  }
  if (!std::isfinite(result.fastest_time)) {
    return result;
  }

  std::vector<const CandidateObjective *> competitive;
  for (const auto & candidate : candidates) {
    if (finite_candidate(candidate) &&
      candidate.common_window_time <= result.fastest_time + time_slack + kEpsilon)
    {
      competitive.push_back(&candidate);
    }
  }
  result.competitive_count = competitive.size();
  if (competitive.empty()) {
    return result;
  }

  double minimum_clearance = std::numeric_limits<double>::infinity();
  double maximum_clearance = -std::numeric_limits<double>::infinity();
  double minimum_angular_speed = std::numeric_limits<double>::infinity();
  double maximum_angular_speed = -std::numeric_limits<double>::infinity();
  double minimum_curvature_energy = std::numeric_limits<double>::infinity();
  double maximum_curvature_energy = -std::numeric_limits<double>::infinity();
  for (const auto * candidate : competitive) {
    minimum_clearance = std::min(minimum_clearance, candidate->clearance);
    maximum_clearance = std::max(maximum_clearance, candidate->clearance);
    minimum_angular_speed = std::min(
      minimum_angular_speed, candidate->max_abs_angular_speed);
    maximum_angular_speed = std::max(
      maximum_angular_speed, candidate->max_abs_angular_speed);
    minimum_curvature_energy = std::min(
      minimum_curvature_energy, candidate->curvature_energy);
    maximum_curvature_energy = std::max(
      maximum_curvature_energy, candidate->curvature_energy);
  }

  for (const auto * candidate : competitive) {
    const double score = (
      weights.clearance * normalized_high(
      candidate->clearance, minimum_clearance, maximum_clearance) +
      weights.angular_speed * normalized_low(
      candidate->max_abs_angular_speed, minimum_angular_speed, maximum_angular_speed) +
      weights.curvature_energy * normalized_low(
      candidate->curvature_energy, minimum_curvature_energy, maximum_curvature_energy)) /
      weight_sum;
    const bool higher_score = score > result.selected_score + kEpsilon;
    const bool equal_score = std::abs(score - result.selected_score) <= kEpsilon;
    const bool faster = candidate->common_window_time < result.selected_time - kEpsilon;
    const bool same_time =
      std::abs(candidate->common_window_time - result.selected_time) <= kEpsilon;
    if (!result.valid || higher_score ||
      (equal_score && (faster ||
      (same_time && candidate->candidate_index < result.candidate_index))))
    {
      result.valid = true;
      result.candidate_index = candidate->candidate_index;
      result.selected_time = candidate->common_window_time;
      result.selected_score = score;
    }
  }
  return result;
}

}  // namespace adaptive_pivot_g2
