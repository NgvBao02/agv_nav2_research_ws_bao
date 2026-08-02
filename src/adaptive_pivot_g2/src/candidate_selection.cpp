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

double stable_candidate_cost(
  double peak_cost,
  double max_abs_angular_speed,
  double curvature_energy,
  double max_angular_speed,
  double curvature_energy_scale,
  const SelectionWeights & weights)
{
  const double weight_sum = weights.clearance + weights.angular_speed +
    weights.curvature_energy;
  if (!std::isfinite(peak_cost) || peak_cost < 0.0 ||
    !std::isfinite(max_abs_angular_speed) || max_abs_angular_speed < 0.0 ||
    !std::isfinite(curvature_energy) || curvature_energy < 0.0 ||
    !std::isfinite(max_angular_speed) || max_angular_speed <= 0.0 ||
    !std::isfinite(curvature_energy_scale) || curvature_energy_scale <= 0.0 ||
    !std::isfinite(weights.clearance) || weights.clearance < 0.0 ||
    !std::isfinite(weights.angular_speed) || weights.angular_speed < 0.0 ||
    !std::isfinite(weights.curvature_energy) ||
    weights.curvature_energy < 0.0 ||
    !std::isfinite(weight_sum) || weight_sum <= kEpsilon)
  {
    return std::numeric_limits<double>::infinity();
  }
  const double risk = std::min(1.0, peak_cost / 252.0);
  const double angular = std::min(1.0, max_abs_angular_speed / max_angular_speed);
  const double energy = curvature_energy /
    (curvature_energy + curvature_energy_scale);
  return (weights.clearance * risk +
         weights.angular_speed * angular +
         weights.curvature_energy * energy) / weight_sum;
}

double stable_path_quality_score(
  const PathQualityMetrics & metrics,
  double reference_path_length,
  double wheel_separation,
  double curvature_energy_scale,
  double maximum_proximity_cost,
  const PathQualityWeights & weights)
{
  const double weight_sum = weights.path_length + weights.max_abs_curvature +
    weights.curvature_energy + weights.pivot_rotation + weights.proximity_cost;
  if (!metrics.valid || !metrics.safe ||
    !std::isfinite(metrics.path_length) || metrics.path_length < 0.0 ||
    !std::isfinite(metrics.max_abs_curvature) || metrics.max_abs_curvature < 0.0 ||
    !std::isfinite(metrics.curvature_energy) || metrics.curvature_energy < 0.0 ||
    !std::isfinite(metrics.pivot_rotation) || metrics.pivot_rotation < 0.0 ||
    !std::isfinite(metrics.peak_proximity_cost) ||
    metrics.peak_proximity_cost < 0.0 ||
    !std::isfinite(reference_path_length) || reference_path_length <= 0.0 ||
    !std::isfinite(wheel_separation) || wheel_separation <= 0.0 ||
    !std::isfinite(curvature_energy_scale) || curvature_energy_scale <= 0.0 ||
    !std::isfinite(maximum_proximity_cost) || maximum_proximity_cost <= 0.0 ||
    !std::isfinite(weights.path_length) || weights.path_length < 0.0 ||
    !std::isfinite(weights.max_abs_curvature) ||
    weights.max_abs_curvature < 0.0 ||
    !std::isfinite(weights.curvature_energy) ||
    weights.curvature_energy < 0.0 ||
    !std::isfinite(weights.pivot_rotation) || weights.pivot_rotation < 0.0 ||
    !std::isfinite(weights.proximity_cost) || weights.proximity_cost < 0.0 ||
    !std::isfinite(weights.raw_fallback_penalty) ||
    weights.raw_fallback_penalty < 0.0 ||
    !std::isfinite(weight_sum) || weight_sum <= kEpsilon)
  {
    return std::numeric_limits<double>::infinity();
  }

  const double length_cost = std::min(2.0, metrics.path_length / reference_path_length);
  const double wheel_reversal_curvature = 2.0 / wheel_separation;
  const double curvature_cost = metrics.max_abs_curvature /
    (metrics.max_abs_curvature + wheel_reversal_curvature);
  // Convert in-place rotation to the same 1/m maneuver-effort dimension as
  // integral(kappa^2 ds). Without this term an all-pivot path appears to have
  // zero curvature energy and can incorrectly dominate a smooth translation.
  const double maneuver_energy = metrics.curvature_energy +
    metrics.pivot_rotation / wheel_separation;
  const double energy_cost = maneuver_energy /
    (maneuver_energy + curvature_energy_scale);
  constexpr double kPi = 3.14159265358979323846;
  const double pivot_cost = metrics.pivot_rotation /
    (metrics.pivot_rotation + kPi);
  const double proximity_cost = std::min(
    1.0, metrics.peak_proximity_cost / maximum_proximity_cost);
  const double score = (
    weights.path_length * length_cost +
    weights.max_abs_curvature * curvature_cost +
    weights.curvature_energy * energy_cost +
    weights.pivot_rotation * pivot_cost +
    weights.proximity_cost * proximity_cost) / weight_sum;
  return score + (metrics.raw_fallback ? weights.raw_fallback_penalty : 0.0);
}

LosBranchSelection select_los_branch(
  const PathQualityMetrics & no_los,
  const PathQualityMetrics & los,
  double reference_path_length,
  double wheel_separation,
  double curvature_energy_scale,
  double maximum_proximity_cost,
  double minimum_improvement,
  const PathQualityWeights & weights)
{
  LosBranchSelection result;
  if (!std::isfinite(minimum_improvement) || minimum_improvement < 0.0) {
    result.reason = "invalid_minimum_improvement";
    return result;
  }
  result.no_los_score = stable_path_quality_score(
    no_los, reference_path_length, wheel_separation,
    curvature_energy_scale, maximum_proximity_cost, weights);
  result.los_score = stable_path_quality_score(
    los, reference_path_length, wheel_separation,
    curvature_energy_scale, maximum_proximity_cost, weights);
  const bool no_los_valid = std::isfinite(result.no_los_score);
  const bool los_valid = std::isfinite(result.los_score);
  if (!no_los_valid && !los_valid) {
    return result;
  }
  result.valid = true;
  if (!no_los_valid) {
    result.use_los = true;
    result.reason = "only_los_valid";
  } else if (!los_valid) {
    result.reason = "only_no_los_valid";
  } else if (result.los_score + minimum_improvement < result.no_los_score) {
    result.use_los = true;
    result.reason = "los_quality_improvement";
  } else {
    result.reason = "no_los_quality_not_worse";
  }
  return result;
}

}  // namespace adaptive_pivot_g2
