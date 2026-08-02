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

#include "adaptive_pivot_g2/hierarchical_shape_search.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

namespace adaptive_pivot_g2
{
namespace
{

constexpr double kEpsilon = 1.0e-12;
constexpr std::array<double, 5> kCoarseFractions{{0.1, 0.2, 0.3, 0.4, 0.5}};
constexpr std::array<double, 4> kRecoveryFractions{{0.15, 0.25, 0.35, 0.45}};
constexpr std::size_t kRefinementIntervals = 10U;

bool is_feasible(const ShapeSearchSample & sample)
{
  return sample.evaluation.feasible &&
         std::isfinite(sample.evaluation.curvature_energy) &&
         sample.evaluation.curvature_energy >= 0.0;
}

std::size_t best_sample_index(const std::vector<ShapeSearchSample> & samples)
{
  std::size_t best = samples.size();
  for (std::size_t index = 0U; index < samples.size(); ++index) {
    if (!is_feasible(samples[index])) {
      continue;
    }
    if (best == samples.size() ||
      samples[index].evaluation.curvature_energy <
      samples[best].evaluation.curvature_energy - kEpsilon ||
      (std::abs(
        samples[index].evaluation.curvature_energy -
        samples[best].evaluation.curvature_energy) <= kEpsilon &&
      samples[index].control_fraction < samples[best].control_fraction))
    {
      best = index;
    }
  }
  return best;
}

}  // namespace

HierarchicalShapeSearchResult search_control_fraction_coarse_to_fine(
  const ShapeEvaluator & evaluator)
{
  HierarchicalShapeSearchResult result;
  if (!evaluator) {
    return result;
  }

  const auto evaluate_unique = [&](double fraction, ShapeSearchStage stage) {
      if (std::any_of(
          result.samples.begin(), result.samples.end(),
          [fraction](const ShapeSearchSample & sample) {
            return std::abs(sample.control_fraction - fraction) <= kEpsilon;
          }))
      {
        return;
      }
      ShapeSearchEvaluation evaluation = evaluator(fraction);
      if (evaluation.feasible &&
        (!std::isfinite(evaluation.curvature_energy) ||
        evaluation.curvature_energy < 0.0))
      {
        evaluation.feasible = false;
        evaluation.curvature_energy = std::numeric_limits<double>::infinity();
        evaluation.rejection_reason = "evaluator returned invalid curvature energy";
      }
      result.samples.push_back({fraction, stage, std::move(evaluation)});
      switch (stage) {
        case ShapeSearchStage::kCoarse:
          ++result.coarse_evaluations;
          break;
        case ShapeSearchStage::kRecovery:
          ++result.recovery_evaluations;
          break;
        case ShapeSearchStage::kRefinement:
          ++result.refinement_evaluations;
          break;
      }
    };

  for (const double fraction : kCoarseFractions) {
    evaluate_unique(fraction, ShapeSearchStage::kCoarse);
  }
  std::size_t winner = best_sample_index(result.samples);
  if (winner == result.samples.size()) {
    for (const double fraction : kRecoveryFractions) {
      evaluate_unique(fraction, ShapeSearchStage::kRecovery);
    }
    winner = best_sample_index(result.samples);
  }
  if (winner == result.samples.size()) {
    return result;
  }

  const double winning_fraction = result.samples[winner].control_fraction;
  double lower = kCoarseFractions.front();
  double upper = kCoarseFractions.back();
  const auto coarse_match = std::find_if(
    kCoarseFractions.begin(), kCoarseFractions.end(),
    [winning_fraction](double value) {
      return std::abs(value - winning_fraction) <= kEpsilon;
    });
  if (coarse_match != kCoarseFractions.end()) {
    const std::size_t coarse_index = static_cast<std::size_t>(
      std::distance(kCoarseFractions.begin(), coarse_match));
    lower = coarse_index == 0U ?
      kCoarseFractions[0U] : kCoarseFractions[coarse_index - 1U];
    upper = coarse_index + 1U == kCoarseFractions.size() ?
      kCoarseFractions.back() : kCoarseFractions[coarse_index + 1U];
  } else {
    const auto right = std::upper_bound(
      kCoarseFractions.begin(), kCoarseFractions.end(), winning_fraction);
    if (right == kCoarseFractions.begin() || right == kCoarseFractions.end()) {
      return result;
    }
    upper = *right;
    lower = *(right - 1);
  }

  for (std::size_t index = 0U; index <= kRefinementIntervals; ++index) {
    const double ratio = static_cast<double>(index) /
      static_cast<double>(kRefinementIntervals);
    evaluate_unique(lower + ratio * (upper - lower), ShapeSearchStage::kRefinement);
  }

  winner = best_sample_index(result.samples);
  if (winner != result.samples.size()) {
    result.valid = true;
    result.selected_sample_index = winner;
  }
  return result;
}

DerivedTrimCandidates derive_two_trim_candidates(
  double incoming_length,
  double outgoing_length,
  bool has_previous_corner,
  bool has_next_corner,
  double maximum_trim,
  double minimum_trim,
  double segment_margin,
  double deduplication_resolution)
{
  DerivedTrimCandidates result;
  if (!std::isfinite(incoming_length) || incoming_length <= 0.0 ||
    !std::isfinite(outgoing_length) || outgoing_length <= 0.0 ||
    !std::isfinite(maximum_trim) || maximum_trim <= 0.0 ||
    !std::isfinite(minimum_trim) || minimum_trim <= 0.0 ||
    maximum_trim < minimum_trim ||
    !std::isfinite(segment_margin) || segment_margin < 0.0 ||
    !std::isfinite(deduplication_resolution) || deduplication_resolution < 0.0)
  {
    return result;
  }

  result.valid = true;
  result.preferred_trim = std::min({maximum_trim, incoming_length, outgoing_length});
  if (result.preferred_trim + kEpsilon >= minimum_trim) {
    result.values.push_back(result.preferred_trim);
  }

  const double incoming_budget = has_previous_corner ?
    0.5 * std::max(0.0, incoming_length - segment_margin) : incoming_length;
  const double outgoing_budget = has_next_corner ?
    0.5 * std::max(0.0, outgoing_length - segment_margin) : outgoing_length;
  result.compatible_trim = std::min({
      result.preferred_trim, incoming_budget, outgoing_budget});
  if (result.compatible_trim + kEpsilon >= minimum_trim &&
    (result.values.empty() ||
    result.preferred_trim - result.compatible_trim > deduplication_resolution + kEpsilon))
  {
    result.values.push_back(result.compatible_trim);
  }
  return result;
}

}  // namespace adaptive_pivot_g2
