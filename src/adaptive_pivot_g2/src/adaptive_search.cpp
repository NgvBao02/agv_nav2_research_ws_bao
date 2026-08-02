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

#include "adaptive_pivot_g2/adaptive_search.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace adaptive_pivot_g2
{
namespace
{

constexpr double kEpsilon = 1.0e-12;

bool options_are_valid(const AdaptiveSearchOptions & options)
{
  return std::isfinite(options.minimum_radius) && options.minimum_radius > 0.0 &&
         std::isfinite(options.maximum_radius) &&
         options.maximum_radius >= options.minimum_radius &&
         options.initial_samples >= 2U &&
         options.maximum_evaluations >= options.initial_samples &&
         std::isfinite(options.radius_tolerance) && options.radius_tolerance > 0.0 &&
         std::isfinite(options.objective_tolerance) &&
         options.objective_tolerance >= 0.0;
}

bool feasible(const SearchSample & sample)
{
  return sample.status == SearchSampleStatus::kFeasible &&
         std::isfinite(sample.objective) && sample.objective >= 0.0;
}

void sort_samples(std::vector<SearchSample> & samples)
{
  std::sort(
    samples.begin(), samples.end(),
    [](const SearchSample & lhs, const SearchSample & rhs) {
      return lhs.trim_distance < rhs.trim_distance;
    });
}

struct IntervalChoice
{
  bool valid{false};
  int priority{std::numeric_limits<int>::max()};
  double width{0.0};
  double left_trim{0.0};
  double midpoint{0.0};
};

bool better_interval(const IntervalChoice & candidate, const IntervalChoice & current)
{
  if (!current.valid || candidate.priority < current.priority) {
    return true;
  }
  if (candidate.priority > current.priority) {
    return false;
  }
  if (candidate.width > current.width + kEpsilon) {
    return true;
  }
  if (std::abs(candidate.width - current.width) <= kEpsilon) {
    return candidate.left_trim < current.left_trim;
  }
  return false;
}

}  // namespace

namespace
{

AdaptiveSearchResult search_trim_domain(
  double absolute_turn_angle,
  double minimum_trim,
  double maximum_trim,
  double effective_trim_tolerance,
  const AdaptiveSearchOptions & options,
  const TrimEvaluator & evaluator)
{
  AdaptiveSearchResult result;
  if (!options_are_valid(options) || !evaluator ||
    !std::isfinite(absolute_turn_angle) || absolute_turn_angle <= 0.0 ||
    absolute_turn_angle >= 3.14159265358979323846 ||
    !std::isfinite(minimum_trim) || minimum_trim <= 0.0 ||
    !std::isfinite(maximum_trim) || maximum_trim < minimum_trim ||
    !std::isfinite(effective_trim_tolerance) || effective_trim_tolerance <= 0.0)
  {
    return result;
  }

  const double tangent = std::tan(0.5 * absolute_turn_angle);
  if (!std::isfinite(tangent) || tangent <= kEpsilon) {
    return result;
  }
  result.minimum_trim = minimum_trim;
  result.maximum_trim = maximum_trim;
  result.effective_trim_tolerance = effective_trim_tolerance;
  if (!std::isfinite(result.minimum_trim) || !std::isfinite(result.maximum_trim) ||
    result.minimum_trim > result.maximum_trim + kEpsilon)
  {
    return result;
  }
  result.valid_domain = true;

  const auto evaluate = [&](double trim) {
      const SearchEvaluation evaluation = evaluator(trim);
      SearchSample sample;
      sample.trim_distance = trim;
      sample.design_radius = trim / tangent;
      sample.status = evaluation.status;
      sample.objective = evaluation.objective;
      sample.payload_index = evaluation.payload_index;
      sample.rejection_reason = evaluation.rejection_reason;
      if (sample.status == SearchSampleStatus::kFeasible &&
        (!std::isfinite(sample.objective) || sample.objective < 0.0))
      {
        sample.status = SearchSampleStatus::kInfeasible;
        sample.objective = std::numeric_limits<double>::infinity();
        sample.rejection_reason = "evaluator returned an invalid objective";
      }
      result.samples.push_back(std::move(sample));
    };

  const std::size_t initial_count = std::min(
    options.initial_samples, options.maximum_evaluations);
  if (result.maximum_trim - result.minimum_trim <= kEpsilon) {
    evaluate(result.minimum_trim);
  } else {
    for (std::size_t index = 0; index < initial_count; ++index) {
      const double fraction = static_cast<double>(index) /
        static_cast<double>(initial_count - 1U);
      evaluate(
        result.minimum_trim +
        fraction * (result.maximum_trim - result.minimum_trim));
    }
  }
  sort_samples(result.samples);

  while (result.samples.size() < options.maximum_evaluations) {
    double best_objective = std::numeric_limits<double>::infinity();
    double worst_objective = -std::numeric_limits<double>::infinity();
    bool has_status_boundary = false;
    for (const auto & sample : result.samples) {
      if (feasible(sample)) {
        best_objective = std::min(best_objective, sample.objective);
        worst_objective = std::max(worst_objective, sample.objective);
      }
    }
    for (std::size_t index = 1; index < result.samples.size(); ++index) {
      has_status_boundary = has_status_boundary ||
        result.samples[index - 1U].status != result.samples[index].status;
    }
    if (!has_status_boundary && std::isfinite(best_objective) &&
      worst_objective - best_objective <= options.objective_tolerance)
    {
      break;
    }

    IntervalChoice selected;
    for (std::size_t index = 1; index < result.samples.size(); ++index) {
      const SearchSample & left = result.samples[index - 1U];
      const SearchSample & right = result.samples[index];
      const double width = right.trim_distance - left.trim_distance;
      if (width <= result.effective_trim_tolerance + kEpsilon) {
        continue;
      }
      IntervalChoice candidate;
      candidate.valid = true;
      candidate.width = width;
      candidate.left_trim = left.trim_distance;
      candidate.midpoint = 0.5 * (left.trim_distance + right.trim_distance);
      const bool adjacent_to_best =
        (feasible(left) &&
        left.objective <= best_objective + options.objective_tolerance) ||
        (feasible(right) &&
        right.objective <= best_objective + options.objective_tolerance);
      const bool varying_feasible_objective =
        feasible(left) && feasible(right) &&
        std::abs(left.objective - right.objective) >
        options.objective_tolerance;
      if (left.status != right.status) {
        candidate.priority = 0;
      } else if (adjacent_to_best) {
        candidate.priority = 1;
      } else if (varying_feasible_objective) {
        candidate.priority = 2;
      } else if (feasible(left) || feasible(right)) {
        candidate.priority = 3;
      } else {
        candidate.priority = 4;
      }
      if (better_interval(candidate, selected)) {
        selected = candidate;
      }
    }
    if (!selected.valid) {
      break;
    }
    evaluate(selected.midpoint);
    sort_samples(result.samples);
  }

  for (std::size_t index = 0; index < result.samples.size(); ++index) {
    if (feasible(result.samples[index])) {
      result.ranked_feasible_samples.push_back(index);
    }
  }
  std::sort(
    result.ranked_feasible_samples.begin(), result.ranked_feasible_samples.end(),
    [&result](std::size_t lhs, std::size_t rhs) {
      const SearchSample & left = result.samples[lhs];
      const SearchSample & right = result.samples[rhs];
      if (std::abs(left.objective - right.objective) > kEpsilon) {
        return left.objective < right.objective;
      }
      return left.trim_distance < right.trim_distance;
    });
  result.feasible_count = result.ranked_feasible_samples.size();
  return result;
}

}  // namespace

AdaptiveSearchResult search_trim_distance(
  double absolute_turn_angle,
  double maximum_geometric_trim,
  double minimum_meaningful_trim_resolution,
  const AdaptiveSearchOptions & options,
  const TrimEvaluator & evaluator)
{
  if (!options_are_valid(options) ||
    !std::isfinite(absolute_turn_angle) || absolute_turn_angle <= 0.0 ||
    absolute_turn_angle >= 3.14159265358979323846 ||
    !std::isfinite(maximum_geometric_trim) || maximum_geometric_trim <= 0.0 ||
    !std::isfinite(minimum_meaningful_trim_resolution) ||
    minimum_meaningful_trim_resolution < 0.0)
  {
    return {};
  }
  const double tangent = std::tan(0.5 * absolute_turn_angle);
  if (!std::isfinite(tangent) || tangent <= kEpsilon) {
    return {};
  }
  const double minimum_trim = options.minimum_radius * tangent;
  const double maximum_trim = std::min(
    options.maximum_radius * tangent, maximum_geometric_trim);
  if (minimum_trim > maximum_trim + kEpsilon) {
    return {};
  }
  return search_trim_domain(
    absolute_turn_angle, minimum_trim, maximum_trim,
    std::max(
      options.radius_tolerance * tangent,
      minimum_meaningful_trim_resolution),
    options, evaluator);
}

AdaptiveSearchResult search_direct_trim_distance(
  double absolute_turn_angle,
  double minimum_trim,
  double maximum_trim,
  double minimum_meaningful_trim_resolution,
  const AdaptiveSearchOptions & options,
  const TrimEvaluator & evaluator)
{
  if (!std::isfinite(minimum_meaningful_trim_resolution) ||
    minimum_meaningful_trim_resolution < 0.0)
  {
    return {};
  }
  return search_trim_domain(
    absolute_turn_angle, minimum_trim, maximum_trim,
    std::max(options.radius_tolerance, minimum_meaningful_trim_resolution),
    options, evaluator);
}

}  // namespace adaptive_pivot_g2
