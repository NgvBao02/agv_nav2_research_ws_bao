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

#ifndef ADAPTIVE_PIVOT_G2__ADAPTIVE_SEARCH_HPP_
#define ADAPTIVE_PIVOT_G2__ADAPTIVE_SEARCH_HPP_

#include <cstddef>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace adaptive_pivot_g2
{

enum class SearchSampleStatus
{
  kFeasible,
  kInfeasible,
  kUnsafe
};

struct AdaptiveSearchOptions
{
  double minimum_radius{0.10};
  double maximum_radius{1.50};
  std::size_t initial_samples{6};
  std::size_t maximum_evaluations{20};
  double radius_tolerance{0.01};
  double objective_tolerance{0.01};
};

struct SearchEvaluation
{
  SearchSampleStatus status{SearchSampleStatus::kInfeasible};
  double objective{std::numeric_limits<double>::infinity()};
  std::size_t payload_index{0};
  std::string rejection_reason;
};

struct SearchSample
{
  double trim_distance{0.0};
  double design_radius{0.0};
  SearchSampleStatus status{SearchSampleStatus::kInfeasible};
  double objective{std::numeric_limits<double>::infinity()};
  std::size_t payload_index{0};
  std::string rejection_reason;
};

struct AdaptiveSearchResult
{
  bool valid_domain{false};
  double minimum_trim{0.0};
  double maximum_trim{0.0};
  double effective_trim_tolerance{0.0};
  std::size_t feasible_count{0};
  std::vector<SearchSample> samples;
  std::vector<std::size_t> ranked_feasible_samples;
};

using TrimEvaluator = std::function<SearchEvaluation(double)>;

/// Deterministically search trim distance rather than design radius.
///
/// The initial grid spans the full geometric domain. Refinement prioritizes
/// feasible/infeasible and safe/unsafe boundaries, then intervals adjacent to
/// the best objective values. The remaining intervals stay eligible so the
/// search does not assume a smooth or unimodal costmap objective.
AdaptiveSearchResult search_trim_distance(
  double absolute_turn_angle,
  double maximum_geometric_trim,
  double minimum_meaningful_trim_resolution,
  const AdaptiveSearchOptions & options,
  const TrimEvaluator & evaluator);

}  // namespace adaptive_pivot_g2

#endif  // ADAPTIVE_PIVOT_G2__ADAPTIVE_SEARCH_HPP_
