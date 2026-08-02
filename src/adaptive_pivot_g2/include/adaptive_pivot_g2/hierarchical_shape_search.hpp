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

#ifndef ADAPTIVE_PIVOT_G2__HIERARCHICAL_SHAPE_SEARCH_HPP_
#define ADAPTIVE_PIVOT_G2__HIERARCHICAL_SHAPE_SEARCH_HPP_

#include <cstddef>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace adaptive_pivot_g2
{

enum class ShapeSearchStage
{
  kCoarse,
  kRecovery,
  kRefinement
};

struct ShapeSearchEvaluation
{
  bool feasible{false};
  double curvature_energy{std::numeric_limits<double>::infinity()};
  std::size_t payload_index{0U};
  std::string rejection_reason;
};

struct ShapeSearchSample
{
  double control_fraction{0.0};
  ShapeSearchStage stage{ShapeSearchStage::kCoarse};
  ShapeSearchEvaluation evaluation;
};

struct HierarchicalShapeSearchResult
{
  bool valid{false};
  std::size_t selected_sample_index{0U};
  std::size_t coarse_evaluations{0U};
  std::size_t recovery_evaluations{0U};
  std::size_t refinement_evaluations{0U};
  std::vector<ShapeSearchSample> samples;
};

using ShapeEvaluator = std::function<ShapeSearchEvaluation(double)>;

/// Search q/d using the fixed PSTMO coarse-to-fine domain.
///
/// The primary grid is {0.1, ..., 0.5}. If it has no feasible sample, the
/// half-step recovery grid {0.15, ..., 0.45} is evaluated. The winning coarse
/// point is refined between both adjacent coarse anchors; a recovery winner is
/// refined inside its containing coarse cell. Every refinement interval is
/// split into ten equal parts and previously evaluated values are deduplicated.
HierarchicalShapeSearchResult search_control_fraction_coarse_to_fine(
  const ShapeEvaluator & evaluator);

struct DerivedTrimCandidates
{
  bool valid{false};
  double preferred_trim{0.0};
  double compatible_trim{0.0};
  std::vector<double> values;
};

/// Derive the preferred and globally compatible trim distances without a d grid.
///
/// An internal shared segment contributes half of its length after the segment
/// margin. Start/goal segments expose their complete length. The compatible
/// value is omitted when it is below the meaningful minimum or duplicates the
/// preferred value within the supplied resolution.
DerivedTrimCandidates derive_two_trim_candidates(
  double incoming_length,
  double outgoing_length,
  bool has_previous_corner,
  bool has_next_corner,
  double maximum_trim,
  double minimum_trim,
  double segment_margin,
  double deduplication_resolution);

}  // namespace adaptive_pivot_g2

#endif  // ADAPTIVE_PIVOT_G2__HIERARCHICAL_SHAPE_SEARCH_HPP_
