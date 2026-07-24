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

#ifndef ADAPTIVE_PIVOT_G2__PATH_OPTIMIZATION_HPP_
#define ADAPTIVE_PIVOT_G2__PATH_OPTIMIZATION_HPP_

#include <cstddef>
#include <limits>
#include <vector>

namespace adaptive_pivot_g2
{

struct CornerState
{
  bool pivot{true};
  double trim_distance{0.0};
  double local_cost{std::numeric_limits<double>::infinity()};
  std::size_t payload_index{0};
};

struct PathOptimizationResult
{
  bool valid{false};
  double total_cost{std::numeric_limits<double>::infinity()};
  std::size_t state_count{0};
  std::size_t compatible_edge_count{0};
  std::size_t pivot_count{0};
  std::vector<std::size_t> selected_state_indices;
};

/// Select one state per corner using O(N K^2) dynamic programming.
///
/// shared_segment_lengths and segment_margins both contain one entry between
/// each pair of adjacent corners. A pivot has trim_distance == 0.
PathOptimizationResult optimize_corner_states(
  const std::vector<std::vector<CornerState>> & corner_states,
  const std::vector<double> & shared_segment_lengths,
  const std::vector<double> & segment_margins);

}  // namespace adaptive_pivot_g2

#endif  // ADAPTIVE_PIVOT_G2__PATH_OPTIMIZATION_HPP_
