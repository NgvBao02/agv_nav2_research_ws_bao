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

#include "adaptive_pivot_g2/path_optimization.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace adaptive_pivot_g2
{
namespace
{

constexpr double kEpsilon = 1.0e-12;
constexpr std::size_t kNoState = std::numeric_limits<std::size_t>::max();

bool valid_state(const CornerState & state)
{
  return std::isfinite(state.trim_distance) && state.trim_distance >= 0.0 &&
         std::isfinite(state.local_cost) && state.local_cost >= 0.0 &&
         (!state.pivot || state.trim_distance <= kEpsilon);
}

bool compatible(
  const CornerState & left,
  const CornerState & right,
  double segment_length,
  double margin)
{
  return left.trim_distance + right.trim_distance + margin <=
         segment_length + kEpsilon;
}

}  // namespace

PathOptimizationResult optimize_corner_states(
  const std::vector<std::vector<CornerState>> & corner_states,
  const std::vector<double> & shared_segment_lengths,
  const std::vector<double> & segment_margins)
{
  PathOptimizationResult result;
  if (corner_states.empty()) {
    result.valid = true;
    result.total_cost = 0.0;
    return result;
  }
  if (shared_segment_lengths.size() + 1U != corner_states.size() ||
    segment_margins.size() != shared_segment_lengths.size())
  {
    return result;
  }
  for (const auto & states : corner_states) {
    if (states.empty()) {
      return result;
    }
    result.state_count += states.size();
    if (std::none_of(states.begin(), states.end(), valid_state)) {
      return result;
    }
  }
  for (std::size_t index = 0; index < shared_segment_lengths.size(); ++index) {
    if (!std::isfinite(shared_segment_lengths[index]) ||
      shared_segment_lengths[index] <= 0.0 ||
      !std::isfinite(segment_margins[index]) || segment_margins[index] < 0.0)
    {
      return result;
    }
  }

  std::vector<std::vector<std::size_t>> predecessor(corner_states.size());
  std::vector<double> previous_cost(corner_states.front().size(),
    std::numeric_limits<double>::infinity());
  std::vector<std::size_t> previous_pivots(corner_states.front().size(), kNoState);
  predecessor.front().assign(corner_states.front().size(), kNoState);
  for (std::size_t state_index = 0; state_index < corner_states.front().size();
    ++state_index)
  {
    const CornerState & state = corner_states.front()[state_index];
    if (valid_state(state)) {
      previous_cost[state_index] = state.local_cost;
      previous_pivots[state_index] = state.pivot ? 1U : 0U;
    }
  }

  for (std::size_t corner_index = 1; corner_index < corner_states.size();
    ++corner_index)
  {
    const auto & previous_states = corner_states[corner_index - 1U];
    const auto & current_states = corner_states[corner_index];
    std::vector<double> current_cost(
      current_states.size(), std::numeric_limits<double>::infinity());
    std::vector<std::size_t> current_pivots(current_states.size(), kNoState);
    predecessor[corner_index].assign(current_states.size(), kNoState);
    for (std::size_t current_index = 0; current_index < current_states.size();
      ++current_index)
    {
      if (!valid_state(current_states[current_index])) {
        continue;
      }
      for (std::size_t previous_index = 0; previous_index < previous_states.size();
        ++previous_index)
      {
        if (!valid_state(previous_states[previous_index]) ||
          !compatible(
            previous_states[previous_index], current_states[current_index],
            shared_segment_lengths[corner_index - 1U],
            segment_margins[corner_index - 1U]))
        {
          continue;
        }
        ++result.compatible_edge_count;
        if (!std::isfinite(previous_cost[previous_index])) {
          continue;
        }
        const double candidate_cost =
          previous_cost[previous_index] + current_states[current_index].local_cost;
        const std::size_t candidate_pivots =
          previous_pivots[previous_index] +
          (current_states[current_index].pivot ? 1U : 0U);
        const bool lower_cost =
          candidate_cost < current_cost[current_index] - kEpsilon;
        const bool same_cost =
          std::abs(candidate_cost - current_cost[current_index]) <= kEpsilon;
        const bool fewer_pivots =
          candidate_pivots < current_pivots[current_index];
        const bool same_pivots =
          candidate_pivots == current_pivots[current_index];
        if (lower_cost ||
          (same_cost && (fewer_pivots ||
          (same_pivots &&
          previous_index < predecessor[corner_index][current_index]))))
        {
          current_cost[current_index] = candidate_cost;
          current_pivots[current_index] = candidate_pivots;
          predecessor[corner_index][current_index] = previous_index;
        }
      }
    }
    previous_cost = std::move(current_cost);
    previous_pivots = std::move(current_pivots);
  }

  std::size_t selected = kNoState;
  for (std::size_t index = 0; index < previous_cost.size(); ++index) {
    if (!std::isfinite(previous_cost[index])) {
      continue;
    }
    if (selected == kNoState ||
      previous_cost[index] < previous_cost[selected] - kEpsilon ||
      (std::abs(previous_cost[index] - previous_cost[selected]) <= kEpsilon &&
      (previous_pivots[index] < previous_pivots[selected] ||
      (previous_pivots[index] == previous_pivots[selected] && index < selected))))
    {
      selected = index;
    }
  }
  if (selected == kNoState) {
    return result;
  }

  result.selected_state_indices.resize(corner_states.size(), 0U);
  for (std::size_t reverse_index = corner_states.size(); reverse_index > 0U;
    --reverse_index)
  {
    const std::size_t corner_index = reverse_index - 1U;
    result.selected_state_indices[corner_index] = selected;
    if (corner_states[corner_index][selected].pivot) {
      ++result.pivot_count;
    }
    if (corner_index > 0U) {
      selected = predecessor[corner_index][selected];
      if (selected == kNoState) {
        result.selected_state_indices.clear();
        result.pivot_count = 0U;
        return result;
      }
    }
  }
  result.valid = true;
  result.total_cost = previous_cost[result.selected_state_indices.back()];
  return result;
}

}  // namespace adaptive_pivot_g2
