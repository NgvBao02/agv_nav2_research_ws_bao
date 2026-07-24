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

#include "adaptive_pivot_g2/path_conditioning.hpp"

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

double point_to_segment_distance(
  const Vec2 & point,
  const Vec2 & start,
  const Vec2 & finish)
{
  const Vec2 delta = finish - start;
  const double squared_length = dot(delta, delta);
  if (squared_length <= kEpsilon) {
    return distance(point, start);
  }
  const double fraction = std::clamp(
    dot(point - start, delta) / squared_length, 0.0, 1.0);
  return distance(point, start + delta * fraction);
}

struct Range
{
  std::size_t first{0};
  std::size_t last{0};
};

double maximum_range_deviation(
  const std::vector<Vec2> & input,
  std::size_t first,
  std::size_t last)
{
  double maximum_deviation = 0.0;
  for (std::size_t index = first + 1U; index < last; ++index) {
    maximum_deviation = std::max(
      maximum_deviation,
      point_to_segment_distance(input[index], input[first], input[last]));
  }
  return maximum_deviation;
}

bool range_is_oscillatory(
  const std::vector<Vec2> & input,
  std::size_t first,
  std::size_t last,
  double minimum_turn_angle,
  std::size_t minimum_sign_changes)
{
  int previous_sign = 0;
  std::size_t sign_changes = 0U;
  for (std::size_t index = first + 1U; index < last; ++index) {
    const Vec2 incoming = input[index] - input[index - 1U];
    const Vec2 outgoing = input[index + 1U] - input[index];
    const double incoming_length = norm(incoming);
    const double outgoing_length = norm(outgoing);
    if (incoming_length <= kEpsilon || outgoing_length <= kEpsilon) {
      continue;
    }
    const double turn = std::atan2(
      cross(incoming, outgoing), dot(incoming, outgoing));
    if (std::abs(turn) < minimum_turn_angle) {
      continue;
    }
    const int sign = turn > 0.0 ? 1 : -1;
    if (previous_sign != 0 && sign != previous_sign) {
      ++sign_changes;
    }
    previous_sign = sign;
  }
  return sign_changes >= minimum_sign_changes;
}

void suppress_safe_oscillations(
  PathConditioningResult & result,
  const PathConditioningOptions & options,
  const SegmentSafetyPredicate & segment_is_safe)
{
  if (result.points.size() < 4U ||
    options.oscillation_maximum_span <= kEpsilon ||
    options.oscillation_maximum_deviation <= kEpsilon)
  {
    return;
  }

  const std::vector<Vec2> input = result.points;
  const std::vector<std::size_t> input_indices = result.retained_input_indices;
  std::vector<Vec2> output{input.front()};
  std::vector<std::size_t> output_indices{input_indices.front()};
  std::size_t anchor = 0U;
  while (anchor + 1U < input.size()) {
    std::size_t selected = anchor + 1U;
    double span = 0.0;
    for (std::size_t finish = anchor + 1U; finish < input.size(); ++finish) {
      span += distance(input[finish - 1U], input[finish]);
      if (span > options.oscillation_maximum_span + kEpsilon) {
        break;
      }
      if (finish < anchor + 3U ||
        !range_is_oscillatory(
          input, anchor, finish, options.oscillation_minimum_turn_angle,
          options.oscillation_minimum_sign_changes))
      {
        continue;
      }
      const double deviation = maximum_range_deviation(
        input, anchor, finish);
      if (deviation > options.oscillation_maximum_deviation + kEpsilon) {
        continue;
      }
      if (!segment_is_safe(input[anchor], input[finish])) {
        ++result.safety_rejected_shortcuts;
        continue;
      }
      selected = finish;
      result.maximum_removed_deviation = std::max(
        result.maximum_removed_deviation, deviation);
    }
    if (selected > anchor + 1U) {
      ++result.accepted_shortcuts;
      ++result.accepted_oscillation_shortcuts;
    }
    output.push_back(input[selected]);
    output_indices.push_back(input_indices[selected]);
    anchor = selected;
  }
  result.points = std::move(output);
  result.retained_input_indices = std::move(output_indices);
}

}  // namespace

PathConditioningResult condition_polyline(
  const std::vector<Vec2> & input,
  double maximum_deviation,
  const SegmentSafetyPredicate & segment_is_safe)
{
  PathConditioningOptions options;
  options.maximum_deviation = maximum_deviation;
  return condition_polyline(input, options, segment_is_safe);
}

PathConditioningResult condition_polyline(
  const std::vector<Vec2> & input,
  const PathConditioningOptions & options,
  const SegmentSafetyPredicate & segment_is_safe)
{
  PathConditioningResult result;
  if (input.size() < 2U) {
    result.rejection_reason = "polyline needs at least two points";
    return result;
  }
  if (!std::isfinite(options.maximum_deviation) ||
    options.maximum_deviation < 0.0 ||
    !std::isfinite(options.oscillation_maximum_span) ||
    options.oscillation_maximum_span < 0.0 ||
    !std::isfinite(options.oscillation_maximum_deviation) ||
    options.oscillation_maximum_deviation < 0.0 ||
    !std::isfinite(options.oscillation_minimum_turn_angle) ||
    options.oscillation_minimum_turn_angle < 0.0 ||
    options.oscillation_minimum_turn_angle > std::acos(-1.0) ||
    options.oscillation_minimum_sign_changes < 1U)
  {
    result.rejection_reason = "path conditioning options are invalid";
    return result;
  }
  if (!segment_is_safe) {
    result.rejection_reason = "segment safety predicate is empty";
    return result;
  }
  if (std::any_of(
      input.begin(), input.end(),
      [](const Vec2 & point) {return !finite(point);}))
  {
    result.rejection_reason = "polyline contains a non-finite point";
    return result;
  }

  if (options.maximum_deviation <= kEpsilon || input.size() == 2U) {
    result.points = input;
    result.retained_input_indices.resize(input.size());
    for (std::size_t index = 0; index < input.size(); ++index) {
      result.retained_input_indices[index] = index;
    }
    result.valid = true;
    suppress_safe_oscillations(result, options, segment_is_safe);
    return result;
  }

  // Iterative bounded Ramer-Douglas-Peucker traversal avoids recursion-depth
  // dependence on long planner paths. Ranges rejected by the swept-footprint
  // predicate are split even when their geometry is perfectly collinear.
  std::vector<bool> retained(input.size(), false);
  retained.front() = true;
  retained.back() = true;
  std::vector<Range> pending{{0U, input.size() - 1U}};
  while (!pending.empty()) {
    const Range range = pending.back();
    pending.pop_back();
    if (range.last <= range.first + 1U) {
      continue;
    }

    std::size_t split_index = range.first + 1U;
    double farthest_deviation = -std::numeric_limits<double>::infinity();
    for (std::size_t index = range.first + 1U; index < range.last; ++index) {
      const double deviation = point_to_segment_distance(
        input[index], input[range.first], input[range.last]);
      if (deviation > farthest_deviation) {
        farthest_deviation = deviation;
        split_index = index;
      }
    }

    if (farthest_deviation <= options.maximum_deviation + kEpsilon) {
      if (segment_is_safe(input[range.first], input[range.last])) {
        ++result.accepted_shortcuts;
        result.maximum_removed_deviation = std::max(
          result.maximum_removed_deviation, farthest_deviation);
        continue;
      }
      ++result.safety_rejected_shortcuts;
      // A collinear unsafe chord has no meaningful farthest point. Bisecting
      // prevents a strongly left-biased split and keeps runtime predictable.
      if (farthest_deviation <= kEpsilon) {
        split_index = range.first + (range.last - range.first) / 2U;
      }
    }

    retained[split_index] = true;
    // Reverse push order gives deterministic left-to-right processing.
    pending.push_back({split_index, range.last});
    pending.push_back({range.first, split_index});
  }

  result.points.reserve(input.size());
  result.retained_input_indices.reserve(input.size());
  for (std::size_t index = 0; index < input.size(); ++index) {
    if (retained[index]) {
      result.points.push_back(input[index]);
      result.retained_input_indices.push_back(index);
    }
  }
  result.valid = result.points.size() >= 2U;
  if (!result.valid) {
    result.rejection_reason = "conditioned polyline has fewer than two points";
  } else {
    suppress_safe_oscillations(result, options, segment_is_safe);
  }
  return result;
}

}  // namespace adaptive_pivot_g2
