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

#ifndef ADAPTIVE_PIVOT_G2__CANDIDATE_SELECTION_HPP_
#define ADAPTIVE_PIVOT_G2__CANDIDATE_SELECTION_HPP_

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace adaptive_pivot_g2
{

struct CandidateObjective
{
  std::size_t candidate_index{0};
  double common_window_time{std::numeric_limits<double>::infinity()};
  // A larger value means more obstacle clearance.  The caller may provide a
  // metric clearance or a monotonic costmap-derived clearance proxy.
  double clearance{0.0};
  double max_abs_angular_speed{std::numeric_limits<double>::infinity()};
  double curvature_energy{std::numeric_limits<double>::infinity()};
};

struct SelectionWeights
{
  double clearance{0.15};
  double angular_speed{0.10};
  double curvature_energy{0.75};
};

struct CandidateSelection
{
  bool valid{false};
  std::size_t candidate_index{0};
  std::size_t competitive_count{0};
  double fastest_time{std::numeric_limits<double>::infinity()};
  double selected_time{std::numeric_limits<double>::infinity()};
  double selected_score{-std::numeric_limits<double>::infinity()};
};

/// Select the highest-utility candidate inside a near-fastest time gate.
///
/// The three objectives are min-max normalized only across candidates whose
/// common-window time is no more than fastest_time + time_slack.  Clearance is
/// maximized; angular speed and curvature energy are minimized.  Equal scores
/// are resolved by lower traversal time and then by candidate index.
CandidateSelection select_competitive_candidate(
  const std::vector<CandidateObjective> & candidates,
  double time_slack,
  const SelectionWeights & weights);

/// Return a stable dimensionless cost for an already safe candidate.
///
/// peak_cost is normalized against 252, angular speed against the configured
/// robot limit, and curvature energy E [1/m] as E / (E + scale). Lower is
/// better. Infinity denotes invalid input.
double stable_candidate_cost(
  double peak_cost,
  double max_abs_angular_speed,
  double curvature_energy,
  double max_angular_speed,
  double curvature_energy_scale,
  const SelectionWeights & weights);

}  // namespace adaptive_pivot_g2

#endif  // ADAPTIVE_PIVOT_G2__CANDIDATE_SELECTION_HPP_
