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

#ifndef ADAPTIVE_PIVOT_G2__QUINTIC_TRANSITION_HPP_
#define ADAPTIVE_PIVOT_G2__QUINTIC_TRANSITION_HPP_

#include <cstddef>
#include <vector>

#include "adaptive_pivot_g2/types.hpp"

namespace adaptive_pivot_g2
{

TransitionCandidate generate_quintic_transition(
  const CornerInput & corner,
  const RobotLimits & limits,
  const TransitionOptions & options);

/// Generate the same quintic G2 curve using an explicit trim distance.
///
/// This entry point deliberately does not apply max_trim_fraction. The caller
/// must derive the geometric domain and enforce inter-corner overlap.
TransitionCandidate generate_quintic_transition_for_trim(
  const CornerInput & corner,
  const RobotLimits & limits,
  const TransitionOptions & options,
  double trim_distance);

/// Return an angle-aware q/d value close to the minimum-curvature-energy
/// shape of this symmetric quintic family. The result is only a search centre;
/// obstacle-aware optimization must still evaluate neighboring shapes.
double recommended_control_fraction(double absolute_turn_angle);

/// Generate a deterministic, sorted q/d search bank around the angle-aware
/// centre. The odd sample count includes the centre and reaches both bounds.
std::vector<double> generate_control_fraction_candidates(
  double absolute_turn_angle,
  double minimum_fraction,
  double maximum_fraction,
  std::size_t sample_count);

/// Generate a quintic G2 transition for an explicit pair (d, q/d).
///
/// The construction keeps P1-P0 == P2-P1 and P5-P4 == P4-P3, so the second
/// derivative and curvature remain zero at both straight-line junctions for
/// every valid control_fraction. The caller owns the two-dimensional search.
TransitionCandidate generate_quintic_transition_for_shape(
  const CornerInput & corner,
  const RobotLimits & limits,
  const TransitionOptions & options,
  double trim_distance,
  double control_fraction);

}  // namespace adaptive_pivot_g2

#endif  // ADAPTIVE_PIVOT_G2__QUINTIC_TRANSITION_HPP_
