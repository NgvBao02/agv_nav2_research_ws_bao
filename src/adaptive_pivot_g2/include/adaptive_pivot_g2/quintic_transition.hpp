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

}  // namespace adaptive_pivot_g2

#endif  // ADAPTIVE_PIVOT_G2__QUINTIC_TRANSITION_HPP_
