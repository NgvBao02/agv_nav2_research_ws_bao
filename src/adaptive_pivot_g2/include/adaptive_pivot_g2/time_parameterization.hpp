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

#ifndef ADAPTIVE_PIVOT_G2__TIME_PARAMETERIZATION_HPP_
#define ADAPTIVE_PIVOT_G2__TIME_PARAMETERIZATION_HPP_

#include <vector>

#include "adaptive_pivot_g2/types.hpp"

namespace adaptive_pivot_g2
{

TimedProfile parameterize_time(
  const std::vector<PathSample> & path,
  const RobotLimits & limits,
  double start_speed,
  double end_speed);

/// Parameterize a transition inside a common symmetric corner window.
///
/// Straight context is inserted before and after candidates whose trim is
/// smaller than window_trim, so acceleration and angular-acceleration limits
/// are solved continuously instead of adding optimistic distance / speed terms.
TimedProfile parameterize_transition_window(
  const TransitionCandidate & candidate,
  double window_trim,
  const RobotLimits & limits,
  double start_speed,
  double end_speed,
  double sample_spacing);

double minimum_translation_time(
  double length,
  double start_speed,
  double end_speed,
  double max_speed,
  double acceleration,
  double deceleration);

double minimum_rotation_time(
  double angle,
  double max_angular_speed,
  double max_angular_acceleration);

double estimate_pivot_window_time(
  double trim_distance,
  double turn_angle,
  const RobotLimits & limits,
  double entry_speed,
  double exit_speed);

}  // namespace adaptive_pivot_g2

#endif  // ADAPTIVE_PIVOT_G2__TIME_PARAMETERIZATION_HPP_
