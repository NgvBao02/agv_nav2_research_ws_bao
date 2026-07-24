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

#include "adaptive_pivot_g2/time_parameterization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace adaptive_pivot_g2
{
namespace
{

constexpr double kEpsilon = 1.0e-10;

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

void enforce_linear_acceleration_limits(
  const std::vector<double> & segment_lengths,
  const RobotLimits & limits,
  const std::vector<double> & caps,
  std::vector<double> & speed)
{
  speed = caps;
  for (int pass = 0; pass < 3; ++pass) {
    for (std::size_t index = 1; index < speed.size(); ++index) {
      const double reachable = std::sqrt(
        std::max(0.0, speed[index - 1] * speed[index - 1] +
        2.0 * limits.max_linear_acceleration * segment_lengths[index - 1]));
      speed[index] = std::min(speed[index], reachable);
    }
    for (std::size_t index = speed.size() - 1; index > 0; --index) {
      const double reachable = std::sqrt(
        std::max(0.0, speed[index] * speed[index] +
        2.0 * limits.max_linear_deceleration * segment_lengths[index - 1]));
      speed[index - 1] = std::min(speed[index - 1], reachable);
    }
  }
}

double segment_time(double length, double start_speed, double end_speed)
{
  const double speed_sum = start_speed + end_speed;
  if (length <= kEpsilon) {
    return 0.0;
  }
  if (speed_sum <= kEpsilon) {
    return std::numeric_limits<double>::infinity();
  }
  return 2.0 * length / speed_sum;
}

}  // namespace

TimedProfile parameterize_time(
  const std::vector<PathSample> & path,
  const RobotLimits & limits,
  double start_speed,
  double end_speed)
{
  TimedProfile profile;
  if (path.size() < 2) {
    profile.rejection_reason = "time parameterization needs at least two path samples";
    return profile;
  }
  if (!finite_positive(limits.max_linear_acceleration) ||
    !finite_positive(limits.max_linear_deceleration) ||
    !finite_positive(limits.max_angular_acceleration))
  {
    profile.rejection_reason = "acceleration limits must be finite and positive";
    return profile;
  }
  if (!std::isfinite(start_speed) || start_speed < 0.0 ||
    !std::isfinite(end_speed) || end_speed < 0.0)
  {
    profile.rejection_reason = "boundary speed caps must be finite and non-negative";
    return profile;
  }

  std::vector<double> segment_lengths(path.size() - 1, 0.0);
  std::vector<double> caps(path.size(), 0.0);
  for (std::size_t index = 0; index < path.size(); ++index) {
    if (!finite(path[index].position) || !std::isfinite(path[index].curvature) ||
      !finite_positive(path[index].speed_limit))
    {
      profile.rejection_reason = "path contains an invalid sample or speed limit";
      return profile;
    }
    caps[index] = path[index].speed_limit;
    if (index > 0) {
      segment_lengths[index - 1] = distance(path[index - 1].position, path[index].position);
      if (!finite_positive(segment_lengths[index - 1])) {
        profile.rejection_reason = "moving path contains duplicate or non-finite positions";
        return profile;
      }
    }
  }

  caps.front() = std::min(caps.front(), std::max(0.0, start_speed));
  caps.back() = std::min(caps.back(), std::max(0.0, end_speed));
  std::vector<double> speed;

  // Reducing velocity by f reduces finite-difference angular acceleration by
  // approximately f^2 because omega scales by f while traversal time scales by 1/f.
  for (int iteration = 0; iteration < 40; ++iteration) {
    enforce_linear_acceleration_limits(segment_lengths, limits, caps, speed);
    bool changed = false;
    for (std::size_t index = 1; index < path.size(); ++index) {
      const double dt = segment_time(
        segment_lengths[index - 1], speed[index - 1], speed[index]);
      if (!std::isfinite(dt)) {
        profile.rejection_reason = "zero-speed interval has non-zero length";
        return profile;
      }
      const double omega_before = speed[index - 1] * path[index - 1].curvature;
      const double omega_after = speed[index] * path[index].curvature;
      const double angular_acceleration = std::abs(omega_after - omega_before) / dt;
      if (angular_acceleration > limits.max_angular_acceleration * (1.0 + 1.0e-6)) {
        const double factor = std::max(
          0.1, 0.995 * std::sqrt(limits.max_angular_acceleration / angular_acceleration));
        caps[index - 1] *= factor;
        caps[index] *= factor;
        changed = true;
      }
    }
    if (!changed) {
      break;
    }
  }

  enforce_linear_acceleration_limits(segment_lengths, limits, caps, speed);
  profile.linear_speed = speed;
  profile.angular_speed.resize(path.size(), 0.0);
  profile.time.resize(path.size(), 0.0);
  for (std::size_t index = 0; index < path.size(); ++index) {
    profile.angular_speed[index] = speed[index] * path[index].curvature;
    if (index > 0) {
      const double dt = segment_time(
        segment_lengths[index - 1], speed[index - 1], speed[index]);
      if (!std::isfinite(dt)) {
        profile.rejection_reason = "time profile contains an infeasible interval";
        return profile;
      }
      profile.time[index] = profile.time[index - 1] + dt;
      profile.max_abs_angular_acceleration = std::max(
        profile.max_abs_angular_acceleration,
        std::abs(profile.angular_speed[index] - profile.angular_speed[index - 1]) / dt);
    }
  }
  if (profile.max_abs_angular_acceleration >
    limits.max_angular_acceleration * (1.0 + 1.0e-4))
  {
    profile.rejection_reason = "angular acceleration iteration did not converge";
    return profile;
  }

  profile.total_time = profile.time.back();
  profile.valid = std::isfinite(profile.total_time);
  if (!profile.valid) {
    profile.rejection_reason = "profile duration is not finite";
  }
  return profile;
}

TimedProfile parameterize_transition_window(
  const TransitionCandidate & candidate,
  double window_trim,
  const RobotLimits & limits,
  double start_speed,
  double end_speed,
  double sample_spacing)
{
  TimedProfile profile;
  if (!candidate.valid || candidate.samples.size() < 2U ||
    !std::isfinite(window_trim) ||
    window_trim + kEpsilon < candidate.trim_distance ||
    !finite_positive(sample_spacing) ||
    !finite_positive(limits.max_linear_speed) ||
    !finite_positive(limits.max_wheel_speed))
  {
    profile.rejection_reason = "transition window inputs are invalid";
    return profile;
  }

  const double extension = std::max(0.0, window_trim - candidate.trim_distance);
  if (extension <= kEpsilon) {
    return parameterize_time(
      candidate.samples, limits, start_speed, end_speed);
  }

  const double straight_speed_limit =
    std::min(limits.max_linear_speed, limits.max_wheel_speed);
  std::vector<PathSample> window;
  const int straight_segments = std::max(
    1, static_cast<int>(std::ceil(extension / sample_spacing)));
  window.reserve(
    candidate.samples.size() + 2U * static_cast<std::size_t>(straight_segments));

  const auto & entry = candidate.samples.front();
  const Vec2 incoming{std::cos(entry.heading), std::sin(entry.heading)};
  const Vec2 window_start = entry.position - incoming * extension;
  for (int index = 0; index < straight_segments; ++index) {
    const double fraction =
      static_cast<double>(index) / static_cast<double>(straight_segments);
    window.push_back(
      {window_start + incoming * (fraction * extension),
        entry.heading, 0.0, straight_speed_limit});
  }
  window.insert(window.end(), candidate.samples.begin(), candidate.samples.end());

  const auto & exit = candidate.samples.back();
  const Vec2 outgoing{std::cos(exit.heading), std::sin(exit.heading)};
  for (int index = 1; index <= straight_segments; ++index) {
    const double fraction =
      static_cast<double>(index) / static_cast<double>(straight_segments);
    window.push_back(
      {exit.position + outgoing * (fraction * extension),
        exit.heading, 0.0, straight_speed_limit});
  }
  return parameterize_time(window, limits, start_speed, end_speed);
}

double minimum_translation_time(
  double length,
  double start_speed,
  double end_speed,
  double max_speed,
  double acceleration,
  double deceleration)
{
  if (!std::isfinite(length) || length < 0.0 || !std::isfinite(start_speed) ||
    !std::isfinite(end_speed) || start_speed < 0.0 || end_speed < 0.0 ||
    !finite_positive(max_speed) || !finite_positive(acceleration) ||
    !finite_positive(deceleration) || start_speed > max_speed || end_speed > max_speed)
  {
    return std::numeric_limits<double>::infinity();
  }
  if (length <= kEpsilon) {
    return (start_speed <= kEpsilon && end_speed <= kEpsilon) ?
           0.0 : std::numeric_limits<double>::infinity();
  }

  if (end_speed > start_speed) {
    const double minimum_length =
      (end_speed * end_speed - start_speed * start_speed) / (2.0 * acceleration);
    if (minimum_length > length + kEpsilon) {
      return std::numeric_limits<double>::infinity();
    }
  } else if (start_speed > end_speed) {
    const double minimum_length =
      (start_speed * start_speed - end_speed * end_speed) / (2.0 * deceleration);
    if (minimum_length > length + kEpsilon) {
      return std::numeric_limits<double>::infinity();
    }
  }

  const double acceleration_length =
    (max_speed * max_speed - start_speed * start_speed) / (2.0 * acceleration);
  const double deceleration_length =
    (max_speed * max_speed - end_speed * end_speed) / (2.0 * deceleration);
  if (acceleration_length + deceleration_length <= length) {
    const double cruise_length = length - acceleration_length - deceleration_length;
    return (max_speed - start_speed) / acceleration +
           cruise_length / max_speed +
           (max_speed - end_speed) / deceleration;
  }

  const double peak_speed_squared =
    (2.0 * acceleration * deceleration * length +
    deceleration * start_speed * start_speed +
    acceleration * end_speed * end_speed) /
    (acceleration + deceleration);
  const double peak_speed = std::sqrt(std::max(0.0, peak_speed_squared));
  if (peak_speed + kEpsilon < std::max(start_speed, end_speed)) {
    return std::numeric_limits<double>::infinity();
  }
  return (peak_speed - start_speed) / acceleration +
         (peak_speed - end_speed) / deceleration;
}

double minimum_rotation_time(
  double angle,
  double max_angular_speed,
  double max_angular_acceleration)
{
  const double absolute_angle = std::abs(angle);
  if (!std::isfinite(absolute_angle) || !finite_positive(max_angular_speed) ||
    !finite_positive(max_angular_acceleration))
  {
    return std::numeric_limits<double>::infinity();
  }
  if (absolute_angle <= kEpsilon) {
    return 0.0;
  }
  const double ramp_angle =
    max_angular_speed * max_angular_speed / max_angular_acceleration;
  if (absolute_angle <= ramp_angle) {
    return 2.0 * std::sqrt(absolute_angle / max_angular_acceleration);
  }
  return 2.0 * max_angular_speed / max_angular_acceleration +
         (absolute_angle - ramp_angle) / max_angular_speed;
}

double estimate_pivot_window_time(
  double trim_distance,
  double turn_angle,
  const RobotLimits & limits,
  double entry_speed,
  double exit_speed)
{
  const double maximum_translation_speed =
    std::min(limits.max_linear_speed, limits.max_wheel_speed);
  const double approach_time = minimum_translation_time(
    trim_distance, entry_speed, 0.0, maximum_translation_speed,
    limits.max_linear_acceleration, limits.max_linear_deceleration);
  const double departure_time = minimum_translation_time(
    trim_distance, 0.0, exit_speed, maximum_translation_speed,
    limits.max_linear_acceleration, limits.max_linear_deceleration);
  const double rotation_time = minimum_rotation_time(
    turn_angle, limits.max_angular_speed, limits.max_angular_acceleration);
  return approach_time + rotation_time + departure_time;
}

}  // namespace adaptive_pivot_g2
