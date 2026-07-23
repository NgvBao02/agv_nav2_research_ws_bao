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

#include "adaptive_pivot_g2/quintic_transition.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace adaptive_pivot_g2
{
namespace
{

constexpr double kEpsilon = 1.0e-12;
constexpr std::array<double, 6> kBinomial5{{1.0, 5.0, 10.0, 10.0, 5.0, 1.0}};
constexpr std::array<double, 5> kBinomial4{{1.0, 4.0, 6.0, 4.0, 1.0}};
constexpr std::array<double, 4> kBinomial3{{1.0, 3.0, 3.0, 1.0}};

Vec2 normalized(const Vec2 & value)
{
  const double length = norm(value);
  if (!finite(value) || length <= kEpsilon) {
    throw std::invalid_argument("corner direction must be finite and non-zero");
  }
  return value / length;
}

double bernstein(unsigned int index, unsigned int degree, double u, double coefficient)
{
  return coefficient * std::pow(1.0 - u, static_cast<int>(degree - index)) *
         std::pow(u, static_cast<int>(index));
}

struct DifferentialState
{
  Vec2 position;
  Vec2 first_derivative;
  Vec2 second_derivative;
};

DifferentialState evaluate(
  const std::array<Vec2, 6> & control_points,
  double u)
{
  DifferentialState state;
  for (unsigned int index = 0; index < 6; ++index) {
    state.position = state.position +
      control_points[index] * bernstein(index, 5, u, kBinomial5[index]);
  }

  for (unsigned int index = 0; index < 5; ++index) {
    const Vec2 delta = control_points[index + 1] - control_points[index];
    state.first_derivative = state.first_derivative +
      delta * (5.0 * bernstein(index, 4, u, kBinomial4[index]));
  }

  for (unsigned int index = 0; index < 4; ++index) {
    const Vec2 delta2 = control_points[index + 2] -
      control_points[index + 1] * 2.0 + control_points[index];
    state.second_derivative = state.second_derivative +
      delta2 * (20.0 * bernstein(index, 3, u, kBinomial3[index]));
  }
  return state;
}

std::vector<PathSample> sample_curve(
  const std::array<Vec2, 6> & control_points,
  double requested_spacing,
  int segment_count)
{
  std::vector<PathSample> samples;
  samples.reserve(static_cast<std::size_t>(segment_count + 1));
  for (int index = 0; index <= segment_count; ++index) {
    const double u = static_cast<double>(index) / static_cast<double>(segment_count);
    const DifferentialState state = evaluate(control_points, u);
    const double derivative_norm = norm(state.first_derivative);
    if (derivative_norm <= kEpsilon || !std::isfinite(derivative_norm)) {
      return {};
    }
    const double curvature =
      cross(state.first_derivative, state.second_derivative) /
      std::pow(derivative_norm, 3.0);
    samples.push_back(
      {state.position,
        std::atan2(state.first_derivative.y, state.first_derivative.x),
        curvature,
        0.0});
  }

  double maximum_chord = 0.0;
  for (std::size_t index = 1; index < samples.size(); ++index) {
    maximum_chord = std::max(
      maximum_chord, distance(samples[index - 1].position, samples[index].position));
  }
  if (maximum_chord > requested_spacing * 1.001 && segment_count < (1 << 18)) {
    return sample_curve(control_points, requested_spacing, segment_count * 2);
  }
  return samples;
}

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

TransitionCandidate generate_quintic_transition(
  const CornerInput & corner,
  const RobotLimits & limits,
  const TransitionOptions & options)
{
  TransitionCandidate candidate;
  candidate.design_radius = options.design_radius;

  if (!finite(corner.vertex) || !finite_positive(corner.incoming_length) ||
    !finite_positive(corner.outgoing_length))
  {
    candidate.rejection_reason = "corner geometry is not finite or has zero-length segments";
    return candidate;
  }
  if (!finite_positive(options.design_radius) ||
    !finite_positive(options.control_fraction) || options.control_fraction >= 0.5 ||
    !finite_positive(options.sample_spacing) ||
    !finite_positive(options.max_trim_fraction) || options.max_trim_fraction >= 0.5)
  {
    candidate.rejection_reason = "transition options are outside their valid range";
    return candidate;
  }
  if (!finite_positive(limits.wheel_separation) ||
    !finite_positive(limits.max_linear_speed) ||
    !finite_positive(limits.max_angular_speed) ||
    !finite_positive(limits.max_wheel_speed))
  {
    candidate.rejection_reason = "robot speed or geometry limits are invalid";
    return candidate;
  }

  Vec2 incoming;
  Vec2 outgoing;
  try {
    incoming = normalized(corner.incoming_direction);
    outgoing = normalized(corner.outgoing_direction);
  } catch (const std::invalid_argument & error) {
    candidate.rejection_reason = error.what();
    return candidate;
  }

  const double turn_angle = std::atan2(cross(incoming, outgoing), dot(incoming, outgoing));
  candidate.turn_angle = turn_angle;
  const double absolute_angle = std::abs(turn_angle);
  if (absolute_angle <= options.minimum_turn_angle ||
    absolute_angle >= options.maximum_turn_angle)
  {
    candidate.rejection_reason = "turn angle is outside the transition domain";
    return candidate;
  }

  const double trim_distance = options.design_radius * std::tan(0.5 * absolute_angle);
  candidate.trim_distance = trim_distance;
  const double maximum_trim = options.max_trim_fraction *
    std::min(corner.incoming_length, corner.outgoing_length);
  if (!std::isfinite(trim_distance) || trim_distance > maximum_trim) {
    candidate.rejection_reason = "transition would overlap an adjacent corner window";
    return candidate;
  }

  const double tangent_offset = options.control_fraction * trim_distance;
  const Vec2 entry = corner.vertex - incoming * trim_distance;
  const Vec2 exit = corner.vertex + outgoing * trim_distance;
  const std::array<Vec2, 6> control_points{{
    entry,
    entry + incoming * tangent_offset,
    entry + incoming * (2.0 * tangent_offset),
    exit - outgoing * (2.0 * tangent_offset),
    exit - outgoing * tangent_offset,
    exit}};

  const int initial_segments = std::max(
    16, static_cast<int>(std::ceil(2.0 * trim_distance / options.sample_spacing)));
  candidate.samples = sample_curve(control_points, options.sample_spacing, initial_segments);
  if (candidate.samples.size() < 2) {
    candidate.rejection_reason = "Bezier derivative became singular";
    return candidate;
  }

  const double expected_sign = turn_angle > 0.0 ? 1.0 : -1.0;
  double curvature_energy = 0.0;
  for (std::size_t index = 0; index < candidate.samples.size(); ++index) {
    PathSample & sample = candidate.samples[index];
    if (!finite(sample.position) || !std::isfinite(sample.curvature)) {
      candidate.rejection_reason = "transition contains a non-finite sample";
      return candidate;
    }
    if (expected_sign * sample.curvature < -1.0e-8) {
      candidate.rejection_reason = "transition contains an unintended curvature reversal";
      return candidate;
    }

    const double half_track_curvature = 0.5 * limits.wheel_separation * sample.curvature;
    const double left_factor = 1.0 - half_track_curvature;
    const double right_factor = 1.0 + half_track_curvature;
    if (left_factor < -1.0e-9 || right_factor < -1.0e-9) {
      candidate.rejection_reason = "transition requires a reversing inner wheel";
      return candidate;
    }

    const double absolute_curvature = std::abs(sample.curvature);
    double speed_limit = limits.max_linear_speed;
    if (absolute_curvature > kEpsilon) {
      speed_limit = std::min(speed_limit, limits.max_angular_speed / absolute_curvature);
    }
    const double wheel_factor = std::max(std::abs(left_factor), std::abs(right_factor));
    speed_limit = std::min(speed_limit, limits.max_wheel_speed / wheel_factor);
    if (!finite_positive(speed_limit)) {
      candidate.rejection_reason = "transition has no positive feasible speed";
      return candidate;
    }
    sample.speed_limit = speed_limit;
    candidate.max_abs_curvature = std::max(candidate.max_abs_curvature, absolute_curvature);

    if (index > 0) {
      const double ds = distance(candidate.samples[index - 1].position, sample.position);
      candidate.path_length += ds;
      const double previous_curvature = candidate.samples[index - 1].curvature;
      curvature_energy += 0.5 *
        (previous_curvature * previous_curvature + sample.curvature * sample.curvature) * ds;
    }
  }
  candidate.curvature_energy = curvature_energy;
  candidate.valid = true;
  return candidate;
}

}  // namespace adaptive_pivot_g2
