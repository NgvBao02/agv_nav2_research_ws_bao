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

#include "adaptive_pivot_g2_controller/adaptive_speed_profile.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace adaptive_pivot_g2_controller
{
namespace
{

constexpr double kEpsilon = 1.0e-9;

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

double planar_distance(
  const geometry_msgs::msg::PoseStamped & first,
  const geometry_msgs::msg::PoseStamped & second)
{
  return std::hypot(
    second.pose.position.x - first.pose.position.x,
    second.pose.position.y - first.pose.position.y);
}

double signed_menger_curvature(
  const SpeedProfilePoint & first,
  const SpeedProfilePoint & middle,
  const SpeedProfilePoint & last)
{
  const double first_to_middle = std::hypot(middle.x - first.x, middle.y - first.y);
  const double middle_to_last = std::hypot(last.x - middle.x, last.y - middle.y);
  const double first_to_last = std::hypot(last.x - first.x, last.y - first.y);
  const double denominator = first_to_middle * middle_to_last * first_to_last;
  if (denominator <= kEpsilon) {
    return 0.0;
  }
  const double cross =
    (middle.x - first.x) * (last.y - first.y) -
    (middle.y - first.y) * (last.x - first.x);
  return 2.0 * cross / denominator;
}

std::string local_limiting_constraint(const InstantaneousSpeedCaps & caps)
{
  const double minimum = caps.combined;
  if (caps.lateral_acceleration <= minimum + kEpsilon) {
    return "lateral_acceleration";
  }
  if (caps.angular_speed <= minimum + kEpsilon) {
    return "angular_speed";
  }
  if (caps.wheel_speed <= minimum + kEpsilon) {
    return "wheel_speed";
  }
  return "max_linear_speed";
}

double interpolate(double first, double second, double ratio)
{
  return first + ratio * (second - first);
}

double normalized_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double reachable_initial_speed(
  double distance,
  double final_speed,
  double maximum_initial_speed,
  double max_deceleration,
  double max_jerk)
{
  if (maximum_initial_speed <= final_speed + kEpsilon) {
    return maximum_initial_speed;
  }
  if (jerk_limited_deceleration_distance(
      maximum_initial_speed, final_speed, max_deceleration, max_jerk) <=
    distance + kEpsilon)
  {
    return maximum_initial_speed;
  }

  double lower = final_speed;
  double upper = maximum_initial_speed;
  for (int iteration = 0; iteration < 80; ++iteration) {
    const double candidate = 0.5 * (lower + upper);
    if (jerk_limited_deceleration_distance(
        candidate, final_speed, max_deceleration, max_jerk) <= distance)
    {
      lower = candidate;
    } else {
      upper = candidate;
    }
  }
  return lower;
}

double reachable_final_speed(
  double distance,
  double initial_speed,
  double maximum_final_speed,
  double max_acceleration,
  double max_jerk)
{
  if (maximum_final_speed <= initial_speed + kEpsilon) {
    return maximum_final_speed;
  }
  if (jerk_limited_acceleration_distance(
      initial_speed, maximum_final_speed, max_acceleration, max_jerk) <=
    distance + kEpsilon)
  {
    return maximum_final_speed;
  }

  double lower = initial_speed;
  double upper = maximum_final_speed;
  for (int iteration = 0; iteration < 80; ++iteration) {
    const double candidate = 0.5 * (lower + upper);
    if (jerk_limited_acceleration_distance(
        initial_speed, candidate, max_acceleration, max_jerk) <= distance)
    {
      lower = candidate;
    } else {
      upper = candidate;
    }
  }
  return lower;
}

void enforce_backward_braking_envelope(
  AdaptiveSpeedProfile & profile,
  const AdaptiveSpeedParameters & parameters)
{
  for (std::size_t index = profile.points.size() - 1U; index > 0U; --index) {
    const double segment_length =
      profile.points[index].distance - profile.points[index - 1U].distance;
    const double future_speed = profile.points[index].speed_cap;
    const double current_cap = profile.points[index - 1U].speed_cap;
    if (current_cap <= future_speed + kEpsilon) {
      continue;
    }
    const double reachable_speed = reachable_initial_speed(
      segment_length, future_speed, current_cap,
      parameters.max_linear_deceleration, parameters.max_linear_jerk);
    if (reachable_speed < current_cap - kEpsilon) {
      profile.points[index - 1U].speed_cap = reachable_speed;
      profile.points[index - 1U].limiting_constraint = "future_braking";
    }
  }
}

void enforce_forward_acceleration_envelope(
  AdaptiveSpeedProfile & profile,
  const AdaptiveSpeedParameters & parameters)
{
  for (std::size_t index = 1U; index < profile.points.size(); ++index) {
    const double segment_length =
      profile.points[index].distance - profile.points[index - 1U].distance;
    const double previous_speed = profile.points[index - 1U].speed_cap;
    const double current_cap = profile.points[index].speed_cap;
    if (current_cap <= previous_speed + kEpsilon) {
      continue;
    }
    const double reachable_speed = reachable_final_speed(
      segment_length, previous_speed, current_cap,
      parameters.max_linear_acceleration, parameters.max_linear_jerk);
    if (reachable_speed < current_cap - kEpsilon) {
      profile.points[index].speed_cap = reachable_speed;
      profile.points[index].limiting_constraint = "past_acceleration";
    }
  }
}

void enforce_longitudinal_envelopes(
  AdaptiveSpeedProfile & profile,
  const AdaptiveSpeedParameters & parameters)
{
  // Both passes can only lower caps. Iterate because a cap lowered by the
  // forward pass may require earlier braking on the next backward pass.
  for (std::size_t iteration = 0U;
    iteration < 2U * profile.points.size(); ++iteration)
  {
    std::vector<double> previous_caps;
    previous_caps.reserve(profile.points.size());
    for (const auto & point : profile.points) {
      previous_caps.push_back(point.speed_cap);
    }
    enforce_backward_braking_envelope(profile, parameters);
    enforce_forward_acceleration_envelope(profile, parameters);
    bool changed = false;
    for (std::size_t index = 0U; index < profile.points.size(); ++index) {
      changed = changed ||
        std::abs(profile.points[index].speed_cap - previous_caps[index]) >
        1.0e-12;
    }
    if (!changed) {
      return;
    }
  }
  throw std::invalid_argument(
          "adaptive longitudinal speed envelope did not converge");
}

double segment_time(double distance, double first_speed, double last_speed)
{
  const double speed_sum = first_speed + last_speed;
  if (distance <= kEpsilon) {
    return 0.0;
  }
  return speed_sum > kEpsilon ?
         2.0 * distance / speed_sum :
         std::numeric_limits<double>::infinity();
}

void enforce_angular_acceleration_envelope(
  AdaptiveSpeedProfile & profile,
  const AdaptiveSpeedParameters & parameters)
{
  for (int iteration = 0; iteration < 40; ++iteration) {
    enforce_longitudinal_envelopes(profile, parameters);
    bool changed = false;
    for (std::size_t index = 1U; index < profile.points.size(); ++index) {
      auto & first = profile.points[index - 1U];
      auto & last = profile.points[index];
      const double distance = last.distance - first.distance;
      const double time_step = segment_time(
        distance, first.speed_cap, last.speed_cap);
      if (!std::isfinite(time_step)) {
        continue;
      }
      const double first_angular_speed = first.curvature * first.speed_cap;
      const double last_angular_speed = last.curvature * last.speed_cap;
      const double angular_acceleration =
        std::abs(last_angular_speed - first_angular_speed) / time_step;
      if (angular_acceleration >
        parameters.max_angular_acceleration * (1.0 + 1.0e-6))
      {
        const double factor = std::max(
          0.1, 0.995 * std::sqrt(
            parameters.max_angular_acceleration / angular_acceleration));
        first.speed_cap *= factor;
        last.speed_cap *= factor;
        first.limiting_constraint = "angular_acceleration";
        last.limiting_constraint = "angular_acceleration";
        changed = true;
      }
    }
    if (!changed) {
      break;
    }
  }
  enforce_longitudinal_envelopes(profile, parameters);
  for (std::size_t index = 1U; index < profile.points.size(); ++index) {
    const auto & first = profile.points[index - 1U];
    const auto & last = profile.points[index];
    const double time_step = segment_time(
      last.distance - first.distance, first.speed_cap, last.speed_cap);
    if (!std::isfinite(time_step)) {
      continue;
    }
    const double angular_acceleration = std::abs(
      last.curvature * last.speed_cap -
      first.curvature * first.speed_cap) / time_step;
    if (angular_acceleration >
      parameters.max_angular_acceleration * (1.0 + 1.0e-4))
    {
      throw std::invalid_argument(
              "adaptive angular-acceleration envelope did not converge");
    }
  }
}

}  // namespace

void validate_adaptive_speed_parameters(const AdaptiveSpeedParameters & parameters)
{
  const double positive_values[]{
    parameters.max_linear_speed,
    parameters.max_angular_speed,
    parameters.max_wheel_linear_speed,
    parameters.wheel_separation,
    parameters.max_lateral_acceleration,
    parameters.max_linear_acceleration,
    parameters.max_linear_deceleration,
    parameters.max_angular_acceleration,
    parameters.max_linear_jerk,
    parameters.curvature_sample_distance,
    parameters.projection_search_forward,
    parameters.projection_heading_weight,
    parameters.feedback_sync_tolerance};
  if (std::any_of(
      std::begin(positive_values), std::end(positive_values),
      [](double value) {return !finite_positive(value);}))
  {
    throw std::invalid_argument("adaptive speed positive limits must be finite and positive");
  }
  const double non_negative_values[]{
    parameters.terminal_linear_speed,
    parameters.terminal_stop_buffer,
    parameters.projection_search_backward,
    parameters.projection_max_regression,
    parameters.cross_track_error_soft,
    parameters.heading_error_soft,
    parameters.angular_tracking_error_soft};
  if (std::any_of(
      std::begin(non_negative_values), std::end(non_negative_values),
      [](double value) {return !std::isfinite(value) || value < 0.0;}))
  {
    throw std::invalid_argument("adaptive speed distances and terminal speed are invalid");
  }
  if (parameters.terminal_linear_speed > parameters.max_linear_speed) {
    throw std::invalid_argument("terminal speed cannot exceed maximum linear speed");
  }
  if (parameters.cross_track_error_hard <= parameters.cross_track_error_soft ||
    parameters.heading_error_hard <= parameters.heading_error_soft ||
    parameters.angular_tracking_error_hard <=
    parameters.angular_tracking_error_soft ||
    !std::isfinite(parameters.cross_track_error_hard) ||
    !std::isfinite(parameters.heading_error_hard) ||
    !std::isfinite(parameters.angular_tracking_error_hard) ||
    !finite_positive(parameters.recovery_min_linear_speed) ||
    parameters.recovery_min_linear_speed > parameters.max_linear_speed)
  {
    throw std::invalid_argument("adaptive tracking-recovery limits are invalid");
  }
}

InstantaneousSpeedCaps instantaneous_speed_caps(
  double curvature,
  const AdaptiveSpeedParameters & parameters)
{
  validate_adaptive_speed_parameters(parameters);
  if (!std::isfinite(curvature)) {
    throw std::invalid_argument("curvature must be finite");
  }

  InstantaneousSpeedCaps caps;
  const double absolute_curvature = std::abs(curvature);
  caps.max_linear = parameters.max_linear_speed;
  caps.lateral_acceleration = parameters.max_linear_speed;
  caps.angular_speed = parameters.max_linear_speed;
  caps.wheel_speed = parameters.max_linear_speed;
  if (absolute_curvature > kEpsilon) {
    caps.lateral_acceleration = std::sqrt(
      parameters.max_lateral_acceleration / absolute_curvature);
    caps.angular_speed = parameters.max_angular_speed / absolute_curvature;
  }
  const double outer_wheel_factor =
    1.0 + 0.5 * parameters.wheel_separation * absolute_curvature;
  caps.wheel_speed = parameters.max_wheel_linear_speed / outer_wheel_factor;
  caps.combined = std::min(
    {caps.max_linear, caps.lateral_acceleration, caps.angular_speed, caps.wheel_speed});
  caps.limiting_constraint = local_limiting_constraint(caps);
  return caps;
}

double jerk_limited_stopping_distance(
  double speed,
  double max_deceleration,
  double max_jerk)
{
  return jerk_limited_deceleration_distance(
    speed, 0.0, max_deceleration, max_jerk);
}

double jerk_limited_deceleration_distance(
  double initial_speed,
  double final_speed,
  double max_deceleration,
  double max_jerk)
{
  if (!std::isfinite(initial_speed) || initial_speed < 0.0 ||
    !std::isfinite(final_speed) || final_speed < 0.0 ||
    final_speed > initial_speed ||
    !finite_positive(max_deceleration) || !finite_positive(max_jerk))
  {
    throw std::invalid_argument("jerk-limited deceleration inputs are invalid");
  }
  const double speed_change = initial_speed - final_speed;
  if (speed_change <= kEpsilon) {
    return 0.0;
  }

  const double transition_speed =
    max_deceleration * max_deceleration / max_jerk;
  if (speed_change <= transition_speed) {
    return (initial_speed + final_speed) *
           std::sqrt(speed_change / max_jerk);
  }
  return 0.5 * (initial_speed + final_speed) *
         (speed_change / max_deceleration + max_deceleration / max_jerk);
}

double jerk_limited_acceleration_distance(
  double initial_speed,
  double final_speed,
  double max_acceleration,
  double max_jerk)
{
  if (!std::isfinite(initial_speed) || initial_speed < 0.0 ||
    !std::isfinite(final_speed) || final_speed < initial_speed ||
    !finite_positive(max_acceleration) || !finite_positive(max_jerk))
  {
    throw std::invalid_argument("jerk-limited acceleration inputs are invalid");
  }
  return jerk_limited_deceleration_distance(
    final_speed, initial_speed, max_acceleration, max_jerk);
}

double jerk_limited_speed_for_stopping_distance(
  double distance,
  double max_deceleration,
  double max_jerk)
{
  if (!std::isfinite(distance) || distance < 0.0 ||
    !finite_positive(max_deceleration) || !finite_positive(max_jerk))
  {
    throw std::invalid_argument("inverse jerk-limited stop inputs are invalid");
  }
  if (distance <= kEpsilon) {
    return 0.0;
  }

  const double transition_distance =
    max_deceleration * max_deceleration * max_deceleration /
    (max_jerk * max_jerk);
  if (distance <= transition_distance) {
    return std::pow(distance * std::sqrt(max_jerk), 2.0 / 3.0);
  }
  const double linear_coefficient =
    max_deceleration * max_deceleration / max_jerk;
  return 0.5 * (
    -linear_coefficient +
    std::sqrt(
      linear_coefficient * linear_coefficient +
         8.0 * max_deceleration * distance));
}

double angular_acceleration_speed_cap(
  double reference_linear_speed,
  double reference_angular_speed,
  double measured_angular_speed,
  double time_step,
  double max_angular_acceleration)
{
  if (!std::isfinite(reference_linear_speed) || reference_linear_speed < 0.0 ||
    !std::isfinite(reference_angular_speed) ||
    !std::isfinite(measured_angular_speed) ||
    !finite_positive(time_step) ||
    !finite_positive(max_angular_acceleration))
  {
    throw std::invalid_argument("angular-acceleration speed-cap inputs are invalid");
  }
  if (reference_linear_speed <= kEpsilon ||
    std::abs(reference_angular_speed) <= kEpsilon)
  {
    return reference_linear_speed;
  }
  const double direction = std::copysign(1.0, reference_angular_speed);
  const double reachable_angular_speed = std::max(
    0.0, direction * measured_angular_speed +
    max_angular_acceleration * time_step);
  const double scale = std::clamp(
    reachable_angular_speed / std::abs(reference_angular_speed), 0.0, 1.0);
  return reference_linear_speed * scale;
}

double tracking_error_speed_cap(
  double error,
  double soft_error,
  double hard_error,
  double minimum_speed,
  double maximum_speed)
{
  if (!std::isfinite(error) ||
    !std::isfinite(soft_error) || soft_error < 0.0 ||
    !std::isfinite(hard_error) || hard_error <= soft_error ||
    !finite_positive(minimum_speed) ||
    !finite_positive(maximum_speed) ||
    minimum_speed > maximum_speed)
  {
    throw std::invalid_argument("tracking-error speed-cap inputs are invalid");
  }
  const double magnitude = std::abs(error);
  if (magnitude <= soft_error) {
    return maximum_speed;
  }
  if (magnitude >= hard_error) {
    return minimum_speed;
  }
  const double ratio =
    (magnitude - soft_error) / (hard_error - soft_error);
  // Smoothstep avoids a slope discontinuity as the robot enters or leaves the
  // recovery region, so this cap does not itself inject longitudinal jerk.
  const double blend = ratio * ratio * (3.0 - 2.0 * ratio);
  return interpolate(maximum_speed, minimum_speed, blend);
}

AdaptiveSpeedProfile build_adaptive_speed_profile(
  const nav_msgs::msg::Path & path,
  const AdaptiveSpeedParameters & parameters)
{
  validate_adaptive_speed_parameters(parameters);
  if (path.poses.size() < 2U) {
    throw std::invalid_argument("adaptive speed profile needs at least two path poses");
  }

  AdaptiveSpeedProfile profile;
  profile.frame_id = path.header.frame_id;
  profile.max_linear_acceleration = parameters.max_linear_acceleration;
  profile.max_linear_deceleration = parameters.max_linear_deceleration;
  profile.max_linear_jerk = parameters.max_linear_jerk;
  profile.points.resize(path.poses.size());
  for (std::size_t index = 0; index < path.poses.size(); ++index) {
    const auto & pose = path.poses[index];
    auto & point = profile.points[index];
    point.x = pose.pose.position.x;
    point.y = pose.pose.position.y;
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw std::invalid_argument("adaptive speed path contains a non-finite position");
    }
    if (index > 0U) {
      const double length = planar_distance(path.poses[index - 1U], pose);
      if (!finite_positive(length)) {
        throw std::invalid_argument("adaptive speed path contains duplicate positions");
      }
      point.distance = profile.points[index - 1U].distance + length;
    }
  }

  // Nav2's goal checker may end FollowPath as soon as the robot enters its
  // positional tolerance.  Put an exact profile knot at a configurable
  // virtual stop location so the base reaches terminal speed before that
  // early completion can remove the controller command.
  const double path_length = profile.points.back().distance;
  const double terminal_stop_distance = std::max(
    0.0, path_length - std::min(parameters.terminal_stop_buffer, path_length));
  if (parameters.terminal_stop_buffer > kEpsilon &&
    terminal_stop_distance > kEpsilon)
  {
    const auto after = std::lower_bound(
      profile.points.begin(), profile.points.end(), terminal_stop_distance,
      [](const SpeedProfilePoint & point, double distance) {
        return point.distance < distance;
      });
    if (after != profile.points.begin() && after != profile.points.end() &&
      std::abs(after->distance - terminal_stop_distance) > kEpsilon)
    {
      const auto before = after - 1;
      const double segment_length = after->distance - before->distance;
      const double ratio =
        (terminal_stop_distance - before->distance) / segment_length;
      SpeedProfilePoint stop_point;
      stop_point.x = interpolate(before->x, after->x, ratio);
      stop_point.y = interpolate(before->y, after->y, ratio);
      stop_point.distance = terminal_stop_distance;
      profile.points.insert(after, stop_point);
    }
  }

  for (std::size_t index = 0; index < profile.points.size(); ++index) {
    std::size_t before = index;
    while (before > 0U &&
      profile.points[index].distance - profile.points[before].distance <
      parameters.curvature_sample_distance)
    {
      --before;
    }
    std::size_t after = index;
    while (after + 1U < profile.points.size() &&
      profile.points[after].distance - profile.points[index].distance <
      parameters.curvature_sample_distance)
    {
      ++after;
    }
    if (before < index && after > index) {
      profile.points[index].curvature = signed_menger_curvature(
        profile.points[before], profile.points[index], profile.points[after]);
    }
  }
  if (profile.points.size() > 2U) {
    profile.points.front().curvature = profile.points[1U].curvature;
    profile.points.back().curvature =
      profile.points[profile.points.size() - 2U].curvature;
  }

  for (auto & point : profile.points) {
    const auto caps = instantaneous_speed_caps(point.curvature, parameters);
    point.local_speed_cap = caps.combined;
    point.speed_cap = caps.combined;
    point.limiting_constraint = caps.limiting_constraint;
  }
  for (auto & point : profile.points) {
    if (point.distance + kEpsilon >= terminal_stop_distance &&
      parameters.terminal_linear_speed < point.speed_cap)
    {
      point.speed_cap = parameters.terminal_linear_speed;
      point.limiting_constraint = "terminal_braking";
    }
  }

  // Propagate every future cap with the exact zero-acceleration S-curve
  // transition distance.  Subtracting two independent stopping distances is
  // only valid when the future speed is zero and is optimistic otherwise.
  enforce_longitudinal_envelopes(profile, parameters);
  // G2 removes curvature discontinuity, but omega = v*kappa can still change
  // too quickly when curvature varies.  Jointly scale neighboring caps and
  // re-propagate braking until the discrete angular-acceleration bound holds.
  enforce_angular_acceleration_envelope(profile, parameters);
  return profile;
}

SpeedProfileProjection project_onto_speed_profile(
  const AdaptiveSpeedProfile & profile,
  double x,
  double y,
  double yaw,
  std::size_t hint_segment,
  double hint_distance,
  double search_backward,
  double search_forward,
  double heading_weight,
  double max_regression)
{
  SpeedProfileProjection projection;
  if (profile.points.size() < 2U ||
    !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
    !std::isfinite(hint_distance) || hint_distance < 0.0 ||
    !std::isfinite(search_backward) || search_backward < 0.0 ||
    !finite_positive(search_forward) ||
    !finite_positive(heading_weight) ||
    !std::isfinite(max_regression) || max_regression < 0.0)
  {
    return projection;
  }

  const std::size_t clamped_hint = std::min(hint_segment, profile.points.size() - 2U);
  hint_distance = std::clamp(
    hint_distance,
    profile.points[clamped_hint].distance,
    profile.points.back().distance);
  const double minimum_distance = std::max(
    0.0, hint_distance - std::min(search_backward, max_regression));
  const double maximum_distance = hint_distance + search_forward;
  double best_score = std::numeric_limits<double>::infinity();
  double best_squared_error = std::numeric_limits<double>::infinity();
  double best_ratio = 0.0;
  double best_distance = 0.0;
  double best_tangent_heading = 0.0;

  for (std::size_t index = 0; index + 1U < profile.points.size(); ++index) {
    const auto & first = profile.points[index];
    const auto & last = profile.points[index + 1U];
    if (last.distance < minimum_distance || first.distance > maximum_distance) {
      continue;
    }
    const double dx = last.x - first.x;
    const double dy = last.y - first.y;
    const double squared_length = dx * dx + dy * dy;
    if (squared_length <= kEpsilon) {
      continue;
    }
    const double segment_length = last.distance - first.distance;
    const double minimum_ratio = std::clamp(
      (minimum_distance - first.distance) / segment_length, 0.0, 1.0);
    const double maximum_ratio = std::clamp(
      (maximum_distance - first.distance) / segment_length, 0.0, 1.0);
    if (maximum_ratio + kEpsilon < minimum_ratio) {
      continue;
    }
    const double ratio = std::clamp(
      ((x - first.x) * dx + (y - first.y) * dy) / squared_length,
      minimum_ratio, maximum_ratio);
    const double projected_x = first.x + ratio * dx;
    const double projected_y = first.y + ratio * dy;
    const double squared_error =
      (x - projected_x) * (x - projected_x) +
      (y - projected_y) * (y - projected_y);
    const double tangent_heading = std::atan2(dy, dx);
    const double heading_error = normalized_angle(yaw - tangent_heading);
    const double score =
      squared_error +
      std::pow(heading_weight * heading_error, 2.0);
    const double candidate_distance =
      interpolate(first.distance, last.distance, ratio);
    if (score < best_score - kEpsilon ||
      (std::abs(score - best_score) <= kEpsilon &&
      candidate_distance > best_distance))
    {
      best_score = score;
      best_squared_error = squared_error;
      projection.segment_index = index;
      best_ratio = ratio;
      best_distance = candidate_distance;
      best_tangent_heading = tangent_heading;
    }
  }
  if (!std::isfinite(best_squared_error)) {
    return projection;
  }

  const auto & first = profile.points[projection.segment_index];
  const auto & last = profile.points[projection.segment_index + 1U];
  projection.valid = true;
  projection.distance = interpolate(first.distance, last.distance, best_ratio);
  projection.remaining_distance =
    std::max(0.0, profile.points.back().distance - projection.distance);
  projection.cross_track_error = std::sqrt(best_squared_error);
  projection.tangent_heading = best_tangent_heading;
  projection.heading_error = normalized_angle(yaw - best_tangent_heading);
  projection.curvature = interpolate(first.curvature, last.curvature, best_ratio);
  projection.local_speed_cap =
    interpolate(first.local_speed_cap, last.local_speed_cap, best_ratio);
  projection.speed_cap = interpolate(first.speed_cap, last.speed_cap, best_ratio);
  projection.limiting_constraint =
    first.speed_cap <= last.speed_cap ? first.limiting_constraint : last.limiting_constraint;
  if (first.speed_cap > last.speed_cap + kEpsilon &&
    finite_positive(profile.max_linear_deceleration) &&
    finite_positive(profile.max_linear_jerk))
  {
    const double remaining_segment_distance =
      (1.0 - best_ratio) * (last.distance - first.distance);
    const double braking_cap = reachable_initial_speed(
      remaining_segment_distance, last.speed_cap, first.speed_cap,
      profile.max_linear_deceleration, profile.max_linear_jerk);
    if (braking_cap < projection.speed_cap) {
      projection.speed_cap = braking_cap;
      projection.limiting_constraint = "future_braking";
    }
  }
  if (first.speed_cap + kEpsilon < last.speed_cap &&
    finite_positive(profile.max_linear_acceleration) &&
    finite_positive(profile.max_linear_jerk))
  {
    const double traveled_segment_distance =
      best_ratio * (last.distance - first.distance);
    const double acceleration_cap = reachable_final_speed(
      traveled_segment_distance, first.speed_cap, last.speed_cap,
      profile.max_linear_acceleration, profile.max_linear_jerk);
    if (acceleration_cap < projection.speed_cap) {
      projection.speed_cap = acceleration_cap;
      projection.limiting_constraint = "past_acceleration";
    }
  }
  return projection;
}

JerkLimitedSpeedResult update_jerk_limited_speed(
  double target_speed,
  double measured_speed,
  double time_step,
  const AdaptiveSpeedParameters & parameters,
  JerkLimitedSpeedState & state)
{
  validate_adaptive_speed_parameters(parameters);
  if (!std::isfinite(target_speed) || target_speed < 0.0 ||
    !std::isfinite(measured_speed) || !finite_positive(time_step))
  {
    throw std::invalid_argument("jerk-limited speed update inputs are invalid");
  }

  target_speed = std::min(target_speed, parameters.max_linear_speed);
  measured_speed = std::max(0.0, measured_speed);
  if (!state.initialized) {
    state.initialized = true;
    state.speed = measured_speed;
    state.acceleration = 0.0;
  }

  const bool recovering_from_safety_override = state.safety_override_active;
  const double previous_speed = state.speed;
  const double previous_acceleration = state.acceleration;
  // The command is the integrator state; snapping it back to a lagging
  // measurement creates a sawtooth command whenever the drivetrain cannot
  // reproduce the requested acceleration in one control period.  When the
  // command leads feedback by the configured tolerance, smoothly bleed its
  // acceleration to zero and let the drivetrain catch up.
  double integration_target = target_speed;
  const bool feedback_limited =
    target_speed > previous_speed + kEpsilon &&
    previous_speed >
    measured_speed + parameters.feedback_sync_tolerance;
  if (feedback_limited) {
    integration_target = previous_speed;
  }
  const double desired_acceleration = std::clamp(
    (integration_target - previous_speed) / time_step,
    -parameters.max_linear_deceleration,
    parameters.max_linear_acceleration);
  const double jerk_step = parameters.max_linear_jerk * time_step;
  const double command_acceleration = std::clamp(
    desired_acceleration,
    std::max(
      -parameters.max_linear_deceleration,
      previous_acceleration - jerk_step),
    std::min(
      parameters.max_linear_acceleration,
      previous_acceleration + jerk_step));
  const double unconstrained_speed =
    previous_speed + command_acceleration * time_step;
  const double shaped_speed = std::max(0.0, unconstrained_speed);

  JerkLimitedSpeedResult result;
  result.speed = std::min(shaped_speed, target_speed);
  const bool upper_cap_override =
    target_speed + kEpsilon < shaped_speed;
  // Speed magnitude cannot cross below zero. Near a stop, an earlier hard
  // cap can leave the S-curve with negative acceleration but insufficient
  // speed to unwind it at the nominal jerk. Report that viability-boundary
  // clip as a hard override instead of misclassifying its jerk as nominal.
  const bool zero_speed_override = unconstrained_speed < -kEpsilon;
  const bool instantaneous_safety_override =
    upper_cap_override || zero_speed_override;
  result.safety_override =
    instantaneous_safety_override || recovering_from_safety_override;
  result.feedback_limited = feedback_limited;
  result.acceleration = (result.speed - previous_speed) / time_step;
  result.jerk = (result.acceleration - previous_acceleration) / time_step;
  state.speed = result.speed;
  // An instantaneous lower safety cap may intentionally violate the nominal
  // jerk bound.  Keep the reported result truthful, but do not feed an
  // out-of-range acceleration back into the next S-curve update.
  state.acceleration =
    result.speed <= kEpsilon ?
    0.0 :
    std::clamp(
    result.acceleration,
    -parameters.max_linear_deceleration,
    parameters.max_linear_acceleration);
  state.safety_override_active = instantaneous_safety_override;
  return result;
}

}  // namespace adaptive_pivot_g2_controller
