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

#ifndef ADAPTIVE_PIVOT_G2_CONTROLLER__ADAPTIVE_SPEED_PROFILE_HPP_
#define ADAPTIVE_PIVOT_G2_CONTROLLER__ADAPTIVE_SPEED_PROFILE_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "nav_msgs/msg/path.hpp"

namespace adaptive_pivot_g2_controller
{

/// Physical and numerical limits used by the predictive speed envelope.
struct AdaptiveSpeedParameters
{
  double max_linear_speed{0.30};
  double max_angular_speed{0.80};
  double max_wheel_linear_speed{0.36};
  double wheel_separation{0.2548};
  double max_lateral_acceleration{0.18};
  double max_linear_acceleration{0.35};
  double max_linear_deceleration{0.45};
  double max_angular_acceleration{1.20};
  double max_linear_jerk{0.90};
  double curvature_sample_distance{0.10};
  double terminal_linear_speed{0.0};
  double terminal_stop_buffer{0.04};
  double projection_search_backward{0.25};
  double projection_search_forward{0.80};
  double projection_heading_weight{0.20};
  double projection_max_regression{0.03};
  double feedback_sync_tolerance{0.06};
  double cross_track_error_soft{0.02};
  double cross_track_error_hard{0.10};
  double heading_error_soft{0.08};
  double heading_error_hard{0.45};
  double angular_tracking_error_soft{0.08};
  double angular_tracking_error_hard{0.35};
  double recovery_min_linear_speed{0.06};
};

struct SpeedProfilePoint
{
  double x{0.0};
  double y{0.0};
  double distance{0.0};
  double curvature{0.0};
  double local_speed_cap{0.0};
  double speed_cap{0.0};
  std::string limiting_constraint{"max_linear_speed"};
};

struct AdaptiveSpeedProfile
{
  std::string frame_id;
  double max_linear_acceleration{0.0};
  double max_linear_deceleration{0.0};
  double max_linear_jerk{0.0};
  std::vector<SpeedProfilePoint> points;
};

struct SpeedProfileProjection
{
  bool valid{false};
  std::size_t segment_index{0};
  double distance{0.0};
  double remaining_distance{0.0};
  double cross_track_error{0.0};
  double tangent_heading{0.0};
  double heading_error{0.0};
  double curvature{0.0};
  double local_speed_cap{0.0};
  double speed_cap{0.0};
  std::string limiting_constraint{"unavailable"};
};

struct InstantaneousSpeedCaps
{
  double max_linear{0.0};
  double lateral_acceleration{0.0};
  double angular_speed{0.0};
  double wheel_speed{0.0};
  double combined{0.0};
  std::string limiting_constraint{"max_linear_speed"};
};

struct JerkLimitedSpeedState
{
  bool initialized{false};
  /// The previous update clipped the S-curve command to a harder safety cap.
  /// Keep the following state-recovery sample out of nominal jerk statistics.
  bool safety_override_active{false};
  double speed{0.0};
  double acceleration{0.0};
};

struct JerkLimitedSpeedResult
{
  double speed{0.0};
  double acceleration{0.0};
  double jerk{0.0};
  bool safety_override{false};
  bool feedback_limited{false};
};

/// Throws std::invalid_argument when a speed parameter is invalid.
void validate_adaptive_speed_parameters(const AdaptiveSpeedParameters & parameters);

/// Differential-drive and lateral-acceleration caps at a given curvature.
InstantaneousSpeedCaps instantaneous_speed_caps(
  double curvature,
  const AdaptiveSpeedParameters & parameters);

/// Distance required for a zero-acceleration S-curve stop.
double jerk_limited_stopping_distance(
  double speed,
  double max_deceleration,
  double max_jerk);

/// Minimum distance for a zero-acceleration S-curve speed transition.
double jerk_limited_deceleration_distance(
  double initial_speed,
  double final_speed,
  double max_deceleration,
  double max_jerk);

/// Minimum distance for a zero-acceleration S-curve acceleration transition.
double jerk_limited_acceleration_distance(
  double initial_speed,
  double final_speed,
  double max_acceleration,
  double max_jerk);

/// Inverse of jerk_limited_stopping_distance().
double jerk_limited_speed_for_stopping_distance(
  double distance,
  double max_deceleration,
  double max_jerk);

/// Safe scalar speed along a reference Twist under a one-step angular ramp.
double angular_acceleration_speed_cap(
  double reference_linear_speed,
  double reference_angular_speed,
  double measured_angular_speed,
  double time_step,
  double max_angular_acceleration);

/// Speed cap that decreases continuously outside a tracking-error deadband.
double tracking_error_speed_cap(
  double error,
  double soft_error,
  double hard_error,
  double minimum_speed,
  double maximum_speed);

/// Build a curvature-aware, bidirectional jerk-aware speed envelope for a path.
AdaptiveSpeedProfile build_adaptive_speed_profile(
  const nav_msgs::msg::Path & path,
  const AdaptiveSpeedParameters & parameters);

/// Project a point onto a monotonic local window of the speed profile.
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
  double max_regression);

/// Shape normal commands with bounded acceleration/jerk; lower safety caps win immediately.
JerkLimitedSpeedResult update_jerk_limited_speed(
  double target_speed,
  double measured_speed,
  double time_step,
  const AdaptiveSpeedParameters & parameters,
  JerkLimitedSpeedState & state);

}  // namespace adaptive_pivot_g2_controller

#endif  // ADAPTIVE_PIVOT_G2_CONTROLLER__ADAPTIVE_SPEED_PROFILE_HPP_
