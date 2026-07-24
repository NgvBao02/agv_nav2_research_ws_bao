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

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "adaptive_pivot_g2_controller/adaptive_speed_profile.hpp"

namespace adaptive_pivot_g2_controller
{
namespace
{

geometry_msgs::msg::PoseStamped pose(double x, double y)
{
  geometry_msgs::msg::PoseStamped output;
  output.header.frame_id = "map";
  output.pose.position.x = x;
  output.pose.position.y = y;
  output.pose.orientation.w = 1.0;
  return output;
}

nav_msgs::msg::Path path_from(
  const std::vector<std::pair<double, double>> & positions)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & position : positions) {
    path.poses.push_back(pose(position.first, position.second));
  }
  return path;
}

TEST(AdaptiveSpeedProfile, StopDistanceAndInverseAgreeAcrossBothRegimes)
{
  constexpr double deceleration = 0.45;
  constexpr double jerk = 0.90;
  for (const double speed : {0.01, 0.10, 0.225, 0.30, 0.60}) {
    const double distance = jerk_limited_stopping_distance(speed, deceleration, jerk);
    EXPECT_NEAR(
      jerk_limited_speed_for_stopping_distance(distance, deceleration, jerk),
      speed, 1.0e-10);
  }
}

TEST(AdaptiveSpeedProfile, NonzeroTargetUsesFullSCurveTransitionDistance)
{
  constexpr double deceleration = 0.45;
  constexpr double jerk = 0.90;
  constexpr double initial_speed = 0.30;
  constexpr double final_speed = 0.12;
  const double transition_distance = jerk_limited_deceleration_distance(
    initial_speed, final_speed, deceleration, jerk);
  const double invalid_coordinate_difference =
    jerk_limited_stopping_distance(initial_speed, deceleration, jerk) -
    jerk_limited_stopping_distance(final_speed, deceleration, jerk);

  EXPECT_GT(transition_distance, invalid_coordinate_difference);
  EXPECT_NEAR(
    jerk_limited_deceleration_distance(
      initial_speed, 0.0, deceleration, jerk),
    jerk_limited_stopping_distance(initial_speed, deceleration, jerk), 1.0e-12);
  EXPECT_THROW(
    jerk_limited_deceleration_distance(
      final_speed, initial_speed, deceleration, jerk),
    std::invalid_argument);
}

TEST(AdaptiveSpeedProfile, AccelerationDistanceIsTheSymmetricSCurveTransition)
{
  constexpr double acceleration = 0.35;
  constexpr double jerk = 0.90;
  constexpr double initial_speed = 0.08;
  constexpr double final_speed = 0.30;

  EXPECT_NEAR(
    jerk_limited_acceleration_distance(
      initial_speed, final_speed, acceleration, jerk),
    jerk_limited_deceleration_distance(
      final_speed, initial_speed, acceleration, jerk),
    1.0e-12);
  EXPECT_THROW(
    jerk_limited_acceleration_distance(
      final_speed, initial_speed, acceleration, jerk),
    std::invalid_argument);
}

TEST(AdaptiveSpeedProfile, AngularAccelerationCapScalesTheWholeReferenceTwist)
{
  EXPECT_NEAR(
    angular_acceleration_speed_cap(0.30, 0.60, 0.0, 0.05, 1.20),
    0.03, 1.0e-12);
  EXPECT_NEAR(
    angular_acceleration_speed_cap(0.30, -0.60, 0.30, 0.05, 1.20),
    0.0, 1.0e-12);
  EXPECT_NEAR(
    angular_acceleration_speed_cap(0.30, 0.0, 0.50, 0.05, 1.20),
    0.30, 1.0e-12);
  EXPECT_THROW(
    angular_acceleration_speed_cap(0.30, 0.60, 0.0, 0.0, 1.20),
    std::invalid_argument);
}

TEST(AdaptiveSpeedProfile, StraightPathCruisesThenBrakesForTerminal)
{
  AdaptiveSpeedParameters parameters;
  const auto path = path_from({
        {0.0, 0.0}, {0.5, 0.0}, {1.0, 0.0}, {1.5, 0.0},
        {2.0, 0.0}, {2.5, 0.0}, {2.9, 0.0}, {3.0, 0.0}});

  const auto profile = build_adaptive_speed_profile(path, parameters);

  ASSERT_GE(profile.points.size(), path.poses.size());
  EXPECT_NEAR(profile.points.front().speed_cap, parameters.max_linear_speed, 1.0e-12);
  EXPECT_NEAR(profile.points.back().speed_cap, 0.0, 1.0e-12);
  EXPECT_EQ(profile.points.back().limiting_constraint, "terminal_braking");
  EXPECT_LT(profile.points[profile.points.size() - 2U].speed_cap, parameters.max_linear_speed);
}

TEST(AdaptiveSpeedProfile, TerminalBufferCreatesExactEarlyStopKnot)
{
  AdaptiveSpeedParameters parameters;
  parameters.terminal_stop_buffer = 0.04;
  const auto profile = build_adaptive_speed_profile(
    path_from({{0.0, 0.0}, {0.5, 0.0}, {0.85, 0.0}, {1.0, 0.0}}), parameters);

  const auto stop = std::find_if(
    profile.points.begin(), profile.points.end(),
    [](const SpeedProfilePoint & point) {
      return std::abs(point.distance - 0.96) < 1.0e-12;
    });
  ASSERT_NE(stop, profile.points.end());
  EXPECT_NEAR(stop->x, 0.96, 1.0e-12);
  EXPECT_DOUBLE_EQ(stop->speed_cap, 0.0);
  EXPECT_EQ(stop->limiting_constraint, "terminal_braking");
  ASSERT_NE(stop, profile.points.begin());
  EXPECT_GT((stop - 1)->speed_cap, 0.0);
  EXPECT_LT((stop - 1)->speed_cap, parameters.max_linear_speed);
  EXPECT_DOUBLE_EQ(profile.points.back().speed_cap, 0.0);
}

TEST(AdaptiveSpeedProfile, CurvatureCapUsesLateralAcceleration)
{
  AdaptiveSpeedParameters parameters;
  parameters.max_linear_speed = 1.0;
  parameters.max_angular_speed = 10.0;
  parameters.max_wheel_linear_speed = 10.0;
  parameters.max_lateral_acceleration = 0.20;
  const auto caps = instantaneous_speed_caps(2.0, parameters);

  EXPECT_NEAR(caps.combined, std::sqrt(0.10), 1.0e-12);
  EXPECT_EQ(caps.limiting_constraint, "lateral_acceleration");
}

TEST(AdaptiveSpeedProfile, CornerConstraintPropagatesBeforeCorner)
{
  AdaptiveSpeedParameters parameters;
  parameters.max_linear_speed = 0.50;
  parameters.max_angular_speed = 10.0;
  parameters.max_wheel_linear_speed = 10.0;
  parameters.max_lateral_acceleration = 0.08;
  parameters.curvature_sample_distance = 0.10;
  const auto path = path_from({
        {0.0, 0.0}, {0.2, 0.0}, {0.4, 0.0}, {0.6, 0.0},
        {0.8, 0.0}, {1.0, 0.0}, {1.2, 0.0}, {1.4, 0.0},
        {1.6, 0.0}, {1.8, 0.0}, {2.0, 0.0}, {2.0, 0.2},
        {2.0, 0.4}});

  const auto profile = build_adaptive_speed_profile(path, parameters);

  const auto first_curvature_constraint = std::find_if(
    profile.points.begin(), profile.points.end(),
    [&parameters](const SpeedProfilePoint & point) {
      return point.local_speed_cap < parameters.max_linear_speed;
    });
  ASSERT_NE(first_curvature_constraint, profile.points.end());
  ASSERT_NE(first_curvature_constraint, profile.points.begin());
  const auto & point_before = *(first_curvature_constraint - 1);
  EXPECT_LT(point_before.speed_cap, point_before.local_speed_cap);
  EXPECT_EQ(point_before.limiting_constraint, "future_braking");
}

TEST(AdaptiveSpeedProfile, EveryBackwardSpeedDropHasEnoughSCurveDistance)
{
  AdaptiveSpeedParameters parameters;
  parameters.max_linear_speed = 0.50;
  parameters.max_angular_speed = 10.0;
  parameters.max_wheel_linear_speed = 10.0;
  parameters.max_lateral_acceleration = 0.08;
  parameters.max_angular_acceleration = 10.0;
  parameters.curvature_sample_distance = 0.10;
  parameters.terminal_stop_buffer = 0.0;
  parameters.terminal_linear_speed = parameters.max_linear_speed;
  const auto profile = build_adaptive_speed_profile(
    path_from({
        {0.0, 0.0}, {0.1, 0.0}, {0.2, 0.0}, {0.3, 0.0},
        {0.4, 0.0}, {0.5, 0.0}, {0.6, 0.0}, {0.7, 0.0},
        {0.8, 0.0}, {0.9, 0.0}, {1.0, 0.0}, {1.0, 0.1},
        {1.0, 0.2}}),
    parameters);

  for (std::size_t index = 1U; index < profile.points.size(); ++index) {
    const auto & first = profile.points[index - 1U];
    const auto & last = profile.points[index];
    if (first.speed_cap > last.speed_cap + 1.0e-10) {
      const double available = last.distance - first.distance;
      const double required = jerk_limited_deceleration_distance(
        first.speed_cap, last.speed_cap,
        parameters.max_linear_deceleration, parameters.max_linear_jerk);
      EXPECT_LE(required, available + 1.0e-8);
    }
  }
}

TEST(AdaptiveSpeedProfile, CurveExitHasEnoughForwardSCurveAccelerationDistance)
{
  AdaptiveSpeedParameters parameters;
  parameters.max_linear_speed = 0.50;
  parameters.max_angular_speed = 10.0;
  parameters.max_wheel_linear_speed = 10.0;
  parameters.max_lateral_acceleration = 0.08;
  parameters.max_angular_acceleration = 10.0;
  parameters.curvature_sample_distance = 0.10;
  parameters.terminal_stop_buffer = 0.0;
  parameters.terminal_linear_speed = parameters.max_linear_speed;
  const auto profile = build_adaptive_speed_profile(
    path_from({
        {0.0, 0.0}, {0.2, 0.0}, {0.4, 0.0}, {0.6, 0.0},
        {0.8, 0.0}, {1.0, 0.0}, {1.0, 0.2}, {1.0, 0.4},
        {1.0, 0.6}, {1.0, 0.8}, {1.0, 1.0}, {1.2, 1.0},
        {1.4, 1.0}, {1.6, 1.0}, {1.8, 1.0}, {2.0, 1.0}}),
    parameters);

  bool saw_forward_limited_point = false;
  for (std::size_t index = 1U; index < profile.points.size(); ++index) {
    const auto & first = profile.points[index - 1U];
    const auto & last = profile.points[index];
    if (last.speed_cap > first.speed_cap + 1.0e-10) {
      const double available = last.distance - first.distance;
      const double required = jerk_limited_acceleration_distance(
        first.speed_cap, last.speed_cap,
        parameters.max_linear_acceleration, parameters.max_linear_jerk);
      EXPECT_LE(required, available + 1.0e-8);
      saw_forward_limited_point =
        saw_forward_limited_point ||
        last.limiting_constraint == "past_acceleration";
    }
  }
  EXPECT_TRUE(saw_forward_limited_point);
}

TEST(AdaptiveSpeedProfile, EnforcesDiscreteAngularAccelerationEnvelope)
{
  AdaptiveSpeedParameters parameters;
  parameters.max_linear_speed = 0.50;
  parameters.max_angular_speed = 10.0;
  parameters.max_wheel_linear_speed = 10.0;
  parameters.max_lateral_acceleration = 10.0;
  parameters.max_angular_acceleration = 0.20;
  parameters.terminal_stop_buffer = 0.0;
  parameters.terminal_linear_speed = parameters.max_linear_speed;
  const auto profile = build_adaptive_speed_profile(
    path_from({
        {0.0, 0.0}, {0.1, 0.0}, {0.2, 0.0},
        {0.3, 0.02}, {0.39, 0.06}, {0.47, 0.12},
        {0.53, 0.20}, {0.57, 0.29}}),
    parameters);

  for (std::size_t index = 1U; index < profile.points.size(); ++index) {
    const auto & first = profile.points[index - 1U];
    const auto & last = profile.points[index];
    const double distance = last.distance - first.distance;
    const double time_step =
      2.0 * distance / (first.speed_cap + last.speed_cap);
    const double angular_acceleration = std::abs(
      last.curvature * last.speed_cap -
      first.curvature * first.speed_cap) / time_step;
    EXPECT_LE(
      angular_acceleration,
      parameters.max_angular_acceleration * (1.0 + 1.0e-4));
  }
}

TEST(AdaptiveSpeedProfile, ProjectionIsLocalAndInterpolated)
{
  AdaptiveSpeedParameters parameters;
  const auto profile = build_adaptive_speed_profile(
    path_from({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}}),
    parameters);

  const auto projection = project_onto_speed_profile(
    profile, 1.25, 0.10, 0.0, 1U, 1.0, 0.25, 0.80, 0.20, 0.03);

  ASSERT_TRUE(projection.valid);
  EXPECT_EQ(projection.segment_index, 1U);
  EXPECT_NEAR(projection.distance, 1.25, 1.0e-12);
  EXPECT_NEAR(projection.cross_track_error, 0.10, 1.0e-12);
  EXPECT_NEAR(projection.remaining_distance, 1.75, 1.0e-12);
}

TEST(AdaptiveSpeedProfile, ProjectionStaysInsideContinuousSCurveEnvelope)
{
  AdaptiveSpeedProfile profile;
  profile.frame_id = "map";
  profile.max_linear_acceleration = 0.35;
  profile.max_linear_deceleration = 0.45;
  profile.max_linear_jerk = 0.90;
  const double initial_speed = 0.30;
  const double final_speed = 0.12;
  const double length = jerk_limited_deceleration_distance(
    initial_speed, final_speed,
    profile.max_linear_deceleration, profile.max_linear_jerk);
  profile.points = {
    {0.0, 0.0, 0.0, 0.0, initial_speed, initial_speed, "future_braking"},
    {length, 0.0, length, 0.0, final_speed, final_speed, "curvature"}};

  const auto projection = project_onto_speed_profile(
    profile, 0.5 * length, 0.0, 0.0, 0U, 0.0, 0.0, length, 0.20, 0.0);

  ASSERT_TRUE(projection.valid);
  const double required = jerk_limited_deceleration_distance(
    projection.speed_cap, final_speed,
    profile.max_linear_deceleration, profile.max_linear_jerk);
  EXPECT_LE(required, 0.5 * length + 1.0e-10);
  EXPECT_LT(projection.speed_cap, 0.5 * (initial_speed + final_speed));
  EXPECT_EQ(projection.limiting_constraint, "future_braking");
}

TEST(AdaptiveSpeedProfile, ProjectionUsesHeadingAndCannotJumpBackwardAtCrossing)
{
  AdaptiveSpeedParameters parameters;
  parameters.terminal_stop_buffer = 0.0;
  parameters.terminal_linear_speed = parameters.max_linear_speed;
  const auto profile = build_adaptive_speed_profile(
    path_from({
        {-1.0, 0.0}, {0.0, 0.0}, {1.0, 0.0},
        {0.0, 0.0}, {-1.0, 0.0}}),
    parameters);

  const auto forward = project_onto_speed_profile(
    profile, 0.0, 0.0, 0.0, 0U, 0.8, 0.8, 3.0, 0.20, 0.03);
  ASSERT_TRUE(forward.valid);
  EXPECT_LT(std::abs(forward.heading_error), 1.0e-12);
  EXPECT_GE(forward.distance, 0.77);

  const auto reverse = project_onto_speed_profile(
    profile, 0.0, 0.0, 3.14159265358979323846,
    2U, 2.2, 0.8, 2.0, 0.20, 0.03);
  ASSERT_TRUE(reverse.valid);
  EXPECT_LT(std::abs(reverse.heading_error), 1.0e-12);
  EXPECT_GE(reverse.distance, 2.17);
}

TEST(AdaptiveSpeedProfile, TrackingRecoveryCapHasSmoothDeadbandAndFloor)
{
  EXPECT_DOUBLE_EQ(
    tracking_error_speed_cap(0.01, 0.02, 0.10, 0.06, 0.30),
    0.30);
  EXPECT_NEAR(
    tracking_error_speed_cap(0.06, 0.02, 0.10, 0.06, 0.30),
    0.18, 1.0e-12);
  EXPECT_DOUBLE_EQ(
    tracking_error_speed_cap(-0.12, 0.02, 0.10, 0.06, 0.30),
    0.06);
  EXPECT_THROW(
    tracking_error_speed_cap(0.02, 0.10, 0.02, 0.06, 0.30),
    std::invalid_argument);
}

TEST(AdaptiveSpeedProfile, JerkLimitsAccelerationButNeverExceedsSafetyCap)
{
  AdaptiveSpeedParameters parameters;
  JerkLimitedSpeedState state;

  const auto first = update_jerk_limited_speed(0.30, 0.0, 0.05, parameters, state);
  EXPECT_NEAR(first.acceleration, parameters.max_linear_jerk * 0.05, 1.0e-12);
  EXPECT_NEAR(first.speed, first.acceleration * 0.05, 1.0e-12);
  EXPECT_FALSE(first.safety_override);

  const auto emergency = update_jerk_limited_speed(0.0, first.speed, 0.05, parameters, state);
  EXPECT_DOUBLE_EQ(emergency.speed, 0.0);
  EXPECT_TRUE(emergency.safety_override);

  const auto recovered = update_jerk_limited_speed(0.30, 0.0, 0.05, parameters, state);
  EXPECT_TRUE(recovered.safety_override);
  EXPECT_TRUE(std::isfinite(recovered.speed));
  EXPECT_TRUE(std::isfinite(recovered.acceleration));
  EXPECT_LE(recovered.acceleration, parameters.max_linear_acceleration);
  EXPECT_GE(recovered.acceleration, -parameters.max_linear_deceleration);

  const auto nominal = update_jerk_limited_speed(
    0.30, recovered.speed, 0.05, parameters, state);
  EXPECT_FALSE(nominal.safety_override);
  EXPECT_LE(std::abs(nominal.jerk), parameters.max_linear_jerk + 1.0e-12);
}

TEST(AdaptiveSpeedProfile, LaggingFeedbackDoesNotResetCommandIntegrator)
{
  AdaptiveSpeedParameters parameters;
  parameters.feedback_sync_tolerance = 0.005;
  JerkLimitedSpeedState state;

  const auto first = update_jerk_limited_speed(
    0.30, 0.0, 0.05, parameters, state);
  const auto second = update_jerk_limited_speed(
    0.30, 0.0, 0.05, parameters, state);
  const auto limited = update_jerk_limited_speed(
    0.30, 0.0, 0.05, parameters, state);
  const auto held = update_jerk_limited_speed(
    0.30, 0.0, 0.05, parameters, state);

  EXPECT_LT(first.speed, second.speed);
  EXPECT_LT(second.speed, limited.speed);
  EXPECT_TRUE(limited.feedback_limited);
  EXPECT_TRUE(held.feedback_limited);
  EXPECT_GE(held.speed, limited.speed);
  EXPECT_LE(std::abs(limited.jerk), parameters.max_linear_jerk + 1.0e-12);
  EXPECT_LE(std::abs(held.jerk), parameters.max_linear_jerk + 1.0e-12);

  const auto caught_up = update_jerk_limited_speed(
    0.30, held.speed, 0.05, parameters, state);
  EXPECT_FALSE(caught_up.feedback_limited);
  EXPECT_GT(caught_up.speed, held.speed);
}

TEST(AdaptiveSpeedProfile, RejectsInvalidLimitsAndDuplicatePathPoints)
{
  AdaptiveSpeedParameters parameters;
  parameters.max_linear_jerk = 0.0;
  EXPECT_THROW(validate_adaptive_speed_parameters(parameters), std::invalid_argument);

  parameters = AdaptiveSpeedParameters();
  parameters.terminal_stop_buffer = -0.01;
  EXPECT_THROW(validate_adaptive_speed_parameters(parameters), std::invalid_argument);

  parameters = AdaptiveSpeedParameters();
  EXPECT_THROW(
    build_adaptive_speed_profile(
      path_from({{0.0, 0.0}, {0.0, 0.0}}), parameters),
    std::invalid_argument);
}

}  // namespace
}  // namespace adaptive_pivot_g2_controller
