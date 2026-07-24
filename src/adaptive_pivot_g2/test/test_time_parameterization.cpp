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

#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

#include "adaptive_pivot_g2/quintic_transition.hpp"
#include "adaptive_pivot_g2/time_parameterization.hpp"

namespace adaptive_pivot_g2
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(TimeParameterization, ProducesTriangularProfileOnStraightPath)
{
  RobotLimits limits;
  limits.max_linear_acceleration = 1.0;
  limits.max_linear_deceleration = 1.0;
  limits.max_angular_acceleration = 2.0;

  std::vector<PathSample> path;
  for (int index = 0; index <= 10; ++index) {
    path.push_back({{0.1 * index, 0.0}, 0.0, 0.0, 2.0});
  }

  const TimedProfile profile = parameterize_time(path, limits, 0.0, 0.0);

  ASSERT_TRUE(profile.valid) << profile.rejection_reason;
  EXPECT_NEAR(profile.total_time, 2.0, 1.0e-9);
  EXPECT_NEAR(profile.linear_speed.front(), 0.0, 1.0e-12);
  EXPECT_NEAR(profile.linear_speed[5], 1.0, 1.0e-9);
  EXPECT_NEAR(profile.linear_speed.back(), 0.0, 1.0e-12);
  EXPECT_NEAR(profile.max_abs_angular_acceleration, 0.0, 1.0e-12);
}

TEST(TimeParameterization, EnforcesAngularAccelerationOnG2Transition)
{
  RobotLimits limits;
  limits.wheel_separation = 0.20;
  limits.max_linear_speed = 1.0;
  limits.max_angular_speed = 10.0;
  limits.max_wheel_speed = 2.0;
  limits.max_linear_acceleration = 4.0;
  limits.max_linear_deceleration = 4.0;
  limits.max_angular_acceleration = 0.40;
  const CornerInput corner{{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, 2.0, 2.0};
  TransitionOptions options;
  options.design_radius = 0.50;
  options.sample_spacing = 0.01;
  const TransitionCandidate candidate = generate_quintic_transition(corner, limits, options);
  ASSERT_TRUE(candidate.valid) << candidate.rejection_reason;

  const TimedProfile profile = parameterize_time(candidate.samples, limits, 0.0, 0.0);

  ASSERT_TRUE(profile.valid) << profile.rejection_reason;
  EXPECT_LE(
    profile.max_abs_angular_acceleration,
    limits.max_angular_acceleration * (1.0 + 1.0e-4));
}

TEST(TimeParameterization, PivotTimeIncludesEquivalentStraightTravel)
{
  RobotLimits limits;
  limits.max_linear_speed = 1.0;
  limits.max_wheel_speed = 1.0;
  limits.max_linear_acceleration = 1.0;
  limits.max_linear_deceleration = 1.0;
  limits.max_angular_speed = 1.0;
  limits.max_angular_acceleration = 2.0;

  const double actual = estimate_pivot_window_time(0.5, 0.5 * kPi, limits, 0.0, 0.0);
  const double expected_translation = 4.0 * std::sqrt(0.5);
  const double expected_rotation = 1.0 + (0.5 * kPi - 0.5);

  EXPECT_NEAR(actual, expected_translation + expected_rotation, 1.0e-12);
}

TEST(TimeParameterization, PivotStraightMotionAlsoRespectsWheelSpeed)
{
  RobotLimits limits;
  limits.max_linear_speed = 1.0;
  limits.max_wheel_speed = 0.5;
  limits.max_linear_acceleration = 1.0;
  limits.max_linear_deceleration = 1.0;
  limits.max_angular_speed = 1.0;
  limits.max_angular_acceleration = 2.0;

  EXPECT_FALSE(std::isfinite(
      estimate_pivot_window_time(0.2, 0.5, limits, 1.0, 1.0)));
  EXPECT_TRUE(std::isfinite(
      estimate_pivot_window_time(0.2, 0.5, limits, 0.5, 0.5)));
}

TEST(TimeParameterization, RejectsImpossibleBoundarySpeedForShortWindow)
{
  const double duration = minimum_translation_time(0.01, 1.0, 0.0, 1.0, 1.0, 1.0);
  EXPECT_FALSE(std::isfinite(duration));
}

TEST(TimeParameterization, RejectsInvalidBoundarySpeedCaps)
{
  RobotLimits limits;
  const std::vector<PathSample> path{
    {{0.0, 0.0}, 0.0, 0.0, limits.max_linear_speed},
    {{1.0, 0.0}, 0.0, 0.0, limits.max_linear_speed}};

  EXPECT_FALSE(parameterize_time(path, limits, -0.1, 0.0).valid);
  EXPECT_FALSE(parameterize_time(
      path, limits, std::numeric_limits<double>::quiet_NaN(), 0.0).valid);
}

TEST(TimeParameterization, TransitionWindowAddsContinuousStraightContext)
{
  RobotLimits limits;
  limits.max_linear_speed = 0.60;
  limits.max_angular_speed = 4.0;
  limits.max_wheel_speed = 2.0;
  limits.max_lateral_acceleration = 10.0;
  limits.max_linear_acceleration = 0.50;
  limits.max_linear_deceleration = 0.50;
  limits.max_angular_acceleration = 0.30;
  const CornerInput corner{{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, 2.0, 2.0};
  TransitionOptions options;
  options.design_radius = 0.25;
  options.sample_spacing = 0.02;
  const auto candidate = generate_quintic_transition(corner, limits, options);
  ASSERT_TRUE(candidate.valid) << candidate.rejection_reason;

  const auto curve_only = parameterize_time(
    candidate.samples, limits, limits.max_linear_speed,
    limits.max_linear_speed);
  const auto window = parameterize_transition_window(
    candidate, 0.60, limits, limits.max_linear_speed,
    limits.max_linear_speed, options.sample_spacing);

  ASSERT_TRUE(curve_only.valid) << curve_only.rejection_reason;
  ASSERT_TRUE(window.valid) << window.rejection_reason;
  EXPECT_GT(window.linear_speed.size(), curve_only.linear_speed.size());
  EXPECT_GT(window.total_time, curve_only.total_time);
  EXPECT_LE(
    window.max_abs_angular_acceleration,
    limits.max_angular_acceleration * (1.0 + 1.0e-4));
}

TEST(TimeParameterization, CommonWindowCanPreferPivotWhenAngularAccelerationIsTight)
{
  RobotLimits limits;
  limits.wheel_separation = 0.20;
  limits.max_linear_speed = 1.0;
  limits.max_angular_speed = 4.0;
  limits.max_wheel_speed = 2.0;
  limits.max_lateral_acceleration = 100.0;
  limits.max_linear_acceleration = 1.0;
  limits.max_linear_deceleration = 1.0;
  limits.max_angular_acceleration = 2.0;
  const CornerInput corner{{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, 2.0, 2.0};
  TransitionOptions options;
  options.design_radius = 0.35;
  options.sample_spacing = 0.02;
  const TransitionCandidate candidate = generate_quintic_transition(corner, limits, options);
  ASSERT_TRUE(candidate.valid) << candidate.rejection_reason;

  const double window_distance = 0.80;
  const TimedProfile transition = parameterize_transition_window(
    candidate, window_distance, limits, 0.0, 0.0, options.sample_spacing);
  const double pivot = estimate_pivot_window_time(
    window_distance, 0.5 * kPi, limits, 0.0, 0.0);

  ASSERT_TRUE(transition.valid) << transition.rejection_reason;
  ASSERT_TRUE(std::isfinite(pivot));
  EXPECT_GT(transition.total_time, pivot);
}

}  // namespace
}  // namespace adaptive_pivot_g2
