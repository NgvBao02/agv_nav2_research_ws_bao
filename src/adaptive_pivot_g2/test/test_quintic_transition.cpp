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

#include "gtest/gtest.h"

#include "adaptive_pivot_g2/quintic_transition.hpp"

namespace adaptive_pivot_g2
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

RobotLimits permissive_limits()
{
  RobotLimits limits;
  limits.wheel_separation = 0.20;
  limits.max_linear_speed = 1.0;
  limits.max_angular_speed = 10.0;
  limits.max_wheel_speed = 2.0;
  return limits;
}

TEST(RobotLimits, DefaultsMatchCurrentRollingTreadGeometry)
{
  const RobotLimits limits;
  EXPECT_DOUBLE_EQ(limits.wheel_separation, 0.2548);
}

TEST(QuinticTransition, SatisfiesEndpointG2Conditions)
{
  const CornerInput corner{{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, 2.0, 2.0};
  TransitionOptions options;
  options.design_radius = 0.35;
  options.sample_spacing = 0.02;

  const TransitionCandidate candidate =
    generate_quintic_transition(corner, permissive_limits(), options);

  ASSERT_TRUE(candidate.valid) << candidate.rejection_reason;
  ASSERT_GE(candidate.samples.size(), 2U);
  EXPECT_NEAR(candidate.turn_angle, 0.5 * kPi, 1.0e-12);
  EXPECT_NEAR(candidate.trim_distance, 0.35, 1.0e-12);
  EXPECT_NEAR(candidate.control_fraction, 0.35, 1.0e-12);
  EXPECT_NEAR(candidate.control_distance, 0.1225, 1.0e-12);
  EXPECT_NEAR(candidate.samples.front().position.x, -0.35, 1.0e-12);
  EXPECT_NEAR(candidate.samples.front().position.y, 0.0, 1.0e-12);
  EXPECT_NEAR(candidate.samples.back().position.x, 0.0, 1.0e-12);
  EXPECT_NEAR(candidate.samples.back().position.y, 0.35, 1.0e-12);
  EXPECT_NEAR(candidate.samples.front().heading, 0.0, 1.0e-12);
  EXPECT_NEAR(candidate.samples.back().heading, 0.5 * kPi, 1.0e-12);
  EXPECT_NEAR(candidate.samples.front().curvature, 0.0, 1.0e-10);
  EXPECT_NEAR(candidate.samples.back().curvature, 0.0, 1.0e-10);

  for (std::size_t index = 1; index < candidate.samples.size(); ++index) {
    EXPECT_LE(
      distance(candidate.samples[index - 1].position, candidate.samples[index].position),
      options.sample_spacing * 1.001);
    EXPECT_GE(candidate.samples[index].curvature, -1.0e-8);
  }
  EXPECT_GT(candidate.curvature_energy, 0.0);
  EXPECT_GT(candidate.path_length, 0.0);
}

TEST(QuinticTransition, PreservesRightTurnCurvatureSign)
{
  const CornerInput corner{{0.0, 0.0}, {1.0, 0.0}, {0.0, -1.0}, 2.0, 2.0};
  const TransitionCandidate candidate =
    generate_quintic_transition(corner, permissive_limits(), TransitionOptions{});

  ASSERT_TRUE(candidate.valid) << candidate.rejection_reason;
  EXPECT_LT(candidate.turn_angle, 0.0);
  for (const PathSample & sample : candidate.samples) {
    EXPECT_LE(sample.curvature, 1.0e-8);
  }
}

TEST(QuinticTransition, RejectsCornerWindowOverlap)
{
  const CornerInput corner{{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, 0.30, 0.30};
  TransitionOptions options;
  options.design_radius = 0.20;

  const TransitionCandidate candidate =
    generate_quintic_transition(corner, permissive_limits(), options);

  EXPECT_FALSE(candidate.valid);
  EXPECT_EQ(candidate.rejection_reason, "transition would overlap an adjacent corner window");
}

TEST(QuinticTransition, RejectsInnerWheelReversalForWideTrack)
{
  RobotLimits limits = permissive_limits();
  limits.wheel_separation = 0.42;
  const CornerInput corner{{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, 2.0, 2.0};
  TransitionOptions options;
  options.design_radius = 0.20;

  const TransitionCandidate candidate = generate_quintic_transition(corner, limits, options);

  EXPECT_FALSE(candidate.valid);
  EXPECT_EQ(candidate.rejection_reason, "transition requires a reversing inner wheel");
}

TEST(QuinticTransition, AngleAwareControlFractionContractsForSharpTurns)
{
  const double shallow = recommended_control_fraction(30.0 * kPi / 180.0);
  const double right_angle = recommended_control_fraction(0.5 * kPi);
  const double sharp = recommended_control_fraction(150.0 * kPi / 180.0);

  EXPECT_GT(shallow, right_angle);
  EXPECT_GT(right_angle, sharp);
  EXPECT_NEAR(shallow, 0.3263, 5.0e-3);
  EXPECT_NEAR(sharp, 0.1630, 1.0e-2);
}

TEST(QuinticTransition, ControlFractionBankIncludesCentreAndBounds)
{
  const auto candidates = generate_control_fraction_candidates(
    0.5 * kPi, 0.08, 0.45, 7U);

  ASSERT_EQ(candidates.size(), 7U);
  EXPECT_DOUBLE_EQ(candidates.front(), 0.08);
  EXPECT_DOUBLE_EQ(candidates.back(), 0.45);
  EXPECT_NEAR(candidates[3], recommended_control_fraction(0.5 * kPi), 1.0e-12);
  EXPECT_TRUE(std::is_sorted(candidates.begin(), candidates.end()));
}

TEST(QuinticTransition, JointShapeSearchCanReduceSharpTurnEnergy)
{
  const double angle = 120.0 * kPi / 180.0;
  const CornerInput corner{
    {0.0, 0.0}, {1.0, 0.0}, {std::cos(angle), std::sin(angle)}, 3.0, 3.0};
  TransitionOptions options;
  options.sample_spacing = 0.005;
  const auto fixed = generate_quintic_transition_for_shape(
    corner, permissive_limits(), options, 1.0, 0.35);
  const auto adaptive = generate_quintic_transition_for_shape(
    corner, permissive_limits(), options, 1.0,
    recommended_control_fraction(angle));

  ASSERT_TRUE(fixed.valid) << fixed.rejection_reason;
  ASSERT_TRUE(adaptive.valid) << adaptive.rejection_reason;
  EXPECT_LT(adaptive.curvature_energy, 0.85 * fixed.curvature_energy);
  EXPECT_LT(adaptive.max_abs_curvature, fixed.max_abs_curvature);
  EXPECT_NE(adaptive.control_fraction, fixed.control_fraction);
}

}  // namespace
}  // namespace adaptive_pivot_g2
