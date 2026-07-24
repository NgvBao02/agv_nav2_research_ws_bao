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

#include "adaptive_pivot_g2/path_conditioning.hpp"

namespace adaptive_pivot_g2
{
namespace
{

const auto kAlwaysSafe = [](const Vec2 &, const Vec2 &) {return true;};

void expect_same_points(
  const std::vector<Vec2> & actual,
  const std::vector<Vec2> & expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t index = 0; index < actual.size(); ++index) {
    EXPECT_DOUBLE_EQ(actual[index].x, expected[index].x);
    EXPECT_DOUBLE_EQ(actual[index].y, expected[index].y);
  }
}

TEST(PathConditioning, CollapsesGridStaircaseInsideBound)
{
  const std::vector<Vec2> staircase{
    {0.00, 0.00}, {0.05, 0.00}, {0.05, 0.05}, {0.10, 0.05},
    {0.10, 0.10}, {0.15, 0.10}, {0.15, 0.15}, {0.20, 0.15},
    {0.20, 0.20}};

  const auto result = condition_polyline(staircase, 0.04, kAlwaysSafe);

  ASSERT_TRUE(result.valid) << result.rejection_reason;
  ASSERT_EQ(result.points.size(), 2U);
  EXPECT_EQ(result.retained_input_indices, (std::vector<std::size_t>{0U, 8U}));
  EXPECT_LE(result.maximum_removed_deviation, 0.04);
  EXPECT_EQ(result.accepted_shortcuts, 1U);
}

TEST(PathConditioning, PreservesTrueCornerOutsideBound)
{
  const std::vector<Vec2> corner{{0.0, 0.0}, {0.5, 0.0}, {0.5, 0.5}};

  const auto result = condition_polyline(corner, 0.04, kAlwaysSafe);

  ASSERT_TRUE(result.valid);
  expect_same_points(result.points, corner);
  EXPECT_EQ(
    result.retained_input_indices,
    (std::vector<std::size_t>{0U, 1U, 2U}));
}

TEST(PathConditioning, RejectsGeometricallyValidUnsafeShortcut)
{
  const std::vector<Vec2> path{
    {0.0, 0.0}, {0.1, 0.01}, {0.2, 0.0}, {0.3, -0.01}, {0.4, 0.0}};
  const auto only_short_segments_are_safe =
    [](const Vec2 & start, const Vec2 & finish) {
      return distance(start, finish) <= 0.21;
    };

  const auto result = condition_polyline(
    path, 0.04, only_short_segments_are_safe);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.points.size(), 2U);
  EXPECT_GT(result.safety_rejected_shortcuts, 0U);
  for (std::size_t index = 1; index < result.points.size(); ++index) {
    EXPECT_TRUE(only_short_segments_are_safe(
        result.points[index - 1U], result.points[index]));
  }
}

TEST(PathConditioning, SuppressesAlternatingPlannerOscillation)
{
  const std::vector<Vec2> oscillation{
    {0.00, 0.00}, {0.20, -0.12}, {0.40, 0.03}, {0.60, -0.11},
    {0.80, 0.04}, {1.00, -0.10}, {1.20, 0.00}};
  PathConditioningOptions options;
  options.maximum_deviation = 0.04;
  options.oscillation_maximum_span = 2.0;
  options.oscillation_maximum_deviation = 0.15;
  options.oscillation_minimum_turn_angle = 0.20;
  options.oscillation_minimum_sign_changes = 2U;

  const auto result = condition_polyline(
    oscillation, options, kAlwaysSafe);

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.points.size(), 2U);
  EXPECT_EQ(result.accepted_oscillation_shortcuts, 1U);
  EXPECT_LE(result.maximum_removed_deviation, 0.15);
}

TEST(PathConditioning, DoesNotEraseMonotonicObstacleCorner)
{
  const std::vector<Vec2> detour{
    {0.0, 0.0}, {0.3, 0.0}, {0.5, 0.2}, {0.5, 0.5}, {0.8, 0.8}};
  PathConditioningOptions options;
  options.maximum_deviation = 0.04;
  options.oscillation_maximum_span = 2.0;
  options.oscillation_maximum_deviation = 0.20;

  const auto result = condition_polyline(detour, options, kAlwaysSafe);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.points.size(), 2U);
  EXPECT_EQ(result.accepted_oscillation_shortcuts, 0U);
}

TEST(PathConditioning, KeepsOscillationWhenShortcutIsUnsafe)
{
  const std::vector<Vec2> oscillation{
    {0.00, 0.00}, {0.20, -0.12}, {0.40, 0.03}, {0.60, -0.11},
    {0.80, 0.04}, {1.00, -0.10}, {1.20, 0.00}};
  PathConditioningOptions options;
  options.maximum_deviation = 0.04;
  options.oscillation_maximum_span = 2.0;
  options.oscillation_maximum_deviation = 0.15;

  const auto result = condition_polyline(
    oscillation, options,
    [](const Vec2 & start, const Vec2 & finish) {
      return distance(start, finish) < 0.5;
    });

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.points.size(), 2U);
  EXPECT_EQ(result.accepted_oscillation_shortcuts, 0U);
  EXPECT_GT(result.safety_rejected_shortcuts, 0U);
}

TEST(PathConditioning, PreservesReversalWithCoincidentRangeEndpoints)
{
  const std::vector<Vec2> reversal{{0.0, 0.0}, {0.5, 0.0}, {0.0, 0.0}};

  const auto result = condition_polyline(reversal, 0.04, kAlwaysSafe);

  ASSERT_TRUE(result.valid);
  expect_same_points(result.points, reversal);
}

TEST(PathConditioning, ZeroDeviationLeavesInputUntouched)
{
  const std::vector<Vec2> path{{0.0, 0.0}, {0.1, 0.01}, {0.2, 0.0}};
  std::size_t safety_calls = 0U;

  const auto result = condition_polyline(
    path, 0.0,
    [&safety_calls](const Vec2 &, const Vec2 &) {
      ++safety_calls;
      return true;
    });

  ASSERT_TRUE(result.valid);
  expect_same_points(result.points, path);
  EXPECT_EQ(safety_calls, 0U);
}

TEST(PathConditioning, RejectsInvalidInputs)
{
  const auto too_short = condition_polyline({{0.0, 0.0}}, 0.04, kAlwaysSafe);
  const auto negative = condition_polyline(
    {{0.0, 0.0}, {1.0, 0.0}}, -0.01, kAlwaysSafe);
  const auto non_finite = condition_polyline(
    {{0.0, 0.0}, {std::numeric_limits<double>::infinity(), 0.0}},
    0.04, kAlwaysSafe);
  const SegmentSafetyPredicate empty_predicate;
  const auto empty = condition_polyline(
    {{0.0, 0.0}, {1.0, 0.0}}, 0.04, empty_predicate);
  PathConditioningOptions invalid_options;
  invalid_options.maximum_deviation = 0.04;
  invalid_options.oscillation_minimum_sign_changes = 0U;
  const auto invalid_oscillation = condition_polyline(
    {{0.0, 0.0}, {1.0, 0.0}}, invalid_options, kAlwaysSafe);

  EXPECT_FALSE(too_short.valid);
  EXPECT_FALSE(negative.valid);
  EXPECT_FALSE(non_finite.valid);
  EXPECT_FALSE(empty.valid);
  EXPECT_FALSE(invalid_oscillation.valid);
}

}  // namespace
}  // namespace adaptive_pivot_g2
