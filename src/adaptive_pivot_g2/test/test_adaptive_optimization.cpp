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
#include <cstddef>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

#include "adaptive_pivot_g2/adaptive_search.hpp"
#include "adaptive_pivot_g2/path_optimization.hpp"
#include "adaptive_pivot_g2/quintic_transition.hpp"

namespace adaptive_pivot_g2
{
namespace
{

constexpr double kHalfPi = 1.57079632679489661923;
constexpr double kPi = 3.14159265358979323846;

TEST(AdaptiveSearch, FindsRadiusOutsideLegacyBank)
{
  const std::vector<double> legacy{
    0.20, 0.30, 0.40, 0.50, 0.60, 0.75, 1.00, 1.25, 1.50};
  AdaptiveSearchOptions options;
  options.initial_samples = 6U;
  options.maximum_evaluations = 20U;
  options.radius_tolerance = 0.005;
  const auto result = search_trim_distance(
    kHalfPi, 1.5, 0.0025, options,
    [](double trim) {
      SearchEvaluation evaluation;
      evaluation.status = SearchSampleStatus::kFeasible;
      evaluation.objective = std::pow(trim - 0.347, 2.0);
      return evaluation;
    });

  ASSERT_TRUE(result.valid_domain);
  ASSERT_FALSE(result.ranked_feasible_samples.empty());
  const double radius =
    result.samples[result.ranked_feasible_samples.front()].design_radius;
  EXPECT_NEAR(radius, 0.347, 0.006);
  EXPECT_TRUE(
    std::none_of(
      legacy.begin(), legacy.end(),
      [radius](double old_radius) {
        return std::abs(radius - old_radius) < 1.0e-6;
      }));
}

TEST(AdaptiveSearch, HandlesSmallLargeAndTooShortDomains)
{
  AdaptiveSearchOptions options;
  options.maximum_evaluations = 12U;
  const auto evaluator = [](double trim) {
      return SearchEvaluation{
      SearchSampleStatus::kFeasible, trim, 0U, ""};
    };
  const auto small = search_trim_distance(0.05, 0.50, 0.002, options, evaluator);
  const auto large = search_trim_distance(2.8, 0.80, 0.002, options, evaluator);
  const auto too_short = search_trim_distance(
    kHalfPi, 0.05, 0.002, options, evaluator);

  EXPECT_TRUE(small.valid_domain);
  EXPECT_TRUE(large.valid_domain);
  EXPECT_FALSE(too_short.valid_domain);
  EXPECT_LE(large.maximum_trim, 0.80);
}

TEST(AdaptiveSearch, DirectTrimDomainDoesNotShrinkAtShallowAngles)
{
  AdaptiveSearchOptions options;
  options.initial_samples = 5U;
  options.maximum_evaluations = 5U;
  options.radius_tolerance = 0.01;
  const auto result = search_direct_trim_distance(
    5.0 * kPi / 180.0, 0.02, 1.20, 0.005, options,
    [](double trim) {
      return SearchEvaluation{SearchSampleStatus::kFeasible, trim, 0U, ""};
    });

  ASSERT_TRUE(result.valid_domain);
  EXPECT_DOUBLE_EQ(result.minimum_trim, 0.02);
  EXPECT_DOUBLE_EQ(result.maximum_trim, 1.20);
  ASSERT_EQ(result.samples.size(), 5U);
  EXPECT_DOUBLE_EQ(result.samples.front().trim_distance, 0.02);
  EXPECT_DOUBLE_EQ(result.samples.back().trim_distance, 1.20);
}

TEST(AdaptiveSearch, DirectTrimSearchRefinesPhysicalFeasibilityBoundary)
{
  AdaptiveSearchOptions options;
  options.initial_samples = 4U;
  options.maximum_evaluations = 14U;
  options.radius_tolerance = 0.005;
  const auto result = search_direct_trim_distance(
    150.0 * kPi / 180.0, 0.02, 1.50, 0.002, options,
    [](double trim) {
      return trim < 0.60 ?
             SearchEvaluation{SearchSampleStatus::kInfeasible,
             std::numeric_limits<double>::infinity(), 0U, "wheel_reversal"} :
             SearchEvaluation{SearchSampleStatus::kFeasible, trim, 0U, ""};
    });

  ASSERT_TRUE(result.valid_domain);
  ASSERT_GT(result.feasible_count, 0U);
  const double first_feasible = result.samples[
    result.ranked_feasible_samples.front()].trim_distance;
  EXPECT_GE(first_feasible, 0.60);
  EXPECT_LT(first_feasible, 0.65);
}

TEST(AdaptiveSearch, RefinesSafetyBoundaryAndIsDeterministic)
{
  AdaptiveSearchOptions options;
  const auto evaluator = [](double trim) {
      if (trim < 0.583) {
        return SearchEvaluation{
        SearchSampleStatus::kUnsafe,
        std::numeric_limits<double>::infinity(), 0U, "collision"};
      }
      return SearchEvaluation{
      SearchSampleStatus::kFeasible, trim, 0U, ""};
    };
  const auto first = search_trim_distance(
    kHalfPi, 1.5, 0.005, options, evaluator);
  const auto second = search_trim_distance(
    kHalfPi, 1.5, 0.005, options, evaluator);

  ASSERT_EQ(first.samples.size(), second.samples.size());
  for (std::size_t index = 0; index < first.samples.size(); ++index) {
    EXPECT_DOUBLE_EQ(
      first.samples[index].trim_distance, second.samples[index].trim_distance);
    EXPECT_EQ(first.samples[index].status, second.samples[index].status);
  }
  ASSERT_FALSE(first.ranked_feasible_samples.empty());
  const double boundary =
    first.samples[first.ranked_feasible_samples.front()].trim_distance;
  EXPECT_GE(boundary, 0.583);
  EXPECT_LT(boundary, 0.60);
}

TEST(PathOptimization, AllowsOneCornerAboveLegacyFortyFivePercent)
{
  const std::vector<std::vector<CornerState>> states{
    {{true, 0.0, 1.0, 0U}, {false, 0.60, 0.1, 1U}},
    {{true, 0.0, 1.0, 0U}, {false, 0.30, 0.1, 1U}}};
  const auto result = optimize_corner_states(states, {1.0}, {0.05});

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.selected_state_indices.size(), 2U);
  EXPECT_EQ(result.selected_state_indices[0], 1U);
  EXPECT_EQ(result.selected_state_indices[1], 1U);
  EXPECT_EQ(result.pivot_count, 0U);
}

TEST(PathOptimization, RejectsOverlapAndUsesPivotBesideTransition)
{
  const std::vector<std::vector<CornerState>> states{
    {{true, 0.0, 0.8, 0U}, {false, 0.60, 0.1, 1U}},
    {{true, 0.0, 0.8, 0U}, {false, 0.40, 0.1, 1U}}};
  const auto result = optimize_corner_states(states, {1.0}, {0.05});

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.selected_state_indices.size(), 2U);
  const bool first_is_pivot =
    states[0][result.selected_state_indices[0]].pivot;
  const bool second_is_pivot =
    states[1][result.selected_state_indices[1]].pivot;
  EXPECT_NE(first_is_pivot, second_is_pivot);
}

TEST(PathOptimization, CoversPivotAndG2SafetyCombinations)
{
  const auto pivot_only = optimize_corner_states(
    {{{true, 0.0, 1.0, 0U}}}, {}, {});
  const auto transition_only = optimize_corner_states(
    {{{false, 0.25, 0.2, 4U}}}, {}, {});
  const auto no_state = optimize_corner_states({{}}, {}, {});

  ASSERT_TRUE(pivot_only.valid);
  EXPECT_EQ(pivot_only.pivot_count, 1U);
  ASSERT_TRUE(transition_only.valid);
  EXPECT_EQ(transition_only.pivot_count, 0U);
  EXPECT_FALSE(no_state.valid);
}

TEST(AdaptiveGeometry, ExplicitTrimCanExceedLegacyFractionWithoutWheelReversal)
{
  RobotLimits limits;
  limits.wheel_separation = 0.10;
  limits.max_linear_speed = 1.0;
  limits.max_angular_speed = 10.0;
  limits.max_wheel_speed = 2.0;
  TransitionOptions options;
  options.sample_spacing = 0.01;
  const CornerInput corner{
    {0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, 1.0, 1.0};

  const auto adaptive = generate_quintic_transition_for_trim(
    corner, limits, options, 0.60);
  options.design_radius = 0.60;
  const auto legacy = generate_quintic_transition(corner, limits, options);

  ASSERT_TRUE(adaptive.valid) << adaptive.rejection_reason;
  EXPECT_FALSE(legacy.valid);
  EXPECT_NEAR(adaptive.trim_distance, 0.60, 1.0e-12);
  for (std::size_t index = 1; index < adaptive.samples.size(); ++index) {
    EXPECT_TRUE(finite(adaptive.samples[index].position));
    EXPECT_GT(
      distance(
        adaptive.samples[index - 1U].position,
        adaptive.samples[index].position),
      0.0);
  }
}

}  // namespace
}  // namespace adaptive_pivot_g2
