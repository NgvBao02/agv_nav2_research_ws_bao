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
#include <vector>

#include "adaptive_pivot_g2/hierarchical_shape_search.hpp"
#include "adaptive_pivot_g2/path_optimization.hpp"
#include "gtest/gtest.h"

namespace adaptive_pivot_g2
{
namespace
{

std::vector<double> fractions_for_stage(
  const HierarchicalShapeSearchResult & result,
  ShapeSearchStage stage)
{
  std::vector<double> values;
  for (const auto & sample : result.samples) {
    if (sample.stage == stage) {
      values.push_back(sample.control_fraction);
    }
  }
  return values;
}

TEST(HierarchicalShapeSearch, RefinesBothSidesOfInteriorCoarseWinner)
{
  const auto result = search_control_fraction_coarse_to_fine(
    [](double alpha) {
      return ShapeSearchEvaluation{true, std::pow(alpha - 0.31, 2.0), 0U, ""};
    });

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(fractions_for_stage(result, ShapeSearchStage::kCoarse),
    (std::vector<double>{0.1, 0.2, 0.3, 0.4, 0.5}));
  EXPECT_EQ(result.coarse_evaluations, 5U);
  EXPECT_EQ(result.recovery_evaluations, 0U);
  EXPECT_EQ(result.refinement_evaluations, 8U);
  const auto refinement = fractions_for_stage(result, ShapeSearchStage::kRefinement);
  ASSERT_FALSE(refinement.empty());
  EXPECT_NEAR(refinement.front(), 0.22, 1.0e-12);
  EXPECT_NEAR(refinement.back(), 0.38, 1.0e-12);
  EXPECT_NEAR(result.samples[result.selected_sample_index].control_fraction, 0.30, 1.0e-12);
}

TEST(HierarchicalShapeSearch, RefinesOneIntervalAtDomainBoundary)
{
  const auto result = search_control_fraction_coarse_to_fine(
    [](double alpha) {
      return ShapeSearchEvaluation{true, alpha, 0U, ""};
    });

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.refinement_evaluations, 9U);
  const auto refinement = fractions_for_stage(result, ShapeSearchStage::kRefinement);
  ASSERT_EQ(refinement.size(), 9U);
  EXPECT_NEAR(refinement.front(), 0.11, 1.0e-12);
  EXPECT_NEAR(refinement.back(), 0.19, 1.0e-12);
  EXPECT_NEAR(result.samples[result.selected_sample_index].control_fraction, 0.1, 1.0e-12);
}

TEST(HierarchicalShapeSearch, RecoveryRunsOnlyWhenCoarseGridHasNoFeasibleShape)
{
  const auto result = search_control_fraction_coarse_to_fine(
    [](double alpha) {
      const bool feasible = std::abs(alpha - 0.35) < 0.011;
      return ShapeSearchEvaluation{
      feasible, feasible ? std::pow(alpha - 0.347, 2.0) :
      std::numeric_limits<double>::infinity(), 0U, feasible ? "" : "unsafe"};
    });

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.coarse_evaluations, 5U);
  EXPECT_EQ(result.recovery_evaluations, 4U);
  EXPECT_GT(result.refinement_evaluations, 0U);
  EXPECT_NEAR(result.samples[result.selected_sample_index].control_fraction, 0.35, 1.0e-12);
}

TEST(HierarchicalShapeSearch, IgnoresInvalidEnergyAndBreaksTiesTowardLowerAlpha)
{
  const auto result = search_control_fraction_coarse_to_fine(
    [](double alpha) {
      if (std::abs(alpha - 0.1) < 1.0e-12) {
        return ShapeSearchEvaluation{true,
        std::numeric_limits<double>::quiet_NaN(), 0U, ""};
      }
      return ShapeSearchEvaluation{true, 1.0, 0U, ""};
    });

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.samples[result.selected_sample_index].control_fraction, 0.12, 1.0e-12);
}

TEST(DerivedTrimCandidates, ProvidesPreferredAndHalfBudgetCompatibleValues)
{
  const auto middle = derive_two_trim_candidates(
    1.0, 0.7, true, true, 0.8, 0.02, 0.05, 0.01);
  ASSERT_TRUE(middle.valid);
  ASSERT_EQ(middle.values.size(), 2U);
  EXPECT_NEAR(middle.preferred_trim, 0.7, 1.0e-12);
  EXPECT_NEAR(middle.compatible_trim, 0.325, 1.0e-12);

  const auto first = derive_two_trim_candidates(
    0.6, 1.0, false, true, 0.8, 0.02, 0.05, 0.01);
  ASSERT_TRUE(first.valid);
  EXPECT_NEAR(first.preferred_trim, 0.6, 1.0e-12);
  EXPECT_NEAR(first.compatible_trim, 0.475, 1.0e-12);
}

TEST(DerivedTrimCandidates, CompatibleValuesSatisfyEverySharedSegment)
{
  const double margin = 0.05;
  const double shared_length = 0.9;
  const auto left = derive_two_trim_candidates(
    1.2, shared_length, false, true, 0.8, 0.02, margin, 0.01);
  const auto right = derive_two_trim_candidates(
    shared_length, 1.1, true, false, 0.8, 0.02, margin, 0.01);

  ASSERT_TRUE(left.valid);
  ASSERT_TRUE(right.valid);
  EXPECT_LE(
    left.compatible_trim + right.compatible_trim + margin,
    shared_length + 1.0e-12);
}

TEST(DerivedTrimCandidates, DynamicProgrammingUsesCompactPairInsteadOfPivots)
{
  const double margin = 0.05;
  const double shared_length = 1.0;
  const auto left = derive_two_trim_candidates(
    1.4, shared_length, false, true, 0.8, 0.02, margin, 0.01);
  const auto right = derive_two_trim_candidates(
    shared_length, 1.4, true, false, 0.8, 0.02, margin, 0.01);
  ASSERT_EQ(left.values.size(), 2U);
  ASSERT_EQ(right.values.size(), 2U);
  const std::vector<std::vector<CornerState>> states{
    {{true, 0.0, 1.0, 0U}, {false, left.preferred_trim, 0.1, 1U},
      {false, left.compatible_trim, 0.2, 2U}},
    {{true, 0.0, 1.0, 0U}, {false, right.preferred_trim, 0.1, 1U},
      {false, right.compatible_trim, 0.2, 2U}}};

  const auto result = optimize_corner_states(states, {shared_length}, {margin});

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.selected_state_indices.size(), 2U);
  EXPECT_EQ(result.selected_state_indices[0], 2U);
  EXPECT_EQ(result.selected_state_indices[1], 2U);
  EXPECT_EQ(result.pivot_count, 0U);
}

}  // namespace
}  // namespace adaptive_pivot_g2
