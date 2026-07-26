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

#include <gtest/gtest.h>

#include <limits>

#include "adaptive_pivot_g2/hybrid_selection.hpp"

namespace adaptive_pivot_g2
{
namespace
{

const HybridSelectionPolicy kPolicy{};

HybridCandidate candidate(
  bool safe, double cost, double effort, double length = 5.0)
{
  return {safe, cost, effort, length};
}

}  // namespace

TEST(HybridSelection, UsesOnlySafeCandidateInEitherDirection)
{
  const auto pivot_result = select_hybrid_candidate(
    candidate(false, 0.0, 0.0), candidate(true, 150.0, 2.0), kPolicy);
  ASSERT_TRUE(pivot_result.valid);
  EXPECT_TRUE(pivot_result.use_pivot);
  EXPECT_EQ(pivot_result.reason, "simple_unsafe");

  const auto simple_result = select_hybrid_candidate(
    candidate(true, 150.0, 2.0), candidate(false, 0.0, 0.0), kPolicy);
  ASSERT_TRUE(simple_result.valid);
  EXPECT_FALSE(simple_result.use_pivot);
  EXPECT_EQ(simple_result.reason, "pivot_unsafe");
}

TEST(HybridSelection, PeakCostDeadbandIsSymmetric)
{
  const auto pivot_result = select_hybrid_candidate(
    candidate(true, 202.0, 1.0), candidate(true, 182.0, 20.0), kPolicy);
  ASSERT_TRUE(pivot_result.valid);
  EXPECT_TRUE(pivot_result.use_pivot);
  EXPECT_EQ(pivot_result.reason, "pivot_lower_peak_cost");

  const auto simple_result = select_hybrid_candidate(
    candidate(true, 182.0, 20.0), candidate(true, 202.0, 1.0), kPolicy);
  ASSERT_TRUE(simple_result.valid);
  EXPECT_FALSE(simple_result.use_pivot);
  EXPECT_EQ(simple_result.reason, "simple_lower_peak_cost");
}

TEST(HybridSelection, ManeuverEffortComparisonIsSymmetricInsideCostDeadband)
{
  const auto pivot_result = select_hybrid_candidate(
    candidate(true, 177.0, 2.65), candidate(true, 177.0, 0.43), kPolicy);
  ASSERT_TRUE(pivot_result.valid);
  EXPECT_TRUE(pivot_result.use_pivot);
  EXPECT_EQ(pivot_result.reason, "pivot_lower_maneuver_effort");

  const auto simple_result = select_hybrid_candidate(
    candidate(true, 177.0, 0.43), candidate(true, 177.0, 2.65), kPolicy);
  ASSERT_TRUE(simple_result.valid);
  EXPECT_FALSE(simple_result.use_pivot);
  EXPECT_EQ(simple_result.reason, "simple_lower_maneuver_effort");
}

TEST(HybridSelection, ResidualCostBreaksAnEffortDeadbandTie)
{
  const HybridSelectionPolicy policy{20.0, 0.05, 0.25, 1.0e-6};
  const auto result = select_hybrid_candidate(
    candidate(true, 190.0, 1.00), candidate(true, 189.0, 1.02), policy);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.use_pivot);
  EXPECT_EQ(result.reason, "pivot_lower_residual_cost");
}

TEST(HybridSelection, PathLengthBreaksAnExactCostAndEffortTie)
{
  const auto result = select_hybrid_candidate(
    candidate(true, 177.0, 1.0, 5.0),
    candidate(true, 177.0, 1.0, 4.9), kPolicy);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.use_pivot);
  EXPECT_EQ(result.reason, "pivot_shorter_path");
}

TEST(HybridSelection, ExactMetricTieIsStable)
{
  const auto result = select_hybrid_candidate(
    candidate(true, 177.0, 1.0), candidate(true, 177.0, 1.0), kPolicy);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.use_pivot);
  EXPECT_EQ(result.reason, "metric_tie_stable_simple");
}

TEST(HybridSelection, RejectsTwoUnsafeCandidates)
{
  const auto result = select_hybrid_candidate(
    candidate(false, 0.0, 0.0), candidate(false, 0.0, 0.0), kPolicy);
  EXPECT_FALSE(result.valid);
}

TEST(HybridSelection, RejectsInvalidParametersAndCandidateMetrics)
{
  HybridSelectionPolicy invalid_policy = kPolicy;
  invalid_policy.relative_effort_deadband = 1.1;
  EXPECT_FALSE(select_hybrid_candidate(
      candidate(true, 202.0, 1.0), candidate(true, 177.0, 1.5),
      invalid_policy).valid);

  EXPECT_FALSE(select_hybrid_candidate(
      candidate(true, 202.0, std::numeric_limits<double>::infinity()),
      candidate(true, 177.0, 1.5), kPolicy).valid);
}

TEST(HybridSelection, UsesRawFallbackWhenBothSmoothedCandidatesAreUnsafe)
{
  const auto result = select_hybrid_candidate_with_raw_fallback(
    candidate(true, 180.0, 2.0),
    candidate(false, 0.0, 0.0), candidate(false, 0.0, 0.0), kPolicy);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.use_raw);
  EXPECT_FALSE(result.use_pivot);
  EXPECT_EQ(result.reason, "smoothed_candidates_unsafe_raw_fallback");
}

TEST(HybridSelection, RejectsWhenRawAndSmoothedCandidatesAreUnsafe)
{
  const auto result = select_hybrid_candidate_with_raw_fallback(
    candidate(false, 0.0, 0.0),
    candidate(false, 0.0, 0.0), candidate(false, 0.0, 0.0), kPolicy);
  EXPECT_FALSE(result.valid);
}

}  // namespace adaptive_pivot_g2
