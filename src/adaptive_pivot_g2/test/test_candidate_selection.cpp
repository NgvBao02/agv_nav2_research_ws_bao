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

#include "adaptive_pivot_g2/candidate_selection.hpp"

namespace adaptive_pivot_g2
{
namespace
{

TEST(CandidateSelection, ChoosesSmoothCandidateInsideTimeGate)
{
  const std::vector<CandidateObjective> candidates{
    {0U, 1.00, 0.10, 0.80, 8.0},
    {1U, 1.10, 0.30, 0.40, 2.0},
    {2U, 1.25, 0.50, 0.20, 1.0}};

  const CandidateSelection selected = select_competitive_candidate(
    candidates, 0.20, SelectionWeights{});

  ASSERT_TRUE(selected.valid);
  EXPECT_EQ(selected.candidate_index, 1U);
  EXPECT_EQ(selected.competitive_count, 2U);
  EXPECT_DOUBLE_EQ(selected.fastest_time, 1.0);
  EXPECT_DOUBLE_EQ(selected.selected_time, 1.1);
  EXPECT_NEAR(selected.selected_score, 1.0, 1.0e-12);
}

TEST(CandidateSelection, ZeroSlackSelectsTheFastestCandidate)
{
  const std::vector<CandidateObjective> candidates{
    {4U, 2.0, 0.10, 0.80, 8.0},
    {7U, 2.1, 0.50, 0.20, 1.0}};

  const CandidateSelection selected = select_competitive_candidate(
    candidates, 0.0, SelectionWeights{});

  ASSERT_TRUE(selected.valid);
  EXPECT_EQ(selected.candidate_index, 4U);
  EXPECT_EQ(selected.competitive_count, 1U);
}

TEST(CandidateSelection, IgnoresInvalidObjectives)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<CandidateObjective> candidates{
    {0U, nan, 0.2, 0.3, 1.0},
    {1U, 1.0, 0.2, -0.3, 1.0},
    {2U, 1.1, 0.2, 0.3, 1.0}};

  const CandidateSelection selected = select_competitive_candidate(
    candidates, 0.2, SelectionWeights{});

  ASSERT_TRUE(selected.valid);
  EXPECT_EQ(selected.candidate_index, 2U);
  EXPECT_EQ(selected.competitive_count, 1U);
}

TEST(CandidateSelection, RejectsInvalidWeights)
{
  SelectionWeights weights;
  weights.clearance = 0.0;
  weights.angular_speed = 0.0;
  weights.curvature_energy = 0.0;

  const CandidateSelection selected = select_competitive_candidate(
    {{0U, 1.0, 0.2, 0.3, 1.0}}, 0.2, weights);

  EXPECT_FALSE(selected.valid);
}

TEST(CandidateSelection, StableCostDoesNotDependOnCandidateSet)
{
  const double first = stable_candidate_cost(
    126.0, 0.425, 1.0, 0.85, 1.0, SelectionWeights{});
  const double repeated = stable_candidate_cost(
    126.0, 0.425, 1.0, 0.85, 1.0, SelectionWeights{});

  EXPECT_DOUBLE_EQ(first, repeated);
  EXPECT_NEAR(first, 0.5, 1.0e-12);
}

TEST(CandidateSelection, StableCostRejectsInvalidEnergyScale)
{
  const double cost = stable_candidate_cost(
    100.0, 0.3, 1.0, 0.85, 0.0, SelectionWeights{});
  EXPECT_FALSE(std::isfinite(cost));
}

TEST(CandidateSelection, StableCostClampsInscribedInflationCost)
{
  SelectionWeights weights;
  weights.clearance = 1.0;
  weights.angular_speed = 0.0;
  weights.curvature_energy = 0.0;

  const double cost = stable_candidate_cost(
    253.0, 0.0, 0.0, 0.85, 1.0, weights);

  EXPECT_DOUBLE_EQ(cost, 1.0);
}

}  // namespace
}  // namespace adaptive_pivot_g2
