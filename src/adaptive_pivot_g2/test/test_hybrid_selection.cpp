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

#include "adaptive_pivot_g2/hybrid_selection.hpp"

namespace adaptive_pivot_g2
{

TEST(HybridSelection, UsesPivotWhenSimpleIsUnsafe)
{
  const auto result = select_hybrid_candidate(
    {false, 0.0, 0.0}, {true, 150.0, 2.0}, 20.0, 2.0, 0.25);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.use_pivot);
  EXPECT_EQ(result.reason, "simple_unsafe");
}

TEST(HybridSelection, UsesPivotForSafetyGainInsideEnergyBudget)
{
  const auto result = select_hybrid_candidate(
    {true, 202.0, 1.8}, {true, 177.0, 3.2}, 20.0, 2.0, 0.25);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.use_pivot);
  EXPECT_EQ(result.reason, "safety_gain_within_energy_budget");
}

TEST(HybridSelection, KeepsSimpleWhenSafetyGainIsTooSmall)
{
  const auto result = select_hybrid_candidate(
    {true, 202.0, 1.8}, {true, 190.0, 2.0}, 20.0, 2.0, 0.25);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.use_pivot);
  EXPECT_EQ(result.reason, "simple_default");
}

TEST(HybridSelection, KeepsSimpleWhenPivotExceedsEnergyBudget)
{
  const auto result = select_hybrid_candidate(
    {true, 253.0, 1.0}, {true, 150.0, 3.0}, 20.0, 2.0, 0.25);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.use_pivot);
}

TEST(HybridSelection, RejectsTwoUnsafeCandidates)
{
  const auto result = select_hybrid_candidate(
    {false, 0.0, 0.0}, {false, 0.0, 0.0}, 20.0, 2.0, 0.25);
  EXPECT_FALSE(result.valid);
}

TEST(HybridSelection, RejectsInvalidParameters)
{
  const auto result = select_hybrid_candidate(
    {true, 202.0, 1.0}, {true, 177.0, 1.5}, -1.0, 2.0, 0.25);
  EXPECT_FALSE(result.valid);
}

}  // namespace adaptive_pivot_g2
