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

#include <set>
#include <string>

#include "adaptive_pivot_g2_rviz/planner_catalog.hpp"
#include "gtest/gtest.h"

TEST(PlannerCatalog, ContainsFiveUniqueConfiguredPlannerIds)
{
  const std::set<std::string> unique(
    adaptive_pivot_g2_rviz::kPlannerIds.begin(),
    adaptive_pivot_g2_rviz::kPlannerIds.end());
  EXPECT_EQ(unique.size(), 5u);
  EXPECT_EQ(unique.size(), adaptive_pivot_g2_rviz::kPlannerIds.size());
}

TEST(PlannerCatalog, AcceptsOnlyExactConfiguredIds)
{
  using adaptive_pivot_g2_rviz::is_supported_planner;
  EXPECT_TRUE(is_supported_planner("NavFnAStar"));
  EXPECT_TRUE(is_supported_planner("NavFnDijkstra"));
  EXPECT_TRUE(is_supported_planner("ThetaStar"));
  EXPECT_TRUE(is_supported_planner("Smac2D"));
  EXPECT_TRUE(is_supported_planner("SmacHybrid"));
  EXPECT_FALSE(is_supported_planner(""));
  EXPECT_FALSE(is_supported_planner("Theta*"));
  EXPECT_FALSE(is_supported_planner("GridBased"));
  EXPECT_FALSE(is_supported_planner("thetastar"));
}
