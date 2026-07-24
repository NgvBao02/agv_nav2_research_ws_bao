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

#include "adaptive_pivot_g2_rviz/environment_catalog.hpp"
#include "adaptive_pivot_g2_rviz/planner_catalog.hpp"
#include "adaptive_pivot_g2_rviz/smoother_catalog.hpp"
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

TEST(EnvironmentCatalog, ContainsSevenUniqueConfiguredEnvironmentIds)
{
  const std::set<std::string> unique(
    adaptive_pivot_g2_rviz::kEnvironmentIds.begin(),
    adaptive_pivot_g2_rviz::kEnvironmentIds.end());
  EXPECT_EQ(unique.size(), 7u);
  EXPECT_EQ(
    unique.size(), adaptive_pivot_g2_rviz::kEnvironmentIds.size());
}

TEST(EnvironmentCatalog, AcceptsOnlyExactConfiguredIds)
{
  using adaptive_pivot_g2_rviz::is_supported_environment;
  EXPECT_TRUE(is_supported_environment("research_warehouse"));
  EXPECT_TRUE(is_supported_environment("warehouse_long_aisles"));
  EXPECT_TRUE(is_supported_environment("warehouse_cross_aisles"));
  EXPECT_TRUE(is_supported_environment("warehouse_dispatch"));
  EXPECT_TRUE(is_supported_environment("narrow_aisles"));
  EXPECT_TRUE(is_supported_environment("office_maze"));
  EXPECT_TRUE(is_supported_environment("open_arena"));
  EXPECT_FALSE(is_supported_environment(""));
  EXPECT_FALSE(is_supported_environment("warehouse"));
  EXPECT_FALSE(is_supported_environment("../research_warehouse"));
  EXPECT_FALSE(is_supported_environment("Research_Warehouse"));
}

TEST(SmootherCatalog, ContainsSevenUniqueConfiguredSmootherIds)
{
  const std::set<std::string> unique(
    adaptive_pivot_g2_rviz::kSmootherIds.begin(),
    adaptive_pivot_g2_rviz::kSmootherIds.end());
  EXPECT_EQ(unique.size(), 7u);
  EXPECT_EQ(unique.size(), adaptive_pivot_g2_rviz::kSmootherIds.size());
}

TEST(SmootherCatalog, AcceptsOnlyExactConfiguredIds)
{
  using adaptive_pivot_g2_rviz::is_supported_smoother;
  EXPECT_TRUE(is_supported_smoother("simple"));
  EXPECT_TRUE(is_supported_smoother("savitzky_golay"));
  EXPECT_TRUE(is_supported_smoother("constrained"));
  EXPECT_TRUE(is_supported_smoother("pivot_g2_fixed"));
  EXPECT_TRUE(is_supported_smoother("pivot_g2"));
  EXPECT_TRUE(is_supported_smoother("adaptive_hybrid_fixed"));
  EXPECT_TRUE(is_supported_smoother("adaptive_hybrid"));
  EXPECT_FALSE(is_supported_smoother(""));
  EXPECT_FALSE(is_supported_smoother("pivot-g2"));
  EXPECT_FALSE(is_supported_smoother("../simple"));
}
