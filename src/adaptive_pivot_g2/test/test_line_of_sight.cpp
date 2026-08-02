// Copyright 2026 PSTMO Research Team
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

#include <cmath>
#include <vector>

#include "adaptive_pivot_g2/line_of_sight.hpp"

namespace adaptive_pivot_g2
{
namespace
{

bool same_point(const Vec2 & lhs, const Vec2 & rhs)
{
  return distance(lhs, rhs) < 1.0e-12;
}

TEST(LineOfSight, ChoosesFarthestSafeEndpoint)
{
  const std::vector<Vec2> input{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}};
  const auto result = prune_line_of_sight(
    input,
    [](const Vec2 &, const Vec2 &) {return true;},
    [](const Vec2 &, const Vec2 &, const Vec2 &) {return true;});

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.fallback_to_input);
  ASSERT_EQ(result.points.size(), 2U);
  EXPECT_TRUE(same_point(result.points.front(), input.front()));
  EXPECT_TRUE(same_point(result.points.back(), input.back()));
  EXPECT_EQ(result.accepted_shortcuts, 1U);
}

TEST(LineOfSight, RejectsUnsafeShortcutAndKeepsSafeIntermediatePoint)
{
  const std::vector<Vec2> input{{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}};
  const auto result = prune_line_of_sight(
    input,
    [](const Vec2 & start, const Vec2 & finish) {
      return std::abs(finish.x - start.x) < 1.5;
    },
    [](const Vec2 &, const Vec2 &, const Vec2 &) {return true;});

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.fallback_to_input);
  ASSERT_EQ(result.points.size(), input.size());
  EXPECT_EQ(result.safety_rejections, 1U);
  EXPECT_EQ(result.accepted_shortcuts, 0U);
}

TEST(LineOfSight, FallsBackInsteadOfAcceptingUnsafeAdjacentEdge)
{
  const std::vector<Vec2> input{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  const auto result = prune_line_of_sight(
    input,
    [](const Vec2 & start, const Vec2 & finish) {
      (void)finish;
      return !same_point(start, {0.0, 0.0});
    },
    [](const Vec2 &, const Vec2 &, const Vec2 &) {return true;});

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.fallback_to_input);
  ASSERT_EQ(result.points.size(), input.size());
  for (std::size_t index = 0U; index < input.size(); ++index) {
    EXPECT_TRUE(same_point(result.points[index], input[index]));
  }
  EXPECT_NE(result.fallback_reason.find("index 0"), std::string::npos);
}

TEST(LineOfSight, AppliesJunctionSafetyToRetainedTurns)
{
  const std::vector<Vec2> input{
    {0.0, 0.0}, {1.0, 0.0}, {2.0, 1.0}, {3.0, 1.0}, {4.0, 0.0}};
  const auto result = prune_line_of_sight(
    input,
    [](const Vec2 & start, const Vec2 & finish) {
      return finish.x - start.x <= 2.1;
    },
    [](const Vec2 &, const Vec2 &, const Vec2 & next) {
      return next.x < 4.0;
    });

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.fallback_to_input);
  EXPECT_GT(result.safety_rejections, 0U);
}

}  // namespace
}  // namespace adaptive_pivot_g2
