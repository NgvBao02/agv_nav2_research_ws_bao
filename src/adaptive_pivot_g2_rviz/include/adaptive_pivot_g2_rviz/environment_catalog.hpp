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

#ifndef ADAPTIVE_PIVOT_G2_RVIZ__ENVIRONMENT_CATALOG_HPP_
#define ADAPTIVE_PIVOT_G2_RVIZ__ENVIRONMENT_CATALOG_HPP_

#include <algorithm>
#include <array>
#include <string_view>

namespace adaptive_pivot_g2_rviz
{

inline constexpr std::array<std::string_view, 7> kEnvironmentIds = {
  "research_warehouse",
  "warehouse_long_aisles",
  "warehouse_cross_aisles",
  "warehouse_dispatch",
  "narrow_aisles",
  "office_maze",
  "open_arena",
};

inline bool is_supported_environment(std::string_view environment_id)
{
  return std::find(
    kEnvironmentIds.begin(), kEnvironmentIds.end(), environment_id) !=
         kEnvironmentIds.end();
}

}  // namespace adaptive_pivot_g2_rviz

#endif  // ADAPTIVE_PIVOT_G2_RVIZ__ENVIRONMENT_CATALOG_HPP_
