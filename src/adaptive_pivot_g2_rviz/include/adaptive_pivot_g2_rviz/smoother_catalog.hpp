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

#ifndef ADAPTIVE_PIVOT_G2_RVIZ__SMOOTHER_CATALOG_HPP_
#define ADAPTIVE_PIVOT_G2_RVIZ__SMOOTHER_CATALOG_HPP_

#include <algorithm>
#include <array>
#include <string_view>

namespace adaptive_pivot_g2_rviz
{

inline constexpr std::array<std::string_view, 5> kSmootherIds = {
  "simple",
  "savitzky_golay",
  "constrained",
  "pstmo",
  "adaptive_hybrid",
};

// RAW is a useful no-smoothing execution baseline, but it is not a Nav2
// smoother plugin and therefore intentionally stays outside kSmootherIds.
inline constexpr std::array<std::string_view, 6> kExecutionMethodIds = {
  "raw",
  "simple",
  "savitzky_golay",
  "constrained",
  "pstmo",
  "adaptive_hybrid",
};

inline bool is_supported_smoother(std::string_view smoother_id)
{
  return std::find(
    kSmootherIds.begin(), kSmootherIds.end(), smoother_id) !=
         kSmootherIds.end();
}

inline bool is_supported_execution_method(std::string_view method_id)
{
  return std::find(
    kExecutionMethodIds.begin(), kExecutionMethodIds.end(), method_id) !=
         kExecutionMethodIds.end();
}

}  // namespace adaptive_pivot_g2_rviz

#endif  // ADAPTIVE_PIVOT_G2_RVIZ__SMOOTHER_CATALOG_HPP_
