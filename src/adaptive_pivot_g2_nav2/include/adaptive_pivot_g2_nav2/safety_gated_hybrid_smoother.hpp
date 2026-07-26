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

#ifndef ADAPTIVE_PIVOT_G2_NAV2__SAFETY_GATED_HYBRID_SMOOTHER_HPP_
#define ADAPTIVE_PIVOT_G2_NAV2__SAFETY_GATED_HYBRID_SMOOTHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "adaptive_pivot_g2_nav2/adaptive_pivot_g2_smoother.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_smoother/simple_smoother.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

namespace adaptive_pivot_g2_nav2
{

/// Select between Simple and Pivot-G2 with a symmetric safety/cost/effort rule.
class SafetyGatedHybridSmoother : public nav2_core::Smoother
{
public:
  SafetyGatedHybridSmoother() = default;
  ~SafetyGatedHybridSmoother() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_subscriber) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  bool smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time) override;

private:
  std::string plugin_name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_subscriber_;
  std::unique_ptr<nav2_smoother::SimpleSmoother> simple_smoother_;
  std::unique_ptr<AdaptivePivotG2Smoother> pivot_smoother_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr
    diagnostics_publisher_;
  rclcpp::Logger logger_{rclcpp::get_logger("SafetyGatedHybridSmoother")};

  double peak_cost_deadband_{20.0};
  double relative_effort_deadband_{0.05};
  double effort_floor_{0.25};
  double path_length_tolerance_{1.0e-6};
  double pivot_rotation_characteristic_length_{0.2548};
  double evaluation_spacing_{0.025};
  double pivot_duplicate_position_tolerance_{1.0e-4};
  double minimum_pivot_angle_{0.0872664626};
  unsigned char maximum_center_cost_{252};
  std::vector<geometry_msgs::msg::Point> fallback_footprint_;
};

}  // namespace adaptive_pivot_g2_nav2

#endif  // ADAPTIVE_PIVOT_G2_NAV2__SAFETY_GATED_HYBRID_SMOOTHER_HPP_
