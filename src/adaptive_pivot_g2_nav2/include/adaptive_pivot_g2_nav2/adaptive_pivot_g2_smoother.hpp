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

#ifndef ADAPTIVE_PIVOT_G2_NAV2__ADAPTIVE_PIVOT_G2_SMOOTHER_HPP_
#define ADAPTIVE_PIVOT_G2_NAV2__ADAPTIVE_PIVOT_G2_SMOOTHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "adaptive_pivot_g2/candidate_selection.hpp"
#include "adaptive_pivot_g2/types.hpp"
#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

namespace adaptive_pivot_g2_nav2
{

class AdaptivePivotG2Smoother : public nav2_core::Smoother
{
public:
  AdaptivePivotG2Smoother() = default;
  ~AdaptivePivotG2Smoother() override = default;

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
  struct CornerDecision
  {
    adaptive_pivot_g2::Vec2 vertex;
    adaptive_pivot_g2::Vec2 incoming;
    adaptive_pivot_g2::Vec2 outgoing;
    bool use_transition{false};
    adaptive_pivot_g2::TransitionCandidate transition;
    double turn_angle{0.0};
    double pivot_time{0.0};
    double transition_time{0.0};
    double selection_score{0.0};
    double clearance_proxy{0.0};
    std::size_t candidate_count{0};
    std::size_t competitive_count{0};
  };

  std::string plugin_name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_subscriber_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr diagnostics_publisher_;
  rclcpp::Logger logger_{rclcpp::get_logger("AdaptivePivotG2Smoother")};

  adaptive_pivot_g2::RobotLimits limits_;
  adaptive_pivot_g2::TransitionOptions transition_options_;
  std::vector<double> radius_candidates_;
  double delta_time_selection_{0.15};
  double time_competitive_slack_{10.0};
  adaptive_pivot_g2::SelectionWeights selection_weights_;
  double corner_angle_threshold_{0.0872664626};
  double output_spacing_{0.05};
  unsigned char max_footprint_cost_{200};
  bool line_of_sight_pruning_{false};
  std::vector<geometry_msgs::msg::Point> fallback_footprint_;
};

}  // namespace adaptive_pivot_g2_nav2

#endif  // ADAPTIVE_PIVOT_G2_NAV2__ADAPTIVE_PIVOT_G2_SMOOTHER_HPP_
