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
#include <map>
#include <string>
#include <vector>

#include "adaptive_pivot_g2/adaptive_search.hpp"
#include "adaptive_pivot_g2/candidate_selection.hpp"
#include "adaptive_pivot_g2/line_of_sight.hpp"
#include "adaptive_pivot_g2/path_conditioning.hpp"
#include "adaptive_pivot_g2/path_optimization.hpp"
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
  bool smooth_single_branch(
    nav_msgs::msg::Path & path, const rclcpp::Duration & max_time);

  struct TransitionState
  {
    adaptive_pivot_g2::TransitionCandidate candidate;
    double curve_time{0.0};
    double common_window_time{0.0};
    double peak_cost{0.0};
    double max_abs_angular_speed{0.0};
    double objective_cost{0.0};
  };

  struct CornerDecision
  {
    adaptive_pivot_g2::Vec2 vertex;
    adaptive_pivot_g2::Vec2 incoming;
    adaptive_pivot_g2::Vec2 outgoing;
    bool pivot_safe{false};
    bool pass_through{false};
    bool use_transition{false};
    adaptive_pivot_g2::TransitionCandidate transition;
    double turn_angle{0.0};
    double pivot_time{0.0};
    double transition_time{0.0};
    double selection_score{0.0};
    double clearance_proxy{0.0};
    std::size_t candidate_count{0};
    std::size_t competitive_count{0};
    double minimum_search_radius{0.0};
    double maximum_search_radius{0.0};
    double minimum_search_trim{0.0};
    double maximum_search_trim{0.0};
    std::size_t trim_evaluation_count{0};
    std::size_t evaluation_count{0};
    std::size_t feasible_count{0};
    std::vector<TransitionState> transition_states;
    std::vector<adaptive_pivot_g2::CornerState> optimization_states;
  };

  struct PreparedPath
  {
    std::vector<adaptive_pivot_g2::Vec2> points;
    adaptive_pivot_g2::PathConditioningResult conditioning;
    adaptive_pivot_g2::LineOfSightPruningResult los_result;
    std::vector<geometry_msgs::msg::Point> safety_footprint;
    double conditioning_maximum_deviation{0.0};
    double oscillation_maximum_deviation{0.0};
  };

  PreparedPath prepare_path(
    const nav_msgs::msg::Path & path,
    const std::vector<adaptive_pivot_g2::Vec2> & raw_points,
    const std::shared_ptr<nav2_costmap_2d::Costmap2D> & costmap,
    const std::vector<geometry_msgs::msg::Point> & footprint) const;

  std::vector<double> control_fractions_for_angle(double turn_angle) const;

  void publish_diagnostics(
    const std::vector<CornerDecision> & decisions,
    const adaptive_pivot_g2::PathOptimizationResult & path_optimization,
    const std::map<std::string, std::size_t> & rejection_counts,
    const adaptive_pivot_g2::PathConditioningResult & conditioning,
    std::size_t input_point_count,
    const adaptive_pivot_g2::LineOfSightPruningResult & los_result,
    double effective_conditioning_deviation,
    double effective_oscillation_deviation,
    double effective_segment_margin,
    const std::string & fallback_status,
    const std::string & selected_stitch_rejection,
    std::size_t output_point_count,
    double runtime_seconds);

  std::vector<adaptive_pivot_g2::CandidateObjective> parameterize_common_window(
    std::vector<TransitionState> & evaluations,
    double common_trim,
    std::map<std::string, std::size_t> & rejection_counts,
    double & fastest_time,
    std::size_t & feasible_count,
    double & common_entry_speed,
    double & common_exit_speed) const;

  nav_msgs::msg::Path build_output_path(
    const std::vector<adaptive_pivot_g2::Vec2> & points,
    const std::vector<CornerDecision> & decisions,
    const std_msgs::msg::Header & header,
    const geometry_msgs::msg::Quaternion & goal_orientation,
    bool force_pivot) const;

  bool stitched_timing_is_valid(
    const std::vector<CornerDecision> & decisions,
    const nav_msgs::msg::Path & candidate_path,
    bool force_pivot,
    double effective_segment_margin,
    std::string * rejection_reason = nullptr) const;

  std::string plugin_name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_subscriber_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr diagnostics_publisher_;
  rclcpp::Logger logger_{rclcpp::get_logger("AdaptivePivotG2Smoother")};

  adaptive_pivot_g2::RobotLimits limits_;
  adaptive_pivot_g2::TransitionOptions transition_options_;
  adaptive_pivot_g2::AdaptiveSearchOptions adaptive_search_options_;
  double minimum_trim_distance_{0.02};
  double maximum_trim_distance_{0.8};
  double minimum_control_fraction_{0.08};
  double maximum_control_fraction_{0.45};
  std::size_t control_fraction_samples_{1U};
  std::size_t retained_candidates_per_corner_{9};
  double curvature_energy_scale_{1.0};
  double segment_margin_override_{0.0};
  double delta_time_selection_{0.15};
  double time_competitive_slack_{10.0};
  adaptive_pivot_g2::SelectionWeights selection_weights_;
  double corner_angle_threshold_{0.0872664626};
  double path_conditioning_max_deviation_{0.0};
  double path_conditioning_resolution_ratio_{1.5};
  double oscillation_maximum_span_{2.0};
  double oscillation_maximum_deviation_{0.0};
  double oscillation_deviation_resolution_ratio_{3.0};
  double oscillation_minimum_turn_angle_{0.20};
  std::size_t oscillation_minimum_sign_changes_{2U};
  double output_spacing_{0.05};
  unsigned char max_footprint_cost_{252};
  bool line_of_sight_pruning_{false};
  double line_of_sight_footprint_padding_{0.15};
  bool compare_los_against_no_los_{true};
  double los_selection_minimum_improvement_{0.005};
  adaptive_pivot_g2::PathQualityWeights path_quality_weights_;
  bool diagnostics_publish_enabled_{true};
  std::string last_diagnostics_message_;
  std::vector<geometry_msgs::msg::Point> fallback_footprint_;
};

}  // namespace adaptive_pivot_g2_nav2

#endif  // ADAPTIVE_PIVOT_G2_NAV2__ADAPTIVE_PIVOT_G2_SMOOTHER_HPP_
