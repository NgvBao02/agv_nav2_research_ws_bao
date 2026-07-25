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

#ifndef ADAPTIVE_PIVOT_G2_CONTROLLER__MANEUVER_AWARE_RPP_CONTROLLER_HPP_
#define ADAPTIVE_PIVOT_G2_CONTROLLER__MANEUVER_AWARE_RPP_CONTROLLER_HPP_

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "adaptive_pivot_g2_controller/adaptive_speed_profile.hpp"
#include "adaptive_pivot_g2_controller/maneuver_path.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"

namespace adaptive_pivot_g2_controller
{

/// Regulated Pure Pursuit with explicit execution of Pivot-G2 path markers.
class ManeuverAwareRPPController
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  ManeuverAwareRPPController() = default;
  ~ManeuverAwareRPPController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void reset() override;

private:
  struct SpeedTelemetry
  {
    std::string mode{"tracking"};
    std::string controller_phase{"tracking"};
    std::string limiting_constraint{"unavailable"};
    double measured_speed{0.0};
    double rpp_speed{0.0};
    double command_speed{0.0};
    double profile_cap{0.0};
    double local_path_cap{0.0};
    double instantaneous_cap{0.0};
    double lateral_acceleration_cap{0.0};
    double angular_speed_cap{0.0};
    double angular_acceleration_cap{0.0};
    double wheel_speed_cap{0.0};
    double tracking_error_cap{0.0};
    double heading_error_cap{0.0};
    double angular_tracking_cap{0.0};
    double remaining_distance{0.0};
    double cross_track_error{0.0};
    double path_heading_error{0.0};
    double path_curvature{0.0};
    double command_curvature{0.0};
    double acceleration{0.0};
    double jerk{0.0};
    bool safety_override{false};
  };

  geometry_msgs::msg::PoseStamped transform_for_control(
    const geometry_msgs::msg::PoseStamped & input,
    const geometry_msgs::msg::PoseStamped & robot_pose) const;
  std::optional<geometry_msgs::msg::PoseStamped> robot_pose_in_active_path_frame(
    const geometry_msgs::msg::PoseStamped & robot_pose) const;
  geometry_msgs::msg::TwistStamped stop_or_rotate(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    double heading_error);
  std::optional<geometry_msgs::msg::TwistStamped> initial_alignment_command(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity);
  geometry_msgs::msg::TwistStamped terminal_pose_command(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    const geometry_msgs::msg::PoseStamped & terminal_pose);
  geometry_msgs::msg::TwistStamped shape_pose_servo_command(
    geometry_msgs::msg::TwistStamped command,
    const geometry_msgs::msg::Twist & velocity,
    double remaining_distance,
    const std::string & mode);
  geometry_msgs::msg::TwistStamped pivot_braking_command(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker,
    double remaining_distance);
  geometry_msgs::msg::TwistStamped track_active_segment(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker);
  double pose_servo_activation_distance(
    const geometry_msgs::msg::Twist & velocity,
    double staging_distance) const;
  void reset_pose_servo_state();
  double measured_control_period();
  void publish_speed_telemetry(const SpeedTelemetry & telemetry);
  void activate_segment(std::size_t index);

  std::vector<ManeuverSegment> segments_;
  std::vector<AdaptiveSpeedProfile> speed_profiles_;
  std::size_t active_segment_{0};
  std::size_t projection_hint_segment_{0};
  double projection_hint_distance_{0.0};
  bool rotating_at_pivot_{false};
  bool initial_alignment_pending_{false};
  bool initial_alignment_active_{false};
  std::optional<geometry_msgs::msg::PoseStamped> initial_alignment_target_;
  bool terminal_maneuver_active_{false};
  bool terminal_aligning_goal_{false};
  bool terminal_driving_to_goal_{false};
  bool terminal_precision_active_{false};
  double terminal_drive_direction_{1.0};
  std::string terminal_phase_{"inactive"};
  std::optional<geometry_msgs::msg::PoseStamped> precision_servo_target_;
  bool adaptive_speed_enabled_{true};
  AdaptiveSpeedParameters adaptive_speed_parameters_;
  JerkLimitedSpeedState linear_speed_state_;
  rclcpp::Clock::SharedPtr clock_;
  std::optional<rclcpp::Time> last_control_time_;
  std::string adaptive_speed_diagnostics_topic_{"/research/adaptive_speed"};
  rclcpp_lifecycle::LifecyclePublisher<
    diagnostic_msgs::msg::DiagnosticArray>::SharedPtr adaptive_speed_diagnostics_pub_;
  double duplicate_position_tolerance_{1.0e-4};
  double minimum_pivot_angle_{0.0872664626};
  double pivot_position_tolerance_{0.10};
  double pivot_yaw_tolerance_{0.015};
  double stopped_linear_velocity_{0.01};
  double stopped_angular_velocity_{0.02};
  double pivot_angular_speed_{0.425};
  double pivot_angular_acceleration_{0.80};
  double pivot_effective_angular_deceleration_{0.18};
  double pivot_heading_gain_{1.8};
  double control_period_{0.10};
  double initial_alignment_preview_distance_{0.30};
  double initial_alignment_enter_angle_{0.15};
  double initial_alignment_exit_angle_{0.035};
  double terminal_position_tolerance_{0.15};
  double terminal_hold_position_tolerance_{0.015};
  double terminal_hold_entry_margin_{0.005};
  double terminal_release_position_tolerance_{0.04};
  double terminal_staging_position_tolerance_{0.15};
  double terminal_stop_margin_{0.03};
  double terminal_effective_deceleration_{0.08};
  double terminal_max_linear_speed_{0.30};
  double terminal_precision_max_linear_speed_{0.05};
  double terminal_position_gain_{2.5};
  double terminal_realign_heading_tolerance_{0.05};
};

}  // namespace adaptive_pivot_g2_controller

#endif  // ADAPTIVE_PIVOT_G2_CONTROLLER__MANEUVER_AWARE_RPP_CONTROLLER_HPP_
