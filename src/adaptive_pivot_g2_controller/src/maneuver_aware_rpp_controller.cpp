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

#include "adaptive_pivot_g2_controller/maneuver_aware_rpp_controller.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace adaptive_pivot_g2_controller
{
namespace
{

constexpr double kPositionComparisonEpsilon = 1.0e-6;

template<typename T>
T declare_and_get(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,
  const T & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
}

double normalized_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

void add_diagnostic_value(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key,
  double value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = std::to_string(value);
  status.values.push_back(std::move(item));
}

void add_diagnostic_value(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(std::move(item));
}

}  // namespace

void ManeuverAwareRPPController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::configure(
    parent, name, std::move(tf), std::move(costmap_ros));
  const auto node = parent.lock();
  if (!node) {
    throw nav2_core::ControllerException(
            "Maneuver-aware RPP lifecycle node expired during configure");
  }
  const std::string prefix = name + ".";
  duplicate_position_tolerance_ = declare_and_get<double>(
    node, prefix + "pivot_duplicate_position_tolerance", 1.0e-4);
  minimum_pivot_angle_ = declare_and_get<double>(
    node, prefix + "minimum_pivot_angle", 0.0872664626);
  pivot_position_tolerance_ = declare_and_get<double>(
    node, prefix + "pivot_position_tolerance", 0.10);
  pivot_yaw_tolerance_ = declare_and_get<double>(
    node, prefix + "pivot_yaw_tolerance", 0.015);
  stopped_linear_velocity_ = declare_and_get<double>(
    node, prefix + "pivot_stopped_linear_velocity", 0.01);
  stopped_angular_velocity_ = declare_and_get<double>(
    node, prefix + "pivot_stopped_angular_velocity", 0.02);
  pivot_angular_speed_ = declare_and_get<double>(
    node, prefix + "pivot_max_angular_speed", 0.425);
  pivot_angular_acceleration_ = declare_and_get<double>(
    node, prefix + "pivot_max_angular_acceleration", 0.80);
  pivot_heading_gain_ = declare_and_get<double>(
    node, prefix + "pivot_heading_gain", 1.8);
  control_period_ = declare_and_get<double>(
    node, prefix + "pivot_control_period", 0.10);
  terminal_position_tolerance_ = declare_and_get<double>(
    node, prefix + "terminal_position_tolerance", 0.15);
  terminal_hold_position_tolerance_ = declare_and_get<double>(
    node, prefix + "terminal_hold_position_tolerance", 0.015);
  terminal_hold_entry_margin_ = declare_and_get<double>(
    node, prefix + "terminal_hold_entry_margin", 0.005);
  terminal_release_position_tolerance_ = declare_and_get<double>(
    node, prefix + "terminal_release_position_tolerance", 0.04);
  terminal_staging_position_tolerance_ = declare_and_get<double>(
    node, prefix + "terminal_staging_position_tolerance", 0.15);
  terminal_stop_margin_ = declare_and_get<double>(
    node, prefix + "terminal_stop_margin", 0.03);
  terminal_effective_deceleration_ = declare_and_get<double>(
    node, prefix + "terminal_effective_deceleration", 0.08);
  terminal_max_linear_speed_ = declare_and_get<double>(
    node, prefix + "terminal_max_linear_speed", 0.30);
  terminal_precision_max_linear_speed_ = declare_and_get<double>(
    node, prefix + "terminal_precision_max_linear_speed", 0.05);
  terminal_position_gain_ = declare_and_get<double>(
    node, prefix + "terminal_position_gain", 2.5);
  terminal_realign_heading_tolerance_ = declare_and_get<double>(
    node, prefix + "terminal_realign_heading_tolerance", 0.05);
  adaptive_speed_enabled_ = declare_and_get<bool>(
    node, prefix + "adaptive_speed_enabled", true);
  adaptive_speed_parameters_.max_linear_speed = declare_and_get<double>(
    node, prefix + "adaptive_max_linear_speed", 0.30);
  adaptive_speed_parameters_.max_angular_speed = declare_and_get<double>(
    node, prefix + "adaptive_max_angular_speed", 0.80);
  adaptive_speed_parameters_.max_wheel_linear_speed = declare_and_get<double>(
    node, prefix + "adaptive_max_wheel_linear_speed", 0.36);
  adaptive_speed_parameters_.wheel_separation = declare_and_get<double>(
    node, prefix + "adaptive_wheel_separation", 0.2548);
  adaptive_speed_parameters_.max_lateral_acceleration = declare_and_get<double>(
    node, prefix + "adaptive_max_lateral_acceleration", 0.18);
  adaptive_speed_parameters_.max_linear_acceleration = declare_and_get<double>(
    node, prefix + "adaptive_max_linear_acceleration", 0.35);
  adaptive_speed_parameters_.max_linear_deceleration = declare_and_get<double>(
    node, prefix + "adaptive_max_linear_deceleration", 0.45);
  adaptive_speed_parameters_.max_angular_acceleration = declare_and_get<double>(
    node, prefix + "adaptive_max_angular_acceleration", 1.20);
  adaptive_speed_parameters_.max_linear_jerk = declare_and_get<double>(
    node, prefix + "adaptive_max_linear_jerk", 0.90);
  adaptive_speed_parameters_.curvature_sample_distance = declare_and_get<double>(
    node, prefix + "adaptive_curvature_sample_distance", 0.10);
  adaptive_speed_parameters_.terminal_linear_speed = declare_and_get<double>(
    node, prefix + "adaptive_terminal_linear_speed", 0.0);
  adaptive_speed_parameters_.terminal_stop_buffer = declare_and_get<double>(
    node, prefix + "adaptive_terminal_stop_buffer", 0.04);
  adaptive_speed_parameters_.projection_search_backward = declare_and_get<double>(
    node, prefix + "adaptive_projection_search_backward", 0.25);
  adaptive_speed_parameters_.projection_search_forward = declare_and_get<double>(
    node, prefix + "adaptive_projection_search_forward", 0.80);
  adaptive_speed_parameters_.projection_heading_weight = declare_and_get<double>(
    node, prefix + "adaptive_projection_heading_weight", 0.20);
  adaptive_speed_parameters_.projection_max_regression = declare_and_get<double>(
    node, prefix + "adaptive_projection_max_regression", 0.03);
  adaptive_speed_parameters_.feedback_sync_tolerance = declare_and_get<double>(
    node, prefix + "adaptive_feedback_sync_tolerance", 0.06);
  adaptive_speed_parameters_.cross_track_error_soft = declare_and_get<double>(
    node, prefix + "adaptive_cross_track_error_soft", 0.02);
  adaptive_speed_parameters_.cross_track_error_hard = declare_and_get<double>(
    node, prefix + "adaptive_cross_track_error_hard", 0.10);
  adaptive_speed_parameters_.heading_error_soft = declare_and_get<double>(
    node, prefix + "adaptive_heading_error_soft", 0.08);
  adaptive_speed_parameters_.heading_error_hard = declare_and_get<double>(
    node, prefix + "adaptive_heading_error_hard", 0.45);
  adaptive_speed_parameters_.angular_tracking_error_soft = declare_and_get<double>(
    node, prefix + "adaptive_angular_tracking_error_soft", 0.08);
  adaptive_speed_parameters_.angular_tracking_error_hard = declare_and_get<double>(
    node, prefix + "adaptive_angular_tracking_error_hard", 0.35);
  adaptive_speed_parameters_.recovery_min_linear_speed = declare_and_get<double>(
    node, prefix + "adaptive_recovery_min_linear_speed", 0.06);
  adaptive_speed_diagnostics_topic_ = declare_and_get<std::string>(
    node, prefix + "adaptive_speed_diagnostics_topic", "/research/adaptive_speed");

  const double values[]{
    duplicate_position_tolerance_, minimum_pivot_angle_, pivot_position_tolerance_,
    pivot_yaw_tolerance_, stopped_linear_velocity_, stopped_angular_velocity_,
    pivot_angular_speed_, pivot_angular_acceleration_, pivot_heading_gain_, control_period_,
    terminal_position_tolerance_, terminal_hold_position_tolerance_,
    terminal_hold_entry_margin_,
    terminal_release_position_tolerance_,
    terminal_staging_position_tolerance_,
    terminal_stop_margin_,
    terminal_effective_deceleration_,
    terminal_max_linear_speed_, terminal_precision_max_linear_speed_,
    terminal_position_gain_,
    terminal_realign_heading_tolerance_};
  if (std::any_of(
      std::begin(values), std::end(values),
      [](double value) {return !std::isfinite(value) || value < 0.0;}) ||
    pivot_position_tolerance_ <= 0.0 || pivot_yaw_tolerance_ <= 0.0 ||
    minimum_pivot_angle_ > 3.14159265358979323846 ||
    pivot_angular_speed_ <= 0.0 || pivot_angular_acceleration_ <= 0.0 ||
    pivot_heading_gain_ <= 0.0 || control_period_ <= 0.0 ||
    terminal_position_tolerance_ <= 0.0 ||
    terminal_hold_position_tolerance_ >= terminal_position_tolerance_ ||
    terminal_hold_entry_margin_ <= 0.0 ||
    terminal_hold_entry_margin_ >= terminal_hold_position_tolerance_ ||
    terminal_release_position_tolerance_ <= terminal_hold_position_tolerance_ ||
    terminal_release_position_tolerance_ >=
    terminal_staging_position_tolerance_ ||
    terminal_staging_position_tolerance_<terminal_position_tolerance_ ||
    pivot_position_tolerance_ <= terminal_release_position_tolerance_ ||
    pivot_position_tolerance_> terminal_staging_position_tolerance_ ||
    terminal_effective_deceleration_ <= 0.0 ||
    terminal_effective_deceleration_ >
    adaptive_speed_parameters_.max_linear_deceleration ||
    terminal_max_linear_speed_ > adaptive_speed_parameters_.max_linear_speed ||
    terminal_max_linear_speed_ <= 0.0 ||
    terminal_precision_max_linear_speed_ <= 0.0 ||
    terminal_precision_max_linear_speed_ > terminal_max_linear_speed_ ||
    terminal_position_gain_ <= 0.0 ||
    terminal_realign_heading_tolerance_ <= 0.0)
  {
    throw nav2_core::ControllerException("Maneuver-aware RPP pivot parameters are invalid");
  }
  try {
    validate_adaptive_speed_parameters(adaptive_speed_parameters_);
  } catch (const std::invalid_argument & error) {
    throw nav2_core::ControllerException(error.what());
  }
  if (adaptive_speed_diagnostics_topic_.empty()) {
    throw nav2_core::ControllerException("Adaptive speed diagnostics topic cannot be empty");
  }
  clock_ = node->get_clock();
  adaptive_speed_diagnostics_pub_ =
    node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    adaptive_speed_diagnostics_topic_, rclcpp::QoS(10));
  RCLCPP_INFO(
    logger_,
    "Configured maneuver-aware RPP with explicit pivots and %s adaptive speed up to %.3f m/s",
    adaptive_speed_enabled_ ? "enabled" : "disabled",
    adaptive_speed_parameters_.max_linear_speed);
}

void ManeuverAwareRPPController::cleanup()
{
  reset();
  adaptive_speed_diagnostics_pub_.reset();
  clock_.reset();
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::cleanup();
}

void ManeuverAwareRPPController::activate()
{
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::activate();
  if (adaptive_speed_diagnostics_pub_) {
    adaptive_speed_diagnostics_pub_->on_activate();
  }
}

void ManeuverAwareRPPController::deactivate()
{
  if (adaptive_speed_diagnostics_pub_) {
    adaptive_speed_diagnostics_pub_->on_deactivate();
  }
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::deactivate();
}

void ManeuverAwareRPPController::setPlan(const nav_msgs::msg::Path & path)
{
  try {
    segments_ = split_path_at_pivots(
      path, duplicate_position_tolerance_, minimum_pivot_angle_);
  } catch (const std::invalid_argument & error) {
    throw nav2_core::InvalidPath(error.what());
  }
  speed_profiles_.clear();
  if (adaptive_speed_enabled_) {
    speed_profiles_.reserve(segments_.size());
    try {
      for (const auto & segment : segments_) {
        speed_profiles_.push_back(
          build_adaptive_speed_profile(segment.path, adaptive_speed_parameters_));
      }
    } catch (const std::invalid_argument & error) {
      throw nav2_core::InvalidPath(error.what());
    }
  }
  active_segment_ = 0;
  projection_hint_segment_ = 0;
  projection_hint_distance_ = 0.0;
  rotating_at_pivot_ = false;
  reset_pose_servo_state();
  linear_speed_state_ = JerkLimitedSpeedState();
  last_control_time_.reset();
  activate_segment(active_segment_);
  const std::size_t pivot_count = static_cast<std::size_t>(std::count_if(
      segments_.begin(), segments_.end(),
      [](const ManeuverSegment & segment) {return segment.ends_with_pivot;}));
  RCLCPP_INFO(
    logger_, "Accepted maneuver path with %zu tracking segments and %zu pivots",
    segments_.size(), pivot_count);
}

void ManeuverAwareRPPController::activate_segment(std::size_t index)
{
  if (index >= segments_.size()) {
    return;
  }
  if (segments_[index].path.poses.size() < 2) {
    throw nav2_core::InvalidPath("Maneuver path contains a unit tracking segment");
  }
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::reset();
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::setPlan(
    segments_[index].path);
  projection_hint_segment_ = 0;
  projection_hint_distance_ = 0.0;
  reset_pose_servo_state();
  linear_speed_state_ = JerkLimitedSpeedState();
}

void ManeuverAwareRPPController::reset_pose_servo_state()
{
  terminal_maneuver_active_ = false;
  terminal_aligning_goal_ = false;
  terminal_driving_to_goal_ = false;
  terminal_precision_active_ = false;
  terminal_drive_direction_ = 1.0;
  terminal_phase_ = "inactive";
  precision_servo_target_.reset();
}

geometry_msgs::msg::PoseStamped ManeuverAwareRPPController::transform_for_control(
  const geometry_msgs::msg::PoseStamped & input,
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  geometry_msgs::msg::PoseStamped current = input;
  if (current.header.frame_id.empty()) {
    current.header.frame_id = segments_[active_segment_].path.header.frame_id;
  }
  current.header.stamp = robot_pose.header.stamp;
  try {
    return tf_->transform(
      current, robot_pose.header.frame_id, tf2::durationFromSec(params_->transform_tolerance));
  } catch (const tf2::TransformException & error) {
    throw nav2_core::ControllerTFError(error.what());
  }
}

std::optional<geometry_msgs::msg::PoseStamped>
ManeuverAwareRPPController::robot_pose_in_active_path_frame(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  if (active_segment_ >= segments_.size()) {
    return std::nullopt;
  }
  std::string target_frame = segments_[active_segment_].path.header.frame_id;
  if (target_frame.empty() && !segments_[active_segment_].path.poses.empty()) {
    target_frame = segments_[active_segment_].path.poses.front().header.frame_id;
  }
  if (target_frame.empty()) {
    return std::nullopt;
  }
  if (robot_pose.header.frame_id == target_frame) {
    return robot_pose;
  }
  try {
    return tf_->transform(
      robot_pose, target_frame, tf2::durationFromSec(params_->transform_tolerance));
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "Adaptive speed profile projection skipped: %s", error.what());
    return std::nullopt;
  }
}

double ManeuverAwareRPPController::measured_control_period()
{
  double period = control_period_;
  if (!clock_) {
    return period;
  }
  const rclcpp::Time now = clock_->now();
  if (last_control_time_.has_value()) {
    const double measured = (now - *last_control_time_).seconds();
    if (std::isfinite(measured) &&
      measured >= 0.5 * control_period_ && measured <= 2.5 * control_period_)
    {
      period = measured;
    }
  }
  last_control_time_ = now;
  return period;
}

void ManeuverAwareRPPController::publish_speed_telemetry(
  const SpeedTelemetry & telemetry)
{
  if (!adaptive_speed_diagnostics_pub_ ||
    !adaptive_speed_diagnostics_pub_->is_activated())
  {
    return;
  }
  diagnostic_msgs::msg::DiagnosticArray message;
  if (clock_) {
    message.header.stamp = clock_->now();
  }
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.name = "adaptive_pivot_g2_controller/speed_envelope";
  status.hardware_id = "vacuum_robot";
  status.message = telemetry.mode + ": " + telemetry.limiting_constraint;
  add_diagnostic_value(status, "mode", telemetry.mode);
  add_diagnostic_value(status, "controller_phase", telemetry.controller_phase);
  add_diagnostic_value(status, "limiting_constraint", telemetry.limiting_constraint);
  add_diagnostic_value(status, "measured_speed_mps", telemetry.measured_speed);
  add_diagnostic_value(status, "rpp_speed_mps", telemetry.rpp_speed);
  add_diagnostic_value(status, "command_speed_mps", telemetry.command_speed);
  add_diagnostic_value(status, "profile_cap_mps", telemetry.profile_cap);
  add_diagnostic_value(status, "local_path_cap_mps", telemetry.local_path_cap);
  add_diagnostic_value(status, "instantaneous_cap_mps", telemetry.instantaneous_cap);
  add_diagnostic_value(
    status, "lateral_acceleration_cap_mps", telemetry.lateral_acceleration_cap);
  add_diagnostic_value(status, "angular_speed_cap_mps", telemetry.angular_speed_cap);
  add_diagnostic_value(
    status, "angular_acceleration_cap_mps",
    telemetry.angular_acceleration_cap);
  add_diagnostic_value(status, "wheel_speed_cap_mps", telemetry.wheel_speed_cap);
  add_diagnostic_value(
    status, "tracking_error_cap_mps", telemetry.tracking_error_cap);
  add_diagnostic_value(
    status, "heading_error_cap_mps", telemetry.heading_error_cap);
  add_diagnostic_value(
    status, "angular_tracking_cap_mps", telemetry.angular_tracking_cap);
  add_diagnostic_value(status, "remaining_distance_m", telemetry.remaining_distance);
  add_diagnostic_value(status, "cross_track_error_m", telemetry.cross_track_error);
  add_diagnostic_value(status, "path_heading_error_rad", telemetry.path_heading_error);
  add_diagnostic_value(status, "path_curvature_1pm", telemetry.path_curvature);
  add_diagnostic_value(status, "command_curvature_1pm", telemetry.command_curvature);
  add_diagnostic_value(status, "command_acceleration_mps2", telemetry.acceleration);
  add_diagnostic_value(status, "command_jerk_mps3", telemetry.jerk);
  add_diagnostic_value(
    status, "safety_override", telemetry.safety_override ? "true" : "false");
  message.status.push_back(std::move(status));
  adaptive_speed_diagnostics_pub_->publish(message);
}

geometry_msgs::msg::TwistStamped ManeuverAwareRPPController::track_active_segment(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  auto command =
    nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::
    computeVelocityCommands(pose, velocity, goal_checker);
  SpeedTelemetry telemetry;
  telemetry.measured_speed = std::abs(velocity.linear.x);
  telemetry.rpp_speed = std::abs(command.twist.linear.x);
  telemetry.command_speed = telemetry.rpp_speed;
  telemetry.profile_cap = adaptive_speed_parameters_.max_linear_speed;
  telemetry.local_path_cap = adaptive_speed_parameters_.max_linear_speed;
  telemetry.instantaneous_cap = adaptive_speed_parameters_.max_linear_speed;
  telemetry.lateral_acceleration_cap = adaptive_speed_parameters_.max_linear_speed;
  telemetry.angular_speed_cap = adaptive_speed_parameters_.max_linear_speed;
  telemetry.angular_acceleration_cap = adaptive_speed_parameters_.max_linear_speed;
  telemetry.wheel_speed_cap = adaptive_speed_parameters_.max_linear_speed;
  telemetry.tracking_error_cap = adaptive_speed_parameters_.max_linear_speed;
  telemetry.heading_error_cap = adaptive_speed_parameters_.max_linear_speed;
  telemetry.angular_tracking_cap = adaptive_speed_parameters_.max_linear_speed;

  if (!adaptive_speed_enabled_ || telemetry.rpp_speed <= 1.0e-9) {
    telemetry.mode = telemetry.rpp_speed <= 1.0e-9 ? "rotate_or_stop" : "tracking";
    telemetry.limiting_constraint =
      adaptive_speed_enabled_ ? "rpp_rotation_or_stop" : "adaptive_speed_disabled";
    if (telemetry.rpp_speed <= 1.0e-9) {
      linear_speed_state_ = JerkLimitedSpeedState();
    }
    publish_speed_telemetry(telemetry);
    return command;
  }

  SpeedProfileProjection projection;
  if (active_segment_ < speed_profiles_.size()) {
    const auto profile_pose = robot_pose_in_active_path_frame(pose);
    if (profile_pose.has_value()) {
      projection = project_onto_speed_profile(
        speed_profiles_[active_segment_],
        profile_pose->pose.position.x,
        profile_pose->pose.position.y,
        tf2::getYaw(profile_pose->pose.orientation),
        projection_hint_segment_,
        projection_hint_distance_,
        adaptive_speed_parameters_.projection_search_backward,
        adaptive_speed_parameters_.projection_search_forward,
        adaptive_speed_parameters_.projection_heading_weight,
        adaptive_speed_parameters_.projection_max_regression);
      if (projection.valid) {
        projection_hint_segment_ = std::max(
          projection_hint_segment_, projection.segment_index);
        projection_hint_distance_ = std::max(
          projection_hint_distance_, projection.distance);
        telemetry.profile_cap = projection.speed_cap;
        telemetry.local_path_cap = projection.local_speed_cap;
        telemetry.remaining_distance = projection.remaining_distance;
        telemetry.cross_track_error = projection.cross_track_error;
        telemetry.path_heading_error = projection.heading_error;
        telemetry.path_curvature = projection.curvature;
      }
    }
  }

  const double command_curvature =
    command.twist.angular.z / command.twist.linear.x;
  const auto caps = instantaneous_speed_caps(
    command_curvature, adaptive_speed_parameters_);
  telemetry.command_curvature = command_curvature;
  telemetry.instantaneous_cap = caps.combined;
  telemetry.lateral_acceleration_cap = caps.lateral_acceleration;
  telemetry.angular_speed_cap = caps.angular_speed;
  telemetry.wheel_speed_cap = caps.wheel_speed;

  double target_speed = telemetry.rpp_speed;
  telemetry.limiting_constraint = "rpp_cost_curvature_or_approach";
  if (telemetry.profile_cap < target_speed) {
    target_speed = telemetry.profile_cap;
    telemetry.limiting_constraint =
      projection.valid ? projection.limiting_constraint : "path_profile";
  }
  if (caps.combined < target_speed) {
    target_speed = caps.combined;
    telemetry.limiting_constraint = caps.limiting_constraint;
  }
  if (projection.valid) {
    telemetry.tracking_error_cap = tracking_error_speed_cap(
      projection.cross_track_error,
      adaptive_speed_parameters_.cross_track_error_soft,
      adaptive_speed_parameters_.cross_track_error_hard,
      adaptive_speed_parameters_.recovery_min_linear_speed,
      adaptive_speed_parameters_.max_linear_speed);
    if (telemetry.tracking_error_cap < target_speed) {
      target_speed = telemetry.tracking_error_cap;
      telemetry.limiting_constraint = "cross_track_recovery";
    }
    telemetry.heading_error_cap = tracking_error_speed_cap(
      projection.heading_error,
      adaptive_speed_parameters_.heading_error_soft,
      adaptive_speed_parameters_.heading_error_hard,
      adaptive_speed_parameters_.recovery_min_linear_speed,
      adaptive_speed_parameters_.max_linear_speed);
    if (telemetry.heading_error_cap < target_speed) {
      target_speed = telemetry.heading_error_cap;
      telemetry.limiting_constraint = "heading_recovery";
    }
  }

  const double time_step = measured_control_period();
  if (std::abs(command.twist.angular.z) > 1.0e-9) {
    telemetry.angular_acceleration_cap = angular_acceleration_speed_cap(
      telemetry.rpp_speed, command.twist.angular.z, velocity.angular.z,
      time_step, adaptive_speed_parameters_.max_angular_acceleration);
    if (telemetry.angular_acceleration_cap < target_speed) {
      target_speed = telemetry.angular_acceleration_cap;
      telemetry.limiting_constraint = "angular_acceleration";
    }
  }
  telemetry.angular_tracking_cap = tracking_error_speed_cap(
    velocity.angular.z - command.twist.angular.z,
    adaptive_speed_parameters_.angular_tracking_error_soft,
    adaptive_speed_parameters_.angular_tracking_error_hard,
    adaptive_speed_parameters_.recovery_min_linear_speed,
    adaptive_speed_parameters_.max_linear_speed);
  if (telemetry.angular_tracking_cap < target_speed) {
    target_speed = telemetry.angular_tracking_cap;
    telemetry.limiting_constraint = "angular_tracking_recovery";
  }

  const auto shaped = update_jerk_limited_speed(
    target_speed,
    telemetry.measured_speed,
    time_step,
    adaptive_speed_parameters_,
    linear_speed_state_);
  const double scale = shaped.speed / telemetry.rpp_speed;
  command.twist.linear.x *= scale;
  command.twist.angular.z *= scale;
  telemetry.command_speed = shaped.speed;
  telemetry.acceleration = shaped.acceleration;
  telemetry.jerk = shaped.jerk;
  telemetry.safety_override = shaped.safety_override;
  if (shaped.feedback_limited && !shaped.safety_override) {
    telemetry.limiting_constraint = "actuator_feedback_tracking";
  }
  publish_speed_telemetry(telemetry);
  return command;
}

geometry_msgs::msg::TwistStamped ManeuverAwareRPPController::stop_or_rotate(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  double heading_error)
{
  geometry_msgs::msg::TwistStamped command;
  command.header = pose.header;
  if (std::abs(velocity.linear.x) > stopped_linear_velocity_) {
    return command;
  }

  double desired_angular_speed = std::clamp(
    pivot_heading_gain_ * heading_error, -pivot_angular_speed_, pivot_angular_speed_);
  if (std::abs(heading_error) <= pivot_yaw_tolerance_) {
    desired_angular_speed = 0.0;
  }
  const double speed_step = pivot_angular_acceleration_ * control_period_;
  command.twist.angular.z = std::clamp(
    desired_angular_speed,
    velocity.angular.z - speed_step,
    velocity.angular.z + speed_step);

  const double predicted_heading = tf2::getYaw(pose.pose.orientation) +
    command.twist.angular.z * control_period_;
  if (collision_checker_->inCollision(
      pose.pose.position.x, pose.pose.position.y, predicted_heading))
  {
    throw nav2_core::NoValidControl("Predicted pivot command is in collision");
  }
  return command;
}

geometry_msgs::msg::TwistStamped ManeuverAwareRPPController::terminal_pose_command(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  const geometry_msgs::msg::PoseStamped & terminal_pose)
{
  // Before the precision phase, the map goal is transformed normally so AMCL
  // corrections can improve the approach. Once stopped inside the staging
  // radius, freeze that target in the controller frame (normally odom). This
  // prevents later map->odom particle-filter corrections from moving a
  // millimetre-scale servo target while the base is aligning its final yaw.
  auto control_target =
    precision_servo_target_.has_value() ?
    *precision_servo_target_ : terminal_pose;
  if (precision_servo_target_.has_value()) {
    // Freeze only translation. The global goal yaw must still be transformed
    // with the latest map->odom estimate; otherwise AMCL yaw correction can
    // create a dead band where the odom-frame servo is satisfied but the
    // global goal checker remains just outside its yaw tolerance.
    control_target.pose.orientation = terminal_pose.pose.orientation;
  }
  const double dx = control_target.pose.position.x - pose.pose.position.x;
  const double dy = control_target.pose.position.y - pose.pose.position.y;
  const double distance = std::hypot(dx, dy);
  const double current_heading = tf2::getYaw(pose.pose.orientation);
  const double goal_heading = tf2::getYaw(control_target.pose.orientation);
  const double goal_heading_error = normalized_angle(
    goal_heading - current_heading);
  const double bearing_error = normalized_angle(std::atan2(dy, dx) - current_heading);
  const auto drive_geometry = shortest_terminal_drive(bearing_error);
  const double position_heading_error = drive_geometry.heading_error;
  const auto set_terminal_phase =
    [this, distance, bearing_error, &velocity](const std::string & phase) {
      if (phase != terminal_phase_) {
        terminal_phase_ = phase;
        RCLCPP_INFO(
          logger_,
          "Terminal phase=%s, distance=%.3f m, bearing_error=%.3f rad, "
          "linear_velocity=%.3f m/s, angular_velocity=%.3f rad/s",
          terminal_phase_.c_str(), distance, bearing_error,
          velocity.linear.x, velocity.angular.z);
      }
    };

  if (!terminal_precision_active_ &&
    distance <= terminal_staging_position_tolerance_ &&
    std::abs(velocity.linear.x) <= stopped_linear_velocity_ &&
    std::abs(velocity.angular.z) <= stopped_angular_velocity_)
  {
    terminal_precision_active_ = true;
    precision_servo_target_ = terminal_pose;
    terminal_driving_to_goal_ = false;
    set_terminal_phase("staging_settled");
  }

  const bool goal_yaw_settled =
    std::abs(goal_heading_error) <= pivot_yaw_tolerance_ &&
    std::abs(velocity.angular.z) <= stopped_angular_velocity_;
  if (terminal_aligning_goal_ &&
    distance > terminal_release_position_tolerance_ &&
    (
      goal_yaw_settled ||
      distance > terminal_staging_position_tolerance_))
  {
    terminal_aligning_goal_ = false;
    terminal_driving_to_goal_ = false;
  } else {
    if (!terminal_aligning_goal_ &&
      distance <=
      terminal_hold_position_tolerance_ + kPositionComparisonEpsilon &&
      std::abs(velocity.linear.x) <= stopped_linear_velocity_)
    {
      // Once translation is settled inside the hold region, bearing to the
      // residual millimetre-scale position error is numerically noisy and no
      // longer a useful rotation target. Latch the requested pose yaw now;
      // stop_or_rotate() safely ramps from the current angular velocity.
      terminal_aligning_goal_ = true;
      terminal_driving_to_goal_ = false;
    }
  }
  if (terminal_aligning_goal_) {
    set_terminal_phase("goal_yaw_alignment");
    return stop_or_rotate(pose, velocity, goal_heading_error);
  }

  // Align and settle before each short terminal translation.  This prevents
  // the differential base from drawing an arc out of the goal region.
  if (!terminal_driving_to_goal_) {
    terminal_drive_direction_ = drive_geometry.direction;
    const bool velocity_direction_is_safe =
      std::abs(velocity.linear.x) <= stopped_linear_velocity_ ||
      terminal_drive_direction_ * velocity.linear.x > 0.0;
    if (std::abs(position_heading_error) <= terminal_realign_heading_tolerance_ &&
      velocity_direction_is_safe &&
      std::abs(velocity.angular.z) <= stopped_angular_velocity_)
    {
      terminal_driving_to_goal_ = true;
    } else {
      set_terminal_phase("position_heading_alignment");
      return stop_or_rotate(pose, velocity, position_heading_error);
    }
  } else {
    if (drive_geometry.direction != terminal_drive_direction_) {
      terminal_driving_to_goal_ = false;
      terminal_drive_direction_ = drive_geometry.direction;
      set_terminal_phase("position_direction_change_settle");
      return stop_or_rotate(pose, velocity, 0.0);
    }
    if (std::abs(position_heading_error) >
      5.0 * terminal_realign_heading_tolerance_)
    {
      terminal_driving_to_goal_ = false;
      set_terminal_phase("position_heading_alignment");
      return stop_or_rotate(pose, velocity, position_heading_error);
    }
  }

  geometry_msgs::msg::TwistStamped command;
  set_terminal_phase(
    terminal_drive_direction_ > 0.0 ?
    "position_approach_forward" : "position_approach_reverse");
  command.header = pose.header;
  const double target_position_tolerance =
    terminal_precision_active_ ?
    terminal_hold_position_tolerance_ - terminal_hold_entry_margin_ :
    terminal_staging_position_tolerance_;
  const double available_distance = std::max(
    0.0, distance - target_position_tolerance);
  // A static inverse S-curve is not time-consistent when recomputed at every
  // control tick: it repeatedly assumes zero initial acceleration and was
  // observed to brake at only ~0.067 m/s^2 for a configured 0.08 m/s^2,
  // overshooting by 0.10--0.13 m.  The constant-deceleration envelope below
  // is closed-loop consistent (dv/dt = -a when v^2 = 2 a d).  The outer
  // command shaper still jerk-limits acceleration and the earlier S-curve
  // activation distance provides room for its acceleration ramp.
  const double braking_cap = std::sqrt(
    2.0 * terminal_effective_deceleration_ * available_distance);
  const double phase_speed_limit =
    terminal_precision_active_ ?
    terminal_precision_max_linear_speed_ :
    terminal_max_linear_speed_;
  command.twist.linear.x = terminal_drive_direction_ * std::min(
    {phase_speed_limit, braking_cap,
      terminal_position_gain_ * available_distance});
  const double desired_angular_speed = std::clamp(
    pivot_heading_gain_ * position_heading_error,
    -0.5 * pivot_angular_speed_,
    0.5 * pivot_angular_speed_);
  const double angular_step = pivot_angular_acceleration_ * control_period_;
  command.twist.angular.z = std::clamp(
    desired_angular_speed,
    velocity.angular.z - angular_step,
    velocity.angular.z + angular_step);

  const double predicted_heading =
    current_heading + command.twist.angular.z * control_period_;
  const double predicted_x =
    pose.pose.position.x +
    command.twist.linear.x * std::cos(current_heading) * control_period_;
  const double predicted_y =
    pose.pose.position.y +
    command.twist.linear.x * std::sin(current_heading) * control_period_;
  if (collision_checker_->inCollision(
      predicted_x, predicted_y, predicted_heading))
  {
    throw nav2_core::NoValidControl("Predicted terminal command is in collision");
  }
  return command;
}

double ManeuverAwareRPPController::pose_servo_activation_distance(
  const geometry_msgs::msg::Twist & velocity,
  double staging_distance) const
{
  return std::max(
    staging_distance,
    jerk_limited_stopping_distance(
      std::abs(velocity.linear.x),
      terminal_effective_deceleration_,
      adaptive_speed_parameters_.max_linear_jerk) +
    staging_distance + terminal_stop_margin_);
}

geometry_msgs::msg::TwistStamped
ManeuverAwareRPPController::shape_pose_servo_command(
  geometry_msgs::msg::TwistStamped command,
  const geometry_msgs::msg::Twist & velocity,
  double remaining_distance,
  const std::string & mode)
{
  SpeedTelemetry telemetry;
  telemetry.mode = mode;
  telemetry.controller_phase = terminal_phase_;
  telemetry.limiting_constraint = "pose_servo_s_curve";
  telemetry.measured_speed = std::abs(velocity.linear.x);
  const double raw_linear_command = command.twist.linear.x;
  if (std::abs(raw_linear_command) > 1.0e-9) {
    const auto shaped = update_jerk_limited_speed(
      std::abs(raw_linear_command),
      telemetry.measured_speed,
      measured_control_period(),
      adaptive_speed_parameters_,
      linear_speed_state_);
    command.twist.linear.x = std::copysign(shaped.speed, raw_linear_command);
    telemetry.acceleration = shaped.acceleration;
    telemetry.jerk = shaped.jerk;
    telemetry.safety_override = shaped.safety_override;
    if (shaped.feedback_limited && !shaped.safety_override) {
      telemetry.limiting_constraint = "actuator_feedback_tracking";
    }
  } else {
    linear_speed_state_ = JerkLimitedSpeedState();
  }
  telemetry.command_speed = std::abs(command.twist.linear.x);
  telemetry.remaining_distance = remaining_distance;
  publish_speed_telemetry(telemetry);
  return command;
}

geometry_msgs::msg::TwistStamped
ManeuverAwareRPPController::pivot_braking_command(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker,
  double remaining_distance)
{
  auto command = track_active_segment(pose, velocity, goal_checker);
  const double available_distance = std::max(
    0.0, remaining_distance - pivot_position_tolerance_);
  const double braking_cap = std::sqrt(
    2.0 * terminal_effective_deceleration_ * available_distance);
  const double input_speed = std::abs(command.twist.linear.x);
  const bool safety_override =
    input_speed > braking_cap && input_speed > 1.0e-9;
  if (safety_override) {
    const double scale = braking_cap / input_speed;
    command.twist.linear.x *= scale;
    command.twist.angular.z *= scale;
  }
  SpeedTelemetry telemetry;
  telemetry.mode = "pivot_braking";
  telemetry.limiting_constraint = "effective_deceleration_envelope";
  telemetry.measured_speed = std::abs(velocity.linear.x);
  telemetry.rpp_speed = input_speed;
  telemetry.command_speed = std::abs(command.twist.linear.x);
  telemetry.profile_cap = braking_cap;
  telemetry.remaining_distance = remaining_distance;
  telemetry.safety_override = safety_override;
  publish_speed_telemetry(telemetry);
  return command;
}

geometry_msgs::msg::TwistStamped ManeuverAwareRPPController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  if (segments_.empty() || active_segment_ >= segments_.size()) {
    throw nav2_core::InvalidPath("Maneuver-aware RPP has no active path segment");
  }
  const auto & segment = segments_[active_segment_];
  const bool is_terminal_segment =
    active_segment_ + 1U == segments_.size() && !segment.ends_with_pivot;
  if (is_terminal_segment) {
    const auto terminal_pose = transform_for_control(segment.path.poses.back(), pose);
    const double terminal_distance = std::hypot(
      pose.pose.position.x - terminal_pose.pose.position.x,
      pose.pose.position.y - terminal_pose.pose.position.y);
    const double activation_distance =
      pose_servo_activation_distance(
      velocity, terminal_staging_position_tolerance_);
    if (!terminal_maneuver_active_ &&
      terminal_distance <= activation_distance)
    {
      terminal_maneuver_active_ = true;
      RCLCPP_INFO(
        logger_,
        "Starting latched terminal maneuver at %.3f m "
        "(dynamic activation %.3f m)",
        terminal_distance, activation_distance);
    }
    if (terminal_maneuver_active_) {
      return shape_pose_servo_command(
        terminal_pose_command(pose, velocity, terminal_pose),
        velocity, terminal_distance, "terminal");
    }
  }
  if (!segment.ends_with_pivot) {
    return track_active_segment(pose, velocity, goal_checker);
  }

  const auto pivot_pose = transform_for_control(segment.path.poses.back(), pose);
  const double pivot_distance = std::hypot(
    pose.pose.position.x - pivot_pose.pose.position.x,
    pose.pose.position.y - pivot_pose.pose.position.y);
  const double pivot_activation_distance =
    pose_servo_activation_distance(velocity, pivot_position_tolerance_);
  if (!terminal_maneuver_active_ &&
    pivot_distance <= pivot_activation_distance)
  {
    terminal_maneuver_active_ = true;
    RCLCPP_INFO(
      logger_,
      "Starting latched pivot approach at %.3f m "
      "(dynamic activation %.3f m)",
      pivot_distance, pivot_activation_distance);
  }
  if (!rotating_at_pivot_ && !terminal_maneuver_active_) {
    return track_active_segment(pose, velocity, goal_checker);
  }
  if (!rotating_at_pivot_ &&
    pivot_distance > pivot_position_tolerance_)
  {
    return pivot_braking_command(
      pose, velocity, goal_checker, pivot_distance);
  }

  if (!rotating_at_pivot_) {
    const bool pivot_position_latched =
      terminal_aligning_goal_ &&
      pivot_distance <= terminal_release_position_tolerance_;
    const bool pivot_entry_settled =
      (
      pivot_position_latched ||
      pivot_distance <=
      terminal_hold_position_tolerance_ + kPositionComparisonEpsilon) &&
      std::abs(velocity.linear.x) <= stopped_linear_velocity_;
    if (!pivot_entry_settled) {
      return shape_pose_servo_command(
        terminal_pose_command(pose, velocity, pivot_pose),
        velocity, pivot_distance, "pivot_approach");
    }
    // The marker's incoming orientation describes the path tangent, not an
    // additional maneuver target. Once translation is settled at the marker,
    // rotate directly toward the outgoing pivot target. stop_or_rotate()
    // acceleration-limits any residual angular motion from position approach.
    rotating_at_pivot_ = true;
    RCLCPP_INFO(
      logger_, "Starting interior pivot %zu at %.4f m entry error",
      active_segment_ + 1U, pivot_distance);
  }
  const auto target_pose = transform_for_control(segment.pivot_target, pose);
  const double heading_error = normalized_angle(
    tf2::getYaw(target_pose.pose.orientation) - tf2::getYaw(pose.pose.orientation));
  if (std::abs(heading_error) <= pivot_yaw_tolerance_ &&
    std::abs(velocity.linear.x) <= stopped_linear_velocity_ &&
    std::abs(velocity.angular.z) <= stopped_angular_velocity_)
  {
    ++active_segment_;
    rotating_at_pivot_ = false;
    activate_segment(active_segment_);
    RCLCPP_INFO(logger_, "Completed interior pivot; advancing to segment %zu",
      active_segment_ + 1U);
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    SpeedTelemetry telemetry;
    telemetry.mode = "pivot_transition";
    telemetry.limiting_constraint = "pivot_complete";
    telemetry.measured_speed = std::abs(velocity.linear.x);
    publish_speed_telemetry(telemetry);
    return command;
  }
  auto command = stop_or_rotate(pose, velocity, heading_error);
  linear_speed_state_ = JerkLimitedSpeedState();
  SpeedTelemetry telemetry;
  telemetry.mode = "pivot";
  telemetry.limiting_constraint = "explicit_pivot";
  telemetry.measured_speed = std::abs(velocity.linear.x);
  telemetry.command_speed = std::abs(command.twist.linear.x);
  publish_speed_telemetry(telemetry);
  return command;
}

void ManeuverAwareRPPController::reset()
{
  segments_.clear();
  speed_profiles_.clear();
  active_segment_ = 0;
  projection_hint_segment_ = 0;
  projection_hint_distance_ = 0.0;
  rotating_at_pivot_ = false;
  reset_pose_servo_state();
  linear_speed_state_ = JerkLimitedSpeedState();
  last_control_time_.reset();
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::reset();
}

}  // namespace adaptive_pivot_g2_controller

PLUGINLIB_EXPORT_CLASS(
  adaptive_pivot_g2_controller::ManeuverAwareRPPController,
  nav2_core::Controller)
