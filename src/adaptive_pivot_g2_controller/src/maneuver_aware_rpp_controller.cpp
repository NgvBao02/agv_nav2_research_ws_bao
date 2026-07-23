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
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "nav2_core/controller_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace adaptive_pivot_g2_controller
{
namespace
{

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
    node, prefix + "pivot_yaw_tolerance", 0.025);
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

  const double values[]{
    duplicate_position_tolerance_, minimum_pivot_angle_, pivot_position_tolerance_,
    pivot_yaw_tolerance_, stopped_linear_velocity_, stopped_angular_velocity_,
    pivot_angular_speed_, pivot_angular_acceleration_, pivot_heading_gain_, control_period_};
  if (std::any_of(
      std::begin(values), std::end(values),
      [](double value) {return !std::isfinite(value) || value < 0.0;}) ||
    pivot_position_tolerance_ <= 0.0 || pivot_yaw_tolerance_ <= 0.0 ||
    pivot_angular_speed_ <= 0.0 || pivot_angular_acceleration_ <= 0.0 ||
    pivot_heading_gain_ <= 0.0 || control_period_ <= 0.0)
  {
    throw nav2_core::ControllerException("Maneuver-aware RPP pivot parameters are invalid");
  }
  RCLCPP_INFO(
    logger_, "Configured maneuver-aware RPP controller with explicit interior pivots");
}

void ManeuverAwareRPPController::setPlan(const nav_msgs::msg::Path & path)
{
  try {
    segments_ = split_path_at_pivots(
      path, duplicate_position_tolerance_, minimum_pivot_angle_);
  } catch (const std::invalid_argument & error) {
    throw nav2_core::InvalidPath(error.what());
  }
  active_segment_ = 0;
  rotating_at_pivot_ = false;
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

geometry_msgs::msg::TwistStamped ManeuverAwareRPPController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  if (segments_.empty() || active_segment_ >= segments_.size()) {
    throw nav2_core::InvalidPath("Maneuver-aware RPP has no active path segment");
  }
  const auto & segment = segments_[active_segment_];
  if (!segment.ends_with_pivot) {
    return nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::
           computeVelocityCommands(pose, velocity, goal_checker);
  }

  const auto pivot_pose = transform_for_control(segment.path.poses.back(), pose);
  const double pivot_distance = std::hypot(
    pose.pose.position.x - pivot_pose.pose.position.x,
    pose.pose.position.y - pivot_pose.pose.position.y);
  if (!rotating_at_pivot_ && pivot_distance > pivot_position_tolerance_) {
    return nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::
           computeVelocityCommands(pose, velocity, goal_checker);
  }

  if (!rotating_at_pivot_) {
    rotating_at_pivot_ = true;
    RCLCPP_INFO(logger_, "Starting interior pivot %zu", active_segment_ + 1U);
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
    return command;
  }
  return stop_or_rotate(pose, velocity, heading_error);
}

void ManeuverAwareRPPController::reset()
{
  segments_.clear();
  active_segment_ = 0;
  rotating_at_pivot_ = false;
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::reset();
}

}  // namespace adaptive_pivot_g2_controller

PLUGINLIB_EXPORT_CLASS(
  adaptive_pivot_g2_controller::ManeuverAwareRPPController,
  nav2_core::Controller)
