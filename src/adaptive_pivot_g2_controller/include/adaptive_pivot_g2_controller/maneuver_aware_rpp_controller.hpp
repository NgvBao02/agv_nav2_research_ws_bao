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
#include <string>
#include <vector>

#include "adaptive_pivot_g2_controller/maneuver_path.hpp"
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

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void reset() override;

private:
  geometry_msgs::msg::PoseStamped transform_for_control(
    const geometry_msgs::msg::PoseStamped & input,
    const geometry_msgs::msg::PoseStamped & robot_pose) const;
  geometry_msgs::msg::TwistStamped stop_or_rotate(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    double heading_error);
  void activate_segment(std::size_t index);

  std::vector<ManeuverSegment> segments_;
  std::size_t active_segment_{0};
  bool rotating_at_pivot_{false};
  double duplicate_position_tolerance_{1.0e-4};
  double minimum_pivot_angle_{0.0872664626};
  double pivot_position_tolerance_{0.10};
  double pivot_yaw_tolerance_{0.025};
  double stopped_linear_velocity_{0.01};
  double stopped_angular_velocity_{0.02};
  double pivot_angular_speed_{0.425};
  double pivot_angular_acceleration_{0.80};
  double pivot_heading_gain_{1.8};
  double control_period_{0.10};
};

}  // namespace adaptive_pivot_g2_controller

#endif  // ADAPTIVE_PIVOT_G2_CONTROLLER__MANEUVER_AWARE_RPP_CONTROLLER_HPP_
