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

#ifndef ADAPTIVE_PIVOT_G2__TYPES_HPP_
#define ADAPTIVE_PIVOT_G2__TYPES_HPP_

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace adaptive_pivot_g2
{

struct Vec2
{
  double x{0.0};
  double y{0.0};

  Vec2 operator+(const Vec2 & other) const {return {x + other.x, y + other.y};}
  Vec2 operator-(const Vec2 & other) const {return {x - other.x, y - other.y};}
  Vec2 operator*(double scale) const {return {x * scale, y * scale};}
  Vec2 operator/(double scale) const {return {x / scale, y / scale};}
};

inline double dot(const Vec2 & lhs, const Vec2 & rhs)
{
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

inline double cross(const Vec2 & lhs, const Vec2 & rhs)
{
  return lhs.x * rhs.y - lhs.y * rhs.x;
}

inline double norm(const Vec2 & value)
{
  return std::hypot(value.x, value.y);
}

inline double distance(const Vec2 & lhs, const Vec2 & rhs)
{
  return norm(lhs - rhs);
}

inline bool finite(const Vec2 & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y);
}

struct RobotLimits
{
  // Distance between the rolling-tread center planes, not the CAD joint shafts
  // or the complete wheel-mesh bounding-box centers.
  double wheel_separation{0.2548};
  double max_linear_speed{0.30};
  double max_angular_speed{0.80};
  double max_wheel_speed{0.36};
  double max_lateral_acceleration{0.18};
  double max_linear_acceleration{0.35};
  double max_linear_deceleration{0.45};
  double max_angular_acceleration{1.20};
};

struct CornerInput
{
  Vec2 vertex;
  Vec2 incoming_direction;
  Vec2 outgoing_direction;
  double incoming_length{0.0};
  double outgoing_length{0.0};
};

struct TransitionOptions
{
  double design_radius{0.35};
  double control_fraction{0.35};
  double max_trim_fraction{0.45};
  double sample_spacing{0.02};
  double minimum_turn_angle{1.0e-3};
  double maximum_turn_angle{2.9670597283903604};  // 170 degrees
};

struct PathSample
{
  Vec2 position;
  double heading{0.0};
  double curvature{0.0};
  double speed_limit{0.0};
};

struct TransitionCandidate
{
  bool valid{false};
  std::string rejection_reason;
  double turn_angle{0.0};
  double design_radius{0.0};
  double trim_distance{0.0};
  double path_length{0.0};
  double max_abs_curvature{0.0};
  double curvature_energy{0.0};
  std::vector<PathSample> samples;
};

struct TimedProfile
{
  bool valid{false};
  std::string rejection_reason;
  std::vector<double> linear_speed;
  std::vector<double> angular_speed;
  std::vector<double> time;
  double total_time{std::numeric_limits<double>::infinity()};
  double max_abs_angular_acceleration{0.0};
};

}  // namespace adaptive_pivot_g2

#endif  // ADAPTIVE_PIVOT_G2__TYPES_HPP_
