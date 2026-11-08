// Copyright 2026. SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include "geometry_msgs/msg/twist.hpp"

namespace robot_feasible_navigation
{
constexpr double kExperimentMaxAngularVelocity = .3;

inline bool validExperimentAngularLimit(double maximum)
{
  return std::isfinite(maximum) && maximum > 0. &&
         maximum <= kExperimentMaxAngularVelocity;
}

// Validate RAW output before limiting. The caller must sweep the resulting
// command and publish that same command. No reverse/lateral/extra-axis fallback.
inline bool boundPlanarAngularVelocity(geometry_msgs::msg::Twist & command, double maximum)
{
  const auto & linear = command.linear;
  const auto & angular = command.angular;
  if (!validExperimentAngularLimit(maximum) ||
    !std::isfinite(linear.x) || !std::isfinite(linear.y) || !std::isfinite(linear.z) ||
    !std::isfinite(angular.x) || !std::isfinite(angular.y) || !std::isfinite(angular.z) ||
    linear.x < 0. || linear.y != 0. || linear.z != 0. ||
    angular.x != 0. || angular.y != 0.)
  {
    return false;
  }
  command.angular.z = std::clamp(command.angular.z, -maximum, maximum);
  return true;
}
}  // namespace robot_feasible_navigation
