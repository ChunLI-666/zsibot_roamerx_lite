// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <geometry_msgs/msg/twist.hpp>
namespace official_lattice_rpp_adapter {
inline geometry_msgs::msg::Twist limitAngularCommand(geometry_msgs::msg::Twist value,double limit) {
  const auto & v=value;
  if (!std::isfinite(limit)||limit<=0.||limit>.3||
    !std::isfinite(v.linear.x)||!std::isfinite(v.linear.y)||!std::isfinite(v.linear.z)||
    !std::isfinite(v.angular.x)||!std::isfinite(v.angular.y)||!std::isfinite(v.angular.z)||
    v.linear.x<0.||v.linear.y!=0.||v.linear.z!=0.||v.angular.x!=0.||v.angular.y!=0.) {
    throw std::invalid_argument("Invalid forward planar command or experimental angular limit");
  }
  value.angular.z=std::clamp(value.angular.z,-limit,limit);return value;
}
}
