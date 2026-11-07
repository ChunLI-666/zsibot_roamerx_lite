// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <cstdint>
#include <geometry_msgs/msg/twist.hpp>

namespace official_lattice_rpp_adapter::goal_region {
inline constexpr double kTerminalTransStoppedVelocity=0.01;
inline constexpr double kTerminalRotStoppedVelocity=0.01;
inline bool terminalMeasuredStopped(const geometry_msgs::msg::Twist & v) {
  return std::isfinite(v.linear.x)&&std::isfinite(v.linear.y)&&std::isfinite(v.linear.z)&&
    std::isfinite(v.angular.x)&&std::isfinite(v.angular.y)&&std::isfinite(v.angular.z)&&
    std::hypot(v.linear.x,v.linear.y)<=kTerminalTransStoppedVelocity&&
    std::abs(v.angular.z)<=kTerminalRotStoppedVelocity;
}
// A permission gate only: callers retain fresh TF/Odom and exact goal geometry,
// preserve every ordered checkpoint, and sweep any correction at the actual origin.
class TerminalStopHold {
public:
  enum class Action {Track,HoldZero,CorrectYaw,Ready,Outside};
  void reset() {generation_=0;holding_=false;zero_odom_stamp_=0;}
  bool holding() const {return holding_;}
  Action observe(std::uint64_t generation,bool ordered_complete,bool in_xy,bool in_yaw,
    bool measured_stopped,std::int64_t odom_stamp) {
    if (generation!=generation_) {reset();generation_=generation;}
    if (!ordered_complete) {holding_=false;return Action::Track;}
    if (!in_xy) {holding_=false;return Action::Outside;}
    if (!holding_) {
      if (!in_yaw) {return Action::Track;}
      holding_=true;zero_odom_stamp_=odom_stamp;return Action::HoldZero;
    }
    // The entry sample precedes this client's zero command. It cannot certify
    // stopping, even on an already matching goal. A fresh advancing sample is required.
    if (!measured_stopped||odom_stamp<=zero_odom_stamp_) {return Action::HoldZero;}
    if (in_yaw) {return Action::Ready;}
    holding_=false;return Action::CorrectYaw;
  }
private:
  std::uint64_t generation_=0;
  bool holding_=false;
  std::int64_t zero_odom_stamp_=0;
};
}
