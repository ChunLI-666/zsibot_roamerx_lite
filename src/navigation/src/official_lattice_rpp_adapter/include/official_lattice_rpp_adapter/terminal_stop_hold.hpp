// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <cstdint>
#include <geometry_msgs/msg/twist.hpp>

namespace official_lattice_rpp_adapter::goal_region {
// Match the existing independent waypoint contract; measured motion and a
// commanded zero have separate limits. Never filter or overwrite odometry.
inline constexpr double kTerminalTransStoppedVelocity=0.03;
inline constexpr double kTerminalRotStoppedVelocity=0.05;
inline constexpr std::int64_t kTerminalStopDwellNs=300000000;
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
  void reset() {generation_=0;holding_=false;zero_odom_stamp_=0;observed_=false;resetDwell();}
  bool holding() const {return holding_;}
  Action observe(std::uint64_t generation,bool ordered_complete,bool in_xy,bool in_yaw,
    bool measured_stopped,std::int64_t odom_stamp,std::int64_t steady_ns) {
    if (generation!=generation_) {reset();generation_=generation;}
    if (observed_&&(steady_ns<last_observation_steady_ns_||
      steady_ns-last_observation_steady_ns_>kTerminalStopDwellNs)) {resetDwell();}
    observed_=true;last_observation_steady_ns_=steady_ns;
    if (!ordered_complete) {holding_=false;resetDwell();return Action::Track;}
    if (!in_xy) {holding_=false;resetDwell();return Action::Outside;}
    if (!holding_) {
      if (!in_yaw) {return Action::Track;}
      holding_=true;zero_odom_stamp_=odom_stamp;resetDwell();return Action::HoldZero;
    }
    // The entry sample precedes this client's zero command. It cannot certify
    // stopping, even on an already matching goal. A fresh advancing sample is required.
    if (!measured_stopped||odom_stamp<=zero_odom_stamp_) {
      resetDwell();return Action::HoldZero;
    }
    if (!in_yaw) {holding_=false;resetDwell();return Action::CorrectYaw;}
    // Every intervening command remains a collision-checked zero. Both real
    // elapsed time and advancing original odometry must cover the full dwell;
    // neither a queued epoch jump nor a frozen measurement can certify it.
    if (!dwelling_||steady_ns<dwell_steady_ns_||odom_stamp<last_odom_stamp_) {
      dwelling_=true;dwell_steady_ns_=steady_ns;dwell_odom_stamp_=odom_stamp;
    }
    last_odom_stamp_=odom_stamp;
    return steady_ns-dwell_steady_ns_>=kTerminalStopDwellNs&&
      odom_stamp-dwell_odom_stamp_>=kTerminalStopDwellNs?Action::Ready:Action::HoldZero;
  }
private:
  std::uint64_t generation_=0;
  bool holding_=false;
  std::int64_t zero_odom_stamp_=0;
  bool dwelling_=false;
  bool observed_=false;
  std::int64_t last_observation_steady_ns_=0;
  std::int64_t dwell_steady_ns_=0,dwell_odom_stamp_=0,last_odom_stamp_=0;
  void resetDwell() {dwelling_=false;dwell_steady_ns_=dwell_odom_stamp_=last_odom_stamp_=0;}
};
}
