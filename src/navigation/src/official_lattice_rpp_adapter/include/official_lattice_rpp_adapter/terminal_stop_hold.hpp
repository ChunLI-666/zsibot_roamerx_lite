// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
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
  void configureStability(bool enabled,std::int64_t phase_limit_ns) {
    if (phase_limit_ns<=0||phase_limit_ns>10000000000LL) {
      throw std::invalid_argument("terminal settling phase must be in (0,10] seconds");
    }
    stability_=enabled;phase_limit_=phase_limit_ns;reset();
  }
  void resetHold() {
    generation_=0;holding_=false;zero_odom_stamp_=zero_steady_=0;
    zero_selected_=false;observed_=false;resetDwell();
  }
  void reset() {resetHold();phase_deadline_=0;has_phase_goal_=false;phase_frame_.clear();}
  void installGoal(const std::string & frame,double x,double y,double heading) {
    const bool same=has_phase_goal_&&frame==phase_frame_&&x==phase_x_&&y==phase_y_&&
      std::abs(std::atan2(std::sin(heading-phase_yaw_),std::cos(heading-phase_yaw_)))<=1e-12;
    if (!same) {phase_deadline_=0;}
    phase_frame_=frame;phase_x_=x;phase_y_=y;phase_yaw_=heading;has_phase_goal_=true;
  }
  bool stabilityEnabled() const {return stability_;}
  std::int64_t zeroStamp() const {return zero_odom_stamp_;}
  std::int64_t zeroSteady() const {return zero_steady_;}
  std::int64_t phaseDeadline() const {return phase_deadline_;}
  bool phaseExpired(std::int64_t steady_ns) const {
    return stability_&&phase_deadline_>0&&steady_ns>=phase_deadline_;
  }
  // Called only when a filled-sweep checked zero is actually selected. A new
  // continuous hold gets a new measurement anchor, but never a new phase budget.
  void selectedZero(std::int64_t odom_stamp,std::int64_t steady_ns) {
    if (stability_&&holding_&&!zero_selected_&&odom_stamp>0&&steady_ns>0) {
      zero_selected_=true;zero_odom_stamp_=odom_stamp;zero_steady_=steady_ns;resetDwell();
      if (!phase_deadline_) {phase_deadline_=steady_ns+phase_limit_;}
    }
  }
  void nonzeroSelected() {if (stability_) {resetHold();}}
  // Called by the control thread after it observes an Odom callback epoch
  // change. The callback only revokes permission; it never touches this cursor.
  void invalidateStopContinuity() {resetDwell();}
  bool holding() const {return holding_;}
  Action observe(std::uint64_t generation,bool ordered_complete,bool in_xy,bool in_yaw,
    bool measured_stopped,std::int64_t odom_stamp,std::int64_t steady_ns,bool entry_aim=true,
    bool yaw_stable=true) {
    if (generation!=generation_) {resetHold();generation_=generation;}
    if (observed_&&(steady_ns<last_observation_steady_ns_||
      steady_ns-last_observation_steady_ns_>kTerminalStopDwellNs)) {resetDwell();}
    observed_=true;last_observation_steady_ns_=steady_ns;
    if (!ordered_complete) {holding_=zero_selected_=false;resetDwell();return Action::Track;}
    if (!in_xy) {holding_=zero_selected_=false;resetDwell();return Action::Outside;}
    if (!holding_) {
      if (!in_yaw||!entry_aim) {return Action::Track;}
      holding_=true;zero_odom_stamp_=odom_stamp;zero_steady_=0;zero_selected_=false;
      resetDwell();return Action::HoldZero;
    }
    if (stability_&&!zero_selected_) {return Action::HoldZero;}
    // The entry sample precedes this client's zero command. It cannot certify
    // stopping, even on an already matching goal. A fresh advancing sample is required.
    if (!measured_stopped||odom_stamp<=zero_odom_stamp_) {
      resetDwell();return Action::HoldZero;
    }
    if (!in_yaw) {holding_=zero_selected_=false;resetDwell();return Action::CorrectYaw;}
    // Every intervening command remains a collision-checked zero. Both real
    // elapsed time and advancing original odometry must cover the full dwell;
    // neither a queued epoch jump nor a frozen measurement can certify it.
    if (!dwelling_||steady_ns<dwell_steady_ns_||odom_stamp<last_odom_stamp_) {
      dwelling_=true;dwell_steady_ns_=steady_ns;dwell_odom_stamp_=odom_stamp;
    }
    last_odom_stamp_=odom_stamp;
    return steady_ns-dwell_steady_ns_>=kTerminalStopDwellNs&&
      odom_stamp-dwell_odom_stamp_>=kTerminalStopDwellNs&&
      (!stability_||yaw_stable)?Action::Ready:Action::HoldZero;
  }
private:
  std::uint64_t generation_=0;
  bool holding_=false;
  std::int64_t zero_odom_stamp_=0;
  bool stability_=false,zero_selected_=false,has_phase_goal_=false;
  std::int64_t zero_steady_=0,phase_deadline_=0,phase_limit_=10000000000LL;
  std::string phase_frame_;
  double phase_x_=0.,phase_y_=0.,phase_yaw_=0.;
  bool dwelling_=false;
  bool observed_=false;
  std::int64_t last_observation_steady_ns_=0;
  std::int64_t dwell_steady_ns_=0,dwell_odom_stamp_=0,last_odom_stamp_=0;
  void resetDwell() {dwelling_=false;dwell_steady_ns_=dwell_odom_stamp_=last_odom_stamp_=0;}
};
}
