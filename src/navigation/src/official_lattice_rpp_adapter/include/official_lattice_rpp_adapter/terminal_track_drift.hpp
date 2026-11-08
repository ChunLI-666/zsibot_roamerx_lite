// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace official_lattice_rpp_adapter::goal_region {
// Observed execution permission only. This never certifies collision clearance,
// consumes a heading, modifies the original goal, or grants final completion.
class TerminalTrackDriftGuard {
public:
  struct Goal {double x=0.,y=0.,yaw=0.;std::string frame;};
  enum class Reentry {Proceed,HoldZero,FollowPath,Reject};
  explicit TerminalTrackDriftGuard(double limit=0.) {configure(limit);}
  void configure(double limit) {
    if (!std::isfinite(limit)||limit<0.||limit>0.05) {
      throw std::invalid_argument("terminal Track drift limit must be in [0,.05]");
    }
    limit_=limit;clear();
  }
  bool enabled() const {return limit_>0.;}
  double limit() const {return limit_;}
  void clear() {
    has_goal_=admitted_=reentry_=false;generation_=0;goal_=Goal{};
    pivot_x_=pivot_y_=admitted_tolerance_=0.;resetReentry();
  }
  // Preserve a same-goal reentry obligation across reset/cancel/deactivation.
  // Otherwise a new plan could silently grant another five centimetres.
  void invalidate() {
    if (enabled()&&admitted_) {reentry_=true;}
    admitted_=false;generation_=0;resetReentry();
  }
  void setPlan(const Goal & goal,std::uint64_t generation) {
    if (!valid(goal)||generation==0) {throw std::invalid_argument("invalid terminal Track goal/generation");}
    const bool same=has_goal_&&sameGoal(goal_,goal);
    reentry_=enabled()&&same&&(admitted_||reentry_);
    goal_=goal;has_goal_=true;generation_=generation;admitted_=false;
    resetReentry();
  }
  bool reentryRequired() const {return reentry_;}
  // Call only AFTER a checked zero has actually been selected for return.
  void checkedZero(std::int64_t odom_stamp,std::int64_t steady_ns) {
    if (reentry_&&!zero_&&odom_stamp>0&&steady_ns>0) {
      zero_=true;zero_stamp_=odom_stamp;zero_steady_=steady_ns;
    }
  }
  Reentry observeReentry(double x,double y,double tolerance,bool measured_stopped,
    std::int64_t odom_stamp,std::int64_t steady_ns) {
    if (!reentry_) {return Reentry::Proceed;}
    if (!coordinates(x,y,tolerance)||tolerance!=admitted_tolerance_||
      odom_stamp<=0||steady_ns<=0) {
      resetDwell();last_observation_=0;return Reentry::Reject;
    }
    if (!inside(x,y,tolerance)) {
      // The old pivot grants no new terminal permission. A newly validated
      // ordinary path may still bring the body back to the original region.
      resetReentry();return Reentry::FollowPath;
    }
    if (!zero_) {return Reentry::HoldZero;}
    if (last_observation_>0&&(steady_ns<last_observation_||
      steady_ns-last_observation_>kReentryDwellNs)) {resetDwell();}
    last_observation_=steady_ns;
    if (!measured_stopped||odom_stamp<=zero_stamp_||steady_ns<=zero_steady_) {
      resetDwell();return Reentry::HoldZero;
    }
    if (!dwelling_||steady_ns<dwell_steady_||odom_stamp<last_stamp_) {
      dwelling_=true;dwell_steady_=steady_ns;dwell_stamp_=odom_stamp;
    }
    last_stamp_=odom_stamp;
    if (steady_ns-dwell_steady_<kReentryDwellNs||odom_stamp-dwell_stamp_<kReentryDwellNs) {
      return Reentry::HoldZero;
    }
    reentry_=false;resetReentry();return Reentry::Proceed;
  }
  bool admit(std::uint64_t generation,double x,double y,double tolerance) {
    if (!has_goal_||generation!=generation_||!inside(x,y,tolerance)) {return false;}
    if (!enabled()) {return true;}
    if (reentry_||admitted_) {return false;}
    pivot_x_=x;pivot_y_=y;admitted_tolerance_=tolerance;admitted_=true;return true;
  }
  bool permits(std::uint64_t generation,double x,double y,double tolerance,
    bool original_mandatory_pending,bool corrective_rotation) const {
    if (!has_goal_||!coordinates(x,y,tolerance)) {return false;}
    const double original_error=std::hypot(x-goal_.x,y-goal_.y);
    if (!enabled()) {return original_error<=tolerance;}
    if (!admitted_||reentry_||generation!=generation_||tolerance!=admitted_tolerance_) {return false;}
    // Preserve every position already permitted by the original goal disc.
    // The fixed-pivot budget restricts only the additional outside-disc rescue.
    if (original_error<=tolerance) {return true;}
    return original_mandatory_pending&&!corrective_rotation&&
      std::hypot(x-pivot_x_,y-pivot_y_)<=limit_&&original_error<=tolerance+limit_;
  }
private:
  inline static constexpr std::int64_t kReentryDwellNs=300000000;
  double limit_=0.,pivot_x_=0.,pivot_y_=0.,admitted_tolerance_=0.;
  Goal goal_;std::uint64_t generation_=0;
  bool has_goal_=false,admitted_=false,reentry_=false,zero_=false,dwelling_=false;
  std::int64_t zero_stamp_=0,zero_steady_=0,last_observation_=0;
  std::int64_t dwell_stamp_=0,dwell_steady_=0,last_stamp_=0;
  static bool valid(const Goal & g) {
    return !g.frame.empty()&&std::isfinite(g.x)&&std::isfinite(g.y)&&std::isfinite(g.yaw);
  }
  static bool sameGoal(const Goal & a,const Goal & b) {
    return a.frame==b.frame&&a.x==b.x&&a.y==b.y&&
      std::abs(std::atan2(std::sin(a.yaw-b.yaw),std::cos(a.yaw-b.yaw)))<=1e-12;
  }
  static bool coordinates(double x,double y,double tolerance) {
    return std::isfinite(x)&&std::isfinite(y)&&std::isfinite(tolerance)&&tolerance>0.;
  }
  bool inside(double x,double y,double tolerance) const {
    return has_goal_&&coordinates(x,y,tolerance)&&std::hypot(x-goal_.x,y-goal_.y)<=tolerance;
  }
  void resetDwell() {dwelling_=false;dwell_stamp_=dwell_steady_=last_stamp_=0;}
  void resetReentry() {zero_=false;zero_stamp_=zero_steady_=last_observation_=0;resetDwell();}
};
}
