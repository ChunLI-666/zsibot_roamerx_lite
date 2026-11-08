// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <cstdint>

namespace robot_feasible_navigation
{
// Final ORIGINAL-goal acceptance only. Segment handoffs keep their own thresholds.
class OriginalGoalStopDwell
{
public:
  static constexpr double kMeasuredLinear = .03;
  static constexpr double kMeasuredAngular = .05;
  static constexpr std::int64_t kDwellNs = 300000000;
  static constexpr std::int64_t kMaximumObservationGapNs = 300000000;
  static bool measuredStopped(double vx, double vy, double wz) {
    return std::isfinite(vx) && std::isfinite(vy) && std::isfinite(wz) &&
      std::hypot(vx,vy)<=kMeasuredLinear && std::abs(wz)<=kMeasuredAngular;
  }
  void reset() {active_=qualified_=false;last_steady_=last_odom_=since_=first_odom_=zero_epoch_=0;}
  bool active() const {return active_;}
  bool qualifying() const {return qualified_;}
  // Called only AFTER this cycle's actual-body zero sweep and zero publication.
  // The first zero establishes a baseline, never proof from a pre-zero sample.
  bool observe(std::int64_t steady_ns, std::int64_t odom_ns, std::int64_t zero_command_ns,
    bool original_geometry, bool measured_stopped, bool checked_zero)
  {
    if (!original_geometry || !checked_zero || steady_ns<=0 || odom_ns<=0 || zero_command_ns<=0) {
      reset();return false;
    }
    if (!active_) {
      active_=true;last_steady_=steady_ns;last_odom_=odom_ns;zero_epoch_=zero_command_ns;return false;
    }
    if (steady_ns<last_steady_ || odom_ns<last_odom_ ||
      steady_ns-last_steady_>kMaximumObservationGapNs) {
      reset();return false;
    }
    const bool duplicate=odom_ns==last_odom_;
    const bool source_gap=odom_ns-last_odom_>kMaximumObservationGapNs;
    last_steady_=steady_ns;last_odom_=odom_ns;
    if (!measured_stopped || duplicate || source_gap || odom_ns<=zero_epoch_) {qualified_=false;return false;}
    if (!qualified_) {qualified_=true;since_=steady_ns;first_odom_=odom_ns;return false;}
    return steady_ns-since_>=kDwellNs && odom_ns-first_odom_>=kDwellNs;
  }
private:
  bool active_=false,qualified_=false;
  std::int64_t last_steady_=0,last_odom_=0,since_=0,first_odom_=0,zero_epoch_=0;
};
}  // namespace robot_feasible_navigation
