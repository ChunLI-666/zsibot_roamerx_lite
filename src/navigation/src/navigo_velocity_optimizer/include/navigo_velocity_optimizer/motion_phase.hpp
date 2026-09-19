// Copyright 2026 Zsibot navigation contributors. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include "geometry_msgs/msg/twist.hpp"
#include "navigo_epoch_msgs/msg/epoch_command.hpp"

namespace navigo_velocity_optimizer::phase
{
using Command = navigo_epoch_msgs::msg::EpochCommand;
using Twist = geometry_msgs::msg::Twist;
inline bool translation(const Twist & v) {return v.linear.x != 0. || v.linear.y != 0.;}
inline bool rotation(const Twist & v) {return v.angular.z != 0.;}
inline bool planar(const Twist & v)
{return v.linear.z == 0. && v.angular.x == 0. && v.angular.y == 0.;}
inline bool zero(const Twist & v) {return v == Twist();}
inline bool valid(uint8_t phase, const Twist & v)
{
  for (const double value : {v.linear.x, v.linear.y, v.linear.z,
    v.angular.x, v.angular.y, v.angular.z})
  {
    if (!std::isfinite(value)) {return false;}
  }
  if (!planar(v)) {return false;}
  switch (phase) {
    case Command::PHASE_UNSPECIFIED: return true;  // Legacy controller compatibility.
    case Command::PHASE_HOLD: return zero(v);
    case Command::PHASE_TRANSLATE: return !rotation(v);
    case Command::PHASE_ROTATE: return !translation(v);
    case Command::PHASE_TRACK: return true;
    default: return false;
  }
}
struct Target {Twist velocity; bool braking;};
// This changes the target before acceleration limiting. It never truncates a
// nonzero output axis. The last braking tick outputs exact zero; only the next
// tick may start the new axis. Exact comparisons concern command state, not a
// tunable sensor tolerance. Check both current feedback and last relay state.
inline Target target(uint8_t phase, const Twist & requested,
  const Twist & current, const Twist & last,
  uint8_t last_phase = Command::PHASE_UNSPECIFIED)
{
  const bool phase_change = phase != last_phase &&
    phase != Command::PHASE_UNSPECIFIED && last_phase != Command::PHASE_UNSPECIFIED &&
    last_phase != Command::PHASE_HOLD;
  const bool incompatible = phase_change ||
    (phase == Command::PHASE_TRANSLATE && (rotation(current) || rotation(last))) ||
    (phase == Command::PHASE_ROTATE && (translation(current) || translation(last)));
  if (phase == Command::PHASE_HOLD || zero(requested) || incompatible) {
    return {Twist(), !zero(current) || !zero(last)};
  }
  return {requested, false};
}
inline bool brakingOutput(const Twist & current, const Twist & output,
  uint8_t from_phase = Command::PHASE_UNSPECIFIED)
{
  if (!valid(from_phase, output) ||
    (translation(output) && rotation(output) && from_phase != Command::PHASE_TRACK))
  {return false;}
  const double before[] = {current.linear.x, current.linear.y, current.angular.z};
  const double after[] = {output.linear.x, output.linear.y, output.angular.z};
  for (int i = 0; i < 3; ++i) {
    if (std::abs(after[i]) > std::abs(before[i]) || after[i] * before[i] < 0.) {return false;}
  }
  return true;
}
// Once braking starts, a newer target cannot cancel it before a zero tick.
class Transition
{
public:
  uint8_t currentPhase() const {return current_phase_;}
  void reset() {current_phase_ = Command::PHASE_HOLD; braking_ = false;}
  Target select(uint8_t requested_phase, const Twist & requested,
    const Twist & current, const Twist & last)
  {
    if (braking_ && (!zero(current) || !zero(last))) {return {Twist(), true};}
    auto result = target(requested_phase, requested, current, last, current_phase_);
    braking_ = result.braking;
    return result;
  }
  void update(const Twist & output, uint8_t requested_phase, bool braking)
  {
    if (zero(output)) {reset();}
    else if (!braking) {current_phase_ = requested_phase;}
  }
private:
  uint8_t current_phase_{Command::PHASE_HOLD};
  bool braking_{false};
};
}  // namespace navigo_velocity_optimizer::phase
