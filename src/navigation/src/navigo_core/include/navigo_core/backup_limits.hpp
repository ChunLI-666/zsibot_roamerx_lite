// Copyright 2026 Zsibot navigation contributors. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
namespace navigo_core::backup
{
constexpr double maxDistance = .3, maxSpeed = .1, maxDuration = 5.;
inline bool valid(double distance, double speed, uint64_t deadline, uint64_t now)
{
  return std::isfinite(distance) && distance >= .03 && distance <= maxDistance &&
    std::isfinite(speed) && speed > 0. && speed <= maxSpeed &&
    deadline > now && deadline-now <= static_cast<uint64_t>(maxDuration*1e9);
}
inline double progress(double x0, double y0, double yaw0, double x, double y)
{return -(x-x0)*std::cos(yaw0) - (y-y0)*std::sin(yaw0);}
inline double lateral(double x0, double y0, double yaw0, double x, double y)
{return -(x-x0)*std::sin(yaw0) + (y-y0)*std::cos(yaw0);}
inline double angle(double a) {return std::atan2(std::sin(a), std::cos(a));}
// Conservative composed delay: oldest accepted odometry + command lifetime +
// one bounded controller, relay and final watchdog period (all <= 50 ms).
inline double reactionTime(uint64_t command_ttl_ns)
{return .3 + static_cast<double>(command_ttl_ns)*1e-9 + .05 + .05 + .05;}
// Solve v*t_reaction + v^2/(2*a_stop) <= remaining - 5 mm.
// The relay admits BACKUP only with >=20 Hz and >=0.2 m/s^2 deceleration.
inline double velocity(double speed, double remaining, double reaction = .75)
{
  const double available = std::max(0., remaining-.005);
  const double cap = 2.*available/(std::sqrt(reaction*reaction+2.*available/.2)+reaction);
  return -std::min(speed, cap);
}
inline double stopDistance(double speed, double reaction = .75)
{return std::abs(speed)*reaction + speed*speed/(2.*.2) + .02;}
}
