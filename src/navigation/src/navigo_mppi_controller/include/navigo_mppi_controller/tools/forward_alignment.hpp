// Copyright 2026. Licensed under the Apache License, Version 2.0.
#ifndef NAVIGO_MPPI_CONTROLLER__TOOLS__FORWARD_ALIGNMENT_HPP_
#define NAVIGO_MPPI_CONTROLLER__TOOLS__FORWARD_ALIGNMENT_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include "navigo_costmap_2d/footprint_sweep.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navigo_costmap_2d/costmap_2d.hpp"
#include "navigo_costmap_2d/cost_values.hpp"

namespace mppi::forward_alignment
{
inline double normalize(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}

enum class Mode {TRACK, ALIGN, FINAL};

// Hysteresis uses the goal checker's outer XY bound; final alignment must never
// hold translation outside the position tolerance accepted by that checker.
inline Mode nextMode(
  Mode previous, double distance, double heading_error, double xy_tolerance,
  double xy_hysteresis, double entry_angle, double exit_angle)
{
  if (distance <= std::max(0.0, xy_tolerance - xy_hysteresis) ||
    (previous == Mode::FINAL && distance <= xy_tolerance))
  {
    return Mode::FINAL;
  }
  if (std::abs(heading_error) > (previous == Mode::ALIGN ? exit_angle : entry_angle)) {
    return Mode::ALIGN;
  }
  return Mode::TRACK;
}

// Follow ordered coordinates, not pose orientations (the existing smoother can
// emit orientations opposite to the ordered path tangent).
inline double pathHeading(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::Pose & pose,
  double lookahead)
{
  if (path.poses.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & p = path.poses[i].pose.position;
    const double d = std::hypot(p.x - pose.position.x, p.y - pose.position.y);
    if (d < best) {best = d; nearest = i;}
  }
  size_t target = nearest;
  double length = 0.0;
  while (target + 1 < path.poses.size() && length < lookahead) {
    const auto & a = path.poses[target].pose.position;
    const auto & b = path.poses[++target].pose.position;
    length += std::hypot(b.x - a.x, b.y - a.y);
  }
  const auto & p = path.poses[target].pose.position;
  if (std::hypot(p.x - pose.position.x, p.y - pose.position.y) < 1e-6) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::atan2(p.y - pose.position.y, p.x - pose.position.x);
}

using navigo_costmap_2d::sweep::footprintFree;
using navigo_costmap_2d::sweep::sweepFree;
}  // namespace mppi::forward_alignment
#endif  // NAVIGO_MPPI_CONTROLLER__TOOLS__FORWARD_ALIGNMENT_HPP_
