// Copyright 2026. Licensed under the Apache License, Version 2.0.
#ifndef NAVIGO_MPPI_CONTROLLER__TOOLS__FORWARD_ALIGNMENT_HPP_
#define NAVIGO_MPPI_CONTROLLER__TOOLS__FORWARD_ALIGNMENT_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include "geometry_msgs/msg/pose.hpp"
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

// Rasterize the full convex padded footprint, including its interior. One cell
// guard covers raster rounding and the sub-cell motion between sweep samples.
// Unknown/out-of-map cells are rejected. Caller holds the costmap mutex.
inline bool footprintFree(
  navigo_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double x, double y, double yaw)
{
  if (footprint.size() < 3 || !std::isfinite(x) || !std::isfinite(y) ||
    !std::isfinite(yaw)) {return false;}
  std::vector<navigo_costmap_2d::MapLocation> polygon;
  for (const auto & p : footprint) {
    unsigned int mx, my;
    if (!map.worldToMap(x + p.x * std::cos(yaw) - p.y * std::sin(yaw),
      y + p.x * std::sin(yaw) + p.y * std::cos(yaw), mx, my)) {return false;}
    polygon.push_back({mx, my});
  }
  std::vector<navigo_costmap_2d::MapLocation> cells;
  map.convexFillCells(polygon, cells);
  if (cells.empty()) {return false;}
  for (const auto & cell : cells) {
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        const int mx = static_cast<int>(cell.x) + dx;
        const int my = static_cast<int>(cell.y) + dy;
        if (mx < 0 || my < 0 || mx >= static_cast<int>(map.getSizeInCellsX()) ||
          my >= static_cast<int>(map.getSizeInCellsY()) ||
          map.getCost(mx, my) >= navigo_costmap_2d::LETHAL_OBSTACLE) {return false;}
      }
    }
  }
  return true;
}

inline bool sweepFree(
  navigo_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double x0, double y0, double yaw0, double x1, double y1, double yaw1,
  double max_angle_step = 0.02)
{
  if (!std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(yaw0) ||
    !std::isfinite(x1) || !std::isfinite(y1) || !std::isfinite(yaw1) ||
    !(max_angle_step > 0.0) || !(map.getResolution() > 0.0)) {return false;}
  double radius = 0.0;
  for (const auto & p : footprint) {radius = std::max(radius, std::hypot(p.x, p.y));}
  const double dyaw = normalize(yaw1 - yaw0);
  const double motion = std::hypot(x1 - x0, y1 - y0) + radius * std::abs(dyaw);
  const int steps = std::max(1, static_cast<int>(std::ceil(std::max(
    motion / (0.5 * map.getResolution()), std::abs(dyaw) / max_angle_step))));
  for (int i = 0; i <= steps; ++i) {
    const double ratio = static_cast<double>(i) / steps;
    if (!footprintFree(map, footprint, x0 + ratio * (x1 - x0),
      y0 + ratio * (y1 - y0), yaw0 + ratio * dyaw)) {return false;}
  }
  return true;
}
}  // namespace mppi::forward_alignment
#endif  // NAVIGO_MPPI_CONTROLLER__TOOLS__FORWARD_ALIGNMENT_HPP_
