// Copyright 2026. SPDX-License-Identifier: Apache-2.0
// Shared production predicate promoted from MPPI forward_alignment.hpp.
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include "geometry_msgs/msg/point.hpp"
#include "navigo_costmap_2d/costmap_2d.hpp"
#include "navigo_costmap_2d/cost_values.hpp"
namespace navigo_costmap_2d::sweep {
inline bool validFootprint(const std::vector<geometry_msgs::msg::Point> & footprint)
{
  if (footprint.size() < 3) {return false;}
  double area = 0.;
  for (size_t i=0; i<footprint.size(); ++i) {
    const auto & a=footprint[i]; const auto & b=footprint[(i+1)%footprint.size()];
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || std::hypot(a.x-b.x,a.y-b.y)<1e-9) {return false;}
    area += a.x*b.y-a.y*b.x;
  }
  if (std::abs(area)<1e-9) {return false;}
  for (size_t i=0; i<footprint.size(); ++i) {
    const auto & a=footprint[i]; const auto & b=footprint[(i+1)%footprint.size()];
    for (const auto & p: footprint) {
      if (((b.x-a.x)*(p.y-a.y)-(b.y-a.y)*(p.x-a.x))*std::copysign(1.,area)<-1e-9) {return false;}
    }
  }
  return true;
}
// Rasterize the full convex padded footprint, including its interior. One cell
// guard covers raster rounding and the sub-cell motion between sweep samples.
// Unknown/out-of-map cells are rejected. Caller holds the costmap mutex.
inline bool footprintFree(
  navigo_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double x, double y, double yaw)
{
  if (!validFootprint(footprint) || !std::isfinite(x) || !std::isfinite(y) ||
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
  if (!validFootprint(footprint) || !std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(yaw0) ||
    !std::isfinite(x1) || !std::isfinite(y1) || !std::isfinite(yaw1) ||
    !(max_angle_step > 0.0) || !(map.getResolution() > 0.0)) {return false;}
  double radius = 0.0;
  for (const auto & p : footprint) {radius = std::max(radius, std::hypot(p.x, p.y));}
  const double dyaw = std::atan2(std::sin(yaw1-yaw0), std::cos(yaw1-yaw0));
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
}
