// Copyright 2026. SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace robot_feasible_navigation::sweep
{
using Footprint = std::vector<geometry_msgs::msg::Point>;

// Request-local broad phase only: a clear tile rectangle proves that every
// cell inspected by the unchanged SAT check is free. Occupied rectangles use
// the original check. The caller must hold the map mutex and must not change
// the map until this scope ends; rebuilding on each request prevents stale
// obstacle data. Nested scopes restore the previous thread-local index.
class ScopedObstacleIndex
{
public:
  explicit ScopedObstacleIndex(nav2_costmap_2d::Costmap2D & map)
  : map_(&map), cells_(map.getCharMap()), width_(map.getSizeInCellsX()),
    height_(map.getSizeInCellsY()), resolution_(map.getResolution()),
    origin_x_(map.getOriginX()), origin_y_(map.getOriginY()), previous_(active())
  {
    const std::size_t columns = (static_cast<std::size_t>(width_) + tile_size - 1) / tile_size;
    const std::size_t rows = (static_cast<std::size_t>(height_) + tile_size - 1) / tile_size;
    if (width_ == 0 || height_ == 0 || !cells_ ||
      columns > std::numeric_limits<std::uint32_t>::max() / rows)
    {
      return;
    }
    stride_ = columns + 1;
    prefix_.assign(stride_ * (rows + 1), 0);
    for (std::size_t ty = 0; ty < rows; ++ty) {
      std::uint32_t row_count = 0;
      for (std::size_t tx = 0; tx < columns; ++tx) {
        bool occupied = false;
        for (std::size_t y = ty * tile_size;
          y < std::min((ty + 1) * tile_size, static_cast<std::size_t>(height_)) && !occupied;
          ++y)
        {
          for (std::size_t x = tx * tile_size;
            x < std::min((tx + 1) * tile_size, static_cast<std::size_t>(width_)); ++x)
          {
            if (cells_[y * width_ + x] >= nav2_costmap_2d::LETHAL_OBSTACLE) {
              occupied = true;
              break;
            }
          }
        }
        row_count += occupied ? 1 : 0;
        prefix_[(ty + 1) * stride_ + tx + 1] =
          prefix_[ty * stride_ + tx + 1] + row_count;
      }
    }
    active() = this;
  }

  ~ScopedObstacleIndex()
  {
    if (active() == this) {active() = previous_;}
  }
  ScopedObstacleIndex(const ScopedObstacleIndex &) = delete;
  ScopedObstacleIndex & operator=(const ScopedObstacleIndex &) = delete;

  static bool activeRectangleFree(
    nav2_costmap_2d::Costmap2D & map,
    unsigned int lower_x, unsigned int lower_y,
    unsigned int upper_x, unsigned int upper_y)
  {
    const auto * index = active();
    if (!index || index->map_ != &map || index->cells_ != map.getCharMap() ||
      index->width_ != map.getSizeInCellsX() || index->height_ != map.getSizeInCellsY() ||
      index->resolution_ != map.getResolution() || index->origin_x_ != map.getOriginX() ||
      index->origin_y_ != map.getOriginY() || upper_x >= index->width_ ||
      upper_y >= index->height_ || lower_x > upper_x || lower_y > upper_y)
    {
      return false;
    }
    const std::size_t x0 = lower_x / tile_size, y0 = lower_y / tile_size;
    const std::size_t x1 = upper_x / tile_size + 1, y1 = upper_y / tile_size + 1;
    const auto & p = index->prefix_;
    const auto s = index->stride_;
    return static_cast<std::uint64_t>(p[y1 * s + x1]) + p[y0 * s + x0] ==
           static_cast<std::uint64_t>(p[y0 * s + x1]) + p[y1 * s + x0];
  }

private:
  static const ScopedObstacleIndex * & active()
  {
    static thread_local const ScopedObstacleIndex * index = nullptr;
    return index;
  }
  static constexpr std::size_t tile_size = 8;
  nav2_costmap_2d::Costmap2D * map_;
  const unsigned char * cells_;
  unsigned int width_, height_;
  double resolution_, origin_x_, origin_y_;
  const ScopedObstacleIndex * previous_;
  std::size_t stride_{0};
  std::vector<std::uint32_t> prefix_;
};

inline bool validFootprint(const Footprint & footprint)
{
  if (footprint.size() < 3) {return false;}
  double area = 0.0;
  for (std::size_t i = 0; i < footprint.size(); ++i) {
    const auto & a = footprint[i];
    const auto & b = footprint[(i + 1) % footprint.size()];
    if (!std::isfinite(a.x) || !std::isfinite(a.y) ||
      std::hypot(a.x - b.x, a.y - b.y) < 1e-9)
    {
      return false;
    }
    area += a.x * b.y - a.y * b.x;
  }
  if (!std::isfinite(area) || std::abs(area) < 1e-9) {return false;}
  for (std::size_t i = 0; i < footprint.size(); ++i) {
    const auto & a = footprint[i];
    const auto & b = footprint[(i + 1) % footprint.size()];
    for (const auto & p : footprint) {
      const double cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
      if (cross * std::copysign(1.0, area) < -1e-9) {return false;}
    }
  }
  return true;
}

// Filled convex polygon versus complete occupied/unknown grid squares. Test
// both polygon edge normals and grid axes (SAT); a cell inside the polygon is
// blocked even when none of the footprint edges touches it. One extra cell is
// retained as a conservative guard for motion between sweep samples. Caller
// owns the costmap mutex; no ROS calls, clocks, or mutable checker caches here.
inline bool footprintFree(
  nav2_costmap_2d::Costmap2D & map, const Footprint & footprint,
  double x, double y, double yaw)
{
  const double resolution = map.getResolution();
  if (!validFootprint(footprint) || !std::isfinite(x) || !std::isfinite(y) ||
    !std::isfinite(yaw) || !std::isfinite(resolution) || resolution <= 0.0 ||
    map.getSizeInCellsX() == 0 || map.getSizeInCellsY() == 0)
  {
    return false;
  }
  struct Axis {double x, y, minimum, maximum;};
  std::vector<geometry_msgs::msg::Point> polygon;
  polygon.reserve(footprint.size());
  const double cosine = std::cos(yaw), sine = std::sin(yaw);
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = min_x, max_x = -min_x, max_y = -min_x;
  for (const auto & p : footprint) {
    geometry_msgs::msg::Point q;
    q.x = x + cosine * p.x - sine * p.y;
    q.y = y + sine * p.x + cosine * p.y;
    polygon.push_back(q);
    min_x = std::min(min_x, q.x); max_x = std::max(max_x, q.x);
    min_y = std::min(min_y, q.y); max_y = std::max(max_y, q.y);
  }
  const double origin_x = map.getOriginX(), origin_y = map.getOriginY();
  const double guard = resolution;
  if (!std::isfinite(origin_x) || !std::isfinite(origin_y) ||
    min_x - guard < origin_x || min_y - guard < origin_y ||
    max_x + guard >= origin_x + map.getSizeInCellsX() * resolution ||
    max_y + guard >= origin_y + map.getSizeInCellsY() * resolution)
  {
    return false;
  }
  std::vector<Axis> axes;
  axes.reserve(polygon.size());
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    Axis axis{a.y - b.y, b.x - a.x,
      std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
    for (const auto & p : polygon) {
      const double projection = axis.x * p.x + axis.y * p.y;
      axis.minimum = std::min(axis.minimum, projection);
      axis.maximum = std::max(axis.maximum, projection);
    }
    axes.push_back(axis);
  }
  const auto lower_x = static_cast<unsigned int>(std::floor((min_x - guard - origin_x) / resolution));
  const auto lower_y = static_cast<unsigned int>(std::floor((min_y - guard - origin_y) / resolution));
  const auto upper_x = static_cast<unsigned int>(std::floor((max_x + guard - origin_x) / resolution));
  const auto upper_y = static_cast<unsigned int>(std::floor((max_y + guard - origin_y) / resolution));
  if (upper_x >= map.getSizeInCellsX() || upper_y >= map.getSizeInCellsY()) {return false;}
  if (ScopedObstacleIndex::activeRectangleFree(map, lower_x, lower_y, upper_x, upper_y)) {
    return true;
  }
  const double half_cell = 0.5 * resolution + guard;
  const auto * cells = map.getCharMap();
  for (unsigned int row = lower_y; row <= upper_y; ++row) {
    const double cy = origin_y + (row + 0.5) * resolution;
    for (unsigned int col = lower_x; col <= upper_x; ++col) {
      if (cells[static_cast<std::size_t>(row) * map.getSizeInCellsX() + col] <
        nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        continue;
      }
      const double cx = origin_x + (col + 0.5) * resolution;
      if (cx + half_cell < min_x || cx - half_cell > max_x ||
        cy + half_cell < min_y || cy - half_cell > max_y)
      {
        continue;
      }
      bool separated = false;
      for (const auto & axis : axes) {
        const double center = axis.x * cx + axis.y * cy;
        const double half_projection = half_cell * (std::abs(axis.x) + std::abs(axis.y));
        if (center + half_projection < axis.minimum - 1e-12 ||
          center - half_projection > axis.maximum + 1e-12)
        {
          separated = true;
          break;
        }
      }
      if (!separated) {return false;}
    }
  }
  return true;
}

inline bool sweepFree(
  nav2_costmap_2d::Costmap2D & map, const Footprint & footprint,
  double x0, double y0, double yaw0, double x1, double y1, double yaw1,
  double max_angle_step = 0.02)
{
  const double resolution = map.getResolution();
  if (!validFootprint(footprint) || !std::isfinite(x0) || !std::isfinite(y0) ||
    !std::isfinite(yaw0) || !std::isfinite(x1) || !std::isfinite(y1) ||
    !std::isfinite(yaw1) || !std::isfinite(max_angle_step) || max_angle_step <= 0.0 ||
    !std::isfinite(resolution) || resolution <= 0.0)
  {
    return false;
  }
  double radius = 0.0;
  for (const auto & p : footprint) {radius = std::max(radius, std::hypot(p.x, p.y));}
  const double dyaw = std::atan2(std::sin(yaw1 - yaw0), std::cos(yaw1 - yaw0));
  const double motion = std::hypot(x1 - x0, y1 - y0) + radius * std::abs(dyaw);
  const double requested_steps = std::ceil(std::max(
    motion / (0.5 * resolution), std::abs(dyaw) / max_angle_step));
  // Bound malformed requests without ever accepting an unchecked interval.
  if (!std::isfinite(requested_steps) || requested_steps > 1000000.0) {return false;}
  const auto steps = std::max<std::size_t>(1, static_cast<std::size_t>(requested_steps));
  for (std::size_t i = 0; i <= steps; ++i) {
    const double ratio = static_cast<double>(i) / steps;
    if (!footprintFree(map, footprint, x0 + ratio * (x1 - x0),
      y0 + ratio * (y1 - y0), yaw0 + ratio * dyaw))
    {
      return false;
    }
  }
  return true;
}
}  // namespace robot_feasible_navigation::sweep
