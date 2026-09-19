// Independent convex polygon/cell SAT oracle for the test plant.
// It does not call the production controller's footprint or sweep predicates.
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <navigo_costmap_2d/costmap_2d.hpp>

namespace feedback_test {
using Polygon = std::vector<geometry_msgs::msg::Point>;

inline void validateConvex(const Polygon & polygon) {
  if (polygon.size() < 3) throw std::runtime_error("Footprint needs at least three vertices");
  double twice_area = 0;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    if (!std::isfinite(a.x) || !std::isfinite(a.y) ||
        std::hypot(b.x - a.x, b.y - a.y) < 1e-9) {
      throw std::runtime_error("Nonfinite or duplicate footprint vertex");
    }
    twice_area += a.x * b.y - a.y * b.x;
  }
  if (std::abs(twice_area) < 1e-9) throw std::runtime_error("Degenerate footprint");
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    for (const auto & point : polygon) {
      const double cross = (b.x-a.x)*(point.y-a.y) - (b.y-a.y)*(point.x-a.x);
      if (cross * std::copysign(1.0, twice_area) < -1e-9) {
        throw std::runtime_error("Footprint must be an ordered convex polygon");
      }
    }
  }
}

inline bool collides(navigo_costmap_2d::Costmap2D * map,
                     const Polygon & footprint, double x, double y, double yaw, std::string * reason=nullptr) {
  if (reason) *reason="free";
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
    if (reason) *reason="nonfinite";
    return true;
  }
  const double c = std::cos(yaw), s = std::sin(yaw);
  Polygon world;
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = min_x, max_x = -min_x, max_y = -min_x;
  for (const auto & p : footprint) {
    geometry_msgs::msg::Point q;
    q.x = x + c*p.x - s*p.y;
    q.y = y + s*p.x + c*p.y;
    world.push_back(q);
    min_x = std::min(min_x, q.x); max_x = std::max(max_x, q.x);
    min_y = std::min(min_y, q.y); max_y = std::max(max_y, q.y);
  }
  // Include both cells at an exact grid boundary: boundary contact is collision.
  // A finite epsilon also survives subtraction of a much larger map origin.
  const double epsilon = std::max(1e-9, map->getResolution()*1e-8);
  unsigned x0, y0, x1, y1;
  if (!map->worldToMap(min_x-epsilon, min_y-epsilon, x0, y0) ||
      !map->worldToMap(max_x+epsilon, max_y+epsilon, x1, y1)) {
    if (reason) *reason="outside_map";
    return true;
  }
  // Each axis caches the polygon projection. The cell projection changes by
  // center only; its support radius is half-resolution * (|ax| + |ay|).
  struct Axis {double x, y, low, high, radius;};
  std::vector<Axis> axes;
  const auto add_axis = [&](double ax, double ay) {
    double low = std::numeric_limits<double>::infinity(), high = -low;
    for (const auto & p : world) {
      const double projection = ax*p.x + ay*p.y;
      low = std::min(low, projection); high = std::max(high, projection);
    }
    axes.push_back({ax, ay, low, high,
      map->getResolution()*.5*(std::abs(ax)+std::abs(ay))});
  };
  add_axis(1, 0); add_axis(0, 1);
  for (size_t i = 0; i < world.size(); ++i) {
    const auto & a = world[i];
    const auto & b = world[(i+1)%world.size()];
    add_axis(-(b.y-a.y), b.x-a.x);
  }
  for (unsigned iy = y0; iy <= y1; ++iy) {
    for (unsigned ix = x0; ix <= x1; ++ix) {
      if (map->getCost(ix, iy) < 254) continue;
      double wx, wy; map->mapToWorld(ix, iy, wx, wy);
      bool separated = false;
      for (const auto & axis : axes) {
        const double center = axis.x*wx + axis.y*wy;
        if (axis.high < center-axis.radius-1e-10 ||
            axis.low > center+axis.radius+1e-10) {
          separated = true; break;
        }
      }
      if (!separated) {
        if (reason) *reason=map->getCost(ix,iy)==255 ? "unknown" : "occupied";
        return true;
      }
    }
  }
  return false;
}
}  // namespace feedback_test
