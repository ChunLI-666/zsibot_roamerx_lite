// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "robot_feasible_navigation/footprint_sweep.hpp"

namespace official_lattice_rpp_adapter::goal_region {
// Pose-goal opt-in only. Call once per plan, after current TF/odometry checks,
// with the costmap locked and the already padded footprint. A blocked complete
// rotation returns no shortcut; the original ordered path remains available.
inline std::optional<std::vector<double>> initialGoalRotation(
  nav2_costmap_2d::Costmap2D & map,
  const robot_feasible_navigation::sweep::Footprint & footprint,
  const geometry_msgs::msg::Pose & actual,const geometry_msgs::msg::Pose & goal,
  double xy_tolerance,double maximum_initial_xy_error=0.25)
{
  namespace sweep=robot_feasible_navigation::sweep;
  const auto planar=[](const geometry_msgs::msg::Pose & p) {
    const auto & q=p.orientation;
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    return std::isfinite(p.position.x)&&std::isfinite(p.position.y)&&
      std::isfinite(p.position.z)&&std::abs(p.position.z)<1e-5&&
      std::isfinite(norm)&&std::abs(norm-1.)<1e-3&&
      std::abs(q.x)<1e-5&&std::abs(q.y)<1e-5;
  };
  if (!planar(actual)||!planar(goal)||!std::isfinite(xy_tolerance)||
    xy_tolerance<=0.||xy_tolerance>0.25||
    !std::isfinite(maximum_initial_xy_error)||maximum_initial_xy_error<=0.||
    maximum_initial_xy_error>0.25||!sweep::validFootprint(footprint)||
    std::hypot(actual.position.x-goal.position.x,actual.position.y-goal.position.y)>
    std::min(xy_tolerance,maximum_initial_xy_error))
  {return std::nullopt;}
  const double start=tf2::getYaw(actual.orientation),end=tf2::getYaw(goal.orientation);
  const double delta=std::atan2(std::sin(end-start),std::cos(end-start));
  if (!sweep::footprintFree(map,footprint,actual.position.x,actual.position.y,start)) {
    return std::nullopt;
  }
  const int steps=std::max(1,static_cast<int>(std::ceil(std::abs(delta)/0.02)));
  std::vector<double> headings{start};
  for (int i=1;i<=steps;++i) {
    const double unwrapped=start+delta*double(i)/steps;
    const double next=i==steps?end:std::atan2(std::sin(unwrapped),std::cos(unwrapped));
    if (!sweep::sweepFree(map,footprint,actual.position.x,actual.position.y,headings.back(),
      actual.position.x,actual.position.y,next)) {return std::nullopt;}
    headings.push_back(next);
  }
  return headings;
}
}
