// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include "official_lattice_rpp_adapter/rotation_constraints.hpp"

namespace official_lattice_rpp_adapter {
// Installed Jazzy 1.3.13's documented same-cell branch copies start.position
// and goal.orientation verbatim into a single pose. This identifies that output
// before any float round-trip restoration. Callers validate planar requests.
inline bool nativeLatticeSameCellSingleton(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const nav2_costmap_2d::Costmap2D & map)
{
  if (path.poses.size()!=1 || path.header.frame_id!=start.header.frame_id ||
    goal.header.frame_id!=start.header.frame_id ||
    path.poses.front().header.frame_id!=path.header.frame_id) {return false;}
  const auto & p=path.poses.front().pose;
  const auto & s=start.pose.position;
  const auto & q=goal.pose.orientation;
  if (p.position.x!=s.x || p.position.y!=s.y || p.position.z!=s.z ||
    p.orientation.x!=q.x || p.orientation.y!=q.y ||
    p.orientation.z!=q.z || p.orientation.w!=q.w) {return false;}
  float sx,sy,gx,gy;
  return map.worldToMapContinuous(s.x,s.y,sx,sy) &&
    map.worldToMapContinuous(goal.pose.position.x,goal.pose.position.y,gx,gy) &&
    std::floor(sx)==std::floor(gx) && std::floor(sy)==std::floor(gy);
}

inline bool withinPathRotationXYEquivalence(
  const geometry_msgs::msg::Pose & start,const geometry_msgs::msg::Pose & goal)
{
  return std::hypot(start.position.x-goal.position.x,
    start.position.y-goal.position.y)<backport::kPathRotationXYEpsilon;
}
}
