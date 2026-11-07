// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <cstdint>
#include <vector>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Pure geometry/protocol extracted from the verified official adapter.
// No completion permission, controller state, or initial-region policy lives here.
namespace robot_feasible_navigation::goal_region {
inline constexpr double kPathRotationXYEpsilon=1e-4;
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

struct CellProof {
  bool valid=false;
  bool native_singleton=false;
  double error=0.;
  double budget=0.;
};
CellProof proveGoalCell(const nav_msgs::msg::Path & raw,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,const nav2_costmap_2d::Costmap2D & map);
double consumerBudget(double resolution,double global_extent,const geometry_msgs::msg::Pose & goal);

struct Binding {
  std::uint64_t generation=0;
  std::uint64_t geometric_digest=0;
  nav_msgs::msg::Path executable;
  geometry_msgs::msg::PoseStamped original_goal;
  double error=0.;
  std::size_t suffix_begin=0;
  std::size_t rotation_begin=0;
  std::vector<double> terminal_headings;
};
// Explicit opt-in protocol only: the final standard goal pose is separated from
// the mandatory executable prefix. Plain FollowPath must not call this parser.
Binding splitBinding(const nav_msgs::msg::Path & path,double resolution,double global_extent);

}
