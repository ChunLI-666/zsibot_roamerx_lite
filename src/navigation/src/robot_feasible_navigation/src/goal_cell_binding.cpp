// SPDX-License-Identifier: Apache-2.0
#include "robot_feasible_navigation/goal_cell_binding.hpp"
#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace robot_feasible_navigation::goal_region {
namespace {
double angle(double a,double b) {return std::atan2(std::sin(b-a),std::cos(b-a));}
double distance(const geometry_msgs::msg::Pose & a,const geometry_msgs::msg::Pose & b) {
  return std::hypot(a.position.x-b.position.x,a.position.y-b.position.y);
}
double roundoff(double value) {
  return 64.*std::numeric_limits<double>::epsilon()*std::max(1.,std::abs(value));
}
double ulp(float value) {
  return std::max(std::abs(double(std::nextafter(value,std::numeric_limits<float>::infinity()))-value),
    std::abs(double(value)-std::nextafter(value,-std::numeric_limits<float>::infinity())));
}
bool recoveredCell(double world,double origin,double resolution,float goal_cell) {
  const float cell=static_cast<float>((world-static_cast<float>(origin))/resolution);
  const double reconstructed=static_cast<float>(origin)+double(cell)*resolution;
  return std::isfinite(cell) && std::abs(world-reconstructed)<=roundoff(world) &&
    std::floor(cell)==std::floor(goal_cell);
}
void need(bool condition,const char * message) {
  if (!condition) {throw std::runtime_error(message);}
}
bool planar(const geometry_msgs::msg::Pose & p) {
  const auto & q=p.orientation;const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  return std::isfinite(p.position.x)&&std::isfinite(p.position.y)&&std::isfinite(p.position.z)&&
    std::abs(p.position.z)<1e-5&&std::isfinite(norm)&&std::abs(norm-1.)<1e-3&&
    std::abs(q.x)<1e-5&&std::abs(q.y)<1e-5;
}
bool straight(const geometry_msgs::msg::Pose & a,const geometry_msgs::msg::Pose & b,double slack) {
  const double h=tf2::getYaw(a.orientation),dx=b.position.x-a.position.x,dy=b.position.y-a.position.y;
  return std::abs(angle(h,tf2::getYaw(b.orientation)))<=1e-5 &&
    std::abs(dx*std::sin(h)-dy*std::cos(h))<=slack &&
    dx*std::cos(h)+dy*std::sin(h)>=-slack;
}
}

CellProof proveGoalCell(const nav_msgs::msg::Path & raw,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,const nav2_costmap_2d::Costmap2D & map)
{
  CellProof result;
  if (raw.poses.empty() || !planar(raw.poses.back().pose)) {return result;}
  float gx,gy;
  if (!map.worldToMapContinuous(goal.pose.position.x,goal.pose.position.y,gx,gy)) {return result;}
  const auto & end=raw.poses.back().pose;const double r=map.getResolution();
  result.error=distance(end,goal.pose);
  const double bx=r*(1.+ulp(gx))+std::abs(double(static_cast<float>(map.getOriginX()))-map.getOriginX())+roundoff(end.position.x);
  const double by=r*(1.+ulp(gy))+std::abs(double(static_cast<float>(map.getOriginY()))-map.getOriginY())+roundoff(end.position.y);
  result.budget=std::hypot(bx,by);
  result.native_singleton=nativeLatticeSameCellSingleton(raw,start,goal,map);
  result.valid=(result.native_singleton ||
    (recoveredCell(end.position.x,map.getOriginX(),r,gx)&&
    recoveredCell(end.position.y,map.getOriginY(),r,gy))) && result.error<=result.budget;
  return result;
}

double consumerBudget(double resolution,double global_extent,const geometry_msgs::msg::Pose & goal) {
  need(std::isfinite(resolution)&&resolution>0.&&std::isfinite(global_extent)&&global_extent>0.,
    "Invalid frozen goal-region geometry");
  const double scale=std::max({1.,global_extent,std::abs(goal.position.x)+global_extent,
    std::abs(goal.position.y)+global_extent});
  return resolution*std::sqrt(2.)+8.*std::numeric_limits<float>::epsilon()*scale;
}

Binding splitBinding(const nav_msgs::msg::Path & path,double resolution,double global_extent) {
  need(path.poses.size()>=2&&!path.header.frame_id.empty(),"Goal binding needs a framed executable prefix and goal");
  Binding result;result.original_goal=path.poses.back();result.original_goal.header.frame_id=path.header.frame_id;
  // Local evidence identity, not a transport tag or cryptographic authority.
  result.geometric_digest=1469598103934665603ULL;
  const auto digest_byte=[&](unsigned char value) {result.geometric_digest^=value;result.geometric_digest*=1099511628211ULL;};
  for (const auto value:path.header.frame_id) {digest_byte(static_cast<unsigned char>(value));}
  for (const auto & p:path.poses) {
    for (const double value:{p.pose.position.x,p.pose.position.y,p.pose.position.z,
      p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w}) {
      unsigned char bytes[sizeof(double)];std::memcpy(bytes,&value,sizeof(double));
      for (const auto byte:bytes) {digest_byte(byte);}
    }
  }
  result.executable=path;result.executable.poses.pop_back();
  const auto & goal=result.original_goal.pose;const auto & end=result.executable.poses.back().pose;
  for (const auto & p:path.poses) {
    need(planar(p.pose)&&(p.header.frame_id.empty()||p.header.frame_id==path.header.frame_id),
      "Invalid planar goal binding path/frame");
  }
  result.error=distance(end,goal);
  need(result.error<=consumerBudget(resolution,global_extent,goal),"Goal binding exceeds frozen cell budget");
  need(std::abs(angle(tf2::getYaw(end.orientation),tf2::getYaw(goal.orientation)))<1e-9,
    "Goal binding must preserve original terminal yaw");
  const double epsilon=kPathRotationXYEpsilon;
  const double slack=8.*std::numeric_limits<float>::epsilon()*std::max({1.,global_extent,
    std::abs(goal.position.x)+global_extent,std::abs(goal.position.y)+global_extent});
  bool in_colocated_run=false;geometry_msgs::msg::Pose colocated_anchor;
  for (std::size_t i=1;i<result.executable.poses.size();++i) {
    const auto & a=result.executable.poses[i-1].pose;const auto & b=result.executable.poses[i].pose;
    if (distance(a,b)<epsilon) {
      if (!in_colocated_run) {colocated_anchor=a;in_colocated_run=true;}
      need(distance(colocated_anchor,b)<epsilon,"Co-located rotation run accumulates translation");
    } else {
      in_colocated_run=false;
      need(straight(a,b,slack),"Goal-region executable prefix is not forward stop-turn geometry");
    }
  }
  result.rotation_begin=result.executable.poses.size()-1;
  while (result.rotation_begin>0 &&
    distance(result.executable.poses[result.rotation_begin-1].pose,end)<epsilon) {--result.rotation_begin;}
  result.suffix_begin=result.rotation_begin;
  const auto heading=tf2::getYaw(result.executable.poses[result.rotation_begin].pose.orientation);
  while (result.suffix_begin>0) {
    const auto & a=result.executable.poses[result.suffix_begin-1].pose;
    const auto & b=result.executable.poses[result.suffix_begin].pose;
    if (std::abs(angle(tf2::getYaw(a.orientation),heading))>1e-5 || !straight(a,b,slack)) {break;}
    --result.suffix_begin;
  }
  for (std::size_t i=result.rotation_begin;i<result.executable.poses.size();++i) {
    result.terminal_headings.push_back(tf2::getYaw(result.executable.poses[i].pose.orientation));
  }
  // Every active nonterminal run is consumed by the same ordered checkpoint
  // mechanism as the final run; matching an entire run's endpoint is not proof.
  return result;
}

}
