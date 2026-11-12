// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "official_lattice_rpp_adapter/planner_goal_contract.hpp"
using namespace official_lattice_rpp_adapter;
static geometry_msgs::msg::PoseStamped at(double x,double y,double a) {
  geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";
  p.pose.position.x=x;p.pose.position.y=y;
  p.pose.orientation.z=std::sin(a/2.);p.pose.orientation.w=std::cos(a/2.);return p;
}
static nav_msgs::msg::Path native(
  const geometry_msgs::msg::PoseStamped & start,const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path p;p.header=start.header;p.poses={start};
  p.poses.front().pose.orientation=goal.pose.orientation;return p;
}
TEST(PlannerGoalContract, RecordedThreeMicrometreSingletonMatchesNativeContract) {
  nav2_costmap_2d::Costmap2D map(2000,2000,.01,-9.05,-8.9,0);
  const auto start=at(.9485818273646971,1.094950615277455,-3.0364735253073127);
  const auto goal=at(.9485784265408937,1.0949519390997173,-1.4656771985124162);
  EXPECT_TRUE(nativeLatticeSameCellSingleton(native(start,goal),start,goal,map));
  EXPECT_TRUE(withinPathRotationXYEquivalence(start.pose,goal.pose));
}
TEST(PlannerGoalContract, SameCellDoesNotAuthorizeLargerSubcellTranslation) {
  nav2_costmap_2d::Costmap2D map(2000,2000,.01,-9.05,-8.9,0);
  const auto start=at(.948,1.094,0.),goal=at(.9485,1.094,M_PI/2.);
  EXPECT_TRUE(nativeLatticeSameCellSingleton(native(start,goal),start,goal,map));
  EXPECT_FALSE(withinPathRotationXYEquivalence(start.pose,goal.pose));
}
TEST(PlannerGoalContract, TwentyMicrometreCrossCellMoveDoesNotMatchNativeContract) {
  nav2_costmap_2d::Costmap2D map(2000,2000,.01,-9.05,-8.9,0);
  const auto start=at(.94999,1.094,0.),goal=at(.95001,1.094,M_PI/2.);
  EXPECT_TRUE(withinPathRotationXYEquivalence(start.pose,goal.pose));
  EXPECT_FALSE(nativeLatticeSameCellSingleton(native(start,goal),start,goal,map));
}
TEST(PlannerGoalContract, RotationEquivalenceRetainsStrictExistingBoundary) {
  const auto start=at(0.,0.,0.);
  EXPECT_TRUE(withinPathRotationXYEquivalence(start.pose,
    at(std::nextafter(backport::kPathRotationXYEpsilon,0.),0.,0.).pose));
  EXPECT_FALSE(withinPathRotationXYEquivalence(start.pose,
    at(backport::kPathRotationXYEpsilon,0.,0.).pose));
  EXPECT_FALSE(withinPathRotationXYEquivalence(start.pose,
    at(std::nextafter(backport::kPathRotationXYEpsilon,1.),0.,0.).pose));
}
TEST(PlannerGoalContract, SingletonPatternMustRemainVerbatim) {
  nav2_costmap_2d::Costmap2D map(2000,2000,.01,-9.05,-8.9,0);
  const auto start=at(.948,1.094,0.),goal=at(.948003,1.094,M_PI/2.);
  const auto raw=native(start,goal);
  auto changed=raw;changed.poses.front().pose.position.x=goal.pose.position.x;
  EXPECT_FALSE(nativeLatticeSameCellSingleton(changed,start,goal,map));
  changed=raw;changed.poses.front().pose.orientation=start.pose.orientation;
  EXPECT_FALSE(nativeLatticeSameCellSingleton(changed,start,goal,map));
  changed=raw;changed.poses.push_back(raw.poses.front());
  EXPECT_FALSE(nativeLatticeSameCellSingleton(changed,start,goal,map));
  changed=raw;changed.poses.front().header.frame_id="odom";
  EXPECT_FALSE(nativeLatticeSameCellSingleton(changed,start,goal,map));
}
