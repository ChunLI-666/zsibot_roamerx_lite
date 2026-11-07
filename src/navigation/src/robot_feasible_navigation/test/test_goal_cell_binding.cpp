// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "robot_feasible_navigation/goal_cell_binding.hpp"
using namespace robot_feasible_navigation;
#include "robot_feasible_navigation/segment_coordinator.hpp"
static geometry_msgs::msg::PoseStamped at(double x,double y,double a) {
  geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.position.x=x;p.pose.position.y=y;
  p.pose.orientation.z=std::sin(a/2.);p.pose.orientation.w=std::cos(a/2.);return p;
}
static nav_msgs::msg::Path path(std::initializer_list<geometry_msgs::msg::PoseStamped> poses) {
  nav_msgs::msg::Path p;p.header.frame_id="map";p.poses=poses;return p;
}
TEST(TerminalGoalBinding, RecordedNative144MicrometreGoalIsInsideOfficialCellBudget) {
  nav2_costmap_2d::Costmap2D map(2000,2000,.01,-9.18,-8.86,0);
  const auto s=at(.8207693804070251,1.1400428188075338,-1.503464284225);
  const auto g=at(.8206269664360291,1.1400660310173036,-3.0742606110200903);
  auto raw=path({s});raw.poses.front().pose.orientation=g.pose.orientation;
  const auto p=goal_region::proveGoalCell(raw,s,g,map);
  EXPECT_TRUE(p.valid);EXPECT_TRUE(p.native_singleton);
  EXPECT_NEAR(p.error,.0001442932632427916,1e-15);EXPECT_LT(p.error,p.budget);
}
TEST(TerminalGoalBinding, FullSearchFloatCellProofAcceptsQuantizedEndpointNotArbitraryNearbyXY) {
  nav2_costmap_2d::Costmap2D map(2000,2000,.01,-9.05,-8.9,0);
  const auto s=at(.94999,1.0949519390997173,0.),g=at(.95001,1.0949519390997173,0.);
  auto raw=path({s,at(.9599900436401363,1.0949521636962896,0.)});
  EXPECT_TRUE(goal_region::proveGoalCell(raw,s,g,map).valid);
  raw.poses.back().pose.position.x+=.0000001;
  EXPECT_FALSE(goal_region::proveGoalCell(raw,s,g,map).valid);
  raw.poses.back()=at(.9699900436401363,1.0949521636962896,0.);
  EXPECT_FALSE(goal_region::proveGoalCell(raw,s,g,map).valid);
}
TEST(TerminalGoalBinding, SplitsOnlyExplicitFinalGoalAndPreservesCompletePrefix) {
  auto p=path({at(0.,0.,0.),at(1.,0.,0.),at(1.,0.,M_PI_2),at(1.009,0.,M_PI_2)});
  const auto b=goal_region::splitBinding(p,.01,20.);
  ASSERT_EQ(b.executable.poses.size(),3u);EXPECT_EQ(b.rotation_begin,1u);EXPECT_EQ(b.suffix_begin,0u);
  EXPECT_DOUBLE_EQ(b.original_goal.pose.position.x,1.009);
  EXPECT_THROW(goal_region::splitBinding(path({at(0.,0.,0.),at(.02,0.,0.)}),.01,20.),std::runtime_error);
  EXPECT_THROW(goal_region::splitBinding(path({at(0.,0.,0.),at(.001,0.,1.)}),.01,20.),std::runtime_error);
}
TEST(TerminalGoalBinding, TerminalSuffixStartsAfterAllPriorTurnsAndCannotBeSelectedByLoopProximity) {
  auto p=path({at(0.,0.,0.),at(1.,0.,0.),at(1.,0.,M_PI_2),at(1.,1.,M_PI_2),
    at(1.,1.,M_PI),at(0.,1.,M_PI),at(0.,1.,-M_PI_2),at(0.,0.,-M_PI_2),at(0.,0.,0.),at(.001,0.,0.)});
  const auto b=goal_region::splitBinding(p,.01,20.);
  EXPECT_EQ(b.suffix_begin,6u);EXPECT_EQ(b.rotation_begin,7u);
  EXPECT_GT(b.suffix_begin,0u);
  auto fake=path({at(0.,0.,0.),at(.01,.01,.3),at(.02,.02,0.),at(.021,.02,0.)});
  EXPECT_THROW(goal_region::splitBinding(fake,.01,20.),std::runtime_error);
}
TEST(TerminalGoalBinding, SameGoalNewRouteHasDifferentImmutableGeometricBinding) {
  const auto a=goal_region::splitBinding(path({at(0.,0.,0.),at(1.,0.,0.),at(1.001,0.,0.)}),.01,20.);
  const auto b=goal_region::splitBinding(path({at(.1,0.,0.),at(1.,0.,0.),at(1.001,0.,0.)}),.01,20.);
  EXPECT_NE(a.geometric_digest,b.geometric_digest);
  EXPECT_DOUBLE_EQ(a.original_goal.pose.position.x,b.original_goal.pose.position.x);
}
TEST(TerminalGoalBinding, NonterminalWindingAndReversalRemainAvailableToOrderedCheckpoints) {
  nav_msgs::msg::Path p;p.header.frame_id="map";
  for (int i=0;i<=64;++i)p.poses.push_back(at(0.,0.,i*2*M_PI/64));
  p.poses.push_back(at(.02,0.,0.));p.poses.push_back(at(.021,0.,0.));
  EXPECT_NO_THROW(goal_region::splitBinding(p,.01,20.));
  p=path({at(0.,0.,0.),at(0.,0.,M_PI_2),at(0.,0.,0.),at(.02,0.,0.),at(.021,0.,0.)});
  EXPECT_NO_THROW(goal_region::splitBinding(p,.01,20.));
}
TEST(TerminalGoalBinding, NonterminalHalfTurnAndNearbySignedRunsRemainSupported) {
  for (double sign:{-1.,1.}) {
    const auto p=path({at(0.,0.,0.),at(0.,0.,sign*M_PI),at(-.02,0.,sign*M_PI),at(-.021,0.,sign*M_PI)});
    EXPECT_NO_THROW(goal_region::splitBinding(p,.01,20.));
    const double h=sign*(M_PI-1e-4);
    const auto near=path({at(0.,0.,0.),at(0.,0.,h),at(.02*std::cos(h),.02*std::sin(h),h),
      at(.021*std::cos(h),.021*std::sin(h),h)});
    EXPECT_NO_THROW(goal_region::splitBinding(near,.01,20.));
  }
}
TEST(TerminalGoalBinding, WrappedNonterminalRotationRemainsMonotonic) {
  const double h=190.*M_PI/180.;
  const auto p=path({at(0.,0.,170.*M_PI/180.),at(0.,0.,179.*M_PI/180.),
    at(0.,0.,-179.*M_PI/180.),at(0.,0.,-170.*M_PI/180.),
    at(.02*std::cos(h),.02*std::sin(h),h),at(.021*std::cos(h),.021*std::sin(h),h)});
  EXPECT_NO_THROW(goal_region::splitBinding(p,.01,20.));
}
TEST(TerminalGoalBinding, ManySubEpsilonStepsCannotHideATranslatingLoopAsOneRotation) {
  nav_msgs::msg::Path p;p.header.frame_id="map";
  for (int i=0;i<=2048;++i) {
    const double a=i*2*M_PI/2048;
    p.poses.push_back(at(.01*(1.-std::cos(a)),.01*std::sin(a),a+M_PI_2));
  }
  p.poses.push_back(at(.001,0.,M_PI_2));
  EXPECT_THROW(goal_region::splitBinding(p,.01,20.),std::runtime_error);
}

TEST(TerminalGoalBinding, OriginalGoalGeometryIsCheckedAtCurrentPose) {
  const auto b=goal_region::splitBinding(path({at(0.,0.,0.),at(1.,0.,0.),at(1.002,0.,0.)}),.01,20.);
  EXPECT_TRUE(SegmentCoordinator::originalGoalGeometryReached(at(1.,0.,0.).pose,b.original_goal.pose,.15,.08));
  EXPECT_FALSE(SegmentCoordinator::originalGoalGeometryReached(at(.84,0.,0.).pose,b.original_goal.pose,.15,.08));
  EXPECT_FALSE(SegmentCoordinator::originalGoalGeometryReached(at(1.,0.,.081).pose,b.original_goal.pose,.15,.08));
}
TEST(TerminalGoalBinding, WrongFrameAndNonplanarWireAreRejected) {
  auto p=path({at(0.,0.,0.),at(1.,0.,0.),at(1.002,0.,0.)});
  p.poses.front().header.frame_id="odom";
  EXPECT_THROW(goal_region::splitBinding(p,.01,20.),std::runtime_error);
  p.poses.front().header.frame_id="map";p.poses.back().pose.orientation.x=.01;
  EXPECT_THROW(goal_region::splitBinding(p,.01,20.),std::runtime_error);
}
