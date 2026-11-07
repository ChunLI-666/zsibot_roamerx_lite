// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "official_lattice_rpp_adapter/initial_goal_rotation.hpp"
#include "official_lattice_rpp_adapter/terminal_goal_binding.hpp"
#include "official_lattice_rpp_adapter/terminal_stop_hold.hpp"
using namespace official_lattice_rpp_adapter;
namespace sweep=robot_feasible_navigation::sweep;
static geometry_msgs::msg::Pose pose(double x,double y,double a) {
  geometry_msgs::msg::Pose p;p.position.x=x;p.position.y=y;
  p.orientation.z=std::sin(a/2.);p.orientation.w=std::cos(a/2.);return p;
}
static sweep::Footprint body() {
  sweep::Footprint f;
  for (const auto & xy:std::vector<std::pair<double,double>>{
    {-.797,-.680},{.797,-.680},{.797,.680},{-.797,.680}})
  {geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;f.push_back(p);}
  return f;
}
static double turn(const std::vector<double> & h) {
  double value=0.;for(std::size_t i=1;i<h.size();++i)
  {value+=std::atan2(std::sin(h[i]-h[i-1]),std::cos(h[i]-h[i-1]));}return value;
}
TEST(InitialGoalRotation, RecordedN000NeedsOnlyOriginalShortAngleInsideOriginalRegion) {
  nav2_costmap_2d::Costmap2D map(600,600,.01,0.,-41.,0);sweep::ScopedObstacleIndex index(map);
  const auto result=goal_region::initialGoalRotation(map,body(),
    pose(2.3986238638985964,-38.84884193250153,-.2521269731370991),
    pose(2.374999549089445,-38.93499825434375,-.3926990816987248),.15);
  ASSERT_TRUE(result);EXPECT_NEAR(turn(*result),-.14057210856162572,1e-12);
  EXPECT_LE(result->size(),10u);goal_region::HeadingCursor cursor;cursor.reset(*result);
  for(const auto a:*result) {cursor.advance(a,.08);}EXPECT_TRUE(cursor.complete());
}
TEST(InitialGoalRotation, HalfTurnCheckpointsRetainTheirSelectedSignedDirection) {
  nav2_costmap_2d::Costmap2D map(600,600,.01,-3.,-3.,0);sweep::ScopedObstacleIndex index(map);
  for(const double sign:{-1.,1.}) {
    const auto result=goal_region::initialGoalRotation(map,body(),pose(0.,0.,0.),pose(.1,0.,sign*M_PI),.15);
    ASSERT_TRUE(result);EXPECT_NEAR(turn(*result),sign*M_PI,1e-12);
    for(std::size_t i=1;i<result->size();++i) {
      const double d=std::atan2(std::sin((*result)[i]-(*result)[i-1]),std::cos((*result)[i]-(*result)[i-1]));
      EXPECT_GT(sign*d,0.);EXPECT_LE(std::abs(d),.02+1e-12);
    }
  }
}
TEST(InitialGoalRotation, ClearEndpointsDoNotPermitBlockedIntermediateFilledBodyRotation) {
  nav2_costmap_2d::Costmap2D map(600,600,.01,-3.,-3.,0);
  unsigned int x,y;ASSERT_TRUE(map.worldToMap(.85,.4,x,y));map.setCost(x,y,254);
  sweep::ScopedObstacleIndex index(map);
  ASSERT_TRUE(sweep::footprintFree(map,body(),0.,0.,0.));
  ASSERT_TRUE(sweep::footprintFree(map,body(),0.,0.,M_PI));
  EXPECT_FALSE(goal_region::initialGoalRotation(map,body(),pose(0.,0.,0.),pose(.1,0.,M_PI),.15));
}
TEST(InitialGoalRotation, OriginalRegionAndFinitePoseAreRequired) {
  nav2_costmap_2d::Costmap2D map(600,600,.01,-3.,-3.,0);sweep::ScopedObstacleIndex index(map);
  EXPECT_FALSE(goal_region::initialGoalRotation(map,body(),pose(0.,0.,0.),pose(.151,0.,.1),.15));
  auto invalid=pose(.1,0.,0.);invalid.orientation.w=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(goal_region::initialGoalRotation(map,body(),pose(0.,0.,0.),invalid,.15));
  EXPECT_FALSE(goal_region::initialGoalRotation(map,body(),pose(0.,0.,0.),pose(0.,0.,0.),.3));
}
TEST(InitialGoalRotation, AlreadyMatchingGoalStillUsesNormalCompletionCursor) {
  nav2_costmap_2d::Costmap2D map(600,600,.01,-3.,-3.,0);sweep::ScopedObstacleIndex index(map);
  const auto result=goal_region::initialGoalRotation(map,body(),pose(0.,0.,.7),pose(.1,0.,.7),.15);
  ASSERT_TRUE(result);EXPECT_NEAR(turn(*result),0.,1e-12);
  goal_region::HeadingCursor cursor;cursor.reset(*result);cursor.advance(.7,.08);EXPECT_TRUE(cursor.complete());
}


TEST(TerminalStopHold, MovingThroughOriginalGoalNeverBecomesReady) {
  goal_region::TerminalStopHold hold;using A=goal_region::TerminalStopHold::Action;
  EXPECT_EQ(hold.observe(1,true,true,true,false,100),A::HoldZero);
  EXPECT_EQ(hold.observe(1,true,true,true,false,101),A::HoldZero);
  EXPECT_EQ(hold.observe(1,true,true,true,true,100),A::HoldZero);
  EXPECT_EQ(hold.observe(1,true,true,true,true,102),A::Ready);
  EXPECT_EQ(hold.observe(1,true,true,true,false,103),A::HoldZero);
}
TEST(TerminalStopHold, AlreadyStoppedGoalStillRequiresZeroThenNewMeasuredSample) {
  goal_region::TerminalStopHold hold;using A=goal_region::TerminalStopHold::Action;
  EXPECT_EQ(hold.observe(7,true,true,true,true,900),A::HoldZero);
  EXPECT_EQ(hold.observe(7,true,true,true,true,900),A::HoldZero);
  EXPECT_EQ(hold.observe(7,true,true,true,true,901),A::Ready);
}
TEST(TerminalStopHold, RecordedB9TailDriftCannotSucceedAndRequestsStoppedCorrection) {
  goal_region::TerminalStopHold hold;using A=goal_region::TerminalStopHold::Action;
  // Original sampled first entry had |yaw error|=.0799545 but angular velocity>.22.
  EXPECT_EQ(hold.observe(9,true,true,true,false,1794036887902296304LL),A::HoldZero);
  EXPECT_EQ(hold.observe(9,true,true,false,false,1794036888800000000LL),A::HoldZero);
  // First strict stop was outside .08 at +.1031358849462336rad.
  EXPECT_EQ(hold.observe(9,true,true,false,true,1794036889000000000LL),A::CorrectYaw);
  EXPECT_FALSE(hold.holding());
  // Swept correction has a distinct cursor; mandatory prefix remains complete.
  EXPECT_EQ(hold.observe(9,false,true,false,true,1794036889100000000LL),A::Track);
  EXPECT_EQ(hold.observe(9,true,true,true,false,1794036889200000000LL),A::HoldZero);
  EXPECT_EQ(hold.observe(9,true,true,true,true,1794036889300000000LL),A::Ready);
}
TEST(TerminalStopHold, OriginalOrderedTurnCannotBeSkippedByMatchingFinalGoal) {
  goal_region::HeadingCursor mandatory;
  std::vector<double> winding;for(int i=0;i<=64;++i) {winding.push_back(i*2*M_PI/64);}
  mandatory.reset(winding);mandatory.advance(0.,.08);ASSERT_FALSE(mandatory.complete());
  goal_region::TerminalStopHold hold;using A=goal_region::TerminalStopHold::Action;
  EXPECT_EQ(hold.observe(1,mandatory.complete(),true,true,true,10),A::Track);
  EXPECT_FALSE(hold.holding());
  for(int i=1;i<=64;++i) {mandatory.advance(i*2*M_PI/64,.08);}
  ASSERT_TRUE(mandatory.complete());
  EXPECT_EQ(hold.observe(1,mandatory.complete(),true,true,true,11),A::HoldZero);
  EXPECT_EQ(hold.observe(1,mandatory.complete(),true,true,true,12),A::Ready);
}
TEST(TerminalStopHold, ActualOriginCorrectionPreservesMandatoryCursorAndOriginalGoal) {
  nav2_costmap_2d::Costmap2D map(600,600,.01,-3.,-3.,0);sweep::ScopedObstacleIndex index(map);
  goal_region::HeadingCursor mandatory;mandatory.reset({.2,.3});mandatory.advance(.2,.08);mandatory.advance(.3,.08);
  ASSERT_TRUE(mandatory.complete());const auto before=mandatory.next();
  const auto actual=pose(.06,-.02,.4031358849462336),original=pose(0.,0.,.3);
  const auto correction=goal_region::initialGoalRotation(map,body(),actual,original,.15);
  ASSERT_TRUE(correction);EXPECT_NEAR(turn(*correction),-.1031358849462336,1e-12);
  EXPECT_DOUBLE_EQ(original.orientation.z,std::sin(.3/2.));
  EXPECT_EQ(mandatory.next(),before);EXPECT_TRUE(mandatory.complete());
  goal_region::HeadingCursor recovery;recovery.reset(*correction);recovery.advance(.4031358849462336,.08);
  EXPECT_FALSE(recovery.complete());
  for(const double heading:*correction) {recovery.advance(heading,.08);}EXPECT_TRUE(recovery.complete());
}
TEST(TerminalStopHold, OutsideXYAndBlockedCorrectionNeverBecomeSuccess) {
  goal_region::TerminalStopHold hold;using A=goal_region::TerminalStopHold::Action;
  EXPECT_EQ(hold.observe(1,true,true,true,false,10),A::HoldZero);
  EXPECT_EQ(hold.observe(1,true,false,true,true,11),A::Outside);EXPECT_FALSE(hold.holding());
  nav2_costmap_2d::Costmap2D map(600,600,.01,-3.,-3.,0);
  unsigned int x,y;ASSERT_TRUE(map.worldToMap(.85,.4,x,y));map.setCost(x,y,255);
  sweep::ScopedObstacleIndex index(map);
  EXPECT_FALSE(goal_region::initialGoalRotation(map,body(),pose(0.,0.,0.),pose(0.,0.,M_PI),.15));
}
TEST(TerminalStopHold, ResetCancelAndNewPlanGenerationCannotReuseStoppedReadiness) {
  goal_region::TerminalStopHold hold;using A=goal_region::TerminalStopHold::Action;
  EXPECT_EQ(hold.observe(1,true,true,true,true,10),A::HoldZero);
  EXPECT_EQ(hold.observe(1,true,true,true,true,11),A::Ready);
  hold.reset();EXPECT_FALSE(hold.holding());
  EXPECT_EQ(hold.observe(1,true,true,true,true,12),A::HoldZero);
  EXPECT_EQ(hold.observe(1,true,true,true,true,13),A::Ready);
  EXPECT_EQ(hold.observe(2,true,true,true,true,14),A::HoldZero);
  EXPECT_EQ(hold.observe(2,true,true,true,true,14),A::HoldZero);
}
TEST(TerminalStopHold, MeasuredStopUsesFiniteTwoDimensionalTranslationAndRotation) {
  geometry_msgs::msg::Twist v;EXPECT_TRUE(goal_region::terminalMeasuredStopped(v));
  v.linear.x=.008;v.linear.y=.008;EXPECT_FALSE(goal_region::terminalMeasuredStopped(v));
  v.linear.x=v.linear.y=0.;v.angular.z=.0100001;EXPECT_FALSE(goal_region::terminalMeasuredStopped(v));
  v.angular.z=.01;EXPECT_TRUE(goal_region::terminalMeasuredStopped(v));
  v.angular.z=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(goal_region::terminalMeasuredStopped(v));
}
