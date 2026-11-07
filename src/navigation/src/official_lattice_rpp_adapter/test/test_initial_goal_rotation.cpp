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



namespace {
using Hold=goal_region::TerminalStopHold;
using Action=Hold::Action;
Action sample(Hold & hold,std::int64_t ms,bool stopped=true,bool in_yaw=true,
  bool in_xy=true,bool ordered=true,std::uint64_t generation=1) {
  return hold.observe(generation,ordered,in_xy,in_yaw,stopped,
    1000000000+ms*1000000,2000000000+ms*1000000);
}
}
TEST(TerminalStopHold, CheckedZeroPrecedesContinuousMeasuredDwell) {
  Hold hold;
  EXPECT_EQ(sample(hold,0),Action::HoldZero);
  EXPECT_EQ(sample(hold,50),Action::HoldZero);
  EXPECT_EQ(sample(hold,349),Action::HoldZero);
  EXPECT_EQ(sample(hold,350),Action::Ready);
}
TEST(TerminalStopHold, RecordedB21ResidualMotionMatchesExistingWaypointStopContract) {
  geometry_msgs::msg::Twist v;v.angular.z=-.0328615546;
  ASSERT_TRUE(goal_region::terminalMeasuredStopped(v));
  Hold hold;EXPECT_EQ(sample(hold,0),Action::HoldZero);
  for(int ms=50;ms<350;ms+=50) {
    EXPECT_EQ(sample(hold,ms,goal_region::terminalMeasuredStopped(v)),Action::HoldZero);
  }
  EXPECT_EQ(sample(hold,350,goal_region::terminalMeasuredStopped(v)),Action::Ready);
}
TEST(TerminalStopHold, RecordedB9MovingYawEntryNeverCertifiesStop) {
  geometry_msgs::msg::Twist v;v.angular.z=.22;
  ASSERT_FALSE(goal_region::terminalMeasuredStopped(v));Hold hold;
  EXPECT_EQ(sample(hold,0,false),Action::HoldZero);
  EXPECT_EQ(sample(hold,1000,false),Action::HoldZero);
  EXPECT_EQ(sample(hold,1050,true,false),Action::CorrectYaw);
  EXPECT_FALSE(hold.holding());
}
TEST(TerminalStopHold, MotionInterruptsAndRestartsEntireDwell) {
  Hold hold;sample(hold,0);sample(hold,50);sample(hold,200);
  EXPECT_EQ(sample(hold,250,false),Action::HoldZero);
  EXPECT_EQ(sample(hold,300),Action::HoldZero);
  EXPECT_EQ(sample(hold,599),Action::HoldZero);
  EXPECT_EQ(sample(hold,600),Action::Ready);
}
TEST(TerminalStopHold, CurrentYawAndXYCannotReuseEarlierMatchingGeometry) {
  Hold hold;sample(hold,0);sample(hold,50);sample(hold,200);
  EXPECT_EQ(sample(hold,250,false,false),Action::HoldZero);
  EXPECT_EQ(sample(hold,300,true,false),Action::CorrectYaw);
  EXPECT_FALSE(hold.holding());
  EXPECT_EQ(sample(hold,350),Action::HoldZero);
  EXPECT_EQ(sample(hold,400),Action::HoldZero);
  EXPECT_EQ(sample(hold,650,true,true,false),Action::Outside);
  EXPECT_FALSE(hold.holding());
}
TEST(TerminalStopHold, FrozenOrRegressingOriginalOdometryCannotSupplyDwell) {
  Hold hold;
  EXPECT_EQ(hold.observe(1,true,true,true,true,1000000000,2000000000),Action::HoldZero);
  EXPECT_EQ(hold.observe(1,true,true,true,true,1050000000,2050000000),Action::HoldZero);
  EXPECT_EQ(hold.observe(1,true,true,true,true,1050000000,2400000000),Action::HoldZero);
  EXPECT_EQ(hold.observe(1,true,true,true,true,1040000000,2450000000),Action::HoldZero);
  EXPECT_EQ(hold.observe(1,true,true,true,true,1340000000,2749999999),Action::HoldZero);
  EXPECT_EQ(hold.observe(1,true,true,true,true,1340000001,2750000000),Action::Ready);
}
TEST(TerminalStopHold, QueuedEpochJumpCannotReplaceRealElapsedTime) {
  Hold hold;
  EXPECT_EQ(hold.observe(1,true,true,true,true,1000000000,2000000000),Action::HoldZero);
  EXPECT_EQ(hold.observe(1,true,true,true,true,1050000000,2050000000),Action::HoldZero);
  EXPECT_EQ(hold.observe(1,true,true,true,true,9050000000,2050000001),Action::HoldZero);
  EXPECT_EQ(hold.observe(1,true,true,true,true,9100000000,2350000000),Action::Ready);
}
TEST(TerminalStopHold, UnobservedControllerPauseCannotBridgeTheDwell) {
  Hold hold;sample(hold,0);sample(hold,50);
  EXPECT_EQ(sample(hold,400),Action::HoldZero);
  EXPECT_EQ(sample(hold,450),Action::HoldZero);
  EXPECT_EQ(sample(hold,699),Action::HoldZero);
  EXPECT_EQ(sample(hold,700),Action::Ready);
}
TEST(TerminalStopHold, OrderedWindingCannotBeSkippedByMatchingFinalGoal) {
  goal_region::HeadingCursor cursor;std::vector<double> winding;
  for(int i=0;i<=64;++i) {winding.push_back(i*2*M_PI/64);}
  cursor.reset(winding);cursor.advance(0.,.08);ASSERT_FALSE(cursor.complete());Hold hold;
  EXPECT_EQ(sample(hold,0,true,true,true,cursor.complete()),Action::Track);
  EXPECT_EQ(sample(hold,1000,true,true,true,cursor.complete()),Action::Track);
  for(int i=1;i<=64;++i) {cursor.advance(i*2*M_PI/64,.08);}
  ASSERT_TRUE(cursor.complete());EXPECT_EQ(sample(hold,1100),Action::HoldZero);
  EXPECT_EQ(sample(hold,1150),Action::HoldZero);EXPECT_EQ(sample(hold,1450),Action::Ready);
}
TEST(TerminalStopHold, ResetAndNewGenerationInvalidateStoppedReadiness) {
  Hold hold;sample(hold,0);sample(hold,50);EXPECT_EQ(sample(hold,350),Action::Ready);
  hold.reset();EXPECT_FALSE(hold.holding());
  EXPECT_EQ(sample(hold,400),Action::HoldZero);EXPECT_EQ(sample(hold,450),Action::HoldZero);
  EXPECT_EQ(sample(hold,750),Action::Ready);
  EXPECT_EQ(sample(hold,800,true,true,true,true,2),Action::HoldZero);
  EXPECT_EQ(sample(hold,850,true,true,true,true,2),Action::HoldZero);
  EXPECT_EQ(sample(hold,1150,true,true,true,true,2),Action::Ready);
}
TEST(TerminalStopHold, SweptCorrectionPreservesCompletedMandatoryCursorAndOriginalGoal) {
  nav2_costmap_2d::Costmap2D map(600,600,.01,-3.,-3.,0);
  goal_region::HeadingCursor mandatory;mandatory.reset({.2,.3});mandatory.advance(.2,.08);mandatory.advance(.3,.08);
  ASSERT_TRUE(mandatory.complete());const auto before=mandatory.next();
  {
    sweep::ScopedObstacleIndex index(map);
    const auto actual=pose(.06,-.02,.4031358849462336),original=pose(0.,0.,.3);
    const auto correction=goal_region::initialGoalRotation(map,body(),actual,original,.15);
    ASSERT_TRUE(correction);EXPECT_NEAR(turn(*correction),-.1031358849462336,1e-12);
    EXPECT_DOUBLE_EQ(original.orientation.z,std::sin(.3/2.));EXPECT_EQ(mandatory.next(),before);
  }
  unsigned int x,y;ASSERT_TRUE(map.worldToMap(.85,.4,x,y));map.setCost(x,y,255);
  sweep::ScopedObstacleIndex blocked_index(map);
  EXPECT_FALSE(goal_region::initialGoalRotation(map,body(),pose(0.,0.,0.),pose(0.,0.,M_PI),.15));
}
TEST(TerminalStopHold, MeasuredStopUsesFinitePlanarNormAndExistingContractLimits) {
  geometry_msgs::msg::Twist v;EXPECT_TRUE(goal_region::terminalMeasuredStopped(v));
  v.linear.x=.025;v.linear.y=.025;EXPECT_FALSE(goal_region::terminalMeasuredStopped(v));
  v.linear.x=.03;v.linear.y=0.;v.angular.z=.05;EXPECT_TRUE(goal_region::terminalMeasuredStopped(v));
  v.angular.z=.0500001;EXPECT_FALSE(goal_region::terminalMeasuredStopped(v));
  v.angular.z=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(goal_region::terminalMeasuredStopped(v));
}
