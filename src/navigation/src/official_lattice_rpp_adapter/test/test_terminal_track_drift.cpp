// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <limits>
#include "official_lattice_rpp_adapter/terminal_track_drift.hpp"
using Guard=official_lattice_rpp_adapter::goal_region::TerminalTrackDriftGuard;
using Reentry=Guard::Reentry;
namespace {
Guard::Goal goal(double x=0.,double yaw=0.,const std::string & frame="map") {return {x,0.,yaw,frame};}
Reentry observe(Guard & g,int ms,bool stopped=true,double x=.14,
  std::int64_t original_ms=-1) {
  if (original_ms<0) {original_ms=ms;}
  const auto stamp=1000000000LL+original_ms*1000000;
  const auto steady=2000000000LL+ms*1000000;
  const auto action=g.observeReentry(x,0.,.15,stopped,stamp,steady);
  if (action==Reentry::HoldZero) {g.checkedZero(stamp,steady);}
  return action;
}
void replan(Guard & g) {
  g.setPlan(goal(),1);ASSERT_TRUE(g.admit(1,.14,0.,.15));g.setPlan(goal(),2);
  ASSERT_TRUE(g.reentryRequired());
}
}
TEST(TerminalTrackDrift, DefaultZeroPreservesStrictOriginalRegionAndNoNewReentry) {
  Guard g;g.setPlan(goal(),1);ASSERT_TRUE(g.admit(1,.138,0.,.15));
  EXPECT_TRUE(g.permits(1,.149,0.,.15,true,false));
  EXPECT_FALSE(g.permits(1,.163,0.,.15,true,false));
  g.setPlan(goal(),2);EXPECT_FALSE(g.reentryRequired());
}
TEST(TerminalTrackDrift, ConfigurationAndGoalInputsFailClosed) {
  EXPECT_THROW(Guard(-.001),std::invalid_argument);
  EXPECT_THROW(Guard(.050001),std::invalid_argument);
  EXPECT_THROW(Guard(std::numeric_limits<double>::quiet_NaN()),std::invalid_argument);
  Guard g(.05);EXPECT_THROW(g.setPlan(goal(),0),std::invalid_argument);
  EXPECT_THROW(g.setPlan(goal(0.,0.,""),1),std::invalid_argument);
  EXPECT_THROW(g.setPlan(goal(std::numeric_limits<double>::infinity()),1),std::invalid_argument);
}
TEST(TerminalTrackDrift, RecordedNearEdgeAdmissionMayTrackButCannotFinishOutside) {
  Guard g(.05);g.setPlan(goal(),1);ASSERT_TRUE(g.admit(1,.13800616863322704,0.,.15));
  EXPECT_TRUE(g.permits(1,.163,0.,.15,true,false));
  // Last original checkpoint completes on this tick: extra Track permission ends now.
  EXPECT_FALSE(g.permits(1,.163,0.,.15,false,false));
  EXPECT_FALSE(g.permits(1,.163,0.,.15,true,true));
  EXPECT_TRUE(g.permits(1,.147,0.,.15,false,false));
}
TEST(TerminalTrackDrift, OnlyStrictActualOriginalRegionCanAdmitAndPivotIsImmutable) {
  Guard g(.05);g.setPlan(goal(),1);EXPECT_FALSE(g.admit(1,.151,0.,.15));
  ASSERT_TRUE(g.admit(1,.14,0.,.15));EXPECT_FALSE(g.admit(1,.17,0.,.15));
  EXPECT_TRUE(g.permits(1,.18,0.,.15,true,false));
  EXPECT_FALSE(g.permits(1,.191,0.,.15,true,false));
}
TEST(TerminalTrackDrift, EntireAccumulatedDisplacementNotPerTickStepIsBounded) {
  Guard g(.05);g.setPlan(goal(),1);ASSERT_TRUE(g.admit(1,.10,0.,.15));
  EXPECT_TRUE(g.permits(1,.08,0.,.15,true,false));
  EXPECT_TRUE(g.permits(1,.06,0.,.15,true,false));
  EXPECT_TRUE(g.permits(1,.04,0.,.15,true,false)); // baseline disc remains permitted
  EXPECT_FALSE(g.permits(1,.201,0.,.15,true,false));
  EXPECT_FALSE(g.permits(2,.10,0.,.15,true,false));
  EXPECT_FALSE(g.permits(1,.10,0.,.20,true,false)); // cannot change final geometry
}
TEST(TerminalTrackDrift, NonfiniteObservationCannotUseTrackPermission) {
  Guard g(.05);g.setPlan(goal(),1);ASSERT_TRUE(g.admit(1,.10,0.,.15));
  EXPECT_FALSE(g.permits(1,std::numeric_limits<double>::quiet_NaN(),0.,.15,true,false));
  EXPECT_FALSE(g.permits(1,.1,0.,std::numeric_limits<double>::infinity(),true,false));
}
TEST(TerminalTrackDrift, SameGoalReplanCannotReanchorOutsideOriginalRegion) {
  Guard g(.05);replan(g);EXPECT_FALSE(g.admit(2,.17,0.,.15));
  EXPECT_EQ(observe(g,0,true,.17),Reentry::Reject);
  EXPECT_EQ(g.observeReentry(.17,0.,.20,true,1000000000,2000000000),Reentry::Reject);
  EXPECT_TRUE(g.reentryRequired());
  g.setPlan(goal(),3);EXPECT_TRUE(g.reentryRequired());
  EXPECT_FALSE(g.admit(3,.14,0.,.15));
}
TEST(TerminalTrackDrift, ReentryNeedsActualCheckedZeroThenBothAdvancingDwellClocks) {
  Guard g(.05);replan(g);
  EXPECT_EQ(g.observeReentry(.14,0.,.15,true,1000000000,2000000000),Reentry::HoldZero);
  EXPECT_EQ(g.observeReentry(.14,0.,.15,true,2000000000,3000000000),Reentry::HoldZero);
  // Neither stopped samples nor elapsed time stand in for a successfully checked zero.
  EXPECT_EQ(observe(g,1100),Reentry::HoldZero);
  EXPECT_EQ(observe(g,1150),Reentry::HoldZero);
  EXPECT_EQ(observe(g,1449),Reentry::HoldZero);
  EXPECT_EQ(observe(g,1450),Reentry::Proceed);
  EXPECT_FALSE(g.reentryRequired());ASSERT_TRUE(g.admit(2,.14,0.,.15));
  EXPECT_FALSE(g.permits(2,.191,0.,.15,true,false));
}
TEST(TerminalTrackDrift, FrozenOrQueuedOriginalTimeCannotGrantReentry) {
  Guard g(.05);replan(g);observe(g,0);observe(g,50);
  EXPECT_EQ(observe(g,200,true,.14,50),Reentry::HoldZero);
  EXPECT_EQ(observe(g,350,true,.14,50),Reentry::HoldZero);
  EXPECT_EQ(observe(g,351,true,.14,5000),Reentry::Proceed); // both clocks now cover .3s
  Guard queued(.05);replan(queued);observe(queued,0);observe(queued,50);
  EXPECT_EQ(observe(queued,51,true,.14,5000),Reentry::HoldZero);
}
TEST(TerminalTrackDrift, MotionAndUnobservedPauseRestartReentryDwell) {
  Guard g(.05);replan(g);observe(g,0);observe(g,50);observe(g,200);
  EXPECT_EQ(observe(g,250,false),Reentry::HoldZero);
  EXPECT_EQ(observe(g,300),Reentry::HoldZero);
  EXPECT_EQ(observe(g,599),Reentry::HoldZero);
  EXPECT_EQ(observe(g,600),Reentry::Proceed);
  Guard paused(.05);replan(paused);observe(paused,0);observe(paused,50);
  EXPECT_EQ(observe(paused,400),Reentry::HoldZero);
  EXPECT_EQ(observe(paused,450),Reentry::HoldZero);
  EXPECT_EQ(observe(paused,699),Reentry::HoldZero);
  EXPECT_EQ(observe(paused,700),Reentry::Proceed);
}
TEST(TerminalTrackDrift, ResetCancelStyleInvalidationPreservesSameGoalObligation) {
  Guard g(.05);g.setPlan(goal(),1);ASSERT_TRUE(g.admit(1,.14,0.,.15));
  g.invalidate();g.invalidate();g.setPlan(goal(),2);EXPECT_TRUE(g.reentryRequired());
  EXPECT_FALSE(g.admit(2,.14,0.,.15));
  g.setPlan(goal(10.),3);EXPECT_FALSE(g.reentryRequired());
  EXPECT_FALSE(g.admit(3,.14,0.,.15)); // new goal still needs normal forward approach
  EXPECT_TRUE(g.admit(3,10.,0.,.15));
}
TEST(TerminalTrackDrift, UnadmittedApproachReplansDoNotRequireBeingAtTheGoal) {
  Guard g(.05);g.setPlan(goal(),1);g.setPlan(goal(),2);
  EXPECT_FALSE(g.reentryRequired());EXPECT_FALSE(g.admit(2,-1.,0.,.15));
  EXPECT_TRUE(g.admit(2,.1,0.,.15));
}

TEST(TerminalTrackDrift, ReentryRegionExcursionBreaksContinuousStoppedDwell) {
  Guard g(.05);replan(g);
  EXPECT_EQ(observe(g,0),Reentry::HoldZero);
  EXPECT_EQ(observe(g,50),Reentry::HoldZero);
  EXPECT_EQ(observe(g,200),Reentry::HoldZero);
  EXPECT_EQ(observe(g,250,true,.16),Reentry::Reject);
  EXPECT_EQ(observe(g,350),Reentry::HoldZero);
  EXPECT_EQ(observe(g,500),Reentry::HoldZero);
  EXPECT_EQ(observe(g,649),Reentry::HoldZero);
  EXPECT_EQ(observe(g,650),Reentry::Proceed);
}

TEST(TerminalTrackDrift, CenteredTurnsPreserveOriginalDiscWithoutGrantingOutsideCompletion) {
  Guard g(.05);g.setPlan(goal(),1);ASSERT_TRUE(g.admit(1,.015,0.,.15));
  EXPECT_TRUE(g.permits(1,.1078,0.,.15,true,false));
  EXPECT_TRUE(g.permits(1,.14,0.,.15,false,false));
  EXPECT_FALSE(g.permits(1,.16,0.,.15,true,false)); // outside rescue exceeds original pivot budget
  Guard edge(.05);edge.setPlan(goal(),1);ASSERT_TRUE(edge.admit(1,.138,0.,.15));
  EXPECT_TRUE(edge.permits(1,.18,0.,.15,true,false));
  EXPECT_FALSE(edge.permits(1,.191,0.,.15,true,false));
  EXPECT_FALSE(edge.permits(1,.18,0.,.15,false,false));
}
