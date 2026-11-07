// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include <cmath>
#include <limits>
#include "gtest/gtest.h"
#include "robot_feasible_navigation/segment_coordinator.hpp"

using robot_feasible_navigation::SegmentCoordinator;
namespace
{
constexpr double pi = 3.14159265358979323846;
geometry_msgs::msg::Pose pose(double x, double y, double yaw)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x; p.position.y = y;
  p.orientation.z = std::sin(yaw * 0.5); p.orientation.w = std::cos(yaw * 0.5);
  return p;
}
nav_msgs::msg::Path path(std::initializer_list<geometry_msgs::msg::Pose> poses)
{
  nav_msgs::msg::Path p; p.header.frame_id = "map";
  for (const auto & value : poses) {
    geometry_msgs::msg::PoseStamped s; s.header = p.header; s.pose = value;
    p.poses.push_back(s);
  }
  return p;
}
void walk(SegmentCoordinator & coordinator, double from, double to, double yaw = 0.0)
{
  const double sign = to > from ? 1.0 : -1.0;
  for (double x = from; (to - x) * sign >= -1e-8; x += sign * .01) {
    coordinator.observe(pose(x, 0, yaw));
  }
}
}  // namespace

TEST(SegmentCoordinator, StraightSamplesAreOnePhase)
{
  SegmentCoordinator c;
  c.setPath(path({pose(0, 0, 0), pose(.1, 0, 0), pose(.2, 0, 0), pose(.2, 0, 0)}));
  ASSERT_EQ(c.size(), 1u);
  EXPECT_FALSE(c.rotating());
  walk(c, 0, .2);
  EXPECT_TRUE(c.geometryReached(pose(.2, 0, 0)));
}
TEST(SegmentCoordinator, IntermediateTurnSeparatesContinuousLegs)
{
  SegmentCoordinator c;
  c.setPath(path({pose(0, 0, 0), pose(.1, 0, 0), pose(.2, 0, 0), pose(.2, 0, pi/4),
    pose(.2, 0, pi/2), pose(.2, .1, pi/2), pose(.2, .2, pi/2)}));
  ASSERT_EQ(c.size(), 3u);
  EXPECT_FALSE(c.geometryReached(pose(0, 0, 0)));
  EXPECT_FALSE(c.orderedProgressReached(.01, .25));
}
TEST(SegmentCoordinator, ReturnToInitialXYDoesNotCompleteOutbound)
{
  nav_msgs::msg::Path p; p.header.frame_id = "map";
  for (int i = 0; i <= 200; ++i) {p.poses.push_back(path({pose(i * .01, 0, 0)}).poses[0]);}
  p.poses.push_back(path({pose(2, 0, pi/2)}).poses[0]);
  p.poses.push_back(path({pose(2, 0, pi)}).poses[0]);
  for (int i = 199; i >= 0; --i) {p.poses.push_back(path({pose(i * .01, 0, pi)}).poses[0]);}
  SegmentCoordinator c; c.setPath(p);
  ASSERT_EQ(c.size(), 4u);
  c.observe(pose(0, 0, pi));
  EXPECT_FALSE(c.finalSegment());
  EXPECT_FALSE(c.geometryReached(pose(0, 0, pi)));
  EXPECT_GT(c.remainingDistance(), 3.9);
  walk(c, 0, 2);
  EXPECT_TRUE(c.geometryReached(pose(2, 0, 0)));
  ASSERT_TRUE(c.advance());
  c.observe(pose(2, 0, 0)); c.observe(pose(2, 0, pi/2));
  EXPECT_TRUE(c.geometryReached(pose(2, 0, pi/2)));
  ASSERT_TRUE(c.advance());
  c.observe(pose(2, 0, pi/2)); c.observe(pose(2, 0, pi));
  ASSERT_TRUE(c.advance()); walk(c, 2, 0, pi);
  EXPECT_TRUE(c.finalSegment()); EXPECT_TRUE(c.geometryReached(pose(0, 0, pi)));
}
TEST(SegmentCoordinator, FullRevolutionIsNotShortestYawShortcut)
{
  SegmentCoordinator c;
  c.setPath(path({pose(0, 0, 0), pose(0, 0, pi/2), pose(0, 0, pi),
    pose(0, 0, -pi/2), pose(0, 0, 0)}));
  ASSERT_EQ(c.size(), 4u);
  c.observe(pose(0, 0, 0));
  EXPECT_FALSE(c.geometryReached(pose(0, 0, 0)));
  for (int i = 1; i <= 4; ++i) {
    c.observe(pose(0, 0, i * pi/2));
    EXPECT_TRUE(c.geometryReached(pose(0, 0, i * pi/2)));
    if (i < 4) {ASSERT_TRUE(c.advance());}
  }
}
TEST(SegmentCoordinator, WrongDirectionDoesNotSatisfyRotationProgress)
{
  SegmentCoordinator c; c.setPath(path({pose(0, 0, 0), pose(0, 0, pi/2)}));
  c.observe(pose(0, 0, 0)); c.observe(pose(0, 0, -pi/2));
  EXPECT_FALSE(c.geometryReached(pose(0, 0, pi/2)));
}
TEST(SegmentCoordinator, IntermediateGeometryDoesNotImplyStopped)
{
  SegmentCoordinator c;
  geometry_msgs::msg::Twist velocity; velocity.angular.z = .08;
  EXPECT_FALSE(c.stopped(velocity));
  velocity.angular.z = 0; EXPECT_TRUE(c.stopped(velocity));
  velocity.linear.y = std::numeric_limits<double>::quiet_NaN(); EXPECT_FALSE(c.stopped(velocity));
}
TEST(SegmentCoordinator, ReplanCompatibilityRequiresSameEndpointAndFrame)
{
  auto a = path({pose(0, 0, 0), pose(.2, 0, 0)});
  auto b = path({pose(.1, 0, 0), pose(.2, 0, 0)});
  EXPECT_TRUE(SegmentCoordinator::compatibleGoal(a, b));
  b.poses.back().pose.position.x += .01;
  EXPECT_FALSE(SegmentCoordinator::compatibleGoal(a, b));
  b = a; b.header.frame_id = "odom";
  EXPECT_FALSE(SegmentCoordinator::compatibleGoal(a, b));
}
TEST(SegmentCoordinator, QueuedPathOnlySkipsReachedLeadingRotation)
{
  SegmentCoordinator c;
  c.setPath(path({pose(0, 0, 0), pose(0, 0, pi/2), pose(0, .2, pi/2)}));
  c.skipReachedLeadingRotations(pose(0, 0, pi/2));
  EXPECT_EQ(c.index(), 1u);
  c.skipReachedLeadingRotations(pose(0, .2, pi/2));
  EXPECT_EQ(c.index(), 1u);  // No global nearest/translation skip.
}
TEST(SegmentCoordinator, InvalidPathDoesNotDestroyCurrentProgress)
{
  SegmentCoordinator c; c.setPath(path({pose(0, 0, 0), pose(.1, 0, 0)}));
  auto invalid = path({pose(0, 0, 0), pose(.1, 0, 0)});
  invalid.poses.back().pose.orientation.w = 0;
  EXPECT_THROW(c.setPath(invalid), std::invalid_argument);
  EXPECT_EQ(c.size(), 1u); EXPECT_FALSE(c.empty());
  c.clear(); EXPECT_TRUE(c.empty());
}
TEST(SegmentCoordinator, QueuedSubCellConnectorDoesNotReplayStaleStartYaw)
{
  SegmentCoordinator c;
  c.setPath(path({pose(.005, 0, 0), pose(0, 0, 0), pose(0, 0, pi/2),
    pose(0, .1, pi/2)}));
  c.skipReachedLeadingRotations(pose(0, 0, pi/2));
  EXPECT_EQ(c.index(), 2u);
  EXPECT_FALSE(c.rotating());
}
TEST(SegmentCoordinator, ReverseAndLateralAreRejectedAcrossDenseSamples)
{
  nav_msgs::msg::Path p; p.header.frame_id = "map";
  for (int i = 0; i < 30; ++i) {p.poses.push_back(path({pose(-i*.001, 0, 0)}).poses[0]);}
  SegmentCoordinator c; EXPECT_THROW(c.setPath(p), std::invalid_argument);
}
TEST(SegmentCoordinator, RotationEntryBrakesOnceRatherThanEveryAngularCycle)
{
  SegmentCoordinator c; c.setPath(path({pose(0, 0, 0), pose(0, 0, pi/2)}));
  geometry_msgs::msg::Twist velocity; velocity.linear.x = .1;
  EXPECT_FALSE(c.executionReady(velocity));
  velocity.linear.x = 0; EXPECT_TRUE(c.executionReady(velocity));
  velocity.angular.z = .1; EXPECT_TRUE(c.executionReady(velocity));
  c.clear(); c.setPath(path({pose(0, 0, 0), pose(0, 0, pi/2)}));
  EXPECT_FALSE(c.executionReady(velocity));  // Different/new task must first stop again.
}
TEST(SegmentCoordinator, CompatibleTrackReplanDoesNotRequireUnnecessaryStop)
{
  SegmentCoordinator c; auto p = path({pose(0, 0, 0), pose(.1, 0, 0)});
  c.setPath(p, false);
  geometry_msgs::msg::Twist velocity; velocity.linear.x = .1;
  EXPECT_TRUE(c.executionReady(velocity));
  c.setPath(p); EXPECT_FALSE(c.executionReady(velocity));
}
TEST(SegmentCoordinator, OdometryRequiresReceivedFreshNonFutureFiniteSample)
{
  SegmentCoordinator c; geometry_msgs::msg::Twist velocity;
  EXPECT_FALSE(c.validOdometry(0, 1000, velocity));
  EXPECT_FALSE(c.validOdometry(1000.01, 1000, velocity));
  EXPECT_FALSE(c.validOdometry(999.49, 1000, velocity));
  EXPECT_TRUE(c.validOdometry(999.5, 1000, velocity));
  velocity.angular.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(c.validOdometry(1000, 1000, velocity));
}
TEST(SegmentCoordinator, CostmapWaitHasClockIndependentCancellationAndDeadline)
{
  using Status = SegmentCoordinator::CostmapWaitStatus;
  // Decision is based on steady elapsed time, not ROS /clock progress.
  EXPECT_EQ(SegmentCoordinator::costmapWaitStatus(true, true, false, false, .1, .3),
    Status::Waiting);
  EXPECT_EQ(SegmentCoordinator::costmapWaitStatus(true, true, false, false, .3, .3),
    Status::TimedOut);
  EXPECT_EQ(SegmentCoordinator::costmapWaitStatus(true, true, true, false, .1, .3),
    Status::Cancelled);
  EXPECT_EQ(SegmentCoordinator::costmapWaitStatus(true, false, false, false, .1, .3),
    Status::Stopped);
  EXPECT_EQ(SegmentCoordinator::costmapWaitStatus(false, true, false, false, .1, .3),
    Status::Stopped);
  EXPECT_EQ(SegmentCoordinator::costmapWaitStatus(true, true, false, true, .1, .3),
    Status::Ready);
}
TEST(SegmentCoordinator, TerminalToleranceWindowAcceptsOnlyLocalRemainingRoute)
{
  SegmentCoordinator c;
  c.setPath(path({pose(0, 0, 0), pose(0, 0, pi/2), pose(0, .04, pi/2),
    pose(0, .04, 0)}));
  EXPECT_TRUE(c.terminalWindowEligible(pose(0, 0, 0), pose(0, .04, 0), .25));
  EXPECT_EQ(c.size(), 3u);  // Eligibility does not overwrite the immutable ordered path.
}
TEST(SegmentCoordinator, TerminalWindowCannotSkipMacroscopicOrLoopingRoute)
{
  nav_msgs::msg::Path p; p.header.frame_id = "map";
  for (int i = 0; i <= 100; ++i) {p.poses.push_back(path({pose(i*.01, 0, 0)}).poses[0]);}
  p.poses.push_back(path({pose(1, 0, pi/2)}).poses[0]);
  p.poses.push_back(path({pose(1, 0, pi)}).poses[0]);
  for (int i = 99; i >= 0; --i) {p.poses.push_back(path({pose(i*.01, 0, pi)}).poses[0]);}
  SegmentCoordinator c; c.setPath(p);
  EXPECT_FALSE(c.terminalWindowEligible(pose(0, 0, 0), pose(0, 0, pi), .25));
}
TEST(SegmentCoordinator, TerminalWindowChecksEveryRemainingPoseNotOnlyLength)
{
  SegmentCoordinator c;
  c.setPath(path({pose(.24, 0, 0), pose(.26, 0, 0)}));
  EXPECT_FALSE(c.terminalWindowEligible(pose(.24, 0, 0), pose(0, 0, 0), .25));
}
TEST(SegmentCoordinator, OriginalGoalGeometryUsesCurrentXYAndYawWithoutStatefulLatch)
{
  const auto goal = pose(0, 0, 0);
  EXPECT_TRUE(SegmentCoordinator::originalGoalGeometryReached(pose(.00972, 0, .1972), goal, .25, .25));
  EXPECT_FALSE(SegmentCoordinator::originalGoalGeometryReached(pose(.26, 0, 0), goal, .25, .25));
  EXPECT_FALSE(SegmentCoordinator::originalGoalGeometryReached(pose(0, 0, .26), goal, .25, .25));
}
TEST(SegmentCoordinator, FinalGeometryCoincidenceCannotSkipRemainingRoute)
{
  nav_msgs::msg::Path p; p.header.frame_id = "map";
  for (int i = 0; i <= 60; ++i) {p.poses.push_back(path({pose(i*.01, 0, 0)}).poses[0]);}
  p.poses.push_back(path({pose(.6, 0, pi/2)}).poses[0]);
  p.poses.push_back(path({pose(.6, 0, pi)}).poses[0]);
  for (int i = 59; i >= 0; --i) {p.poses.push_back(path({pose(i*.01, 0, pi)}).poses[0]);}
  p.poses.push_back(path({pose(0, 0, -pi/2)}).poses[0]);
  p.poses.push_back(path({pose(0, 0, 0)}).poses[0]);
  for (int i = 1; i <= 30; ++i) {p.poses.push_back(path({pose(i*.01, 0, 0)}).poses[0]);}
  SegmentCoordinator c; c.setPath(p); walk(c, 0, .3);
  const auto goal = pose(.3, 0, 0);
  ASSERT_TRUE(SegmentCoordinator::originalGoalGeometryReached(goal, goal, .05, .08));
  EXPECT_FALSE(c.terminalWindowEligible(goal, goal, .05));
  while (!c.finalSegment()) {
    for (const auto & sample : c.active().path.poses) {c.observe(sample.pose);}
    ASSERT_TRUE(c.advance());
  }
  walk(c, 0, .3);
  EXPECT_TRUE(c.terminalWindowEligible(goal, goal, .05));
}
TEST(SegmentCoordinator, TrackHandoffRespectsStricterFinalToleranceWithoutChangingRotationAnchor)
{
  SegmentCoordinator::Config config; config.xy_tolerance = .06; config.yaw_tolerance = .08;
  SegmentCoordinator c(config);
  nav_msgs::msg::Path p; p.header.frame_id = "map";
  for (int i = 0; i <= 20; ++i) {p.poses.push_back(path({pose(i*.01, 0, 0)}).poses[0]);}
  p.poses.push_back(path({pose(.2, 0, pi/2)}).poses[0]);
  c.setPath(p); walk(c, 0, .14);
  EXPECT_TRUE(c.geometryReached(pose(.141, 0, 0)));
  EXPECT_FALSE(c.geometryReached(pose(.141, 0, 0), .05));
  c.observe(pose(.151, 0, 0));
  EXPECT_TRUE(c.geometryReached(pose(.151, 0, 0), .05));
  EXPECT_FALSE(c.geometryReached(pose(.151, 0, 0), 0.0));
  ASSERT_TRUE(c.advance());
  EXPECT_DOUBLE_EQ(c.config().xy_tolerance, .06);
  EXPECT_TRUE(c.rotationAnchorReached(pose(.141, 0, 0)));
  c.observe(pose(.141, 0, 0)); c.observe(pose(.141, 0, pi/2));
  EXPECT_TRUE(c.geometryReached(pose(.141, 0, pi/2), .05));
}
TEST(SegmentCoordinator, TerminalWindowCountsOnlyUncompletedPartOfCurrentEdge)
{
  nav_msgs::msg::Path p; p.header.frame_id = "map";
  for (int i = 0; i <= 60; ++i) {p.poses.push_back(path({pose(i*.01, 0, 0)}).poses[0]);}
  SegmentCoordinator c; c.setPath(p); walk(c, 0, .55);
  c.observe(pose(.553, 0, 0));
  EXPECT_TRUE(c.terminalWindowEligible(pose(.553, 0, 0), pose(.6, 0, 0), .049));
  EXPECT_FALSE(c.terminalWindowEligible(pose(.549, 0, 0), pose(.6, 0, 0), .049));
  EXPECT_FALSE(c.terminalWindowEligible(pose(.553, .02, 0), pose(.6, 0, 0), .049));
}
TEST(SegmentCoordinator, TerminalWindowDoesNotAddLateralErrorToEuclideanGoalTolerance)
{
  nav_msgs::msg::Path p; p.header.frame_id = "map";
  for (int i = 0; i <= 60; ++i) {p.poses.push_back(path({pose(i*.01, 0, 0)}).poses[0]);}
  SegmentCoordinator c; c.setPath(p); walk(c, 0, .57);
  const auto actual = pose(.57, .03, .28);
  c.observe(actual);
  // XY error is sqrt(.03^2+.03^2), while remaining route is only .03m.
  // The explicit terminal rotation must be allowed to correct the .28rad yaw.
  EXPECT_TRUE(c.terminalWindowEligible(actual, pose(.6, 0, 0), .05));
  EXPECT_FALSE(c.terminalWindowEligible(pose(.57, .05, .28), pose(.6, 0, 0), .05));
}
TEST(SegmentCoordinator, UnstartedRotationRejectsStalePlanOriginWithoutRelaxingTolerance)
{
  SegmentCoordinator c;
  c.setPath(path({pose(2.504, 1.837, 0), pose(2.504, 1.837, .464)}));
  const auto actual = pose(2.63314383, 1.89879362, .47901458);
  EXPECT_FALSE(c.rotationAnchorReached(actual));
  EXPECT_TRUE(c.entryStopPending());
  // A new plan based on the actual stopped pose can replace an unstarted phase.
  c.setPath(path({actual, pose(actual.position.x, actual.position.y, .464)}));
  EXPECT_TRUE(c.rotationAnchorReached(actual));
  geometry_msgs::msg::Twist zero;
  EXPECT_TRUE(c.executionReady(zero));
  EXPECT_FALSE(c.entryStopPending());
}
TEST(SegmentCoordinator, InvalidDataWaitIsBoundedInSteadyTimeAndDoesNotMeanStopped)
{
  using Status = SegmentCoordinator::InputWaitStatus;
  EXPECT_EQ(SegmentCoordinator::inputWaitStatus(false, .0, .5), Status::Waiting);
  EXPECT_EQ(SegmentCoordinator::inputWaitStatus(false, .49, .5), Status::Waiting);
  EXPECT_EQ(SegmentCoordinator::inputWaitStatus(false, .5, .5), Status::TimedOut);
  EXPECT_EQ(SegmentCoordinator::inputWaitStatus(true, .49, .5), Status::Ready);
  SegmentCoordinator c; geometry_msgs::msg::Twist default_zero;
  EXPECT_FALSE(c.validOdometry(0, 1000, default_zero));
  // Server never calls executionReady on that invalid/default zero sample.
}
TEST(SegmentCoordinator, TrackEndpointBrakesBeforeYawCorrectionThenHandsToExplicitTurn)
{
  SegmentCoordinator c;
  c.setPath(path({pose(0, 0, pi/2), pose(0, .1, pi/2), pose(0, .2, pi/2),
    pose(0, .2, pi/2 + .4), pose(0, .2, pi/2 + .8)}));
  // Door trace: XY is 12mm from the endpoint but yaw residual is just over .03.
  const auto actual = pose(.00051, .188, pi/2 + .03044532);
  c.observe(actual);
  ASSERT_FALSE(c.rotating());
  EXPECT_TRUE(c.geometryReached(actual));
  geometry_msgs::msg::Twist velocity; velocity.linear.x = .05;
  EXPECT_FALSE(c.stopped(velocity));  // Server must publish zero before advance.
  velocity.linear.x = 0;
  ASSERT_TRUE(c.stopped(velocity));
  ASSERT_TRUE(c.advance());
  ASSERT_TRUE(c.rotating());
  EXPECT_TRUE(c.rotationAnchorReached(actual));
  c.observe(actual);
  EXPECT_FALSE(c.geometryReached(actual));
}
TEST(SegmentCoordinator, IncomingRotationUsesExactRequestYawAndRejectsOldSnapshot)
{
  SegmentCoordinator c;
  const auto exact_start = pose(.6888435, .7687101, 1.231);
  c.setPath(path({exact_start, pose(.6888435, .7687101, 1.1071487),
    pose(.7888435, .9687101, 1.1071487)}));
  EXPECT_TRUE(c.incomingRotationStartReached(exact_start));
  // Same origin does not make a snapshot captured during an earlier turn fresh.
  const auto actual = pose(.6888435, .7687101, pi/2);
  EXPECT_TRUE(c.rotationAnchorReached(actual));
  EXPECT_FALSE(c.incomingRotationStartReached(actual));
  c.setPath(path({actual, pose(.6888435, .7687101, 1.1071487),
    pose(.7888435, .9687101, 1.1071487)}));
  EXPECT_TRUE(c.incomingRotationStartReached(actual));
}
TEST(SegmentCoordinator, RotationProgressIncludesActualEntryYawOffset)
{
  for (double offset : {-.04, .04}) {
    SegmentCoordinator c;
    c.setPath(path({pose(0, 0, .4), pose(0, 0, .8), pose(0, 0, 1.2)}));
    c.observe(pose(0, 0, .4 + offset));
    c.observe(pose(0, 0, .8));
    EXPECT_FALSE(c.geometryReached(pose(0, 0, .8)));
    c.observe(pose(0, 0, 1.2));
    EXPECT_TRUE(c.geometryReached(pose(0, 0, 1.2)));
  }
}
TEST(SegmentCoordinator, FreshRotationWaitCoversPlanningBudgetButRemainsBounded)
{
  SegmentCoordinator c;
  EXPECT_DOUBLE_EQ(c.config().rotation_anchor_wait_timeout, 15.0);
  using Status = SegmentCoordinator::InputWaitStatus;
  EXPECT_EQ(SegmentCoordinator::inputWaitStatus(false, 5.0, 15.0), Status::Waiting);
  EXPECT_EQ(SegmentCoordinator::inputWaitStatus(false, 15.0, 15.0), Status::TimedOut);
}

TEST(SegmentCoordinator, OppositeRotationDecelerationRequestsZeroHold)
{
  using Decision = SegmentCoordinator::RotationCommandDecision;
  // RPP's .5rad/s^2 acceleration clamp at 20Hz retains the measured sign
  // while reducing an opposite .08rad/s rotation to .055rad/s.
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(.1, -.055, -.08), Decision::HoldZero);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(-.1, .055, .08), Decision::HoldZero);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(.1, -.08, -.08), Decision::HoldZero);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(-.1, .08, .08), Decision::HoldZero);
}

TEST(SegmentCoordinator, OppositeRotationAccelerationAndUnmeasuredDirectionAreRejected)
{
  using Decision = SegmentCoordinator::RotationCommandDecision;
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(.1, -.09, -.08), Decision::Reject);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(-.1, .09, .08), Decision::Reject);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(.1, -.055, .08), Decision::Reject);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(-.1, .055, -.08), Decision::Reject);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(.1, -.055, 0.), Decision::Reject);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(-.1, .055, 0.), Decision::Reject);
}

TEST(SegmentCoordinator, RotationTowardCurrentTargetAllowsOvershootCorrection)
{
  using Decision = SegmentCoordinator::RotationCommandDecision;
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(.1, .055, -.08), Decision::Allow);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(-.1, -.055, .08), Decision::Allow);
  // A positive planned turn may overshoot. The checked remaining sweep then
  // points backwards; the original turn sign must not lock out that correction.
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(-.04, -.025, .01), Decision::Allow);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(.04, .025, -.01), Decision::Allow);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(.1, 0., -.08), Decision::Allow);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(0., 0., 0.), Decision::Allow);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(0., .055, .08), Decision::HoldZero);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(0., .055, 0.), Decision::Reject);
}

TEST(SegmentCoordinator, RotationDecisionRejectsEveryNonfiniteInput)
{
  using Decision = SegmentCoordinator::RotationCommandDecision;
  for (double value : {std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()})
  {
    EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(value, .05, .08), Decision::Reject);
    EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(.1, value, .08), Decision::Reject);
    EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(.1, .05, value), Decision::Reject);
  }
}


TEST(SegmentCoordinator, RecordedAInitialTurnNearEndpointPublishesOnlyZero)
{
  using Decision = SegmentCoordinator::RotationCommandDecision;
  // Exact closed A setup Return failure: wrong-way .15 exceeds measured .0749.
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(
    .002680148, -.150000000, -.074945915), Decision::Reject);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(
    .002680148, -.150000000, -.074945915, .08), Decision::HoldZero);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(
    -.002680148, .150000000, .074945915, .08), Decision::HoldZero);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(.08, .15, .07, .08), Decision::HoldZero);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(-.08, -.15, -.07, .08), Decision::HoldZero);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(0., -.15, 0., .08), Decision::HoldZero);
}
TEST(SegmentCoordinator, EndpointSettlingPreservesOutsideToleranceDirectionProtection)
{
  using Decision = SegmentCoordinator::RotationCommandDecision;
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(.0800001, -.15, -.0749, .08), Decision::Reject);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(-.0800001, .15, .0749, .08), Decision::Reject);
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(.0800001, .15, -.0749, .08), Decision::Allow);
  for(double invalid:{-.01,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
    EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(.002, -.15, -.07, invalid), Decision::Reject);
  }
}
TEST(SegmentCoordinator, YawZeroHoldDoesNotWaiveDisplacedXYAnchorOrOrderedProgress)
{
  SegmentCoordinator::Config cfg;cfg.xy_tolerance=.15;cfg.yaw_tolerance=.08;
  SegmentCoordinator c(cfg);
  const double target=1.1071487177940904,start=target+pi/2.;
  c.setPath(path({pose(-9.915,-38.861,start),pose(-9.915,-38.861,target),
    pose(-9.815,-38.661,target)}),false);
  c.observe(pose(-9.915,-38.861,start));
  const auto actual=pose(-10.02336,-38.71349,target-.002680148);c.observe(actual);
  ASSERT_TRUE(c.rotating());EXPECT_TRUE(c.orderedProgressReached(.3,.08));
  EXPECT_FALSE(c.rotationAnchorReached(actual));EXPECT_FALSE(c.geometryReached(actual));
  EXPECT_EQ(SegmentCoordinator::rotationCommandDecision(
    .002680148,-.15,-.074945915,.08),SegmentCoordinator::RotationCommandDecision::HoldZero);
  geometry_msgs::msg::Twist stopped;EXPECT_TRUE(c.stopped(stopped));
  EXPECT_FALSE(c.geometryReached(actual));EXPECT_EQ(c.index(),0u);
}
TEST(SegmentCoordinator, EndpointOnlyAdvancesAfterUnchangedGeometryAndMeasuredStop)
{
  SegmentCoordinator::Config cfg;cfg.xy_tolerance=.15;cfg.yaw_tolerance=.08;
  SegmentCoordinator c(cfg);c.setPath(path({pose(0.,0.,.4),pose(0.,0.,.8),pose(.1,.1,.8)}),false);
  c.observe(pose(0.,0.,.4));const auto actual=pose(.02,.03,.797319852);c.observe(actual);
  ASSERT_TRUE(c.geometryReached(actual));geometry_msgs::msg::Twist v;v.angular.z=-.074945915;
  EXPECT_FALSE(c.stopped(v));EXPECT_EQ(c.index(),0u);
  v.angular.z=0.;ASSERT_TRUE(c.stopped(v));ASSERT_TRUE(c.geometryReached(actual));
  EXPECT_TRUE(c.advance());EXPECT_EQ(c.index(),1u);EXPECT_FALSE(c.rotating());
}


TEST(SegmentCoordinator, ActualOriginControlPathPreservesOriginalOrderedAuthority)
{
  const double target=1.1071487177940904;
  auto original=path({pose(-9.915,-38.861,target+pi/2.),pose(-9.915,-38.861,target),
    pose(-9.815,-38.661,target)});
  auto actual=original.poses.front();actual.pose=pose(-10.024448854,-38.719629926,1.104765730);
  actual.header.stamp.sec=1794038316;actual.header.stamp.nanosec=150314331;
  const auto saved=original;
  const auto control=SegmentCoordinator::actualOriginRotationPath(actual,original.poses[1]);
  ASSERT_EQ(control.poses.size(),2u);
  EXPECT_EQ(control.header,actual.header);EXPECT_EQ(control.poses.front(),actual);
  EXPECT_EQ(control.poses.back().pose.position,actual.pose.position);
  EXPECT_EQ(control.poses.back().pose.orientation,original.poses[1].pose.orientation);
  EXPECT_EQ(original,saved);
  auto foreign=original.poses[1];foreign.header.frame_id="odom";
  EXPECT_THROW(SegmentCoordinator::actualOriginRotationPath(actual,foreign),std::invalid_argument);
}
TEST(SegmentCoordinator, StoppedOutsideAnchorRequiresFailureReplanWithoutAdvancing)
{
  SegmentCoordinator::Config cfg;cfg.xy_tolerance=.15;cfg.yaw_tolerance=.08;
  SegmentCoordinator c(cfg);const double target=1.1071487177940904;
  c.setPath(path({pose(-9.915,-38.861,target+pi/2.),pose(-9.915,-38.861,target),
    pose(-9.815,-38.661,target)}),false);
  c.observe(pose(-9.915,-38.861,target+pi/2.));
  const auto actual=pose(-10.024448854,-38.719629926,1.104765730);c.observe(actual);
  geometry_msgs::msg::Twist measured;measured.angular.z=-.075103827;
  EXPECT_FALSE(c.stoppedRotationNeedsReplan(actual,measured));
  measured.angular.z=0.;EXPECT_TRUE(c.stoppedRotationNeedsReplan(actual,measured));
  EXPECT_EQ(c.index(),0u);EXPECT_FALSE(c.geometryReached(actual));
  EXPECT_FALSE(c.stoppedRotationNeedsReplan(pose(-9.915,-38.861,target),measured));
  EXPECT_FALSE(c.stoppedRotationNeedsReplan(pose(-10.024448854,-38.719629926,target-.09),measured));
}

#include "robot_feasible_navigation/original_goal_stop_dwell.hpp"
using FinalDwell = robot_feasible_navigation::OriginalGoalStopDwell;
namespace {
constexpr std::int64_t dwell_s=1000000000,dwell_o=1000000000000000000,dwell_step=50000000;
bool dwell_tick(FinalDwell & d,int i,bool geometry=true,bool stopped=true,bool zero=true) {
  return d.observe(dwell_s+i*dwell_step,dwell_o+i*dwell_step,
    dwell_o+i*dwell_step,geometry,stopped,zero);
}
}
TEST(OriginalGoalStopDwell, MovingInsideOriginalGeometryNeverReady) {
  FinalDwell d;for(int i=0;i<20;++i) {EXPECT_FALSE(dwell_tick(d,i,true,false));}
}
TEST(OriginalGoalStopDwell, FirstZeroIsBaselineAndBothClocksNeed300ms) {
  FinalDwell d;for(int i=0;i<7;++i) {EXPECT_FALSE(dwell_tick(d,i));}
  EXPECT_TRUE(dwell_tick(d,7));  // .3s after the first NEW post-zero observation.
}
TEST(OriginalGoalStopDwell, MeasuredPolicyMatchesPilotAndProductionSegmentDefaultStaysSeparate) {
  EXPECT_TRUE(FinalDwell::measuredStopped(.03,0.,.05));
  EXPECT_TRUE(FinalDwell::measuredStopped(0.,-.03,-.05));
  EXPECT_FALSE(FinalDwell::measuredStopped(.03,.03,0.));
  EXPECT_FALSE(FinalDwell::measuredStopped(0.,0.,std::nextafter(.05,1.)));
  EXPECT_FALSE(FinalDwell::measuredStopped(std::numeric_limits<double>::quiet_NaN(),0.,0.));
  SegmentCoordinator c;geometry_msgs::msg::Twist v;v.linear.x=.02;v.angular.z=.04;
  EXPECT_FALSE(c.stopped(v));  // Production Config is still .01/.01.
  EXPECT_TRUE(FinalDwell::measuredStopped(v.linear.x,v.linear.y,v.angular.z));
}
TEST(OriginalGoalStopDwell, SourceClockSpanCannotBeReplacedBySteadyTime) {
  FinalDwell d;
  for(int i=0;i<13;++i) {EXPECT_FALSE(d.observe(dwell_s+i*dwell_step,
    dwell_o+i*dwell_step/2,dwell_o+i*dwell_step,true,true,true));}
  EXPECT_TRUE(d.observe(dwell_s+13*dwell_step,dwell_o+13*dwell_step/2,
    dwell_o+13*dwell_step,true,true,true));
}
TEST(OriginalGoalStopDwell, SteadySpanCannotBeReplacedBySourceClock) {
  FinalDwell d;
  for(int i=0;i<13;++i) {EXPECT_FALSE(d.observe(dwell_s+i*dwell_step/2,
    dwell_o+i*dwell_step,dwell_o+i*dwell_step,true,true,true));}
  EXPECT_TRUE(d.observe(dwell_s+13*dwell_step/2,dwell_o+13*dwell_step,
    dwell_o+13*dwell_step,true,true,true));
}
TEST(OriginalGoalStopDwell, PreZeroEpochsDoNotCountEvenWhenAdvancing) {
  FinalDwell d;
  for(int i=0;i<11;++i) {EXPECT_FALSE(d.observe(dwell_s+i*dwell_step,
    dwell_o-4*dwell_step+i*dwell_step,dwell_o+i*dwell_step,true,true,true));}
  EXPECT_TRUE(d.observe(dwell_s+11*dwell_step,dwell_o+7*dwell_step,
    dwell_o+11*dwell_step,true,true,true));
}
TEST(OriginalGoalStopDwell, DuplicateOriginalOdomNeverMatures) {
  FinalDwell d;
  for(int i=0;i<20;++i) {EXPECT_FALSE(d.observe(dwell_s+i*dwell_step,
    dwell_o,dwell_o+i*dwell_step,true,true,true));}
}
TEST(OriginalGoalStopDwell, RegressionInvalidatesEntireHold) {
  FinalDwell d;for(int i=0;i<6;++i) {EXPECT_FALSE(dwell_tick(d,i));}
  EXPECT_FALSE(d.observe(dwell_s+6*dwell_step,dwell_o+4*dwell_step,
    dwell_o+6*dwell_step,true,true,true));EXPECT_FALSE(d.active());
}
TEST(OriginalGoalStopDwell, ControlPauseBeyondFreshnessCannotBeBridged) {
  FinalDwell d;for(int i=0;i<6;++i) {EXPECT_FALSE(dwell_tick(d,i));}
  EXPECT_FALSE(d.observe(dwell_s+6*dwell_step+300000001,
    dwell_o+6*dwell_step+300000001,dwell_o+6*dwell_step+300000001,true,true,true));
  EXPECT_FALSE(d.active());
}
TEST(OriginalGoalStopDwell, SourceEpochGapBeyondFreshnessCannotBeBridged) {
  FinalDwell d;for(int i=0;i<6;++i) {EXPECT_FALSE(dwell_tick(d,i));}
  EXPECT_FALSE(d.observe(dwell_s+6*dwell_step,dwell_o+6*dwell_step+300000001,
    dwell_o+6*dwell_step+300000001,true,true,true));
}
TEST(OriginalGoalStopDwell, GeometryOrCheckedZeroLossResets) {
  for(bool lose_geometry:{false,true}) {
    FinalDwell d;for(int i=0;i<6;++i) {EXPECT_FALSE(dwell_tick(d,i));}
    EXPECT_FALSE(dwell_tick(d,6,!lose_geometry,true,lose_geometry));EXPECT_FALSE(d.active());
    for(int i=7;i<14;++i) {EXPECT_FALSE(dwell_tick(d,i));}
    EXPECT_TRUE(dwell_tick(d,14));
  }
}
TEST(OriginalGoalStopDwell, OneMovingSampleBreaksContinuousStop) {
  FinalDwell d;for(int i=0;i<6;++i) {EXPECT_FALSE(dwell_tick(d,i));}
  EXPECT_FALSE(dwell_tick(d,6,true,false));
  for(int i=7;i<13;++i) {EXPECT_FALSE(dwell_tick(d,i));}
  EXPECT_TRUE(dwell_tick(d,13));
}
TEST(OriginalGoalStopDwell, NewPlanCancelAndResetCannotReuseQualifiedHistory) {
  FinalDwell d;for(int i=0;i<7;++i) {EXPECT_FALSE(dwell_tick(d,i));}
  d.reset();EXPECT_FALSE(d.active());
  for(int i=7;i<14;++i) {EXPECT_FALSE(dwell_tick(d,i));}
  EXPECT_TRUE(dwell_tick(d,14));
}
