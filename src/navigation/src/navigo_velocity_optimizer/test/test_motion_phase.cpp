// Copyright 2026 Zsibot navigation contributors. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "navigo_velocity_optimizer/motion_phase.hpp"
#include "navigo_velocity_optimizer/velocity_smoother.hpp"
namespace phase = navigo_velocity_optimizer::phase;
using Command = phase::Command;
using Twist = phase::Twist;
class Probe : public navigo_velocity_optimizer::VelocityOptimizer
{
public:
  Probe() {smoothing_frequency_ = 20.;}
  Twist step(uint8_t requested_phase, const Twist & requested, const Twist & current,
    uint8_t last_phase = Command::PHASE_UNSPECIFIED)
  {
    const auto target = phase::target(requested_phase, requested, current, current, last_phase);
    Twist result;
    result.linear.x = applyConstraints(current.linear.x, target.velocity.linear.x, 1.5, -.2, 1.);
    result.linear.y = applyConstraints(current.linear.y, target.velocity.linear.y, 1.5, -.2, 1.);
    result.angular.z = applyConstraints(current.angular.z, target.velocity.angular.z, 1.5, -.2, 1.);
    EXPECT_FALSE(phase::translation(result) && phase::rotation(result) &&
      requested_phase != Command::PHASE_TRACK &&
      !(target.braking && last_phase == Command::PHASE_TRACK));
    if (target.braking) {EXPECT_TRUE(phase::brakingOutput(current, result, last_phase));}
    return result;
  }
};
class MotionPhase : public testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};
TEST_F(MotionPhase, RotationTailReachesExactZeroBeforeTranslation)
{
  Probe node; Twist current, desired;
  current.angular.z = -.05; desired.linear.x = .1;
  bool saw_zero = false, started_translation = false;
  for (int i = 0; i < 20; ++i) {
    const auto next = node.step(Command::PHASE_TRANSLATE, desired, current);
    if (next.linear.x != 0.) {EXPECT_TRUE(saw_zero); started_translation = true;}
    if (phase::zero(next)) {saw_zero = true;}
    if (current.angular.z != 0.) {EXPECT_LE(std::abs(next.angular.z-current.angular.z), .010000000000001);}
    current = next;
  }
  EXPECT_TRUE(started_translation);
}
TEST_F(MotionPhase, TranslationTailReachesExactZeroBeforeRotation)
{
  Probe node; Twist current, desired; current.linear.x = .1; desired.angular.z = .1;
  bool saw_zero = false, started_rotation = false;
  for (int i = 0; i < 25; ++i) {
    const auto next = node.step(Command::PHASE_ROTATE, desired, current);
    if (next.angular.z != 0.) {EXPECT_TRUE(saw_zero); started_rotation = true;}
    if (phase::zero(next)) {saw_zero = true;}
    current = next;
  }
  EXPECT_TRUE(started_rotation);
}
TEST_F(MotionPhase, TinyResidualIsNotHiddenByTolerance)
{
  Probe node; Twist current, desired; current.angular.z = 1e-10; desired.linear.x = .1;
  const auto zero = node.step(Command::PHASE_TRANSLATE, desired, current);
  EXPECT_TRUE(phase::zero(zero));
  EXPECT_GT(node.step(Command::PHASE_TRANSLATE, desired, zero).linear.x, 0.);
}
TEST_F(MotionPhase, HoldDeceleratesInsteadOfSnapping)
{
  Probe node; Twist current; current.angular.z = .1;
  const auto next = node.step(Command::PHASE_HOLD, Twist(), current);
  EXPECT_NEAR(next.angular.z, .09, 1e-12);
  EXPECT_TRUE(phase::target(Command::PHASE_HOLD, Twist(), current, current).braking);
}
TEST_F(MotionPhase, IncompatibleRawPhaseRejected)
{
  Twist mixed; mixed.linear.x = .1; mixed.angular.z = .1;
  EXPECT_FALSE(phase::valid(Command::PHASE_TRANSLATE, mixed));
  EXPECT_FALSE(phase::valid(Command::PHASE_ROTATE, mixed));
  EXPECT_FALSE(phase::valid(Command::PHASE_HOLD, mixed));
  EXPECT_FALSE(phase::valid(99, Twist()));
  EXPECT_TRUE(phase::valid(Command::PHASE_UNSPECIFIED, mixed));
}
TEST_F(MotionPhase, FeedbackZeroDoesNotEraseLastRelayTail)
{
  Twist last, desired; last.angular.z = .02; desired.linear.x = .1;
  const auto target = phase::target(Command::PHASE_TRANSLATE, desired, Twist(), last);
  EXPECT_TRUE(target.braking); EXPECT_TRUE(phase::zero(target.velocity));
}

TEST_F(MotionPhase, PlannedCurvedTrackIsPreserved)
{
  Probe node; Twist current, desired;
  current.linear.x = desired.linear.x = .1;
  current.angular.z = .05; desired.angular.z = .03;
  EXPECT_TRUE(phase::valid(Command::PHASE_TRACK, desired));
  const auto next = node.step(Command::PHASE_TRACK, desired, current, Command::PHASE_TRACK);
  EXPECT_DOUBLE_EQ(next.linear.x, .1); EXPECT_NEAR(next.angular.z, .04, 1e-12);
}
TEST_F(MotionPhase, CurvedTrackBrakesBothAxesBeforePureRotation)
{
  Probe node; Twist current, desired;
  current.linear.x = .1; current.angular.z = .05; desired.angular.z = .1;
  uint8_t last_phase = Command::PHASE_TRACK;
  bool saw_zero = false, began_new_rotation = false;
  for (int i = 0; i < 25; ++i) {
    const auto target = phase::target(Command::PHASE_ROTATE, desired, current, current, last_phase);
    const auto next = node.step(Command::PHASE_ROTATE, desired, current, last_phase);
    if (!target.braking && next.angular.z > 0.) {EXPECT_TRUE(saw_zero); began_new_rotation = true;}
    if (phase::zero(next)) {saw_zero = true; last_phase = Command::PHASE_HOLD;}
    else if (!target.braking) {last_phase = Command::PHASE_ROTATE;}
    current = next;
  }
  EXPECT_TRUE(began_new_rotation);
}
TEST_F(MotionPhase, CoupledBrakingNeedsActualTrackOrigin)
{
  Twist previous, output; previous.linear.x=.1; previous.angular.z=.1;
  output.linear.x=.09; output.angular.z=.09;
  EXPECT_TRUE(phase::brakingOutput(previous, output, Command::PHASE_TRACK));
  EXPECT_FALSE(phase::brakingOutput(previous, output, Command::PHASE_ROTATE));
  output.angular.z=-.01;
  EXPECT_FALSE(phase::brakingOutput(previous, output, Command::PHASE_TRACK));
}
TEST_F(MotionPhase, EveryPhaseRejectsNonfiniteComponents)
{
  Twist value; value.linear.y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(phase::valid(Command::PHASE_UNSPECIFIED, value));
  EXPECT_FALSE(phase::valid(Command::PHASE_TRACK, value));
}

TEST_F(MotionPhase, NewTargetCannotCancelBrakingBeforeZero)
{
  phase::Transition state; Twist rotating, desired;
  rotating.angular.z=.05; desired.linear.x=.1;
  state.update(rotating, Command::PHASE_ROTATE, false);
  EXPECT_TRUE(state.select(Command::PHASE_HOLD, Twist(), rotating, rotating).braking);
  // Even resuming the old rotation must finish the previously requested stop.
  EXPECT_TRUE(state.select(Command::PHASE_ROTATE, rotating, rotating, rotating).braking);
  state.update(Twist(), Command::PHASE_HOLD, true);
  const auto next=state.select(Command::PHASE_TRACK, desired, Twist(), Twist());
  EXPECT_FALSE(next.braking); EXPECT_DOUBLE_EQ(next.velocity.linear.x,.1);
}
