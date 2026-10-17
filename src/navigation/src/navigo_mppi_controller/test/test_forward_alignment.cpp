// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include <gtest/gtest.h>
#include "navigo_mppi_controller/tools/forward_alignment.hpp"
#include "navigo_mppi_controller/optimizer.hpp"

using namespace mppi::forward_alignment;  // NOLINT

TEST(ForwardAlignment, HysteresisAndGoalDrift)
{
  EXPECT_EQ(nextMode(Mode::TRACK, 2., .8, .25, .05, .7, .2), Mode::ALIGN);
  EXPECT_EQ(nextMode(Mode::ALIGN, 2., .3, .25, .05, .7, .2), Mode::ALIGN);
  EXPECT_EQ(nextMode(Mode::TRACK, 2., .3, .25, .05, .7, .2), Mode::TRACK);
  EXPECT_EQ(nextMode(Mode::ALIGN, 2., .1, .25, .05, .7, .2), Mode::TRACK);
  EXPECT_EQ(nextMode(Mode::TRACK, .19, 2., .25, .05, .7, .2), Mode::FINAL);
  EXPECT_EQ(nextMode(Mode::FINAL, .24, 2., .25, .05, .7, .2), Mode::FINAL);
  EXPECT_EQ(nextMode(Mode::FINAL, .26, 2., .25, .05, .7, .2), Mode::ALIGN);
}

TEST(ForwardAlignment, HeadingIgnoresReversedOrientations)
{
  nav_msgs::msg::Path path;
  for (int i = 0; i <= 10; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = .1 * i;
    p.pose.orientation.z = 1.0;
    path.poses.push_back(p);
  }
  geometry_msgs::msg::Pose pose;
  EXPECT_NEAR(pathHeading(path, pose, .5), 0., 1e-9);
  pose.position.y = .5;
  EXPECT_LT(pathHeading(path, pose, .5), 0.);
  path.poses.clear();
  EXPECT_TRUE(std::isnan(pathHeading(path, pose, .5)));
}

class ConstraintOptimizer : public mppi::Optimizer
{
public:
  ConstraintOptimizer()
  {
    motion_model_ = std::make_shared<mppi::OmniMotionModel>();
    settings_.forward_alignment = true;
    settings_.minimum_forward_velocity = .05f;
    settings_.minimum_angular_velocity = .02f;
    settings_.constraints = {.15f, -.15f, .15f, .1f};
    settings_.base_constraints = settings_.constraints;
    settings_.batch_size = 100;
    settings_.time_steps = 5;
    settings_.model_dt = .1f;
    settings_.sampling_std = {.1f, .1f, .08f};
    control_sequence_.reset(5);
  }
  void checkSamplingAndReset()
  {
    reset();
    generateNoisedTrajectories();
    size_t moving = 0;
    for (const auto vx : state_.cvx) {
      EXPECT_TRUE(vx == 0.f || (vx >= .05f && vx <= .15f));
      if (vx > 0.f) {++moving;}
    }
    EXPECT_GT(moving, 0u);
    for (const auto vy : state_.cvy) {EXPECT_FLOAT_EQ(vy, 0.f);}
    for (const auto wz : state_.cwz) {
      EXPECT_TRUE(wz == 0.f || (std::abs(wz) >= .02f && std::abs(wz) <= .1f));
    }
    setSpeedLimit(50.0, true);
    reset();
    EXPECT_FLOAT_EQ(settings_.constraints.vx_max, .075f);
    EXPECT_FLOAT_EQ(settings_.constraints.wz, .05f);
    // Dynamic base limit changes must still apply while a speed zone is active.
    settings_.base_constraints.vx_max = .10f;
    reset();
    EXPECT_FLOAT_EQ(settings_.constraints.vx_max, .05f);
    setSpeedLimit(.05, false);
    settings_.base_constraints.vx_max = .30f;
    reset();
    EXPECT_NEAR(settings_.constraints.vx_max, .05f, 1e-7);
    EXPECT_NEAR(settings_.constraints.wz, .1f / 6.f, 1e-7);
    EXPECT_GE(settings_.constraints.vx_max, settings_.minimum_forward_velocity);
    for (const auto vx : control_sequence_.vx) {EXPECT_FLOAT_EQ(vx, .05f);}
    // A feasible seed does not falsify the initial measured velocity or history.
    EXPECT_DOUBLE_EQ(state_.speed.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(control_history_[0].vx, 0.0);
    generateNoisedTrajectories();
    for (const auto vx : state_.cvx) {EXPECT_TRUE(vx == 0.f || vx == .05f);}
    setSpeedLimit(10.0, true);
    reset();
    generateNoisedTrajectories();
    for (const auto vx : state_.cvx) {EXPECT_FLOAT_EQ(vx, 0.f);}
    for (const auto wz : state_.cwz) {EXPECT_FLOAT_EQ(wz, 0.f);}
  }
  void check()
  {
    control_sequence_.vx = {-.15f, .049f, .05f, .10f, .30f};
    control_sequence_.vy = {.15f, -.15f, .01f, 0.f, .1f};
    control_sequence_.wz = {-.009f, .019f, .02f, -.08f, .2f};
    applyControlSequenceConstraints();
    EXPECT_FLOAT_EQ(control_sequence_.vx(0), 0.f);
    EXPECT_FLOAT_EQ(control_sequence_.vx(1), .05f);
    EXPECT_FLOAT_EQ(control_sequence_.vx(2), .05f);
    EXPECT_FLOAT_EQ(control_sequence_.vx(4), .15f);
    EXPECT_FLOAT_EQ(control_sequence_.wz(0), 0.f);
    EXPECT_FLOAT_EQ(control_sequence_.wz(1), .02f);
    EXPECT_FLOAT_EQ(control_sequence_.wz(2), .02f);
    EXPECT_FLOAT_EQ(control_sequence_.wz(4), .1f);
    for (const auto vy : control_sequence_.vy) {EXPECT_FLOAT_EQ(vy, 0.f);}
    settings_.constraints.vx_max = .03f;
    settings_.constraints.wz = .01f;
    applyControlSequenceConstraints();
    for (const auto vx : control_sequence_.vx) {EXPECT_FLOAT_EQ(vx, 0.f);}
    for (const auto wz : control_sequence_.wz) {EXPECT_FLOAT_EQ(wz, 0.f);}
  }
};

TEST(ForwardAlignment, OptimizerEnforcesLimitsAndExecutableActions)
{
  ConstraintOptimizer optimizer;
  optimizer.check();
}

TEST(ForwardAlignment, SampledRolloutsAndResetRespectSpeedLimits)
{
  ConstraintOptimizer optimizer;
  optimizer.checkSamplingAndReset();
}

std::vector<geometry_msgs::msg::Point> footprint()
{
  std::vector<geometry_msgs::msg::Point> points(4);
  points[0].x = .4; points[0].y = .15;
  points[1].x = .4; points[1].y = -.15;
  points[2].x = -.4; points[2].y = -.15;
  points[3].x = -.4; points[3].y = .15;
  return points;
}

TEST(ForwardAlignment, SweptRotationDetectsIntermediateObstacle)
{
  navigo_costmap_2d::Costmap2D map(200, 200, .02, -2., -2., 0);
  unsigned int mx, my;
  map.worldToMap(.26, .26, mx, my);
  map.setCost(mx, my, navigo_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(footprintFree(map, footprint(), 0., 0., 0.));
  EXPECT_TRUE(footprintFree(map, footprint(), 0., 0., M_PI_2));
  EXPECT_FALSE(sweepFree(map, footprint(), 0., 0., 0., 0., 0., M_PI_2));
}

TEST(ForwardAlignment, RejectsInteriorUnknownAndBounds)
{
  navigo_costmap_2d::Costmap2D map(200, 200, .02, -2., -2., 0);
  EXPECT_TRUE(sweepFree(map, footprint(), 0., 0., 0., 0., 0., M_PI));
  map.setCost(100, 100, navigo_costmap_2d::NO_INFORMATION);
  EXPECT_FALSE(footprintFree(map, footprint(), 0., 0., 0.));
  EXPECT_FALSE(footprintFree(map, footprint(), 1.9, 0., 0.));
  EXPECT_FALSE(footprintFree(map, {}, 0., 0., 0.));
  EXPECT_FALSE(sweepFree(map, footprint(), 0., 0., 0., 0., 0., NAN));
}

TEST(ForwardAlignment, SweptTranslationDetectsThinObstacle)
{
  navigo_costmap_2d::Costmap2D map(200, 200, .02, -2., -2., 0);
  map.setCost(100, 100, navigo_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(footprintFree(map, footprint(), -.8, 0., 0.));
  EXPECT_TRUE(footprintFree(map, footprint(), .8, 0., 0.));
  EXPECT_FALSE(sweepFree(map, footprint(), -.8, 0., 0., .8, 0., 0.));
}
