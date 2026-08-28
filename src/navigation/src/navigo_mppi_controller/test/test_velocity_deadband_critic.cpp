#include <gtest/gtest.h>

#include <xtensor/xarray.hpp>

#include "navigo_mppi_controller/critics/velocity_deadband_critic.hpp"

TEST(VelocityDeadbandCritic, AllowsZeroAndExecutableVelocities)
{
  const xt::xarray<float> velocity = {0.0f, 0.02f, 0.05f, 0.08f, -0.03f, -0.10f};
  const auto cost = mppi::critics::executableDeadbandCost(velocity, 0.05f);

  EXPECT_FLOAT_EQ(cost(0), 0.0f);
  EXPECT_FLOAT_EQ(cost(1), 0.03f);
  EXPECT_FLOAT_EQ(cost(2), 0.0f);
  EXPECT_FLOAT_EQ(cost(3), 0.0f);
  EXPECT_FLOAT_EQ(cost(4), 0.02f);
  EXPECT_FLOAT_EQ(cost(5), 0.0f);
}

TEST(VelocityDeadbandCritic, TreatsNumericalZeroAsStopped)
{
  const xt::xarray<float> velocity = {1.0e-8f, -1.0e-8f};
  const auto cost = mppi::critics::executableDeadbandCost(velocity, 0.10f);

  EXPECT_FLOAT_EQ(cost(0), 0.0f);
  EXPECT_FLOAT_EQ(cost(1), 0.0f);
}
