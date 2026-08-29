#include <limits>

#include "gtest/gtest.h"

#include "robot_navigo/motion_limits.hpp"

namespace robot_navigo::motion_limits
{

TEST(MotionLimits, RejectsSubMinimumAndInvalidCommands)
{
  EXPECT_DOUBLE_EQ(executableOrZero(0.049, kExecutableMinVx), 0.0);
  EXPECT_DOUBLE_EQ(executableOrZero(-0.099, kExecutableMinVy), 0.0);
  EXPECT_DOUBLE_EQ(executableOrZero(0.019, kExecutableMinWz), 0.0);
  EXPECT_DOUBLE_EQ(
    executableOrZero(std::numeric_limits<double>::quiet_NaN(), kExecutableMinVx), 0.0);
}

TEST(MotionLimits, KeepsZeroAndExecutableCommands)
{
  EXPECT_DOUBLE_EQ(executableOrZero(0.0, kExecutableMinVx), 0.0);
  EXPECT_DOUBLE_EQ(executableOrZero(0.05, kExecutableMinVx), 0.05);
  EXPECT_DOUBLE_EQ(executableOrZero(-0.10, kExecutableMinVy), -0.10);
  EXPECT_DOUBLE_EQ(executableOrZero(0.02, kExecutableMinWz), 0.02);
}

}  // namespace robot_navigo::motion_limits
