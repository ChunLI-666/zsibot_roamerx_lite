#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "robot_navigo/livox_scan_projection.hpp"

namespace robot_navigo
{

constexpr double kPi = 3.14159265358979323846;

TEST(LivoxScanProjection, KeepsNearestPointAndCountsOneFiniteBin)
{
  ScanProjectionConfig config;
  config.min_height = -0.1;
  config.max_height = 0.5;
  config.range_min = 0.1;
  config.exclude_robot_footprint = false;
  config.angle_min = -kPi;
  config.angle_max = kPi;
  config.angle_increment = kPi / 2.0;
  auto ranges = makeEmptyScan(config);
  ScanProjectionStats stats;

  EXPECT_TRUE(projectPoint(2.0, 0.0, 0.2, config, ranges, stats));
  EXPECT_TRUE(projectPoint(1.0, 0.0, 0.2, config, ranges, stats));
  EXPECT_EQ(stats.points_used, 2U);
  EXPECT_EQ(stats.finite_bins, 1U);
  EXPECT_FLOAT_EQ(ranges[2], 1.0F);
}

TEST(LivoxScanProjection, AppliesPitchBeforeHeightFilter)
{
  ScanProjectionConfig config;
  config.min_height = 0.4;
  config.max_height = 0.6;
  config.range_min = 0.1;
  config.exclude_robot_footprint = false;
  config.rotation = rotationFromRpy(0.0, kPi / 2.0, 0.0);
  auto ranges = makeEmptyScan(config);
  ScanProjectionStats stats;

  EXPECT_TRUE(projectPoint(-0.5, 1.0, 0.0, config, ranges, stats));
  EXPECT_EQ(stats.points_used, 1U);
}

TEST(LivoxScanProjection, RejectsInvalidHeightRangeAndNan)
{
  ScanProjectionConfig config;
  config.min_height = 0.1;
  config.max_height = 0.3;
  config.range_min = 0.1;
  config.exclude_robot_footprint = false;
  auto ranges = makeEmptyScan(config);
  ScanProjectionStats stats;

  EXPECT_FALSE(projectPoint(1.0, 0.0, 0.5, config, ranges, stats));
  EXPECT_FALSE(projectPoint(
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.2, config, ranges, stats));
  EXPECT_EQ(stats.points_used, 0U);
  EXPECT_EQ(stats.finite_bins, 0U);
}

TEST(LivoxScanProjection, RemovesRobotBodyButKeepsNearExternalObstacle)
{
  ScanProjectionConfig config;
  config.min_height = 0.0;
  config.max_height = 0.5;
  config.range_min = 0.1;
  auto ranges = makeEmptyScan(config);
  ScanProjectionStats stats;

  EXPECT_FALSE(projectPoint(-0.30, 0.0, 0.2, config, ranges, stats));
  EXPECT_TRUE(projectPoint(-0.40, 0.0, 0.2, config, ranges, stats));
  EXPECT_EQ(stats.points_used, 1U);
}

}  // namespace robot_navigo
