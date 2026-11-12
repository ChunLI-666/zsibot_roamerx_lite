// Copyright 2026. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <thread>
#include "robot_feasible_navigation/footprint_sweep.hpp"

namespace rs = robot_feasible_navigation::sweep;

static rs::Footprint body()
{
  rs::Footprint result(4);
  result[0].x = 0.335; result[0].y = 0.185;
  result[1].x = -0.335; result[1].y = 0.185;
  result[2].x = -0.335; result[2].y = -0.185;
  result[3].x = 0.335; result[3].y = -0.185;
  return result;
}

static void block(nav2_costmap_2d::Costmap2D & map, double x, double y, unsigned char cost = 254)
{
  unsigned int mx = 0, my = 0;
  ASSERT_TRUE(map.worldToMap(x, y, mx, my));
  map.setCost(mx, my, cost);
}

TEST(FilledFootprint, InteriorObstacleAndUnknownAreBlocked)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, 0.0, 0.0, 0);
  EXPECT_TRUE(rs::footprintFree(map, body(), 1.0, 1.0, 0.0));
  block(map, 1.0, 1.0);
  EXPECT_FALSE(rs::footprintFree(map, body(), 1.0, 1.0, 0.0));
  block(map, 1.0, 1.0, 255);
  EXPECT_FALSE(rs::footprintFree(map, body(), 1.0, 1.0, 0.0));
}

TEST(FilledFootprint, CommandArcChecksMiddleAndRejectsInvalidVelocity)
{
  nav2_costmap_2d::Costmap2D map(400, 400, 0.01, -2.0, -2.0, 0);
  EXPECT_TRUE(rs::commandSweepFree(map, body(), 0.0, 0.0, 0.0, 1.0, M_PI_2, 1.0));
  block(map, 0.4, 0.08);
  EXPECT_TRUE(rs::footprintFree(map, body(), 0.0, 0.0, 0.0));
  EXPECT_TRUE(rs::footprintFree(map, body(), 2.0 / M_PI, 2.0 / M_PI, M_PI_2));
  EXPECT_FALSE(rs::commandSweepFree(map, body(), 0.0, 0.0, 0.0, 1.0, M_PI_2, 1.0));
  EXPECT_FALSE(rs::commandSweepFree(map, body(), 0.0, 0.0, 0.0,
    std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0));
  EXPECT_FALSE(rs::commandSweepFree(map, body(), 0.0, 0.0, 0.0, 0.1, 0.0, 0.0));
}

TEST(FilledFootprint, RotationMiddleCollisionDespiteClearEndpoints)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, 0.0, 0.0, 0);
  block(map, 1.22, 1.22);
  EXPECT_TRUE(rs::footprintFree(map, body(), 1.0, 1.0, 0.0));
  EXPECT_TRUE(rs::footprintFree(map, body(), 1.0, 1.0, M_PI_2));
  EXPECT_FALSE(rs::footprintFree(map, body(), 1.0, 1.0, M_PI_4));
  EXPECT_FALSE(rs::sweepFree(map, body(), 1.0, 1.0, 0.0, 1.0, 1.0, M_PI_2));
  EXPECT_FALSE(rs::sweepFree(map, body(), 1.0, 1.0, M_PI_2, 1.0, 1.0, 0.0));
}

TEST(FilledFootprint, TranslationMiddleCollisionDespiteClearEndpoints)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, 0.0, 0.0, 0);
  block(map, 1.0, 1.0);
  EXPECT_TRUE(rs::footprintFree(map, body(), 0.6, 1.0, 0.0));
  EXPECT_TRUE(rs::footprintFree(map, body(), 1.4, 1.0, 0.0));
  EXPECT_FALSE(rs::sweepFree(map, body(), 0.6, 1.0, 0.0, 1.4, 1.0, 0.0));
}

TEST(FilledFootprint, SatDoesNotBlockAnEmptyBoundingBoxCorner)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, 0.0, 0.0, 0);
  block(map, 1.30, 1.30);
  EXPECT_TRUE(rs::footprintFree(map, body(), 1.0, 1.0, M_PI_4));
  auto reversed = body();
  std::reverse(reversed.begin(), reversed.end());
  EXPECT_TRUE(rs::footprintFree(map, reversed, 1.0, 1.0, M_PI_4));
}

TEST(FilledFootprint, RejectsMapBoundaryAndInvalidInputs)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, 0.0, 0.0, 0);
  EXPECT_FALSE(rs::footprintFree(map, body(), 0.2, 0.2, 0.0));
  EXPECT_FALSE(rs::footprintFree(map, body(), 1.0, 1.0,
    std::numeric_limits<double>::quiet_NaN()));
  auto invalid = body();
  invalid[1] = invalid[0];
  EXPECT_FALSE(rs::sweepFree(map, invalid, 1.0, 1.0, 0.0, 1.0, 1.0, 0.5));
  EXPECT_FALSE(rs::sweepFree(map, body(), 1.0, 1.0, 0.0, 1.0, 1.0, 0.5, 0.0));
  EXPECT_FALSE(rs::sweepFree(map, body(), 1.0, 1.0, 0.0, 1.0, 1.0, 0.5,
    std::numeric_limits<double>::infinity()));
}

TEST(FilledFootprint, WrapAroundUsesTheShortContinuousRotation)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, 0.0, 0.0, 0);
  EXPECT_TRUE(rs::sweepFree(map, body(), 1.0, 1.0, M_PI - 0.1, 1.0, 1.0, -M_PI + 0.1));
}

TEST(FilledFootprint, ScopedIndexIsEquivalentForClearOccupiedUnknownAndMapEdges)
{
  // Partial tiles, nonzero origin, finite inflated costs and both blocked costs.
  nav2_costmap_2d::Costmap2D map(201, 199, 0.01, -0.31, -0.27, 0);
  struct Query {double x, y, yaw; bool expected;};
  for (int scenario = 0; scenario < 3; ++scenario) {
    if (scenario == 1) {
      block(map, 0.69, 0.73, nav2_costmap_2d::LETHAL_OBSTACLE);
      block(map, 0.99, 1.03, nav2_costmap_2d::NO_INFORMATION);
      block(map, 0.29, 0.33, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    } else if (scenario == 2) {
      for (unsigned int y = 1; y < map.getSizeInCellsY(); y += 23) {
        for (unsigned int x = 2; x < map.getSizeInCellsX(); x += 29) {
          map.setCost(x, y, (x + y) % 2 ? 254 : 255);
        }
      }
    }
    std::vector<Query> queries;
    for (int yi = -1; yi <= 9; ++yi) {
      for (int xi = -1; xi <= 9; ++xi) {
        for (int angle = 0; angle < 24; ++angle) {
          const double x = -0.31 + 0.2 * xi, y = -0.27 + 0.2 * yi;
          const double yaw = angle * 2.0 * M_PI / 24;
          queries.push_back({x, y, yaw, rs::footprintFree(map, body(), x, y, yaw)});
        }
      }
    }
    const rs::ScopedObstacleIndex index(map);
    for (const auto & q : queries) {
      EXPECT_EQ(q.expected, rs::footprintFree(map, body(), q.x, q.y, q.yaw)) <<
        "scenario=" << scenario << " x=" << q.x << " y=" << q.y << " yaw=" << q.yaw;
    }
    EXPECT_FALSE(rs::footprintFree(map, body(), 0.5, 0.5,
      std::numeric_limits<double>::quiet_NaN()));
    auto invalid = body();
    invalid[1] = invalid[0];
    EXPECT_FALSE(rs::footprintFree(map, invalid, 0.5, 0.5, 0.0));
  }
}

TEST(FilledFootprint, ScopedIndexKeepsOriginalSatAndSweepCollisionSemantics)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, 0.0, 0.0, 0);
  block(map, 1.30, 1.30);
  {
    const rs::ScopedObstacleIndex index(map);
    // This tile rectangle is occupied, but the original SAT accepts the corner.
    EXPECT_TRUE(rs::footprintFree(map, body(), 1.0, 1.0, M_PI_4));
  }
  block(map, 1.22, 1.22, nav2_costmap_2d::NO_INFORMATION);
  {
    const rs::ScopedObstacleIndex index(map);
    EXPECT_TRUE(rs::footprintFree(map, body(), 1.0, 1.0, 0.0));
    EXPECT_TRUE(rs::footprintFree(map, body(), 1.0, 1.0, M_PI_2));
    EXPECT_FALSE(rs::sweepFree(map, body(), 1.0, 1.0, 0.0, 1.0, 1.0, M_PI_2));
  }
}

TEST(FilledFootprint, ScopedIndexRebuildsAfterUpdatesAndDoesNotApplyToOtherMaps)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, 0.0, 0.0, 0);
  nav2_costmap_2d::Costmap2D other(200, 200, 0.01, 0.0, 0.0, 0);
  block(other, 1.0, 1.0);
  {
    const rs::ScopedObstacleIndex index(map);
    EXPECT_TRUE(rs::footprintFree(map, body(), 1.0, 1.0, 0.0));
    EXPECT_FALSE(rs::footprintFree(other, body(), 1.0, 1.0, 0.0));
    {
      const rs::ScopedObstacleIndex nested(other);
      EXPECT_FALSE(rs::footprintFree(other, body(), 1.0, 1.0, 0.0));
      EXPECT_TRUE(rs::footprintFree(map, body(), 1.0, 1.0, 0.0));
    }
    EXPECT_TRUE(rs::ScopedObstacleIndex::activeRectangleFree(map, 0, 0, 199, 199));
    bool other_thread_sees_index = true;
    std::thread worker([&]() {
        other_thread_sees_index = rs::ScopedObstacleIndex::activeRectangleFree(map, 0, 0, 199, 199);
      });
    worker.join();
    EXPECT_FALSE(other_thread_sees_index);
  }
  EXPECT_FALSE(rs::ScopedObstacleIndex::activeRectangleFree(map, 0, 0, 199, 199));
  block(map, 1.0, 1.0);
  {
    const rs::ScopedObstacleIndex rebuilt(map);
    EXPECT_FALSE(rs::footprintFree(map, body(), 1.0, 1.0, 0.0));
  }
  block(map, 1.0, 1.0, 0);
  {
    const rs::ScopedObstacleIndex rebuilt(map);
    EXPECT_TRUE(rs::footprintFree(map, body(), 1.0, 1.0, 0.0));
  }
}


TEST(FilledFootprint, SparseRowsExactlyRetainBlockedCellsInOriginalOrder)
{
  nav2_costmap_2d::Costmap2D map(43, 39, 0.01, -0.215, -0.195, 0);
  for (unsigned int y = 0; y < 39; ++y) {
    for (unsigned int x = 0; x < 43; ++x) {
      map.setCost(x, y, (x + 3 * y) % 7 == 0 ? 255 :
        (2 * x + y) % 11 == 0 ? 254 : (x + y) % 2 ? 253 : 252);
    }
  }
  rs::ScopedObstacleIndex index(map);
  for (unsigned int y = 0; y < 39; ++y) {
    std::vector<unsigned int> expected;
    for (unsigned int x = 0; x < 43; ++x) {
      if (map.getCost(x, y) >= nav2_costmap_2d::LETHAL_OBSTACLE) {expected.push_back(x);}
    }
    const auto * actual = rs::ScopedObstacleIndex::activeBlockedColumns(map, y);
    ASSERT_NE(actual, nullptr);
    EXPECT_EQ(*actual, expected);
  }
  EXPECT_EQ(rs::ScopedObstacleIndex::activeBlockedColumns(map, 39), nullptr);
  nav2_costmap_2d::Costmap2D foreign(43, 39, 0.01, -0.215, -0.195, 0);
  EXPECT_EQ(rs::ScopedObstacleIndex::activeBlockedColumns(foreign, 0), nullptr);
}

TEST(FilledFootprint, SparseSatMatchesUnindexedFilledPoseAndSweepPredicates)
{
  nav2_costmap_2d::Costmap2D map(403, 399, 0.01, -2.015, -1.995, 0);
  auto footprint = body();
  for (auto & p : footprint) {p.x = std::copysign(0.797, p.x); p.y = std::copysign(0.680, p.y);}
  struct Query {double x, y, yaw; bool pose, sweep;};
  for (unsigned int scenario = 0; scenario < 3; ++scenario) {
    map.resetMap(0, 0, 403, 399);
    if (scenario > 0) {
      for (unsigned int y = 7; y < 399; y += scenario == 1 ? 127 : 31) {
        for (unsigned int x = 3; x < 403; x += scenario == 1 ? 131 : 29) {
          map.setCost(x, y, (x + y) % 2 ? 254 : 255);
        }
      }
    }
    std::vector<Query> queries;
    for (unsigned int i = 0; i < 120; ++i) {
      const double x = (int((i * 37) % 4001) - 2000) / 1000.0;
      const double y = (int((i * 61) % 4001) - 2000) / 1000.0;
      const double yaw = (int((i * 97) % 6284) - 3142) / 1000.0;
      queries.push_back({x, y, yaw, rs::footprintFree(map, footprint, x, y, yaw),
        rs::sweepFree(map, footprint, x, y, yaw, x + 0.12, y - 0.08, yaw + 0.393)});
    }
    rs::ScopedObstacleIndex index(map);
    for (const auto & q : queries) {
      EXPECT_EQ(rs::footprintFree(map, footprint, q.x, q.y, q.yaw), q.pose);
      EXPECT_EQ(rs::sweepFree(map, footprint, q.x, q.y, q.yaw,
        q.x + 0.12, q.y - 0.08, q.yaw + 0.393), q.sweep);
    }
  }
}
