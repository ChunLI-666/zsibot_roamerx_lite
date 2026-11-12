// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <limits>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include "official_lattice_rpp_adapter/command_limits.hpp"
#include "robot_feasible_navigation/footprint_sweep.hpp"
namespace sweep=robot_feasible_navigation::sweep;
TEST(CommandLimits, CollisionChecksUseTheSameLimitedTrajectoryAsPublication) {
  nav2_costmap_2d::Costmap2D map(600,600,.01,-3.,-3.,0);
  unsigned int mx,my;ASSERT_TRUE(map.worldToMap(.955,.555,mx,my));map.setCost(mx,my,254);
  sweep::Footprint fp;for(const auto & xy:std::vector<std::pair<double,double>>{{-.797,-.680},{.797,-.680},{.797,.680},{-.797,.680}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;fp.push_back(p);
  }
  sweep::ScopedObstacleIndex index(map);geometry_msgs::msg::Twist raw;raw.linear.x=.4;raw.angular.z=-1.56640161;
  ASSERT_TRUE(sweep::commandSweepFree(map,fp,0.,0.,0.,raw.linear.x,raw.angular.z,1.));
  const auto sent=official_lattice_rpp_adapter::limitAngularCommand(raw,.3);
  EXPECT_FALSE(sweep::commandSweepFree(map,fp,0.,0.,0.,sent.linear.x,sent.angular.z,1.));
}
TEST(CommandLimits, InvalidRawCommandsCannotBeHiddenByLimiting) {
  geometry_msgs::msg::Twist raw;raw.linear.x=.4;raw.angular.z=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(official_lattice_rpp_adapter::limitAngularCommand(raw,.3),std::invalid_argument);
  raw.angular.z=.1;raw.linear.x=-.1;EXPECT_THROW(official_lattice_rpp_adapter::limitAngularCommand(raw,.3),std::invalid_argument);
  raw.linear.x=.4;raw.linear.y=.1;EXPECT_THROW(official_lattice_rpp_adapter::limitAngularCommand(raw,.3),std::invalid_argument);
}
