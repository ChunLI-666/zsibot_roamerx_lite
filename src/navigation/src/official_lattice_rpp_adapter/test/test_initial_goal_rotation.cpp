// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "official_lattice_rpp_adapter/initial_goal_rotation.hpp"
#include "official_lattice_rpp_adapter/terminal_goal_binding.hpp"
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
