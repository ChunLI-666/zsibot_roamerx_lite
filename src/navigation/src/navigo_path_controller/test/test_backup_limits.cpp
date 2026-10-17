#include <gtest/gtest.h>
#include "navigo_core/backup_limits.hpp"
#include "navigo_core/epoch_contract.hpp"
#include "navigo_costmap_2d/footprint_sweep.hpp"
namespace backup = navigo_core::backup;
TEST(BackupLimits, RejectsInvalidAndUnboundedRequests)
{
  EXPECT_TRUE(backup::valid(.3,.1,5000000001ULL,1));
  EXPECT_FALSE(backup::valid(.301,.1,2,1));
  EXPECT_FALSE(backup::valid(.2,.101,2,1));
  EXPECT_FALSE(backup::valid(.2,.1,1,1));
  EXPECT_FALSE(backup::valid(.2,.1,5000000002ULL,1));
  EXPECT_FALSE(backup::valid(NAN,.1,2,1));
  EXPECT_FALSE(backup::valid(.2,0.,2,1));
}
TEST(BackupLimits, RearwardProjectionAndBraking)
{
  EXPECT_NEAR(backup::progress(0,0,0,-.2,0),.2,1e-12);
  EXPECT_NEAR(backup::progress(0,0,M_PI/2,0,-.2),.2,1e-12);
  EXPECT_NEAR(backup::progress(0,0,0,0,.2),0.,1e-12);
  EXPECT_NEAR(backup::lateral(0,0,0,0,.2),.2,1e-12);
  EXPECT_EQ(backup::velocity(.1,0.),0.);
  EXPECT_LT(std::abs(backup::velocity(.1,.001)),.1);
  EXPECT_GT(backup::stopDistance(.1),.03);
}
TEST(BackupLimits, OnlyExplicitBoundedReverseAllowed)
{
  navigo_epoch_msgs::msg::NavigationToken token;
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x=-.05;
  EXPECT_FALSE(navigo_core::epoch::motionCommandValid(token,velocity,1));
  token.execution_kind=token.BACKUP;token.backup_max_speed=.1;token.execution_deadline_ns=100;
  EXPECT_TRUE(navigo_core::epoch::motionCommandValid(token,velocity,1));
  EXPECT_FALSE(navigo_core::epoch::motionCommandValid(token,velocity,100));
  velocity.angular.z=.01;
  EXPECT_FALSE(navigo_core::epoch::motionCommandValid(token,velocity,1));
  velocity.angular.z=0;velocity.linear.x=-.11;
  EXPECT_FALSE(navigo_core::epoch::motionCommandValid(token,velocity,1));
}
TEST(BackupSweep, FilledInteriorRearEndpointUnknownAndBoundary)
{
  navigo_costmap_2d::Costmap2D map(120,120,.05,-3.,-3.,0);
  std::vector<geometry_msgs::msg::Point> polygon(4);
  polygon[0].x=.4;polygon[0].y=.2;polygon[1].x=.4;polygon[1].y=-.2;
  polygon[2].x=-.4;polygon[2].y=-.2;polygon[3].x=-.4;polygon[3].y=.2;
  auto free=[&]{return navigo_costmap_2d::sweep::sweepFree(map,polygon,0,0,0,-.3,0,0);};
  EXPECT_TRUE(free());
  unsigned x,y;map.worldToMap(0,0,x,y);map.setCost(x,y,254);EXPECT_FALSE(free());
  map.setCost(x,y,0);map.worldToMap(-.69,0,x,y);map.setCost(x,y,255);EXPECT_FALSE(free());
  map.setCost(x,y,0);
  EXPECT_FALSE(navigo_costmap_2d::sweep::sweepFree(map,polygon,-2.7,0,0,-3.,0,0));
}

TEST(BackupLimits, SpeedCapIncludesReactionAndBrakingBeforeDistanceBound)
{
  for (double remaining: {.2,.1,.03,.005,.002,0.}) {
    const double v=std::abs(backup::velocity(.1,remaining));
    EXPECT_LE(v*.75+v*v/(2.*.2),std::max(0.,remaining-.005)+1e-12);
  }
}

TEST(BackupLimits, ReactionBudgetTracksCommandLifetime)
{
  EXPECT_NEAR(backup::reactionTime(300000000),.75,1e-12);
  EXPECT_NEAR(backup::reactionTime(500000000),.95,1e-12);
  EXPECT_LT(std::abs(backup::velocity(.1,.05,.95)),std::abs(backup::velocity(.1,.05,.75)));
}
