// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "official_lattice_rpp_adapter/rotation_constraints.hpp"
using namespace official_lattice_rpp_adapter::backport;
static geometry_msgs::msg::PoseStamped at(double x,double y,double a) {
  geometry_msgs::msg::PoseStamped p;p.pose.position.x=x;p.pose.position.y=y;
  p.pose.orientation.z=std::sin(a/2.);p.pose.orientation.w=std::cos(a/2.);return p;
}
TEST(RotationConstraint, StopsAfterEntireMidPathRotation) {
  nav_msgs::msg::Path p;p.poses.push_back(at(-1.,0.,0.));
  for (int i=0;i<=32;++i) {p.poses.push_back(at(0.,0.,M_PI*i/32.));}
  p.poses.push_back(at(-.1,0.,M_PI));
  EXPECT_EQ(removePosesAfterFirstConstraint(p,true,.05),34u);
  ASSERT_EQ(p.poses.size(),34u);
  EXPECT_NEAR(std::abs(tf2::getYaw(p.poses.back().pose.orientation)),M_PI,1e-9);
}
TEST(RotationConstraint, FinalTurnKeepsOriginalGoal) {
  nav_msgs::msg::Path p;p.poses={at(0.,0.,0.),at(1.,0.,0.),at(1.,0.,1.92)};
  EXPECT_EQ(removePosesAfterFirstConstraint(p,true,.05),0u);
  ASSERT_EQ(p.poses.size(),3u);EXPECT_NEAR(tf2::getYaw(p.poses.back().pose.orientation),1.92,1e-9);
}
