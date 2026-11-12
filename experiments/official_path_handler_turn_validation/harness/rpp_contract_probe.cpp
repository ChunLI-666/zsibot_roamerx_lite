#include <iostream>
#include <stdexcept>
#include "rpp_decision_fixture.hpp"
#include <nav2_controller/plugins/simple_goal_checker.hpp>
geometry_msgs::msg::PoseStamped p(double x,double y,double degrees) {
  geometry_msgs::msg::PoseStamped a; a.header.frame_id="map";
  a.pose.position.x=x; a.pose.position.y=y;
  a.pose.orientation.w=std::cos(degrees*M_PI/360); a.pose.orientation.z=std::sin(degrees*M_PI/360);
  return a;
}
int main() {
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController rpp;
  nav2_controller::SimpleGoalChecker checker;
  geometry_msgs::msg::Twist speed;
  nav_msgs::msg::Path segment; segment.header.frame_id="map";
  segment.poses={p(1,0,0),p(1,0,30)};
  double heading=0,sign=1;
  bool mid_goal=rpp.shouldRotateToGoalHeading(&checker,p(1,0,0),p(1,1,90),speed,segment);
  bool mid_path=rpp.shouldRotateToPath(p(0,0,30),heading,sign);
  checker.reset();
  bool final_goal=rpp.shouldRotateToGoalHeading(&checker,p(1,1,0),p(1,1,90),speed,segment);
  double linear=1,angular=0;
  rpp.rotateToHeading(linear,angular,M_PI/2,speed);
  std::cout<<"{\"mid_rotation_global_goal_branch\":"<<mid_goal
    <<",\"mid_rotation_zero_xy_carrot_branch\":"<<mid_path
    <<",\"final_rotation_branch\":"<<final_goal
    <<",\"rotation_linear_command\":"<<linear<<",\"rotation_angular_command\":"<<angular<<"}\n";
  if(!final_goal || linear!=0 || angular<=0) throw std::runtime_error("Unexpected terminal rotation behavior");
  // Return 2 for the uncovered intermediate rotation, not a build or process failure.
  return !mid_goal && !mid_path ? 2 : 0;
}
