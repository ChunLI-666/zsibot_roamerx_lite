#pragma once
// Declaration-only fixture for selected exact upstream RPP methods. NOT the full plugin.
#include <cmath>
#include <algorithm>
#include <angles/angles.h>
#include <nav2_core/goal_checker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
namespace nav2_regulated_pure_pursuit_controller {
struct Parameters {
  bool use_rotate_to_heading=true;
  double rotate_to_heading_min_angle=0.785;
  double rotate_to_heading_angular_vel=0.5;
  double max_angular_accel=1.0;
};
class RegulatedPurePursuitController {
public:
  Parameters values;
  Parameters * params_=&values;
  double control_duration_=0.1;
  bool shouldRotateToPath(const geometry_msgs::msg::PoseStamped &,double &,double &);
  bool shouldRotateToGoalHeading(nav2_core::GoalChecker *,const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &,const geometry_msgs::msg::Twist &,const nav_msgs::msg::Path &);
  void rotateToHeading(double &,double &,const double &,const geometry_msgs::msg::Twist &);
};
}
