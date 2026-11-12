#pragma once
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav2_controller/plugins/feasible_path_handler.hpp>
#include <nav2_controller/plugins/simple_goal_checker.hpp>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

geometry_msgs::msg::PoseStamped pose(double x,double y,double angle) {
  geometry_msgs::msg::PoseStamped p; p.header.frame_id="probe_map";
  p.pose.position.x=x; p.pose.position.y=y;
  p.pose.orientation.w=std::cos(angle/2);p.pose.orientation.z=std::sin(angle/2);return p;
}
// Compatibility boundary: old generated FollowPath Goal, new official internal controller API.
// This adapter itself has no publishers or hardware access. Test runners may expose an isolated action.
class LegacyGoalBridge {
public:
  nav2_controller::FeasiblePathHandler handler;
  nav2_controller::SimpleGoalChecker checker;
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController controller;
  nav_msgs::msg::Path segment;
  bool active=false;
  void submit(const nav2_msgs::action::FollowPath::Goal & request) {
    if(request.path.poses.empty()) throw std::runtime_error("Empty path");
    if(!request.controller_id.empty() && request.controller_id!="RPP")throw std::runtime_error("Invalid controller id");
    if(!request.goal_checker_id.empty() && request.goal_checker_id!="Checker")throw std::runtime_error("Invalid goal checker id");
    if(!request.progress_checker_id.empty())throw std::runtime_error("Progress checker not implemented by probe");
    controller.reset();checker.reset();handler.setPlan(request.path);
    controller.newPathReceived(request.path);active=true;
  }
  geometry_msgs::msg::Twist step(const geometry_msgs::msg::PoseStamped & p,
    const geometry_msgs::msg::Twist & speed,bool & reached) {
    reached=false;if(!active)return geometry_msgs::msg::Twist();
    auto goal=handler.getTransformedGoal(p.header.stamp);
    auto [a,b]=handler.findPlanSegment(p);segment=handler.transformLocalPlan(a,b);
    auto cmd=controller.computeVelocityCommands(p,speed,&checker,segment,goal).twist;
    reached=checker.isGoalReached(p.pose,goal.pose,speed,segment);
    if(reached){active=false;return geometry_msgs::msg::Twist();}
    return cmd;
  }
  void cancel(){active=false;controller.reset();checker.reset();}
};
