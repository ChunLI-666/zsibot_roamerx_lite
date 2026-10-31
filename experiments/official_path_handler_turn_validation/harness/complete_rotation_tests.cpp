#include <cmath>
#include <iostream>
#include <stdexcept>
#include <nav2_util/path_utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>
geometry_msgs::msg::PoseStamped p(double x,double y,double degrees) {
  geometry_msgs::msg::PoseStamped v;v.pose.position.x=x;v.pose.position.y=y;
  v.pose.orientation.w=std::cos(degrees*M_PI/360);v.pose.orientation.z=std::sin(degrees*M_PI/360);return v;
}
nav_msgs::msg::Path path(std::initializer_list<geometry_msgs::msg::PoseStamped> v) {
  nav_msgs::msg::Path result;result.poses=v;return result;
}
void check(bool value,const char * name){if(!value)throw std::runtime_error(name);std::cout<<"PASS "<<name<<'\n';}
int main() {
  auto turn=path({p(0,0,0),p(1,0,0),p(1,0,30),p(1,0,60),p(1,0,90),p(1,1,90)});
  auto cut=nav2_util::removePosesAfterFirstConstraint(turn,false,.785);
  check(cut==5 && turn.poses.size()==5 && std::abs(tf2::getYaw(turn.poses.back().pose.orientation)-M_PI/2)<1e-5,
    "90deg_run_keeps_final_rotation_pose_and_excludes_translation");
  auto two_turns=path({p(0,0,0),p(0,0,90),p(1,0,90),p(1,0,180),p(1,1,180)});
  check(nav2_util::removePosesAfterFirstConstraint(two_turns,false,.785)==2 && two_turns.poses.size()==2,
    "first_rotation_does_not_skip_to_second_rotation");
  auto wrap=path({p(0,0,170),p(0,0,-170),p(0,0,-110),p(-1,0,-110)});
  check(nav2_util::removePosesAfterFirstConstraint(wrap,false,.785)==3,"wraparound_rotation_preserved");
  auto small=path({p(0,0,0),p(0,0,10),p(1,0,10)});
  check(nav2_util::removePosesAfterFirstConstraint(small,false,.785)==0,"subthreshold_turn_not_forced");
  auto straight=path({p(0,0,0),p(1,0,0),p(2,0,0)});
  check(nav2_util::removePosesAfterFirstConstraint(straight,false,.785)==0,"translation_path_unchanged");
  auto reverse=path({p(0,0,0),p(1,0,0),p(0,0,0)});
  check(nav2_util::removePosesAfterFirstConstraint(reverse,true,0)==2,"existing_inversion_behavior_retained");
  auto terminal=path({p(0,0,0),p(0,0,90),p(0,0,180)});
  check(nav2_util::removePosesAfterFirstConstraint(terminal,false,.785)==0 && terminal.poses.size()==3,
    "terminal_rotation_remains_visible_to_goal_checker");
}
