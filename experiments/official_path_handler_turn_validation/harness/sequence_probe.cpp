#include <cmath>
#include <iostream>
#include <stdexcept>
#include <nav2_controller/plugins/feasible_path_handler.hpp>
#include <nav2_controller/plugins/simple_goal_checker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

geometry_msgs::msg::PoseStamped makePose(double x,double y,double degrees) {
  geometry_msgs::msg::PoseStamped p; p.header.frame_id="map";
  p.pose.position.x=x; p.pose.position.y=y;
  p.pose.orientation.z=std::sin(degrees*M_PI/360);
  p.pose.orientation.w=std::cos(degrees*M_PI/360); return p;
}
class Probe : public nav2_controller::FeasiblePathHandler {
public:
  Probe(double threshold=0.785) {
    tf_=std::make_shared<nav2::TransformBuffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    costmap_ros_=std::make_shared<nav2_costmap_2d::Costmap2DROS>(tf_);
    transform_tolerance_=0; prune_distance_=2; max_robot_pose_search_dist_=4;
    reject_unit_path_=false; enforce_path_rotation_=true; enforce_path_inversion_=false;
    inversion_xy_tolerance_=0.01; inversion_yaw_tolerance_=0.05;
    minimum_rotation_angle_=threshold;
  }
  nav_msgs::msg::Path step(const geometry_msgs::msg::PoseStamped & p) {
    auto [a,b]=findPlanSegment(p); return transformLocalPlan(a,b);
  }
  unsigned constraint() const { return constraint_locale_; }
};
int main(int argc, char **) {
  // No ROS context/node or DDS. Exact upstream methods run with a native costmap wrapper fixture.
  if (argc>1) {
    for(int sample:{1,5,15,30}) for(double threshold:{0.5,5.,10.,45.}) {
      nav_msgs::msg::Path matrix_path; matrix_path.header.frame_id="map";
      matrix_path.poses={makePose(0,0,0),makePose(1,0,0)};
      for(int a=sample;a<=90;a+=sample) matrix_path.poses.push_back(makePose(1,0,a));
      matrix_path.poses.push_back(makePose(1,1,90));
      Probe candidate(threshold*M_PI/180); candidate.setPlan(matrix_path);
      int first=-1;
      for(int a=0;a<=90 && first<0;++a) for(int repeat=0;repeat<2 && first<0;++repeat) {
        auto segment=candidate.step(makePose(1,0,a));
        for(const auto & point:segment.poses) if(point.pose.position.y>0.5) first=a;
      }
      std::cout<<"{\"sample_deg\":"<<sample<<",\"threshold_deg\":"<<threshold
        <<",\"first_translation_visible_yaw_deg\":"<<first
        <<",\"remaining_yaw_deg\":"<<(first<0?-1:90-first)<<"}\n";
    }
    return 0;
  }
  nav_msgs::msg::Path path; path.header.frame_id="map";
  path.poses={makePose(0,0,0),makePose(1,0,0),makePose(1,0,30),
    makePose(1,0,60),makePose(1,0,90),makePose(1,1,90)};
  Probe handler; handler.setPlan(path);
  bool exposed_early=false;
  for (double yaw : {0.,30.,30.,60.,90.}) {
    auto segment=handler.step(makePose(1,0,yaw));
    bool future=false; for(const auto & p:segment.poses) future |= p.pose.position.y>0.5;
    exposed_early |= future && yaw<89.0;
    std::cout<<"{\"test\":\"rotation_sequence\",\"robot_yaw_deg\":"<<yaw
      <<",\"segment_end_yaw_deg\":"<<tf2::getYaw(segment.poses.back().pose.orientation)*180/M_PI
      <<",\"future_translation_visible\":"<<future
      <<",\"remaining_constraint\":"<<handler.constraint()<<"}\n";
  }
  nav2_controller::SimpleGoalChecker checker;
  geometry_msgs::msg::Twist speed;
  nav_msgs::msg::Path pure_rotation; pure_rotation.header.frame_id="map";
  pure_rotation.poses={makePose(1,0,0),makePose(1,0,30)};
  bool mid_xy=checker.isGoalXYReached(makePose(1,0,0).pose,makePose(1,1,90).pose,speed,pure_rotation);
  checker.reset();
  bool end_xy=checker.isGoalXYReached(makePose(1,1,0).pose,makePose(1,1,90).pose,speed,pure_rotation);
  checker.reset();
  bool end_wrong_yaw=checker.isGoalReached(makePose(1,1,0).pose,makePose(1,1,90).pose,speed,pure_rotation);
  checker.reset();
  bool end_right_yaw=checker.isGoalReached(makePose(1,1,90).pose,makePose(1,1,90).pose,speed,pure_rotation);
  std::cout<<"{\"test\":\"goal_contract\",\"mid_rotation_xy_reached\":"<<mid_xy
    <<",\"terminal_xy_reached\":"<<end_xy<<",\"terminal_wrong_yaw_success\":"<<end_wrong_yaw
    <<",\"terminal_right_yaw_success\":"<<end_right_yaw<<"}\n";
  std::cout<<"{\"requirement\":\"withhold_translation_until_full_90deg\",\"satisfied_in_fixture\":"<<!exposed_early<<"}\n";
  if (mid_xy || !end_xy || end_wrong_yaw || !end_right_yaw) throw std::runtime_error("goal API semantics changed");
  return exposed_early ? 2 : 0;
}
