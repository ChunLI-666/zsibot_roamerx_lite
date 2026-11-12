#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <nav2_controller/plugins/feasible_path_handler.hpp>
#include <nav2_util/path_utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw) {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x; p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}
nav_msgs::msg::Path path(std::initializer_list<geometry_msgs::msg::PoseStamped> poses) {
  nav_msgs::msg::Path p; p.header.frame_id = "map"; p.poses = poses; return p;
}
class Probe : public nav2_controller::FeasiblePathHandler {
public:
  Probe(bool rotation = true, double threshold = 0.785) {
    enforce_path_rotation_ = rotation; enforce_path_inversion_ = false;
    minimum_rotation_angle_ = rotation ? threshold : 0.0;
    inversion_xy_tolerance_ = 0.01; inversion_yaw_tolerance_ = 0.05;
    reject_unit_path_ = false; max_robot_pose_search_dist_ = 4.;
    // Native same-frame fixture: no TF thread and no waiting are required.
    prune_distance_ = 2.; transform_tolerance_ = 0.0;
    tf_ = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  }
  unsigned constraint() const { return constraint_locale_; }
  size_t prefixSize() const { return global_plan_up_to_constraint_.poses.size(); }
  double prefixYaw() const { return tf2::getYaw(global_plan_up_to_constraint_.poses.back().pose.orientation); }
  bool within(double x, double y, double yaw) { return isWithinInversionTolerances(pose(x,y,yaw)); }
};
void check(bool result, const char * name) {
  if (!result) { throw std::runtime_error(name); }
  std::cout << "PASS " << name << '\n';
}
int main() {
  // No ROS context, node, DDS, device, ControllerServer, or robot process is started.
  const double pi = std::acos(-1.0);
  auto straight = path({pose(0,0,0),pose(1,0,0),pose(2,0,0)});
  Probe a; a.setPlan(straight);
  check(a.constraint() == 0 && a.prefixSize() == 3, "straight_not_segmented");

  auto turn = path({pose(0,0,0),pose(1,0,0),pose(1,0,pi/6),
      pose(1,0,pi/3),pose(1,0,pi/2),pose(1,1,pi/2)});
  Probe b; b.setPlan(turn);
  check(b.constraint() > 0 && b.prefixSize() < turn.poses.size(), "rotation_crops_future_translation");
  check(!b.within(0,0,b.prefixYaw()), "constraint_requires_position");
  check(!b.within(1,0,0), "constraint_requires_yaw_progress");
  check(b.within(1,0,b.prefixYaw()), "official_prefix_endpoint_is_releasable");
  std::cout << "OBS prefix_yaw_deg=" << b.prefixYaw()*180/pi
            << " requested_turn_deg=90 constraint_index=" << b.constraint() << '\n';
  // Record the exact snapshot's boundary: threshold truncation is not a promise
  // that the entire 90 degree segment has been completed before releasing it.
  check(b.prefixYaw() < pi/2 - .1, "snapshot_releases_before_entire_rotation_end");

  Probe disabled(false); disabled.setPlan(turn);
  check(disabled.constraint() == 0 && disabled.prefixSize() == turn.poses.size(), "rotation_flag_defaults_matter");

  auto wrap = path({pose(0,0,170*pi/180),pose(0,0,-170*pi/180),
      pose(0,0,-110*pi/180),pose(-1,0,-110*pi/180)});
  Probe c; c.setPlan(wrap);
  check(c.constraint()>0, "wraparound_rotation_detected");

  auto two = path({pose(0,0,0),pose(0,0,pi)});
  Probe d; d.setPlan(two);
  check(d.constraint()==0, "two_pose_rotation_not_cropped_in_this_snapshot");

  auto one = path({pose(0,0,0)}); Probe e; e.setPlan(one);
  check(e.constraint()==0, "single_pose_not_a_rotation_segment");

  auto segments = b.findPlanSegment(pose(0,0,0));
  check(segments.first->pose.position.x == 0, "same_frame_closest_segment_native_method");
  std::cout << "SCOPE exact upstream source native component tests with explicit lifecycle adapter; no plugin activation or closed loop\n";
  return 0;
}
