// Copyright 2026. SPDX-License-Identifier: Apache-2.0
#pragma once

#include <mutex>
#include <string>
#include "nav2_smac_planner/smac_planner_lattice.hpp"
#include "robot_feasible_navigation/footprint_sweep.hpp"

namespace robot_feasible_navigation
{
// Optional software candidate. The official search is private and unchanged
// except the recorded incoming-edge safety/cost patches. Exact goal adapters
// use the same filled-footprint sweep as those edges.
class SafeLatticePlanner : public robot_feasible_navigation_smac::SmacPlannerLattice
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

  static void validateControlSet(const nlohmann::json & controls, double resolution);
  static void restoreSearchStartRoundoff(
    geometry_msgs::msg::PoseStamped & searched_start,
    const geometry_msgs::msg::PoseStamped & exact_start,
    nav2_costmap_2d::Costmap2D & map);
  static void restoreSearchRoundoff(
    nav_msgs::msg::Path & searched,
    const geometry_msgs::msg::PoseStamped & exact_start,
    const geometry_msgs::msg::PoseStamped & exact_goal,
    nav2_costmap_2d::Costmap2D & map);
  static void appendCheckedSegment(
    nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & target,
    nav2_costmap_2d::Costmap2D & map, const sweep::Footprint & footprint,
    const std::function<bool()> & cancel_checker = {});

private:
  std::mutex request_mutex_;
  std::string control_file_;
  bool terminal_goal_binding_{false};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr safe_parameter_guard_;
};
}  // namespace robot_feasible_navigation
