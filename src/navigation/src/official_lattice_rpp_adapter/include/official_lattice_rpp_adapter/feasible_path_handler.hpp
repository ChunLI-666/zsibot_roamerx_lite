// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Dexory
// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_CONTROLLER__PLUGINS__FEASIBLE_PATH_HANDLER_HPP_
#define NAV2_CONTROLLER__PLUGINS__FEASIBLE_PATH_HANDLER_HPP_

#include "official_lattice_rpp_adapter/terminal_goal_binding.hpp"

#include <string>
#include <vector>
#include <memory>
#include <utility>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/path.hpp>


namespace official_lattice_rpp_adapter::backport
{
/**
* @class FeasiblePathHandler
* @brief This plugin manages the global plan by clipping it to the local
* segment, typically bounded by the local costmap size
* and transforming the resulting path into the odom frame.
*/

using PathIterator = std::vector<geometry_msgs::msg::PoseStamped>::iterator;
using PathSegment = std::pair<PathIterator, PathIterator>;
class FeasiblePathHandler
{
public:
  /**
   * @brief Initialize parameters
   * @param parent Lifecycle node pointer
   * @param logger Node logging interface
   * @param plugin_name Name of the plugin
   * @param costmap_ros Costmap2DROS object
   * @param tf Shared ptr of TF2 buffer
   */
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const rclcpp::Logger & logger,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    std::shared_ptr<tf2_ros::Buffer> tf);

  /**
   * @brief Set new reference plan
   * @param Path Path to use
   */
  void setPlan(const nav_msgs::msg::Path & path);

  /**
   * @brief Determines the portion of the global plan to be used for local control.
   * This function locates the start and end iterators of the global plan segment
   * that is relevant for controller computation based on the robot's current pose and local costmap.
   * @param pose Robot pose in odom frame
   * @return PathSegment A pair of iterators defining the start and end of the
   *         selected plan segment.
   */
  PathSegment findPlanSegment(
    const geometry_msgs::msg::PoseStamped & pose);

  /**
    * @brief Transforms a predefined segment of the global plan into the costmap global frame.
    * @param closest_point Iterator to the starting pose of the path segment.
    * @param pruned_plan_end Iterator to the ending pose of the path segment.
    * @return nav_msgs::msg::Path The transformed local plan segment in the costmap global frame.
    */
  nav_msgs::msg::Path transformLocalPlan(
    const PathIterator & closest_point,
    const PathIterator & pruned_plan_end);

  /**
   * @brief Get the global goal pose transformed to the costmap global frame
   * @param stamp Time to get the goal pose at
   * @return Transformed goal pose
   */
  geometry_msgs::msg::PoseStamped getTransformedGoal(
    const builtin_interfaces::msg::Time & stamp);

  /**
   * @brief Gets the global plan
   * @return The global plan
   */
  nav_msgs::msg::Path getPlan() {return global_plan_;}

  // Jazzy's server has no path-aware goal checker. Expose ordered progress;
  // do not search later path intersections again when deciding completion.
  bool finalPrefixWithin(double remaining_length) const;
  void enableOrderedRotations(bool enabled) {ordered_rotations_=enabled;}
  const goal_region::HeadingCursor & constraintHeadings() const {return constraint_headings_;}
  bool constraintRotationActive() const {return ordered_rotations_&&constraint_locale_!=0u&&constraint_near_anchor_;}
  bool finalPrefixActive() const {return constraint_locale_==0u;}
  struct DiagnosticState {
    std::size_t global_offset,prefix_offset,constraint_locale,constraint_heading_start;
    std::size_t remaining_plan_poses,current_prefix_poses,heading_next,heading_count;
    bool ordered_rotations,near_anchor,rotation_active,final_prefix;
  };
  DiagnosticState diagnosticState() const {
    return {global_offset_,prefix_offset_,constraint_locale_,constraint_heading_start_,
      global_plan_.poses.size(),global_plan_up_to_constraint_.poses.size(),
      constraint_headings_.next(),constraint_headings_.headings().size(),
      ordered_rotations_,constraint_near_anchor_,constraintRotationActive(),finalPrefixActive()};
  }
  bool terminalHandoffReady(std::size_t suffix_begin,double limit) const {
    return prefix_offset_>=suffix_begin && finalPrefixWithin(limit);
  }
  // Current ordered final TRACK, not a later repeated occurrence of this XY.
  bool terminalTrackActive(std::size_t suffix_begin,std::size_t rotation_begin) const {
    return finalPrefixActive() && prefix_offset_>=suffix_begin && prefix_offset_<rotation_begin;
  }

protected:
  void refreshConstraintHeadings();
  /**
    * @brief Transform a pose to the global reference frame
    * @param pose Current pose
    * @return output poose in global reference frame
    */
  geometry_msgs::msg::PoseStamped transformToGlobalPlanFrame(
    const geometry_msgs::msg::PoseStamped & pose);

  /**
   * Get the greatest extent of the costmap in meters from the center.
   * @return max of distance from center in meters to edge of costmap
   */
  double getCostmapMaxExtent() const;

  /**
    * @brief Check if the robot pose is within the set inversion tolerances
    * @param robot_pose Robot's current pose to check
    * @return bool If the robot pose is within the set inversion tolerances
    */
  bool isWithinInversionTolerances(const geometry_msgs::msg::PoseStamped & robot_pose);

  /**
    * @brief Prune a path to only interesting portions
    * @param plan Plan to prune
    * @param end Final path iterator
    */
  void prunePlan(nav_msgs::msg::Path & plan, const PathIterator end);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  rclcpp::Logger logger_ {rclcpp::get_logger("FeasiblePathHandler")};
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
  nav_msgs::msg::Path global_plan_up_to_constraint_;
  geometry_msgs::msg::PoseStamped global_pose_;
  unsigned int constraint_locale_{0u};
  std::size_t global_offset_{0u},prefix_offset_{0u};
  bool ordered_rotations_=false;
  bool constraint_near_anchor_=false;
  std::size_t constraint_heading_start_=0;
  goal_region::HeadingCursor constraint_headings_;
  bool reject_unit_path_, enforce_path_inversion_, enforce_path_rotation_;
  double max_robot_pose_search_dist_, transform_tolerance_, prune_distance_;
  float inversion_xy_tolerance_, inversion_yaw_tolerance_, minimum_rotation_angle_;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};
}  // namespace official_lattice_rpp_adapter::backport

#endif  // NAV2_CONTROLLER__PLUGINS__FEASIBLE_PATH_HANDLER_HPP_
