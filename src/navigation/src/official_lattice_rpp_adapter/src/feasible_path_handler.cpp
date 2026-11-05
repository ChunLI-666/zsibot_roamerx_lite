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

#include <memory>
#include "angles/angles.h"
#include "pluginlib/class_list_macros.hpp"
#include "official_lattice_rpp_adapter/feasible_path_handler.hpp"
#include "official_lattice_rpp_adapter/rotation_constraints.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_core/controller_exceptions.hpp"


using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace official_lattice_rpp_adapter::backport
{
using nav2_util::geometry_utils::euclidean_distance;

// Mechanical Jazzy lifecycle parameter compatibility; no new Controller ABI.
template<class T> T declareOrGet(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
 const std::string & key, T value) {
 if (!node->has_parameter(key)) {node->declare_parameter<T>(key, value);}
 return node->get_parameter(key).get_value<T>();
}


void FeasiblePathHandler::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const rclcpp::Logger & logger,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  std::shared_ptr<tf2_ros::Buffer> tf)
{
  logger_ = logger;
  plugin_name_ = plugin_name;
  auto node = parent.lock();
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  transform_tolerance_ = costmap_ros_->getTransformTolerance();
  reject_unit_path_ = declareOrGet(node,
    plugin_name + ".reject_unit_path", false);
  max_robot_pose_search_dist_ = declareOrGet(node,
    plugin_name + ".max_robot_pose_search_dist", getCostmapMaxExtent());
  prune_distance_ = declareOrGet(node,
    plugin_name + ".prune_distance", 2.0);
  enforce_path_inversion_ = declareOrGet(node,
    plugin_name + ".enforce_path_inversion", false);
  enforce_path_rotation_ = declareOrGet(node,
    plugin_name + ".enforce_path_rotation", false);
  inversion_xy_tolerance_ = declareOrGet(node,
    plugin_name + ".inversion_xy_tolerance", 0.2);
  inversion_yaw_tolerance_ = declareOrGet(node,
    plugin_name + ".inversion_yaw_tolerance", 0.4);
  minimum_rotation_angle_ = declareOrGet(node,
    plugin_name + ".minimum_rotation_angle", 0.785);
  if (max_robot_pose_search_dist_ < 0.0) {
    RCLCPP_WARN(
      logger_, "Max robot search distance is negative, setting to max to search"
      " every point on path for the closest value.");
    max_robot_pose_search_dist_ = std::numeric_limits<double>::max();
  }
  constraint_locale_ = 0u;
  if (!enforce_path_rotation_) {
    minimum_rotation_angle_ = 0.0f;
  }

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&FeasiblePathHandler::dynamicParametersCallback, this, _1));
}

double FeasiblePathHandler::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters = std::max(
    costmap_ros_->getCostmap()->getSizeInMetersX(),
    costmap_ros_->getCostmap()->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

void FeasiblePathHandler::prunePlan(nav_msgs::msg::Path & plan, const PathIterator end)
{
  plan.poses.erase(plan.poses.begin(), end);
}

bool FeasiblePathHandler::isWithinInversionTolerances(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // Keep full path if we are within tolerance of the inversion pose
  const auto last_pose = global_plan_up_to_constraint_.poses.back();
  float distance = hypotf(
    robot_pose.pose.position.x - last_pose.pose.position.x,
    robot_pose.pose.position.y - last_pose.pose.position.y);

  float angle_distance = angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation),
    tf2::getYaw(last_pose.pose.orientation));

  return distance <= inversion_xy_tolerance_ &&
         fabs(angle_distance) <= inversion_yaw_tolerance_;
}

void FeasiblePathHandler::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  global_plan_up_to_constraint_ = global_plan_;
  if (enforce_path_inversion_ || enforce_path_rotation_) {
    constraint_locale_ = official_lattice_rpp_adapter::backport::removePosesAfterFirstConstraint(global_plan_up_to_constraint_,
      enforce_path_inversion_, minimum_rotation_angle_);
  }
}

geometry_msgs::msg::PoseStamped FeasiblePathHandler::transformToGlobalPlanFrame(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_up_to_constraint_.poses.empty()) {
    throw nav2_core::InvalidPath("Received plan with zero length");
  }

  if (reject_unit_path_ && global_plan_up_to_constraint_.poses.size() == 1) {
    throw nav2_core::InvalidPath("Received plan with length of one");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::transformPoseInTargetFrame(pose, robot_pose, *tf_,
      global_plan_up_to_constraint_.header.frame_id,
      transform_tolerance_))
  {
    throw nav2_core::ControllerTFError("Unable to transform robot pose into global plan's frame");
  }

  return robot_pose;
}

PathSegment FeasiblePathHandler::findPlanSegment(
  const geometry_msgs::msg::PoseStamped & pose)
{
  global_pose_ = transformToGlobalPlanFrame(pose);

  // Limit the search for the closest pose up to max_robot_pose_search_dist on the path
  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_up_to_constraint_.poses.begin(), global_plan_up_to_constraint_.poses.end(),
      max_robot_pose_search_dist_);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto closest_point =
    nav2_util::geometry_utils::min_by(
    global_plan_up_to_constraint_.poses.begin(), closest_pose_upper_bound,
    [this](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(global_pose_, ps);
    });

  // Make sure we always have at least 2 points on the transformed plan and that we don't prune
  // the global plan below 2 points in order to have always enough point to interpolate the
  // end of path direction
  if (std::distance(global_plan_up_to_constraint_.poses.begin(), closest_pose_upper_bound) >= 2 &&
    global_plan_up_to_constraint_.poses.size() > 1 &&
    closest_point == std::prev(closest_pose_upper_bound))
  {
    closest_point = std::prev(std::prev(closest_pose_upper_bound));
  }

  auto pruned_plan_end =
    nav2_util::geometry_utils::first_after_integrated_distance(
    closest_point, global_plan_up_to_constraint_.poses.end(), prune_distance_);

  return {closest_point, pruned_plan_end};
}


nav_msgs::msg::Path FeasiblePathHandler::transformLocalPlan(
  const PathIterator & closest_point,
  const PathIterator & pruned_plan_end)
{
  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  transformed_plan.header.stamp = global_pose_.header.stamp;
  unsigned int mx, my;
  // Find the furthest relevant pose on the path to consider within costmap
  // bounds
  // Transforming it to the costmap frame in the same loop
  for (auto global_plan_pose = closest_point; global_plan_pose != pruned_plan_end;
    ++global_plan_pose)
  {
    // Transform from global plan frame to costmap frame
    geometry_msgs::msg::PoseStamped costmap_plan_pose;
    global_plan_pose->header.stamp = global_pose_.header.stamp;
    global_plan_pose->header.frame_id = global_plan_.header.frame_id;
    if (!nav2_util::transformPoseInTargetFrame(*global_plan_pose, costmap_plan_pose, *tf_,
      costmap_ros_->getGlobalFrameID(), transform_tolerance_)) {
      throw nav2_core::ControllerTFError("Unable to transform constraint pose");
    }

    // Check if pose is inside the costmap
    if (!costmap_ros_->getCostmap()->worldToMap(
        costmap_plan_pose.pose.position.x, costmap_plan_pose.pose.position.y, mx, my))
    {
      break;
    }

    // Filling the transformed plan to return with the transformed pose
    transformed_plan.poses.push_back(costmap_plan_pose);
  }

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  prunePlan(global_plan_up_to_constraint_, closest_point);

  if ((enforce_path_inversion_ || enforce_path_rotation_) && constraint_locale_ != 0u) {
    if (isWithinInversionTolerances(global_pose_)) {
      prunePlan(global_plan_, global_plan_.poses.begin() + constraint_locale_);
      global_plan_up_to_constraint_ = global_plan_;
      constraint_locale_ = official_lattice_rpp_adapter::backport::removePosesAfterFirstConstraint(global_plan_up_to_constraint_,
        enforce_path_inversion_, minimum_rotation_angle_);
    }
  }

  if (transformed_plan.poses.empty()) {
    throw nav2_core::InvalidPath("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

geometry_msgs::msg::PoseStamped FeasiblePathHandler::getTransformedGoal(
  const builtin_interfaces::msg::Time & stamp)
{
  auto goal = global_plan_.poses.back();
  goal.header.frame_id = global_plan_.header.frame_id;
  goal.header.stamp = stamp;
  if (goal.header.frame_id.empty()) {
    throw nav2_core::ControllerTFError("Goal pose has an empty frame_id");
  }
  geometry_msgs::msg::PoseStamped transformed_goal;
  if (!nav2_util::transformPoseInTargetFrame(goal, transformed_goal, *costmap_ros_->getTfBuffer(),
      costmap_ros_->getGlobalFrameID(), transform_tolerance_))
  {
    throw nav2_core::ControllerTFError("Unable to transform goal pose into costmap frame");
  }
  return transformed_goal;
}

bool FeasiblePathHandler::finalPrefixWithin(double limit) const
{
  if (constraint_locale_ != 0u || global_plan_up_to_constraint_.poses.empty()) {return false;}
  const auto & poses = global_plan_up_to_constraint_.poses;
  double remaining = 0.0;
  for (std::size_t i = 1; i < poses.size(); ++i) {
    remaining += euclidean_distance(poses[i - 1], poses[i]);
  }
  // Upstream deliberately retains two points. Account for travel on that
  // first edge without selecting a later spatially nearby crossing/loop.
  if (poses.size() > 1) {
    const auto & a = poses[0].pose.position;
    const auto & b = poses[1].pose.position;
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length = std::hypot(dx, dy);
    if (length > 1e-9) {
      const double projection = ((global_pose_.pose.position.x - a.x) * dx +
        (global_pose_.pose.position.y - a.y) * dy) / length;
      remaining -= std::clamp(projection, 0.0, length);
    }
  }
  return std::isfinite(remaining) && remaining <= limit;
}

rcl_interfaces::msg::SetParametersResult
FeasiblePathHandler::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".max_robot_pose_search_dist") {
        max_robot_pose_search_dist_ = parameter.as_double();
      } else if (param_name == plugin_name_ + ".inversion_xy_tolerance") {
        inversion_xy_tolerance_ = parameter.as_double();
      } else if (param_name == plugin_name_ + ".inversion_yaw_tolerance") {
        inversion_yaw_tolerance_ = parameter.as_double();
      } else if (param_name == plugin_name_ + ".prune_distance") {
        prune_distance_ = parameter.as_double();
      } else if (param_name == plugin_name_ + ".minimum_rotation_angle") {
        minimum_rotation_angle_ = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == plugin_name_ + ".enforce_path_inversion") {
        enforce_path_inversion_ = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".enforce_path_rotation") {
        enforce_path_rotation_ = parameter.as_bool();
        if (!enforce_path_rotation_) {
          minimum_rotation_angle_ = 0.0f;
        }
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace official_lattice_rpp_adapter::backport
