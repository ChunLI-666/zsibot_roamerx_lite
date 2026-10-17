// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include <stdint.h>
#include <chrono>
#include <sstream>
#include "navigo_costmap_2d/costmap_filters/filter_values.hpp"
#include "navigo_mppi_controller/controller.hpp"
#include "navigo_mppi_controller/tools/utils.hpp"

// #define BENCHMARK_TESTING

namespace navigo_mppi_controller
{

void MPPIController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<navigo_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  name_ = name;
  parameters_handler_ = std::make_unique<ParametersHandler>(parent);

  auto node = parent_.lock();
  clock_ = node->get_clock();
  last_time_called_ = clock_->now();
  // Get high-level controller parameters
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(visualize_, "visualize", false);
  getParam(visualize_all_sampled_traj_, "visualize_all_sampled_traj", false);
  getParam(visualize_optimized_traj_, "visualize_optimized_traj", true);
  getParam(reset_period_, "reset_period", 1.0);

  auto alignment = parameters_handler_->getParamGetter(name_ + ".forward_alignment");
  alignment(forward_alignment_enabled_, "enabled", false, ParameterType::Static);
  alignment(entry_angle_, "entry_angle", 0.70, ParameterType::Static);
  alignment(exit_angle_, "exit_angle", 0.20, ParameterType::Static);
  alignment(lookahead_distance_, "lookahead_distance", 0.5, ParameterType::Static);
  alignment(max_angular_velocity_, "max_angular_velocity", 0.1, ParameterType::Static);
  alignment(min_angular_velocity_, "min_angular_velocity", 0.02, ParameterType::Static);
  alignment(angular_acceleration_, "angular_acceleration", 0.2, ParameterType::Static);
  alignment(xy_hysteresis_, "xy_hysteresis", 0.05, ParameterType::Static);
  alignment(rotation_collision_step_, "rotation_collision_step", 0.02, ParameterType::Static);
  alignment(stopped_linear_velocity_, "stopped_linear_velocity", 0.02, ParameterType::Static);
  // Never bypass the optimizer's angular speed ceiling.
  double optimizer_wz = 0.1;
  getParam(optimizer_wz, "wz_max", 1.9, ParameterType::Static);
  if (forward_alignment_enabled_ && !std::isfinite(max_angular_velocity_)) {
    throw std::invalid_argument("Nonfinite alignment angular ceiling");
  }
  max_angular_velocity_ = std::min(max_angular_velocity_, optimizer_wz);
  double frequency = 10.0;
  parameters_handler_->getParamGetter("")(frequency, "controller_frequency", 10.0,
    ParameterType::Static);
  if (forward_alignment_enabled_) {
    for (const double value : {entry_angle_, exit_angle_, lookahead_distance_,
      max_angular_velocity_, min_angular_velocity_, angular_acceleration_, xy_hysteresis_,
      rotation_collision_step_, stopped_linear_velocity_, frequency, optimizer_wz})
    {
      if (!std::isfinite(value)) {throw std::invalid_argument("Nonfinite forward parameters");}
    }
    if (!(exit_angle_ > 0.0 && entry_angle_ > exit_angle_ && entry_angle_ < M_PI &&
      lookahead_distance_ > 0.0 && min_angular_velocity_ > 0.0 &&
      max_angular_velocity_ >= min_angular_velocity_ && angular_acceleration_ > 0.0 &&
      rotation_collision_step_ > 0.0 && xy_hysteresis_ >= 0.0 &&
      stopped_linear_velocity_ >= 0.0 && frequency > 0.0))
    {
      throw std::invalid_argument("Invalid forward_alignment parameters");
    }
  }
  control_period_ = 1.0 / frequency;
  mode_publisher_ = node->create_publisher<std_msgs::msg::String>(
    "~/" + name_ + "/motion_mode", rclcpp::QoS(10));

  // Configure composed objects
  optimizer_.initialize(parent_, name_, costmap_ros_, parameters_handler_.get());
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
  trajectory_visualizer_.on_configure(
    parent_, name_,
    costmap_ros_->getGlobalFrameID(), parameters_handler_.get());

  RCLCPP_INFO(logger_, "Configured MPPI Controller: %s", name_.c_str());
}

void MPPIController::cleanup()
{
  optimizer_.shutdown();
  mode_publisher_.reset();
  trajectory_visualizer_.on_cleanup();
  parameters_handler_.reset();
  RCLCPP_INFO(logger_, "Cleaned up MPPI Controller: %s", name_.c_str());
}

void MPPIController::activate()
{
  trajectory_visualizer_.on_activate();
  mode_publisher_->on_activate();
  parameters_handler_->start();
  RCLCPP_INFO(logger_, "Activated MPPI Controller: %s", name_.c_str());
}

void MPPIController::deactivate()
{
  trajectory_visualizer_.on_deactivate();
  mode_publisher_->on_deactivate();
  RCLCPP_INFO(logger_, "Deactivated MPPI Controller: %s", name_.c_str());
}

void MPPIController::reset()
{
  source_motion_phase_ = last_nonzero_motion_phase_ = 1;
  motion_mode_ = forward_alignment::Mode::TRACK;
  optimizer_.reset();
}

geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  navigo_core::GoalChecker * goal_checker)
{
#ifdef BENCHMARK_TESTING
  auto start = std::chrono::system_clock::now();
#endif

  if (clock_->now() - last_time_called_ > rclcpp::Duration::from_seconds(reset_period_)) {
    reset();
  }
  const double dt = control_period_;
  last_time_called_ = clock_->now();

  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  geometry_msgs::msg::Pose goal = path_handler_.getTransformedGoal(robot_pose.header.stamp).pose;

  nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);

  navigo_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<navigo_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));

  source_motion_phase_ = 1;  // Any exception or zero result remains HOLD.
  geometry_msgs::msg::TwistStamped cmd;
  if (forward_alignment_enabled_ &&
    alignmentCommand(robot_pose, robot_speed, transformed_plan, goal, goal_checker, cmd, dt))
  {
    source_motion_phase_ = cmd.twist.angular.z != 0.0 ? 3 : 1;
    if (source_motion_phase_ != 1) {last_nonzero_motion_phase_ = source_motion_phase_;}
    return cmd;
  }
  try {
    cmd = optimizer_.evalControl(robot_pose, robot_speed, transformed_plan, goal, goal_checker);
  } catch (const std::exception &) {
    publishMode("HOLD", "optimizer_or_collision_rejected", 0.0, robot_pose.header.stamp);
    throw;
  }

  if (forward_alignment_enabled_) {
    const bool translation = cmd.twist.linear.x != 0.0 || cmd.twist.linear.y != 0.0;
    const bool rotation = cmd.twist.angular.z != 0.0;
    // A small measured residual is not a command to snap an axis to zero:
    // the phase-aware relay must still brake its exact command state to zero.
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist stop_tolerance;
    if (!goal_checker || !goal_checker->getTolerances(pose_tolerance, stop_tolerance) ||
      !std::isfinite(stop_tolerance.angular.z) || stop_tolerance.angular.z <= 0.0)
    {
      throw std::runtime_error("Forward tracking requires a finite stopped angular tolerance");
    }
    if (translation && last_nonzero_motion_phase_ == 3 &&
      std::abs(robot_speed.angular.z) > stop_tolerance.angular.z) {
      optimizer_.reset();
      publishMode("HOLD", "waiting_rotation_stop", robot_speed.angular.z, robot_pose.header.stamp);
      cmd.twist = geometry_msgs::msg::Twist();
    } else if (!translation && rotation && last_nonzero_motion_phase_ == 4 &&
      std::hypot(robot_speed.linear.x, robot_speed.linear.y) > stopped_linear_velocity_)
    {
      optimizer_.reset();
      publishMode("HOLD", "waiting_translation_stop", 0.0, robot_pose.header.stamp);
      cmd.twist = geometry_msgs::msg::Twist();
    } else {
      // TRACK preserves the optimizer's explicitly predicted curved movement.
      source_motion_phase_ = translation ? 4 : (rotation ? 3 : 1);
      if (source_motion_phase_ != 1) {last_nonzero_motion_phase_ = source_motion_phase_;}
    }
  }

#ifdef BENCHMARK_TESTING
  auto end = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);
#endif

  if (visualize_) {
    visualize(std::move(transformed_plan));
  }

  return cmd;
}

void MPPIController::visualize(nav_msgs::msg::Path transformed_plan)
{
  if (visualize_all_sampled_traj_) {
    trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), "Candidate Trajectories");
  }
  if (visualize_optimized_traj_) {
    trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(), "Optimal Trajectory");
  }
  trajectory_visualizer_.visualize(std::move(transformed_plan));
}

void MPPIController::setPlan(const nav_msgs::msg::Path & path)
{
  if (forward_alignment_enabled_ && !path.poses.empty()) {
    const auto & goal = path.poses.back();
    if (!have_goal_ || previous_goal_.header.frame_id != path.header.frame_id ||
      std::hypot(previous_goal_.pose.position.x - goal.pose.position.x,
      previous_goal_.pose.position.y - goal.pose.position.y) > 0.05 ||
      std::abs(forward_alignment::normalize(tf2::getYaw(previous_goal_.pose.orientation) -
      tf2::getYaw(goal.pose.orientation))) > 0.05)
    {
      reset();
    }
    previous_goal_ = goal;
    previous_goal_.header.frame_id = path.header.frame_id;
    have_goal_ = true;
  }
  path_handler_.setPath(path);
}

void MPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  if (!std::isfinite(speed_limit) || speed_limit < 0.0) {
    throw std::invalid_argument("Invalid controller speed limit");
  }
  optimizer_.setSpeedLimit(speed_limit, percentage);

}


void MPPIController::publishMode(
  const std::string & mode, const std::string & reason, double error,
  const builtin_interfaces::msg::Time & stamp)
{
  if (!mode_publisher_ || !mode_publisher_->is_activated()) {return;}
  std_msgs::msg::String message;
  std::ostringstream out;
  out << "mode=" << mode << " reason=" << reason << " heading_error=" << error
      << " stamp_sec=" << stamp.sec << " stamp_nanosec=" << stamp.nanosec;
  message.data = out.str();
  mode_publisher_->publish(message);
}

bool MPPIController::alignmentCommand(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::Pose & goal,
  navigo_core::GoalChecker * checker, geometry_msgs::msg::TwistStamped & command, double dt)
{
  for (const double value : {pose.pose.position.x, pose.pose.position.y,
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
    pose.pose.orientation.w, goal.position.x, goal.position.y,
    speed.linear.x, speed.linear.y, speed.angular.z})
  {
    if (!std::isfinite(value)) {
      publishMode("HOLD", "nonfinite_pose_or_velocity", 0.0, pose.header.stamp);
      throw std::runtime_error("Nonfinite forward alignment input");
    }
  }
  command.header.stamp = pose.header.stamp;
  command.header.frame_id = costmap_ros_->getBaseFrameID();
  geometry_msgs::msg::Pose tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (!checker || !checker->getTolerances(tolerance, velocity_tolerance) ||
    !std::isfinite(tolerance.position.x) || !(tolerance.position.x > xy_hysteresis_))
  {
    publishMode("HOLD", "invalid_goal_tolerance", 0.0, pose.header.stamp);
    throw std::runtime_error("Forward alignment requires valid goal checker tolerances");
  }
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double distance = std::hypot(goal.position.x - pose.pose.position.x,
    goal.position.y - pose.pose.position.y);
  const double heading = forward_alignment::pathHeading(path, pose.pose, lookahead_distance_);
  const double heading_error = forward_alignment::normalize(heading - yaw);
  const auto old_mode = motion_mode_;
  motion_mode_ = forward_alignment::nextMode(motion_mode_, distance, heading_error,
    tolerance.position.x, xy_hysteresis_, entry_angle_, exit_angle_);
  if (motion_mode_ != forward_alignment::Mode::FINAL && !std::isfinite(heading)) {
    publishMode("HOLD", "invalid_path_heading", 0.0, pose.header.stamp);
    throw std::runtime_error("No geometric path heading outside final alignment");
  }
  if (motion_mode_ == forward_alignment::Mode::TRACK && old_mode != motion_mode_) {
    if (!std::isfinite(velocity_tolerance.angular.z) || velocity_tolerance.angular.z <= 0.0) {
      throw std::runtime_error("Alignment transition requires a stopped angular tolerance");
    }
    if (std::abs(speed.angular.z) > velocity_tolerance.angular.z) {
      motion_mode_ = old_mode;
      publishMode("HOLD", "waiting_rotation_stop", speed.angular.z, pose.header.stamp);
      return true;
    }
  }
  if (old_mode != motion_mode_) {optimizer_.reset();}
  if (motion_mode_ == forward_alignment::Mode::TRACK) {
    publishMode("FORWARD_TRACK", "path_aligned", heading_error, pose.header.stamp);
    return false;
  }
  const bool final = motion_mode_ == forward_alignment::Mode::FINAL;
  const double error = final ? forward_alignment::normalize(tf2::getYaw(goal.orientation) - yaw) :
    heading_error;
  const double yaw_tolerance = std::abs(tf2::getYaw(tolerance.orientation));
  const double angular_tolerance = final ? yaw_tolerance : exit_angle_;
  if (!(yaw_tolerance > 0.0) || !std::isfinite(error)) {
    throw std::runtime_error("Invalid yaw tolerance or alignment error");
  }
  if (std::hypot(speed.linear.x, speed.linear.y) > stopped_linear_velocity_) {
    publishMode("HOLD", "waiting_translation_stop", error, pose.header.stamp);
    return true;
  }
  if (std::abs(error) <= angular_tolerance) {
    publishMode(final ? "FINAL_ALIGN" : "ALIGN_TO_PATH", "waiting_stop", error,
      pose.header.stamp);
    return true;
  }
  const double effective_angular_limit = std::min(
    max_angular_velocity_, static_cast<double>(optimizer_.getConstraints().wz));
  if (!std::isfinite(effective_angular_limit) ||
    effective_angular_limit < min_angular_velocity_) {
    publishMode("HOLD", "angular_limit_below_executable_minimum", error, pose.header.stamp);
    return true;
  }
  // Test the entire intended rotation, not just the next command. The costmap
  // footprint must include the measured stance/leg envelope on the real robot.
  auto * map = costmap_ros_->getCostmap();
  if (!forward_alignment::sweepFree(*map, costmap_ros_->getRobotFootprint(),
    pose.pose.position.x, pose.pose.position.y, yaw,
    pose.pose.position.x, pose.pose.position.y, yaw + error, rotation_collision_step_))
  {
    publishMode("HOLD", "rotation_footprint_collision", error, pose.header.stamp);
    throw std::runtime_error("Forward alignment rotation footprint is blocked");
  }
  const double target = std::copysign(std::clamp(
    std::sqrt(2.0 * angular_acceleration_ * std::max(0.0, std::abs(error) - angular_tolerance)),
    min_angular_velocity_, effective_angular_limit), error);
  const double bounded = std::clamp(target,
    speed.angular.z - angular_acceleration_ * dt, speed.angular.z + angular_acceleration_ * dt);
  // Waiting at zero also handles direction changes without issuing SDK deadband commands.
  command.twist.angular.z = bounded * error > 0.0 &&
    std::abs(bounded) + 1e-9 >= min_angular_velocity_ ?
    std::clamp(bounded, -effective_angular_limit, effective_angular_limit) : 0.0;
  publishMode(final ? "FINAL_ALIGN" : "ALIGN_TO_PATH", final ? "goal_yaw" : "path_heading",
    error, pose.header.stamp);
  return true;
}

}  // namespace navigo_mppi_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(navigo_mppi_controller::MPPIController, navigo_core::Controller)
