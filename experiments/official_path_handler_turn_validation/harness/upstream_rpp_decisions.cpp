// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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

#include "rpp_decision_fixture.hpp"
namespace nav2_regulated_pure_pursuit_controller {
bool RegulatedPurePursuitController::shouldRotateToPath(
  const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path,
  double & x_vel_sign)
{
  // Whether we should rotate robot to rough path heading
  angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  // In case we are reversing
  if (x_vel_sign < 0.0) {
    angle_to_path = angles::normalize_angle(angle_to_path + M_PI);
  }
  return params_->use_rotate_to_heading &&
         fabs(angle_to_path) > params_->rotate_to_heading_min_angle;
}

bool RegulatedPurePursuitController::shouldRotateToGoalHeading(
  nav2_core::GoalChecker * goal_checker,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const geometry_msgs::msg::Twist & speed,
  const nav_msgs::msg::Path & transformed_plan)
{
  // Whether we should rotate robot to goal heading
  if (!params_->use_rotate_to_heading) {
    return false;
  }
  return goal_checker->isGoalXYReached(robot_pose.pose, goal_pose.pose, speed,
    transformed_plan);
}

void RegulatedPurePursuitController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
{
  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  angular_vel = sign * params_->rotate_to_heading_angular_vel;

  const double & dt = control_duration_;
  const double min_feasible_angular_speed = curr_speed.angular.z - params_->max_angular_accel * dt;
  const double max_feasible_angular_speed = curr_speed.angular.z + params_->max_angular_accel * dt;
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);

  // Check if we need to slow down to avoid overshooting
  double max_vel_to_stop = std::sqrt(2 * params_->max_angular_accel * fabs(angle_to_path));
  if (fabs(angular_vel) > max_vel_to_stop) {
    angular_vel = sign * max_vel_to_stop;
  }
}
}
