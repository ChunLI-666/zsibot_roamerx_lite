// Copyright (c) 2025 Berkan Tali
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

#include "nav2_util/path_utils.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace nav2_util {
bool transformPathInTargetFrame(
  const nav_msgs::msg::Path & input_path,
  nav_msgs::msg::Path & transformed_path,
  tf2_ros::Buffer & tf_buffer, const std::string target_frame,
  const double transform_timeout)
{
  static rclcpp::Logger logger = rclcpp::get_logger("transformPathInTargetFrame");

  if (input_path.header.frame_id == target_frame) {
    transformed_path = input_path;
    return true;
  }

  transformed_path.header.frame_id = target_frame;
  transformed_path.header.stamp = input_path.header.stamp;
  transformed_path.poses.reserve(input_path.poses.size());

  for (const auto & input_pose : input_path.poses) {
    geometry_msgs::msg::PoseStamped source_pose, transformed_pose;
    source_pose.header.frame_id = input_path.header.frame_id;
    source_pose.header.stamp = input_path.header.stamp;
    source_pose.pose = input_pose.pose;

    if (!nav2_util::transformPoseInTargetFrame(
          source_pose, transformed_pose, tf_buffer, target_frame, transform_timeout))
    {
      RCLCPP_ERROR(
        logger,
        "Failed to transform path from '%s' to '%s'.",
        input_path.header.frame_id.c_str(), target_frame.c_str());
      return false;
    }

    transformed_pose.pose.position.z = 0.0;
    transformed_path.poses.push_back(std::move(transformed_pose));
  }

  return true;
}
}
