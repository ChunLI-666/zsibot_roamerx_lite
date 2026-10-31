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
namespace nav2_util {
unsigned int findFirstPathConstraint(
  nav_msgs::msg::Path & path,
  bool enforce_path_inversion,
  float rotation_threshold)
{
  // At least 3 poses for a possible inversion
  if (path.poses.size() < 3) {
    return path.poses.size();
  }

  const bool check_rotation = fabs(rotation_threshold) < 1e-6f ? false : true;
  unsigned int rotation_idx = path.poses.size();
  unsigned int inversion_idx = path.poses.size();
  float prev_dx = 0.0f;
  float prev_dy = 0.0f;

  // Iterating through the path to determine the position of the path inversion or rotation
  for (unsigned int idx = 0; idx < path.poses.size() - 1; ++idx) {
    float dx = path.poses[idx + 1].pose.position.x -
      path.poses[idx].pose.position.x;
    float dy = path.poses[idx + 1].pose.position.y -
      path.poses[idx].pose.position.y;
    float trans = hypot(dx, dy);

    // No smaller index can exist beyond this point, terminate early
    if (rotation_idx <= idx + 1) {
      break;
    }

    // Check inversion
    if (enforce_path_inversion && trans > 1e-4) {
      if (idx >= 1) {
        // Checking for the existence of cusp, in the path, using the dot product.
        float dot_product = prev_dx * dx + prev_dy * dy;
        if (dot_product < 0.0f) {
          inversion_idx = idx + 1;
          break;
        }
      }
      prev_dx = dx;
      prev_dy = dy;
    }

    // Check in place rotation
    if (check_rotation && trans < 1e-4 && rotation_idx == path.poses.size()) {
      float accumulated_rotation = 0.0f;
      unsigned int end_idx = idx;

      // Continue checking while translation remains small
      // until accumulated rotation is larger than threshold
      while (end_idx < path.poses.size() - 1) {
        float current_yaw = tf2::getYaw(path.poses[end_idx].pose.orientation);
        float next_yaw = tf2::getYaw(path.poses[end_idx + 1].pose.orientation);
        accumulated_rotation += fabs(angles::shortest_angular_distance(current_yaw, next_yaw));
        if (accumulated_rotation > rotation_threshold) {
          rotation_idx = end_idx + 1;
          break;
        }
        if (end_idx + 2 < path.poses.size()) {
          float ndx = path.poses[end_idx + 2].pose.position.x -
            path.poses[end_idx + 1].pose.position.x;
          float ndy = path.poses[end_idx + 2].pose.position.y -
            path.poses[end_idx + 1].pose.position.y;
          // Stop if translation resumes
          if (hypot(ndx, ndy) > 1e-4) {
            break;
          }
        } else {
          // We have reached the end of the path
          break;
        }
        end_idx++;
      }
    }
  }

  return std::min(rotation_idx, inversion_idx);
}

unsigned int removePosesAfterFirstConstraint(
  nav_msgs::msg::Path & path,
  bool enforce_path_inversion,
  float rotation_threshold)
{
  nav_msgs::msg::Path cropped_path = path;
  const unsigned int first_after_constraint = findFirstPathConstraint(cropped_path,
    enforce_path_inversion, rotation_threshold);
  if (first_after_constraint == path.poses.size()) {
    return 0u;
  }

  cropped_path.poses.erase(
    cropped_path.poses.begin() + first_after_constraint, cropped_path.poses.end());
  path = cropped_path;
  return first_after_constraint;
}


}
