// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <nav_msgs/msg/path.hpp>
#include <angles/angles.h>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace official_lattice_rpp_adapter::backport {
unsigned int findFirstPathConstraint(nav_msgs::msg::Path &, bool, float);
unsigned int removePosesAfterFirstConstraint(nav_msgs::msg::Path &, bool, float);
}
