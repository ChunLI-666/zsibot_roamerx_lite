// Copyright 2026. SPDX-License-Identifier: Apache-2.0
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "robot_feasible_navigation/segment_controller_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_feasible_navigation::SegmentControllerServer>(rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
