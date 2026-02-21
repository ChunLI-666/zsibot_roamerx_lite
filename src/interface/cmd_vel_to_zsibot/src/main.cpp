// Copyright 2024 ZsiBot Team
// Licensed under the Apache License, Version 2.0

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "cmd_vel_to_zsibot/cmd_vel_to_zsibot.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<cmd_vel_to_zsibot::CmdVelToZsibot>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
