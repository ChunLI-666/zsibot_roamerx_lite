// Copyright 2024 ZsiBot Team
// Licensed under the Apache License, Version 2.0

#ifndef CMD_VEL_TO_ZSIBOT__CMD_VEL_TO_ZSIBOT_HPP_
#define CMD_VEL_TO_ZSIBOT__CMD_VEL_TO_ZSIBOT_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "zsl-1/highlevel.h"

namespace cmd_vel_to_zsibot
{

/**
 * @class CmdVelToZsibot
 * @brief ROS2 node that converts geometry_msgs/Twist commands to ZsiBot SDK calls
 *
 * This node subscribes to /cmd_vel topic and forwards the velocity commands
 * to the ZsiBot robot using the mc_sdk HighLevel interface.
 */
class CmdVelToZsibot : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for CmdVelToZsibot
   * @param options Node options for configuration
   */
  explicit CmdVelToZsibot(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~CmdVelToZsibot();

protected:
  /**
   * @brief Callback function for cmd_vel messages
   * @param msg The Twist message containing velocity commands
   */
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Callback for enable/disable topic
   * @param msg Bool message to enable or disable the bridge
   */
  void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * @brief Timer callback to send commands at fixed rate
   */
  void timerCallback();

  /**
   * @brief Initialize connection to ZsiBot
   * @return true if connection successful
   */
  bool initializeRobot();

  /**
   * @brief Clamp a value between min and max
   * @param value Input value
   * @param min_val Minimum value
   * @param max_val Maximum value
   * @return Clamped value
   */
  double clamp(double value, double min_val, double max_val);

  /**
   * @brief Service callback to make robot stand up
   */
  void standUpCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Service callback to make robot lie down
   */
  void lieDownCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

private:
  // Subscriber for velocity commands
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  
  // Subscriber for enable/disable
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  
  // Publisher for connection status
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
  
  // Timer for sending commands at fixed rate
  rclcpp::TimerBase::SharedPtr timer_;

  // Services for robot control
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stand_up_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr lie_down_srv_;

  // ZsiBot SDK interface
  std::unique_ptr<mc_sdk::zsl_1::HighLevel> highlevel_;

  // Latest velocity command
  geometry_msgs::msg::Twist latest_cmd_;
  
  // Mutex for thread safety
  std::mutex cmd_mutex_;

  // Parameters
  std::string local_ip_;
  int local_port_;
  std::string robot_ip_;
  double max_linear_x_;
  double max_linear_y_;
  double max_angular_z_;
  double cmd_timeout_;
  double publish_rate_;
  bool enabled_;

  // Timestamp of last received command
  rclcpp::Time last_cmd_time_;
  
  // Connection status
  bool connected_;
  
  // Robot standing status - only send move commands when standing
  bool standing_;
};

}  // namespace cmd_vel_to_zsibot

#endif  // CMD_VEL_TO_ZSIBOT__CMD_VEL_TO_ZSIBOT_HPP_
