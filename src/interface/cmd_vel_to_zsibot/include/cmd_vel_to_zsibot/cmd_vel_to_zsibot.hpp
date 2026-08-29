// Copyright 2024 ZsiBot Team
// Licensed under the Apache License, Version 2.0

#ifndef CMD_VEL_TO_ZSIBOT__CMD_VEL_TO_ZSIBOT_HPP_
#define CMD_VEL_TO_ZSIBOT__CMD_VEL_TO_ZSIBOT_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <netinet/in.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "robots_dog_msgs/msg/cmd_vel_to_zsibot_debug.hpp"

namespace cmd_vel_to_zsibot
{

/**
 * @class CmdVelToZsibot
 * @brief ROS2 node that converts geometry_msgs/Twist commands to yz-robot-ctrl UDP payloads
 *
 * This node subscribes to /cmd_vel and forwards normalized velocity commands
 * to the local yz-robot-ctrl service, which owns the ZsiBot SDK connection.
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
   * @brief Timer callback to process pending commands and timeouts
   */
  void commandTimerCallback();

  /**
   * @brief Timer callback for low-rate status polling
   */
  void statusTimerCallback();

  /**
   * @brief Initialize the local UDP command client.
   */
  bool initializeUdpClient();

  /**
   * @brief Clamp a value between min and max
   */
  double clamp(double value, double min_val, double max_val);

  /**
   * @brief Apply velocity limits and SDK minimum command magnitudes
   */
  geometry_msgs::msg::Twist normalizeCommand(const geometry_msgs::msg::Twist & cmd);

  /**
   * @brief Return true if two commands differ enough to resend to the SDK
   */
  bool commandsDiffer(
    const geometry_msgs::msg::Twist & lhs,
    const geometry_msgs::msg::Twist & rhs) const;

  /**
   * @brief Return true if the command is zero in all controlled axes
   */
  bool isZeroCommand(const geometry_msgs::msg::Twist & cmd) const;

  /**
   * @brief Send a normalized move command to the SDK
   */
  void sendMoveCommand(
    const geometry_msgs::msg::Twist & cmd,
    bool force = false,
    const std::string & reason = "cmd_vel");

  /**
   * @brief Send a zero velocity command immediately if needed
   */
  void sendStopCommand(bool force = false, const std::string & reason = "stop");

  /**
   * @brief Send a raw command payload to the configured UDP endpoint.
   */
  bool sendUdpPayload(
    const std::string & payload,
    const std::string & reason,
    bool force = false,
    const geometry_msgs::msg::Twist * normalized_cmd = nullptr);

  /**
   * @brief Return true when commands should be sent through yz-robot-ctrl UDP.
   */
  bool usingUdpOutput() const;

  /**
   * @brief Return true when commands should be converted and emitted.
   */
  bool canEmitCommands() const;

  /**
   * @brief Return true when commands should only be published to debug topics.
   */
  bool usingFakeOutput() const;

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

  // Publisher for battery state
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;

  // Publisher for debug visibility into actual outgoing UDP payloads
  rclcpp::Publisher<robots_dog_msgs::msg::CmdVelToZsibotDebug>::SharedPtr sent_command_pub_;

  // Publisher for full command conversion debug records
  rclcpp::Publisher<robots_dog_msgs::msg::CmdVelToZsibotDebug>::SharedPtr debug_command_pub_;

  // Timers for command processing and status polling
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // Services for robot control
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stand_up_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr lie_down_srv_;

  // Latest velocity command
  geometry_msgs::msg::Twist latest_cmd_;
  geometry_msgs::msg::Twist latest_input_cmd_;
  geometry_msgs::msg::Twist last_sent_cmd_;

  // Mutex for thread safety
  std::mutex cmd_mutex_;

  // Parameters
  std::string local_ip_;
  int local_port_;
  std::string robot_ip_;
  std::string output_mode_;
  std::string control_host_;
  int control_port_;
  double max_linear_x_;
  double max_linear_y_;
  double max_angular_z_;
  double min_linear_x_;
  double min_linear_y_;
  double min_angular_z_;
  double cmd_timeout_;
  double command_check_rate_;
  double status_rate_;
  double min_command_interval_;
  double command_epsilon_;
  bool enabled_;

  // Timestamp of last received command
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_send_time_;
  bool has_pending_cmd_;
  bool has_sent_cmd_;
  bool robot_stopped_;
  uint32_t debug_seq_;

  // Connection status
  bool connected_;

  // UDP client for yz-robot-ctrl local command ingress
  int udp_sock_;
  struct sockaddr_in udp_addr_;

  // Robot standing status - only send move commands when standing
  std::atomic<bool> standing_;
};

}  // namespace cmd_vel_to_zsibot

#endif  // CMD_VEL_TO_ZSIBOT__CMD_VEL_TO_ZSIBOT_HPP_
