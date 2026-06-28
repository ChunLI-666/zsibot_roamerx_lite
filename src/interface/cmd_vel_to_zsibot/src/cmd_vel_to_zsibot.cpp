// Copyright 2024 ZsiBot Team
// Licensed under the Apache License, Version 2.0

#include "cmd_vel_to_zsibot/cmd_vel_to_zsibot.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <sstream>
#include <thread>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace
{

geometry_msgs::msg::Twist zeroTwist()
{
  return geometry_msgs::msg::Twist();
}

}  // namespace

namespace cmd_vel_to_zsibot
{

CmdVelToZsibot::CmdVelToZsibot(const rclcpp::NodeOptions & options)
: Node("cmd_vel_to_zsibot", options),
  enabled_(true),
  debug_seq_(0),
  connected_(false),
  udp_sock_(-1),
  standing_(false)
{
  // Declare parameters
  // Default IPs match yz_robot_ctrl configuration
  this->declare_parameter<std::string>("local_ip", "192.168.168.2");  // RK3588/NanoPC eth0 IP
  this->declare_parameter<int>("local_port", 43988);
  this->declare_parameter<std::string>("robot_ip", "192.168.168.168");  // Robot control board IP
  this->declare_parameter<std::string>("output_mode", "udp");
  this->declare_parameter<std::string>("control_host", "127.0.0.1");
  this->declare_parameter<int>("control_port", 6002);
  this->declare_parameter<double>("max_linear_x", 0.15);
  this->declare_parameter<double>("max_linear_y", 0.15);
  this->declare_parameter<double>("max_angular_z", 0.1);
  this->declare_parameter<double>("cmd_timeout", 0.5);
  this->declare_parameter<double>("command_check_rate", 20.0);
  this->declare_parameter<double>("status_rate", 1.0);
  this->declare_parameter<double>("min_command_interval", 0.1);
  this->declare_parameter<double>("command_epsilon", 1e-3);
  this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel_safe");
  this->declare_parameter<bool>("auto_standup", false);

  // Get parameters
  local_ip_ = this->get_parameter("local_ip").as_string();
  local_port_ = this->get_parameter("local_port").as_int();
  robot_ip_ = this->get_parameter("robot_ip").as_string();
  output_mode_ = this->get_parameter("output_mode").as_string();
  control_host_ = this->get_parameter("control_host").as_string();
  control_port_ = this->get_parameter("control_port").as_int();
  max_linear_x_ = this->get_parameter("max_linear_x").as_double();
  max_linear_y_ = this->get_parameter("max_linear_y").as_double();
  max_angular_z_ = this->get_parameter("max_angular_z").as_double();
  cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();
  command_check_rate_ = this->get_parameter("command_check_rate").as_double();
  status_rate_ = this->get_parameter("status_rate").as_double();
  min_command_interval_ = this->get_parameter("min_command_interval").as_double();
  command_epsilon_ = this->get_parameter("command_epsilon").as_double();
  std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
  bool auto_standup = this->get_parameter("auto_standup").as_bool();

  if (command_check_rate_ <= 0.0) {
    command_check_rate_ = 20.0;
  }
  if (status_rate_ <= 0.0) {
    status_rate_ = 1.0;
  }
  if (min_command_interval_ < 0.0) {
    min_command_interval_ = 0.0;
  }

  RCLCPP_INFO(this->get_logger(), "Initializing CmdVelToZsibot node");
  RCLCPP_INFO(this->get_logger(), "  Output mode: %s", output_mode_.c_str());
  if (usingUdpOutput()) {
    RCLCPP_INFO(this->get_logger(), "  Control UDP: %s:%d", control_host_.c_str(), control_port_);
  } else if (usingFakeOutput()) {
    RCLCPP_INFO(this->get_logger(), "  Fake output enabled: commands are only published to debug topics");
  } else {
    RCLCPP_INFO(this->get_logger(), "  Local IP: %s:%d", local_ip_.c_str(), local_port_);
    RCLCPP_INFO(this->get_logger(), "  Robot IP: %s", robot_ip_.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "  Max velocities: vx=%.2f, vy=%.2f, wz=%.2f",
    max_linear_x_, max_linear_y_, max_angular_z_);
  RCLCPP_INFO(this->get_logger(), "  Cmd timeout: %.2f s", cmd_timeout_);
  RCLCPP_INFO(this->get_logger(), "  Command check rate: %.1f Hz", command_check_rate_);
  RCLCPP_INFO(this->get_logger(), "  Status rate: %.1f Hz", status_rate_);
  RCLCPP_INFO(this->get_logger(), "  Min command interval: %.3f s", min_command_interval_);

  // Initialize latest command to zero
  latest_cmd_.linear.x = 0.0;
  latest_cmd_.linear.y = 0.0;
  latest_cmd_.linear.z = 0.0;
  latest_cmd_.angular.x = 0.0;
  latest_cmd_.angular.y = 0.0;
  latest_cmd_.angular.z = 0.0;
  latest_input_cmd_ = latest_cmd_;
  last_sent_cmd_ = latest_cmd_;

  // Initialize last command time
  last_cmd_time_ = this->now();
  last_send_time_ = this->now();
  has_pending_cmd_ = false;
  has_sent_cmd_ = false;
  robot_stopped_ = true;

  if (usingFakeOutput()) {
    connected_ = true;
    RCLCPP_INFO(this->get_logger(), "Fake command output ready");
  } else if (!usingUdpOutput()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Unsupported output_mode '%s'. SDK direct control is disabled; use output_mode=udp or fake.",
      output_mode_.c_str());
  } else if (!initializeUdpClient()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize UDP command client");
  }

  // Auto standup if configured
  if (auto_standup) {
    RCLCPP_WARN(
      this->get_logger(),
      "auto_standup is disabled for safety; stand up explicitly via web control or ~/stand_up");
  }

  // Create subscribers
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&CmdVelToZsibot::cmdVelCallback, this, _1));

  enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "~/enable",
    10,
    std::bind(&CmdVelToZsibot::enableCallback, this, _1));

  // Create publisher for connection status
  connected_pub_ = this->create_publisher<std_msgs::msg::Bool>("~/connected", 10);

  // Create publisher for battery state
  battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
    "/battery/state", rclcpp::QoS(10).reliable());

  // Create publisher for outgoing command debug
  sent_command_pub_ =
    this->create_publisher<robots_dog_msgs::msg::CmdVelToZsibotDebug>("~/sent_command", 10);
  debug_command_pub_ =
    this->create_publisher<robots_dog_msgs::msg::CmdVelToZsibotDebug>("~/debug_command", 10);

  // Match yz_robot_ctrl semantics: do not stream move() continuously.
  double command_period_ms = 1000.0 / command_check_rate_;
  command_timer_ = this->create_wall_timer(
    std::chrono::duration<double, std::milli>(command_period_ms),
    std::bind(&CmdVelToZsibot::commandTimerCallback, this));

  double status_period_ms = 1000.0 / status_rate_;
  status_timer_ = this->create_wall_timer(
    std::chrono::duration<double, std::milli>(status_period_ms),
    std::bind(&CmdVelToZsibot::statusTimerCallback, this));

  // Create services for robot stance control
  stand_up_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/stand_up",
    std::bind(&CmdVelToZsibot::standUpCallback, this, std::placeholders::_1, std::placeholders::_2));

  lie_down_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/lie_down",
    std::bind(&CmdVelToZsibot::lieDownCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "CmdVelToZsibot node initialized successfully");
  RCLCPP_INFO(this->get_logger(), "  Services: ~/stand_up, ~/lie_down");
}

CmdVelToZsibot::~CmdVelToZsibot()
{
  // Send zero velocity before shutdown
  if (connected_) {
    RCLCPP_INFO(this->get_logger(), "Sending zero velocity before shutdown");
    sendStopCommand(true, "shutdown");
  }
  if (udp_sock_ >= 0) {
    close(udp_sock_);
    udp_sock_ = -1;
  }
}

bool CmdVelToZsibot::initializeUdpClient()
{
  udp_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_sock_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
    connected_ = false;
    return false;
  }

  udp_addr_ = {};
  udp_addr_.sin_family = AF_INET;
  udp_addr_.sin_port = htons(static_cast<uint16_t>(control_port_));
  if (inet_pton(AF_INET, control_host_.c_str(), &udp_addr_.sin_addr) != 1) {
    RCLCPP_ERROR(this->get_logger(), "Invalid control_host: %s", control_host_.c_str());
    close(udp_sock_);
    udp_sock_ = -1;
    connected_ = false;
    return false;
  }

  connected_ = true;
  RCLCPP_INFO(this->get_logger(), "UDP command client ready for yz-robot-ctrl at %s:%d",
              control_host_.c_str(), control_port_);
  return true;
}

void CmdVelToZsibot::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    latest_cmd_ = *msg;
    latest_input_cmd_ = *msg;
    last_cmd_time_ = this->now();
    has_pending_cmd_ = true;
  }

  if (isZeroCommand(normalizeCommand(*msg))) {
    sendStopCommand(false, "cmd_vel_zero");
  }
}

void CmdVelToZsibot::enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  enabled_ = msg->data;
  RCLCPP_INFO(this->get_logger(), "Bridge %s", enabled_ ? "enabled" : "disabled");

  // If disabled, send zero velocity
  if (!enabled_ && connected_) {
    sendStopCommand(true, "disabled");
  }
}

void CmdVelToZsibot::statusTimerCallback()
{
  auto connected_msg = std_msgs::msg::Bool();
  connected_msg.data = connected_;
  connected_pub_->publish(connected_msg);
}

void CmdVelToZsibot::commandTimerCallback()
{
  if (!connected_ || !enabled_) {
    return;
  }

  if (!canEmitCommands()) {
    return;
  }

  double time_since_cmd = (this->now() - last_cmd_time_).seconds();

  if (time_since_cmd > cmd_timeout_) {
    sendStopCommand(false, "cmd_timeout");
    return;
  }

  geometry_msgs::msg::Twist cmd;
  bool has_pending = false;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd = latest_cmd_;
    has_pending = has_pending_cmd_;
    has_pending_cmd_ = false;
  }

  if (!has_pending) {
    return;
  }

  sendMoveCommand(normalizeCommand(cmd), false, "cmd_vel");
}

double CmdVelToZsibot::clamp(double value, double min_val, double max_val)
{
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

geometry_msgs::msg::Twist CmdVelToZsibot::normalizeCommand(
  const geometry_msgs::msg::Twist & cmd)
{
  geometry_msgs::msg::Twist normalized;
  normalized.linear.x = clamp(cmd.linear.x, -max_linear_x_, max_linear_x_);
  normalized.linear.y = clamp(cmd.linear.y, -max_linear_y_, max_linear_y_);
  normalized.angular.z = clamp(cmd.angular.z, -max_angular_z_, max_angular_z_);

  if (std::abs(normalized.linear.x) <= command_epsilon_) {
    normalized.linear.x = 0.0;
  }
  if (std::abs(normalized.linear.y) <= command_epsilon_) {
    normalized.linear.y = 0.0;
  }
  if (std::abs(normalized.angular.z) <= command_epsilon_) {
    normalized.angular.z = 0.0;
  }

  // Same minimums used by yz_robot_ctrl's RobotController::move().
  constexpr double MIN_VX = 0.05;
  constexpr double MIN_VY = 0.1;
  constexpr double MIN_YAW = 0.02;

  if (normalized.linear.x != 0.0 && std::abs(normalized.linear.x) < MIN_VX) {
    normalized.linear.x = normalized.linear.x > 0.0 ? MIN_VX : -MIN_VX;
  }
  if (normalized.linear.y != 0.0 && std::abs(normalized.linear.y) < MIN_VY) {
    normalized.linear.y = normalized.linear.y > 0.0 ? MIN_VY : -MIN_VY;
  }
  if (normalized.angular.z != 0.0 && std::abs(normalized.angular.z) < MIN_YAW) {
    normalized.angular.z = normalized.angular.z > 0.0 ? MIN_YAW : -MIN_YAW;
  }

  return normalized;
}

bool CmdVelToZsibot::commandsDiffer(
  const geometry_msgs::msg::Twist & lhs,
  const geometry_msgs::msg::Twist & rhs) const
{
  return std::abs(lhs.linear.x - rhs.linear.x) > command_epsilon_ ||
         std::abs(lhs.linear.y - rhs.linear.y) > command_epsilon_ ||
         std::abs(lhs.angular.z - rhs.angular.z) > command_epsilon_;
}

bool CmdVelToZsibot::isZeroCommand(const geometry_msgs::msg::Twist & cmd) const
{
  return std::abs(cmd.linear.x) <= command_epsilon_ &&
         std::abs(cmd.linear.y) <= command_epsilon_ &&
         std::abs(cmd.angular.z) <= command_epsilon_;
}

void CmdVelToZsibot::sendMoveCommand(
  const geometry_msgs::msg::Twist & cmd,
  bool force,
  const std::string & reason)
{
  if (!connected_) {
    return;
  }

  const bool zero = isZeroCommand(cmd);
  const double since_last_send = (this->now() - last_send_time_).seconds();
  const bool interval_ok = since_last_send >= min_command_interval_;

  if (!force && !zero && !interval_ok) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    latest_cmd_ = cmd;
    has_pending_cmd_ = true;
    return;
  }

  if (!force && has_sent_cmd_ && !canEmitCommands() && !commandsDiffer(cmd, last_sent_cmd_)) {
    return;
  }

  if (!canEmitCommands()) {
    return;
  }

  std::ostringstream payload;
  payload << "{\"type\":\"twist\",\"vx\":" << cmd.linear.x
          << ",\"vy\":" << cmd.linear.y
          << ",\"wz\":" << cmd.angular.z << "}";
  if (!sendUdpPayload(payload.str(), reason, force, &cmd)) {
    return;
  }

  last_sent_cmd_ = cmd;
  last_send_time_ = this->now();
  has_sent_cmd_ = true;
  robot_stopped_ = zero;
}

void CmdVelToZsibot::sendStopCommand(bool force, const std::string & reason)
{
  geometry_msgs::msg::Twist stop_cmd;
  sendMoveCommand(stop_cmd, force, reason);
}

bool CmdVelToZsibot::sendUdpPayload(
  const std::string & payload,
  const std::string & reason,
  bool force,
  const geometry_msgs::msg::Twist * normalized_cmd)
{
  bool ok = false;
  const bool fake = usingFakeOutput();
  if (fake) {
    ok = true;
  } else if (udp_sock_ >= 0) {
    ssize_t sent = sendto(
      udp_sock_, payload.data(), payload.size(), 0,
      reinterpret_cast<struct sockaddr *>(&udp_addr_), sizeof(udp_addr_));
    ok = sent == static_cast<ssize_t>(payload.size());
  }

  geometry_msgs::msg::Twist input_cmd;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    input_cmd = latest_input_cmd_;
  }

  auto debug = robots_dog_msgs::msg::CmdVelToZsibotDebug();
  debug.header.stamp = this->now();
  debug.header.frame_id = "cmd_vel_to_zsibot";
  debug.seq = ++debug_seq_;
  debug.reason = reason;
  debug.output_mode = output_mode_;
  debug.fake = fake;
  debug.send_ok = ok;
  debug.connected = connected_;
  debug.enabled = enabled_;
  debug.force = force;
  debug.input_cmd = input_cmd;
  debug.has_normalized_cmd = normalized_cmd != nullptr;
  debug.normalized_cmd = normalized_cmd != nullptr ? *normalized_cmd : zeroTwist();
  debug.zero_command = normalized_cmd != nullptr && isZeroCommand(*normalized_cmd);
  debug.robot_stopped = robot_stopped_;
  debug.payload = payload;
  debug.cmd_age_sec = (this->now() - last_cmd_time_).seconds();
  debug.last_send_age_sec = (this->now() - last_send_time_).seconds();

  if (debug_command_pub_) {
    debug_command_pub_->publish(debug);
  }

  if (!ok) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Failed to send UDP command to %s:%d", control_host_.c_str(), control_port_);
  } else if (sent_command_pub_) {
    sent_command_pub_->publish(debug);
  }
  return ok;
}

bool CmdVelToZsibot::usingUdpOutput() const
{
  return output_mode_ == "udp";
}

bool CmdVelToZsibot::canEmitCommands() const
{
  return usingUdpOutput() || usingFakeOutput();
}

bool CmdVelToZsibot::usingFakeOutput() const
{
  return output_mode_ == "fake";
}

void CmdVelToZsibot::standUpCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!connected_) {
    response->success = false;
    response->message = "Robot not connected";
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Standing up robot...");
  if (!canEmitCommands()) {
    response->success = false;
    response->message = "SDK direct control is disabled";
    return;
  }
  if (!sendUdpPayload("c", "stand_up", true)) {
    response->success = false;
    response->message = "Failed to send stand-up command";
    return;
  }

  // Wait for standup to complete
  std::this_thread::sleep_for(3000ms);
  standing_ = true;

  response->success = true;
  response->message = "Robot stood up";
  RCLCPP_INFO(this->get_logger(), "Robot stood up successfully, ready for commands");
}

void CmdVelToZsibot::lieDownCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!connected_) {
    response->success = false;
    response->message = "Robot not connected";
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Lying down robot...");

  // Mark as not standing first to stop move commands
  standing_ = false;

  // Stop first, then lie down (like yz_robot_ctrl gracefulShutdown)
  sendStopCommand(true, "lie_down_stop");
  std::this_thread::sleep_for(500ms);
  if (!canEmitCommands()) {
    response->success = false;
    response->message = "SDK direct control is disabled";
    return;
  }
  if (!sendUdpPayload("x", "lie_down", true)) {
    response->success = false;
    response->message = "Failed to send lie-down command";
    return;
  }

  // Wait for lie down to complete
  std::this_thread::sleep_for(2000ms);

  response->success = true;
  response->message = "Robot laid down";
  RCLCPP_INFO(this->get_logger(), "Robot laid down successfully");
}

}  // namespace cmd_vel_to_zsibot

// Register the component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cmd_vel_to_zsibot::CmdVelToZsibot)
