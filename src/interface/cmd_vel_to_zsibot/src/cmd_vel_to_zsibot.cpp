// Copyright 2024 ZsiBot Team
// Licensed under the Apache License, Version 2.0

#include "cmd_vel_to_zsibot/cmd_vel_to_zsibot.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <thread>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace cmd_vel_to_zsibot
{

CmdVelToZsibot::CmdVelToZsibot(const rclcpp::NodeOptions & options)
: Node("cmd_vel_to_zsibot", options),
  enabled_(true),
  connected_(false),
  standing_(false)
{
  // Declare parameters
  // Default IPs match yz_robot_ctrl configuration
  this->declare_parameter<std::string>("local_ip", "192.168.168.2");  // RK3588/NanoPC eth0 IP
  this->declare_parameter<int>("local_port", 43988);
  this->declare_parameter<std::string>("robot_ip", "192.168.168.168");  // Robot control board IP
  this->declare_parameter<double>("max_linear_x", 0.15);
  this->declare_parameter<double>("max_linear_y", 0.15);
  this->declare_parameter<double>("max_angular_z", 0.25);
  this->declare_parameter<double>("cmd_timeout", 0.5);
  this->declare_parameter<double>("command_check_rate", 20.0);
  this->declare_parameter<double>("status_rate", 1.0);
  this->declare_parameter<double>("min_command_interval", 1.0);
  this->declare_parameter<double>("command_epsilon", 1e-3);
  this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel_safe");
  this->declare_parameter<bool>("auto_standup", false);

  // Get parameters
  local_ip_ = this->get_parameter("local_ip").as_string();
  local_port_ = this->get_parameter("local_port").as_int();
  robot_ip_ = this->get_parameter("robot_ip").as_string();
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
  RCLCPP_INFO(this->get_logger(), "  Local IP: %s:%d", local_ip_.c_str(), local_port_);
  RCLCPP_INFO(this->get_logger(), "  Robot IP: %s", robot_ip_.c_str());
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
  last_sent_cmd_ = latest_cmd_;

  // Initialize last command time
  last_cmd_time_ = this->now();
  last_send_time_ = this->now();
  has_pending_cmd_ = false;
  has_sent_cmd_ = false;
  robot_stopped_ = true;

  // Initialize ZsiBot SDK
  if (!initializeRobot()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize robot connection");
  }

  // Auto standup if configured
  if (auto_standup && connected_) {
    RCLCPP_INFO(this->get_logger(), "Auto standing up robot...");
    {
      std::lock_guard<std::mutex> lock(sdk_mutex_);
      highlevel_->standUp();
    }
    // Wait for standup to complete before allowing move commands
    std::this_thread::sleep_for(3000ms);
    standing_ = true;
    RCLCPP_INFO(this->get_logger(), "Robot stood up, ready for commands");
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
  if (highlevel_ && connected_) {
    RCLCPP_INFO(this->get_logger(), "Sending zero velocity before shutdown");
    sendStopCommand(true);
  }
}

bool CmdVelToZsibot::initializeRobot()
{
  try {
    highlevel_ = std::make_unique<mc_sdk::zsl_1::HighLevel>();

    RCLCPP_INFO(this->get_logger(), "Initializing robot connection...");
    highlevel_->initRobot(local_ip_, local_port_, robot_ip_);

    // Wait for connection to establish (yz_robot_ctrl uses 500ms)
    std::this_thread::sleep_for(500ms);

    // Test connection by actually communicating (like yz_robot_ctrl does)
    // Use getBatteryPower() instead of checkConnect() as it's more reliable
    try {
      uint32_t battery = highlevel_->getBatteryPower();
      connected_ = true;
      RCLCPP_INFO(this->get_logger(), "Successfully connected to ZsiBot at %s (Battery: %u%%)",
                  robot_ip_.c_str(), battery);
    } catch (...) {
      connected_ = false;
      RCLCPP_WARN(this->get_logger(), "Connection test failed - robot may not be responding");
    }

    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during robot initialization: %s", e.what());
    return false;
  }
}

void CmdVelToZsibot::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    latest_cmd_ = *msg;
    last_cmd_time_ = this->now();
    has_pending_cmd_ = true;
  }

  if (isZeroCommand(normalizeCommand(*msg))) {
    sendStopCommand(false);
  }
}

void CmdVelToZsibot::enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  enabled_ = msg->data;
  RCLCPP_INFO(this->get_logger(), "Bridge %s", enabled_ ? "enabled" : "disabled");

  // If disabled, send zero velocity
  if (!enabled_ && highlevel_ && connected_) {
    sendStopCommand(true);
  }
}

void CmdVelToZsibot::statusTimerCallback()
{
  if (!highlevel_) {
    return;
  }

  try {
    uint32_t battery = 0;
    {
      std::lock_guard<std::mutex> lock(sdk_mutex_);
      battery = highlevel_->getBatteryPower();
    }
    if (!connected_) {
      connected_ = true;
      RCLCPP_INFO(this->get_logger(), "Connection restored (Battery: %u%%)", battery);
    }

    auto battery_msg = sensor_msgs::msg::BatteryState();
    battery_msg.header.stamp = this->now();
    battery_msg.header.frame_id = "base_link";
    battery_msg.percentage = static_cast<float>(battery) / 100.0f;
    battery_msg.voltage = std::numeric_limits<float>::quiet_NaN();
    battery_msg.current = std::numeric_limits<float>::quiet_NaN();
    battery_msg.temperature = std::numeric_limits<float>::quiet_NaN();
    battery_msg.power_supply_status =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    battery_msg.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    battery_msg.present = true;
    battery_pub_->publish(battery_msg);
  } catch (...) {
    if (connected_) {
      connected_ = false;
      RCLCPP_WARN(this->get_logger(), "Connection lost");
    }
  }

  auto connected_msg = std_msgs::msg::Bool();
  connected_msg.data = connected_;
  connected_pub_->publish(connected_msg);
}

void CmdVelToZsibot::commandTimerCallback()
{
  if (!connected_ || !enabled_) {
    return;
  }

  // Only send move commands when robot is standing
  if (!standing_) {
    return;
  }

  double time_since_cmd = (this->now() - last_cmd_time_).seconds();

  if (time_since_cmd > cmd_timeout_) {
    sendStopCommand(false);
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

  sendMoveCommand(normalizeCommand(cmd));
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

void CmdVelToZsibot::sendMoveCommand(const geometry_msgs::msg::Twist & cmd, bool force)
{
  if (!highlevel_ || !connected_) {
    return;
  }

  std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);

  const bool zero = isZeroCommand(cmd);
  const double since_last_send = (this->now() - last_send_time_).seconds();
  const bool interval_ok = since_last_send >= min_command_interval_;

  if (!force && !zero && !interval_ok) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    latest_cmd_ = cmd;
    has_pending_cmd_ = true;
    return;
  }

  if (!force && has_sent_cmd_ && !commandsDiffer(cmd, last_sent_cmd_)) {
    return;
  }

  highlevel_->move(
    static_cast<float>(cmd.linear.x),
    static_cast<float>(cmd.linear.y),
    static_cast<float>(cmd.angular.z));

  last_sent_cmd_ = cmd;
  last_send_time_ = this->now();
  has_sent_cmd_ = true;
  robot_stopped_ = zero;
}

void CmdVelToZsibot::sendStopCommand(bool force)
{
  geometry_msgs::msg::Twist stop_cmd;
  sendMoveCommand(stop_cmd, force);
}

void CmdVelToZsibot::standUpCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!highlevel_ || !connected_) {
    response->success = false;
    response->message = "Robot not connected";
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Standing up robot...");
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    highlevel_->standUp();
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
  if (!highlevel_ || !connected_) {
    response->success = false;
    response->message = "Robot not connected";
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Lying down robot...");

  // Mark as not standing first to stop move commands
  standing_ = false;

  // Stop first, then lie down (like yz_robot_ctrl gracefulShutdown)
  sendStopCommand(true);
  std::this_thread::sleep_for(500ms);
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    highlevel_->lieDown();
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
