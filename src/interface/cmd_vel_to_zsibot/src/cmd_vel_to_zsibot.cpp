// Copyright 2024 ZsiBot Team
// Licensed under the Apache License, Version 2.0

#include "cmd_vel_to_zsibot/cmd_vel_to_zsibot.hpp"

#include <chrono>
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
  this->declare_parameter<double>("max_linear_x", 2.0);
  this->declare_parameter<double>("max_linear_y", 1.0);
  this->declare_parameter<double>("max_angular_z", 2.0);
  this->declare_parameter<double>("cmd_timeout", 0.5);
  this->declare_parameter<double>("publish_rate", 100.0);
  this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
  this->declare_parameter<bool>("auto_standup", false);

  // Get parameters
  local_ip_ = this->get_parameter("local_ip").as_string();
  local_port_ = this->get_parameter("local_port").as_int();
  robot_ip_ = this->get_parameter("robot_ip").as_string();
  max_linear_x_ = this->get_parameter("max_linear_x").as_double();
  max_linear_y_ = this->get_parameter("max_linear_y").as_double();
  max_angular_z_ = this->get_parameter("max_angular_z").as_double();
  cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
  bool auto_standup = this->get_parameter("auto_standup").as_bool();

  RCLCPP_INFO(this->get_logger(), "Initializing CmdVelToZsibot node");
  RCLCPP_INFO(this->get_logger(), "  Local IP: %s:%d", local_ip_.c_str(), local_port_);
  RCLCPP_INFO(this->get_logger(), "  Robot IP: %s", robot_ip_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Max velocities: vx=%.2f, vy=%.2f, wz=%.2f",
    max_linear_x_, max_linear_y_, max_angular_z_);
  RCLCPP_INFO(this->get_logger(), "  Cmd timeout: %.2f s", cmd_timeout_);
  RCLCPP_INFO(this->get_logger(), "  Publish rate: %.1f Hz", publish_rate_);

  // Initialize latest command to zero
  latest_cmd_.linear.x = 0.0;
  latest_cmd_.linear.y = 0.0;
  latest_cmd_.linear.z = 0.0;
  latest_cmd_.angular.x = 0.0;
  latest_cmd_.angular.y = 0.0;
  latest_cmd_.angular.z = 0.0;

  // Initialize last command time
  last_cmd_time_ = this->now();

  // Initialize ZsiBot SDK
  if (!initializeRobot()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize robot connection");
  }

  // Auto standup if configured
  if (auto_standup && connected_) {
    RCLCPP_INFO(this->get_logger(), "Auto standing up robot...");
    highlevel_->standUp();
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

  // Create timer for fixed-rate command sending
  double period_ms = 1000.0 / publish_rate_;
  timer_ = this->create_wall_timer(
    std::chrono::duration<double, std::milli>(period_ms),
    std::bind(&CmdVelToZsibot::timerCallback, this));

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
    highlevel_->move(0.0, 0.0, 0.0);
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
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  latest_cmd_ = *msg;
  last_cmd_time_ = this->now();
}

void CmdVelToZsibot::enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  enabled_ = msg->data;
  RCLCPP_INFO(this->get_logger(), "Bridge %s", enabled_ ? "enabled" : "disabled");
  
  // If disabled, send zero velocity
  if (!enabled_ && highlevel_ && connected_) {
    highlevel_->move(0.0, 0.0, 0.0);
  }
}

void CmdVelToZsibot::timerCallback()
{
  if (!highlevel_) {
    return;
  }

  // Heartbeat: periodically check connection by getting battery (like yz_robot_ctrl)
  // Do this every ~1 second (100 calls at 100Hz)
  static int heartbeat_counter = 0;
  if (++heartbeat_counter >= 100) {
    heartbeat_counter = 0;
    try {
      uint32_t battery = highlevel_->getBatteryPower();
      if (!connected_) {
        connected_ = true;
        RCLCPP_INFO(this->get_logger(), "Connection restored (Battery: %u%%)", battery);
      }

      // Publish battery state
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
  }

  // Publish connection status
  auto connected_msg = std_msgs::msg::Bool();
  connected_msg.data = connected_;
  connected_pub_->publish(connected_msg);

  if (!connected_ || !enabled_) {
    return;
  }

  // Only send move commands when robot is standing
  if (!standing_) {
    return;
  }

  // Check for command timeout
  double time_since_cmd = (this->now() - last_cmd_time_).seconds();
  
  double vx, vy, wz;
  
  if (time_since_cmd > cmd_timeout_) {
    // Timeout - send zero velocity
    vx = 0.0;
    vy = 0.0;
    wz = 0.0;
  } else {
    // Get latest command
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    vx = clamp(latest_cmd_.linear.x, -max_linear_x_, max_linear_x_);
    vy = clamp(latest_cmd_.linear.y, -max_linear_y_, max_linear_y_);
    wz = clamp(latest_cmd_.angular.z, -max_angular_z_, max_angular_z_);
  }

  // Send velocity command to robot
  // Note: ZsiBot SDK move() takes (vx, vy, yaw_rate)
  // vx: forward velocity (m/s)
  // vy: lateral velocity (m/s), positive = left
  // yaw_rate: angular velocity (rad/s), positive = counter-clockwise
  highlevel_->move(static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(wz));
}

double CmdVelToZsibot::clamp(double value, double min_val, double max_val)
{
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
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
  highlevel_->standUp();
  
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
  highlevel_->move(0.0, 0.0, 0.0);
  std::this_thread::sleep_for(500ms);
  highlevel_->lieDown();
  
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
