// Adapted from ros-navigation/navigation2 tag 1.3.13: nav2_controller/src/controller_server.cpp
// Jazzy ABI is retained; ordered segment coordination is local to this optional server.
// Copyright (c) 2019 Intel Corporation
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

#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <limits>
#include <functional>
#include <cmath>
#include <iomanip>
#include <sstream>
#include "robot_feasible_navigation/footprint_sweep.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "robot_feasible_navigation/segment_controller_server.hpp"
#include "robot_feasible_navigation/goal_cell_binding.hpp"
#include "robot_feasible_navigation/bounded_planar_command.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace robot_feasible_navigation
{
namespace
{
// RPP reads the XY tolerance to decide when to rotate. Supplying the ordinary .25m
// final tolerance at an intermediate turn would stop translation prematurely.
class IntermediateGoalChecker final : public nav2_core::GoalChecker
{
public:
  IntermediateGoalChecker(const SegmentCoordinator::Config & config,
    std::function<double()> phase_xy_tolerance)
  : config_(config), phase_xy_tolerance_(std::move(phase_xy_tolerance)) {}
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    const std::string &, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(const geometry_msgs::msg::Pose & pose,
    const geometry_msgs::msg::Pose & goal, const geometry_msgs::msg::Twist & velocity) override
  {
    return std::hypot(pose.position.x - goal.position.x, pose.position.y - goal.position.y) <=
           xyTolerance() && std::abs(SegmentCoordinator::angle(
             SegmentCoordinator::yaw(goal) - SegmentCoordinator::yaw(pose))) <=
           config_.yaw_tolerance && std::hypot(velocity.linear.x, velocity.linear.y) <=
           config_.stopped_linear_velocity &&
           std::abs(velocity.angular.z) <= config_.stopped_angular_velocity;
  }
  bool getTolerances(geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & velocity) override
  {
    pose.position.x = xyTolerance(); pose.position.y = pose.position.x;
    pose.position.z = std::numeric_limits<double>::lowest();
    pose.orientation.z = std::sin(config_.yaw_tolerance * .5);
    pose.orientation.w = std::cos(config_.yaw_tolerance * .5);
    velocity.linear.x = config_.stopped_linear_velocity;
    velocity.linear.y = config_.stopped_linear_velocity;
    velocity.angular.z = config_.stopped_angular_velocity;
    return true;
  }
private:
  double xyTolerance() const {return std::min(config_.xy_tolerance, phase_xy_tolerance_());}
  SegmentCoordinator::Config config_;
  std::function<double()> phase_xy_tolerance_;
};
struct ScopeExit
{
  std::function<void()> callback;
  ~ScopeExit() {callback();}
};
}  // namespace

SegmentControllerServer::SegmentControllerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("controller_server", "", options),
  progress_checker_loader_("nav2_core", "nav2_core::ProgressChecker"),
  default_progress_checker_ids_{"progress_checker"},
  default_progress_checker_types_{"nav2_controller::SimpleProgressChecker"},
  goal_checker_loader_("nav2_core", "nav2_core::GoalChecker"),
  default_goal_checker_ids_{"goal_checker"},
  default_goal_checker_types_{"nav2_controller::SimpleGoalChecker"},
  lp_loader_("nav2_core", "nav2_core::Controller"),
  default_ids_{"FollowPath"},
  default_types_{"dwb_core::DWBLocalPlanner"},
  costmap_update_timeout_(300ms)
{
  RCLCPP_INFO(get_logger(), "Creating controller server");

  declare_parameter("controller_frequency", 20.0);

  declare_parameter("action_server_result_timeout", 10.0);

  declare_parameter("progress_checker_plugins", default_progress_checker_ids_);
  declare_parameter("goal_checker_plugins", default_goal_checker_ids_);
  declare_parameter("controller_plugins", default_ids_);
  declare_parameter("min_x_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_y_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_theta_velocity_threshold", rclcpp::ParameterValue(0.0001));

  declare_parameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));

  declare_parameter("failure_tolerance", rclcpp::ParameterValue(0.0));
  declare_parameter("use_realtime_priority", rclcpp::ParameterValue(false));
  declare_parameter("publish_zero_velocity", rclcpp::ParameterValue(true));
  declare_parameter("costmap_update_timeout", 0.30);  // 300ms

  // Fixed for each lifecycle configuration: a dynamic update must not change a live phase.
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  const SegmentCoordinator::Config config;
  declare_parameter("segments.xy_tolerance", config.xy_tolerance, descriptor);
  declare_parameter("segments.yaw_tolerance", config.yaw_tolerance, descriptor);
  declare_parameter("segments.final_stop_dwell_sec", .3, descriptor);
  declare_parameter("segments.stopped_linear_velocity", config.stopped_linear_velocity, descriptor);
  declare_parameter("segments.stopped_angular_velocity", config.stopped_angular_velocity, descriptor);
  declare_parameter("segments.same_xy_epsilon", config.same_xy_epsilon, descriptor);
  declare_parameter("segments.progress_search_distance", config.progress_search_distance, descriptor);
  declare_parameter("segments.odom_timeout", config.odom_timeout, descriptor);
  declare_parameter("segments.rotation_anchor_wait_timeout", config.rotation_anchor_wait_timeout, descriptor);
  declare_parameter("segments.command_sweep_horizon", 1.0, descriptor);
  declare_parameter("segments.max_angular_velocity", kExperimentMaxAngularVelocity, descriptor);
  declare_parameter("segments.terminal_goal_binding", false, descriptor);
  declare_parameter("segments.terminal_global_resolution", .01, descriptor);
  declare_parameter("segments.terminal_global_extent", 20., descriptor);

  // The costmap node is used in the implementation of the controller
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "local_costmap", std::string{get_namespace()}, "local_costmap",
    get_parameter("use_sim_time").as_bool());
}

SegmentControllerServer::~SegmentControllerServer()
{
  // Existing SimpleActionServer::deactivate waits for its execution future to finish.
  // Stop/join it BEFORE any plugin or costmap dependency can be destroyed.
  if (action_server_) {action_server_->deactivate();}
  action_server_.reset();
  clearSegmentState();
  progress_checkers_.clear();
  goal_checkers_.clear();
  controllers_.clear();
  costmap_thread_.reset();
}

nav2_util::CallbackReturn
SegmentControllerServer::on_configure(const rclcpp_lifecycle::State & state)
{
  auto node = shared_from_this();
  get_parameter("segments.terminal_goal_binding",terminal_goal_binding_);
  get_parameter("segments.terminal_global_resolution",terminal_global_resolution_);
  get_parameter("segments.terminal_global_extent",terminal_global_extent_);
  if (terminal_goal_binding_ && (!std::isfinite(terminal_global_resolution_) ||
    terminal_global_resolution_<=0. || !std::isfinite(terminal_global_extent_) || terminal_global_extent_<=0.))
  {
    RCLCPP_ERROR(get_logger(),"Invalid fixed goal binding geometry");
    return nav2_util::CallbackReturn::FAILURE;
  }

  SegmentCoordinator::Config segment_config;
  get_parameter("segments.xy_tolerance", segment_config.xy_tolerance);
  get_parameter("segments.yaw_tolerance", segment_config.yaw_tolerance);
  if (get_parameter("segments.final_stop_dwell_sec").as_double()!=
      OriginalGoalStopDwell::kDwellNs/1.e9)
  {throw std::runtime_error("Final original-goal stop dwell must be .3 seconds");}
  get_parameter("segments.stopped_linear_velocity", segment_config.stopped_linear_velocity);
  get_parameter("segments.stopped_angular_velocity", segment_config.stopped_angular_velocity);
  get_parameter("segments.same_xy_epsilon", segment_config.same_xy_epsilon);
  get_parameter("segments.progress_search_distance", segment_config.progress_search_distance);
  get_parameter("segments.odom_timeout", segment_config.odom_timeout);
  get_parameter("segments.rotation_anchor_wait_timeout", segment_config.rotation_anchor_wait_timeout);
  try {
    if (!validExperimentAngularLimit(
      get_parameter("segments.max_angular_velocity").as_double()))
    {
      throw std::invalid_argument("segments.max_angular_velocity must be finite in (0, .3]");
    }
    coordinator_ = SegmentCoordinator(segment_config);
    intermediate_goal_checker_ = std::make_unique<IntermediateGoalChecker>(segment_config,
      [this]() {return coordinator_.rotating() ? coordinator_.config().xy_tolerance :
        originalGoalTolerances().first;});
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(get_logger(), "Invalid segment configuration: %s", e.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Configuring controller interface");

  RCLCPP_INFO(get_logger(), "getting progress checker plugins..");
  get_parameter("progress_checker_plugins", progress_checker_ids_);
  if (progress_checker_ids_ == default_progress_checker_ids_) {
    for (size_t i = 0; i < default_progress_checker_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_progress_checker_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_progress_checker_types_[i]));
    }
  }

  RCLCPP_INFO(get_logger(), "getting goal checker plugins..");
  get_parameter("goal_checker_plugins", goal_checker_ids_);
  if (goal_checker_ids_ == default_goal_checker_ids_) {
    for (size_t i = 0; i < default_goal_checker_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_goal_checker_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_goal_checker_types_[i]));
    }
  }

  get_parameter("controller_plugins", controller_ids_);
  if (controller_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_types_[i]));
    }
  }

  controller_types_.resize(controller_ids_.size());
  goal_checker_types_.resize(goal_checker_ids_.size());
  progress_checker_types_.resize(progress_checker_ids_.size());

  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("min_x_velocity_threshold", min_x_velocity_threshold_);
  get_parameter("min_y_velocity_threshold", min_y_velocity_threshold_);
  get_parameter("min_theta_velocity_threshold", min_theta_velocity_threshold_);
  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  std::string speed_limit_topic;
  get_parameter("speed_limit_topic", speed_limit_topic);
  get_parameter("failure_tolerance", failure_tolerance_);
  get_parameter("use_realtime_priority", use_realtime_priority_);

  costmap_ros_->configure();
  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  for (size_t i = 0; i != progress_checker_ids_.size(); i++) {
    try {
      progress_checker_types_[i] = nav2_util::get_plugin_type_param(
        node, progress_checker_ids_[i]);
      nav2_core::ProgressChecker::Ptr progress_checker =
        progress_checker_loader_.createUniqueInstance(progress_checker_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created progress_checker : %s of type %s",
        progress_checker_ids_[i].c_str(), progress_checker_types_[i].c_str());
      progress_checker->initialize(node, progress_checker_ids_[i]);
      progress_checkers_.insert({progress_checker_ids_[i], progress_checker});
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create progress_checker. Exception: %s", ex.what());
      on_cleanup(state);
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != progress_checker_ids_.size(); i++) {
    progress_checker_ids_concat_ += progress_checker_ids_[i] + std::string(" ");
  }
  if (progress_checker_ids_concat_.empty()) {
    progress_checker_ids_concat_ = "(none)";
  }

  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s progress checkers available.", progress_checker_ids_concat_.c_str());

  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    try {
      goal_checker_types_[i] = nav2_util::get_plugin_type_param(node, goal_checker_ids_[i]);
      nav2_core::GoalChecker::Ptr goal_checker =
        goal_checker_loader_.createUniqueInstance(goal_checker_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created goal checker : %s of type %s",
        goal_checker_ids_[i].c_str(), goal_checker_types_[i].c_str());
      goal_checker->initialize(node, goal_checker_ids_[i], costmap_ros_);
      goal_checkers_.insert({goal_checker_ids_[i], goal_checker});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create goal checker. Exception: %s", ex.what());
      on_cleanup(state);
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    goal_checker_ids_concat_ += goal_checker_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s goal checkers available.", goal_checker_ids_concat_.c_str());

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    try {
      controller_types_[i] = nav2_util::get_plugin_type_param(node, controller_ids_[i]);
      nav2_core::Controller::Ptr controller =
        lp_loader_.createUniqueInstance(controller_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created controller : %s of type %s",
        controller_ids_[i].c_str(), controller_types_[i].c_str());
      controller->configure(
        node, controller_ids_[i],
        costmap_ros_->getTfBuffer(), costmap_ros_);
      controllers_.insert({controller_ids_[i], controller});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create controller. Exception: %s", ex.what());
      on_cleanup(state);
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    controller_ids_concat_ += controller_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Controller Server has %s controllers available.", controller_ids_concat_.c_str());

  odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);
  vel_publisher_ = std::make_unique<nav2_util::TwistPublisher>(node, "cmd_vel", 1);

  double action_server_result_timeout;
  get_parameter("action_server_result_timeout", action_server_result_timeout);
  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

  double costmap_update_timeout_dbl;
  get_parameter("costmap_update_timeout", costmap_update_timeout_dbl);
  costmap_update_timeout_ = rclcpp::Duration::from_seconds(costmap_update_timeout_dbl);

  // Create the action server that we implement with our followPath method
  // This may throw due to real-time prioritzation if user doesn't have real-time permissions
  try {
    action_server_ = std::make_unique<ActionServer>(
      shared_from_this(),
      "follow_path",
      std::bind(&SegmentControllerServer::computeControl, this),
      nullptr,
      std::chrono::milliseconds(500),
      true /*spin thread*/, server_options, use_realtime_priority_ /*soft realtime*/);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(get_logger(), "Error creating action server! %s", e.what());
    on_cleanup(state);
    return nav2_util::CallbackReturn::FAILURE;
  }

  // Set subscribtion to the speed limiting topic
  speed_limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(
    speed_limit_topic, rclcpp::QoS(10),
    std::bind(&SegmentControllerServer::speedLimitCallback, this, std::placeholders::_1));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SegmentControllerServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Nav2 1.3.13 Costmap2DROS::on_activate builds its initial TF deadline from ROS
  // now(). Starting at zero then receiving a bag epoch can expire that deadline
  // immediately. Require the child's real simulation clock, not a wall/epoch offset.
  if (costmap_ros_->get_parameter("use_sim_time").as_bool()) {
    auto clock = costmap_ros_->get_clock();
    const auto begin = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Waiting for child simulation clock: active=%d now_ns=%lld",
      clock->ros_time_is_active(), static_cast<long long>(clock->now().nanoseconds()));
    rclcpp::WallRate wait(100);
    while (rclcpp::ok() && !(clock->ros_time_is_active() && clock->now().nanoseconds() > 0)) {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - begin).count();
      if (SegmentCoordinator::inputWaitStatus(false, elapsed, 5.0) ==
        SegmentCoordinator::InputWaitStatus::TimedOut)
      {
        RCLCPP_ERROR(get_logger(), "Child simulation clock did not start within steady 5s: "
          "active=%d now_ns=%lld", clock->ros_time_is_active(),
          static_cast<long long>(clock->now().nanoseconds()));
        return nav2_util::CallbackReturn::FAILURE;
      }
      wait.sleep();
    }
    if (!rclcpp::ok()) {return nav2_util::CallbackReturn::FAILURE;}
    RCLCPP_INFO(get_logger(), "Child simulation clock ready: active=%d now_ns=%lld",
      clock->ros_time_is_active(), static_cast<long long>(clock->now().nanoseconds()));
  }

  const auto costmap_ros_state = costmap_ros_->activate();
  if (costmap_ros_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return nav2_util::CallbackReturn::FAILURE;
  }
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->activate();
  }
  vel_publisher_->on_activate();
  action_server_->activate();

  auto node = shared_from_this();
  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SegmentControllerServer::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SegmentControllerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();
  clearSegmentState();
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->deactivate();
  }

  /*
   * The costmap is also a lifecycle node, so it may have already fired on_deactivate
   * via rcl preshutdown cb. Despite the rclcpp docs saying on_shutdown callbacks fire
   * in the order added, the preshutdown callbacks clearly don't per se, due to using an
   * unordered_set iteration. Once this issue is resolved, we can maybe make a stronger
   * ordering assumption: https://github.com/ros2/rclcpp/issues/2096
   */
  costmap_ros_->deactivate();

  // Always publish a zero velocity when deactivating the controller server
  geometry_msgs::msg::TwistStamped velocity;
  velocity.twist.angular.x = 0;
  velocity.twist.angular.y = 0;
  velocity.twist.angular.z = 0;
  velocity.twist.linear.x = 0;
  velocity.twist.linear.y = 0;
  velocity.twist.linear.z = 0;
  velocity.header.frame_id = costmap_ros_->getBaseFrameID();
  velocity.header.stamp = now();
  publishVelocity(velocity);

  vel_publisher_->on_deactivate();

  remove_on_set_parameters_callback(dyn_params_handler_.get());
  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SegmentControllerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  if (action_server_) {action_server_->deactivate();}

  // Cleanup the helper classes
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->cleanup();
  }
  controllers_.clear();

  goal_checkers_.clear();
  progress_checkers_.clear();

  costmap_ros_->cleanup();


  // Release any allocated resources
  action_server_.reset();
  clearSegmentState();
  intermediate_goal_checker_.reset();
  odom_sub_.reset();
  costmap_thread_.reset();
  vel_publisher_.reset();
  speed_limit_sub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SegmentControllerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  if (action_server_) {action_server_->deactivate();}
  clearSegmentState();
  if (vel_publisher_) {publishZeroVelocity();}
  return nav2_util::CallbackReturn::SUCCESS;
}

bool SegmentControllerServer::findControllerId(
  const std::string & c_name,
  std::string & current_controller)
{
  if (controllers_.find(c_name) == controllers_.end()) {
    if (controllers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No controller was specified in action call."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.", controller_ids_concat_.c_str());
      current_controller = controllers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with controller name %s, "
        "which does not exist. Available controllers are: %s.",
        c_name.c_str(), controller_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected controller: %s.", c_name.c_str());
    current_controller = c_name;
  }

  return true;
}

bool SegmentControllerServer::findGoalCheckerId(
  const std::string & c_name,
  std::string & current_goal_checker)
{
  if (goal_checkers_.find(c_name) == goal_checkers_.end()) {
    if (goal_checkers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No goal checker was specified in parameter 'current_goal_checker'."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.", goal_checker_ids_concat_.c_str());
      current_goal_checker = goal_checkers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with goal_checker name %s in parameter"
        " 'current_goal_checker', which does not exist. Available goal checkers are: %s.",
        c_name.c_str(), goal_checker_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected goal checker: %s.", c_name.c_str());
    current_goal_checker = c_name;
  }

  return true;
}

bool SegmentControllerServer::findProgressCheckerId(
  const std::string & c_name,
  std::string & current_progress_checker)
{
  if (progress_checkers_.size() == 0) {
    if (c_name.empty()) {
      RCLCPP_DEBUG(
        get_logger(),
        "No progress checker configured and none requested. Progress checking will be bypassed.");
      current_progress_checker = "";
      return true;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with progress_checker name %s in parameter"
        " 'current_progress_checker', but no progress checkers are configured.",
        c_name.c_str());
      return false;
    }
  }

  if (progress_checkers_.find(c_name) == progress_checkers_.end()) {
    if (progress_checkers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No progress checker was specified in parameter 'current_progress_checker'."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.", progress_checker_ids_concat_.c_str());
      current_progress_checker = progress_checkers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with progress_checker name %s in parameter"
        " 'current_progress_checker', which does not exist. Available progress checkers are: %s.",
        c_name.c_str(), progress_checker_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected progress checker: %s.", c_name.c_str());
    current_progress_checker = c_name;
  }

  return true;
}

void SegmentControllerServer::computeControl()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  clearSegmentState();
  ScopeExit clear_segments{[this]() {clearSegmentState();}};

  RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort.");

  try {
    auto goal = action_server_->get_current_goal();
    if (!goal) {
      return;  //  goal would be nullptr if action_server_ is inactivate.
    }

    std::string c_name = goal->controller_id;
    std::string current_controller;
    if (findControllerId(c_name, current_controller)) {
      current_controller_ = current_controller;
    } else {
      throw nav2_core::InvalidController("Failed to find controller name: " + c_name);
    }

    std::string gc_name = goal->goal_checker_id;
    std::string current_goal_checker;
    if (findGoalCheckerId(gc_name, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      throw nav2_core::ControllerException("Failed to find goal checker name: " + gc_name);
    }

    std::string pc_name = goal->progress_checker_id;
    std::string current_progress_checker;
    if (findProgressCheckerId(pc_name, current_progress_checker)) {
      current_progress_checker_ = current_progress_checker;
    } else {
      throw nav2_core::ControllerException("Failed to find progress checker name: " + pc_name);
    }

    setPlannerPath(goal->path);

    last_valid_cmd_time_ = now();
    rclcpp::WallRate loop_rate(controller_frequency_);
    while (rclcpp::ok()) {
      auto start_time = this->now();

      if (action_server_ == nullptr || !action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
        publishZeroVelocity();
        if (action_server_) {action_server_->terminate_all();}
        return;
      }

      if (action_server_->is_cancel_requested()) {
        resetFinalGoalStop();  // Pending cancellation cannot mature final-stop history.
        if (controllers_[current_controller_]->cancel()) {
          RCLCPP_INFO(get_logger(), "Cancellation was successful. Stopping the robot.");
          action_server_->terminate_all();
          publishZeroVelocity();
          return;
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000, "Waiting for the controller to finish cancellation");
        }
      }

      // Don't compute a trajectory until costmap is valid (after clear costmap)
      rclcpp::WallRate wait_rate(100);
      const auto waiting_start = std::chrono::steady_clock::now();
      while (true) {
        const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - waiting_start).count();
        const auto state = SegmentCoordinator::costmapWaitStatus(
          rclcpp::ok(), action_server_ && action_server_->is_server_active(),
          action_server_ && action_server_->is_cancel_requested(), costmap_ros_->isCurrent(),
          elapsed, costmap_update_timeout_.seconds());
        if (state == SegmentCoordinator::CostmapWaitStatus::Ready) {break;}
        if (state == SegmentCoordinator::CostmapWaitStatus::Stopped ||
          state == SegmentCoordinator::CostmapWaitStatus::Cancelled)
        {
          publishZeroVelocity();
          if (action_server_) {action_server_->terminate_all();}
          return;  // ScopeExit clears phase/queue before deactivate's future completes.
        }
        if (state == SegmentCoordinator::CostmapWaitStatus::TimedOut) {
          throw nav2_core::ControllerTimedOut("Costmap timed out waiting for update");
        }
        wait_rate.sleep();
      }

      updateGlobalPath();

      // Latest TF may return a cached transform while odometry stays fresh.
      // Check the real chain stamp before either completion or command output.
      validateTransformFreshness();

      if (isGoalReached()) {
        RCLCPP_INFO(get_logger(), "Reached the goal!");
        break;
      }

      computeAndPublishVelocity();

      auto cycle_duration = this->now() - start_time;
      if (!loop_rate.sleep()) {
        RCLCPP_WARN(
          get_logger(),
          "Control loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz.",
          controller_frequency_, 1 / cycle_duration.seconds());
        loop_rate.reset();
      }
    }
  } catch (nav2_core::InvalidController & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::INVALID_CONTROLLER;
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::ControllerTFError & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::TF_ERROR;
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::NoValidControl & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::NO_VALID_CONTROL;
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::FailedToMakeProgress & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::FAILED_TO_MAKE_PROGRESS;
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::PatienceExceeded & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::PATIENCE_EXCEEDED;
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::InvalidPath & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::INVALID_PATH;
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::ControllerTimedOut & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::CONTROLLER_TIMED_OUT;
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::ControllerException & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::UNKNOWN;
    action_server_->terminate_current(result);
    return;
  } catch (std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::UNKNOWN;
    action_server_->terminate_current(result);
    return;
  }

  if (!rclcpp::ok()) {
    publishZeroVelocity();
    action_server_->terminate_current();
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Controller succeeded, setting result");

  publishZeroVelocity();

  // TODO(orduno) #861 Handle a pending preemption and set controller name
  action_server_->succeeded_current();
}

void SegmentControllerServer::clearSegmentState()
{
  resetFinalGoalStop();
  final_stop_progress_priority_since_.reset();
  coordinator_.clear();
  queued_path_.reset();
  segment_braking_ = false;
  original_goal_.reset();
  current_wire_path_=nav_msgs::msg::Path{};
  terminal_braking_ = terminal_rotation_active_ = false;
  terminal_retry_blocked_ = false;
  invalid_odom_since_.reset();
  rotation_anchor_wait_since_.reset();
  rotation_anchor_waiting_ = false;
  incoming_rotation_snapshot_ = false;
}

nav_msgs::msg::Path SegmentControllerServer::executionInput(const nav_msgs::msg::Path & path) const
{
  if (!terminal_goal_binding_) {return path;}
  try {
    return goal_region::splitBinding(
      path,terminal_global_resolution_,terminal_global_extent_).executable;
  } catch (const std::exception & e) {throw nav2_core::InvalidPath(e.what());}
}

void SegmentControllerServer::setPlannerPath(
  const nav_msgs::msg::Path & path, bool require_entry_stop)
{
  resetFinalGoalStop();  // Every newly installed plan starts a new final-stop observation.
  const bool preserve_goal = original_goal_ &&
    SegmentCoordinator::compatibleGoal(current_wire_path_, path);
  try {
    coordinator_.setPath(executionInput(path), require_entry_stop);
  } catch (const std::invalid_argument & e) {
    throw nav2_core::InvalidPath(e.what());
  }
  current_wire_path_=path;
  if (!preserve_goal) {
    original_goal_ = path.poses.back();
    original_goal_->header.frame_id = path.header.frame_id;
    final_stop_progress_priority_since_.reset();
    rotation_anchor_wait_since_.reset();
    rotation_anchor_waiting_ = false;
    terminal_retry_blocked_ = false;
    if (!current_progress_checker_.empty()) {
      progress_checkers_[current_progress_checker_]->reset();
    }
  }
  // Same-goal phase/path installation must not erase its lack-of-progress budget.
  // Clear only the per-cycle hold mode when a fresh plan selects TRACK, not its timer.
  if (!coordinator_.rotating()) {rotation_anchor_waiting_ = false;}
  terminal_braking_ = terminal_rotation_active_ = false;
  incoming_rotation_snapshot_ = coordinator_.rotating();
  queued_path_.reset();
  installActiveSegment();
}

void SegmentControllerServer::installActiveSegment()
{
  resetFinalGoalStop();
  current_path_ = coordinator_.active().path;
  for (auto & pose : current_path_.poses) {
    if (pose.header.frame_id.empty()) {pose.header.frame_id = current_path_.header.frame_id;}
  }
  end_pose_ = current_path_.poses.back();
  end_pose_.header.frame_id = current_path_.header.frame_id;
  controllers_[current_controller_]->reset();
  controllers_[current_controller_]->setPlan(current_path_);
  goal_checkers_[current_goal_checker_]->reset();
  intermediate_goal_checker_->reset();
  segment_braking_ = false;
  RCLCPP_INFO(get_logger(), "Ordered segment %zu/%zu: %s, endpoint (%.3f, %.3f, %.3f)",
    coordinator_.index() + 1, coordinator_.size(), coordinator_.rotating() ? "ROTATE" : "TRACK",
    end_pose_.pose.position.x, end_pose_.pose.position.y, SegmentCoordinator::yaw(end_pose_.pose));
}

geometry_msgs::msg::PoseStamped SegmentControllerServer::actualPathPose()
{
  geometry_msgs::msg::PoseStamped actual;
  if (!getRobotPose(actual)) {
    throw nav2_core::ControllerTFError("Failed to obtain actual pose for ordered progress");
  }
  return toPathPose(actual);
}

geometry_msgs::msg::PoseStamped SegmentControllerServer::toPathPose(
  const geometry_msgs::msg::PoseStamped & actual)
{
  geometry_msgs::msg::PoseStamped transformed;
  auto tolerance = rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance());
  if (!nav_2d_utils::transformPose(costmap_ros_->getTfBuffer(),
    current_path_.header.frame_id, actual, transformed, tolerance))
  {
    throw nav2_core::ControllerTFError("Failed to transform actual pose to ordered path frame");
  }
  return transformed;
}

bool SegmentControllerServer::measuredVelocity(geometry_msgs::msg::Twist & measured,
  std::int64_t * original_stamp_ns)
{
  const auto sample = odom_sub_->getTwistStamped();
  const rclcpp::Time stamp(sample.header.stamp, get_clock()->get_clock_type());
  const auto current = now();
  const auto velocity = nav_2d_utils::twist2Dto3D(sample.velocity);
  if (stamp.nanoseconds() <= 0 || stamp > current ||
    (current - stamp).seconds() > coordinator_.config().odom_timeout ||
    !coordinator_.validOdometry(stamp.seconds(), current.seconds(), velocity))
  {
    const bool finite = std::isfinite(velocity.linear.x) && std::isfinite(velocity.linear.y) &&
      std::isfinite(velocity.linear.z) && std::isfinite(velocity.angular.x) &&
      std::isfinite(velocity.angular.y) && std::isfinite(velocity.angular.z);
    const std::string reason = stamp.nanoseconds() <= 0 ? "missing" : stamp > current ?
      "future" : !finite ? "nonfinite" : "stale";
    std::ostringstream diagnostic;
    diagnostic << std::setprecision(17) << "Invalid measured odometry: " << reason <<
      " raw_stamp_ns=" << stamp.nanoseconds() << " now_ns=" << current.nanoseconds() <<
      " age_s=" << (current - stamp).seconds() <<
      " timeout_s=" << coordinator_.config().odom_timeout << " finite=" << finite <<
      " vx=" << velocity.linear.x << " vy=" << velocity.linear.y << " wz=" << velocity.angular.z;
    const auto steady = std::chrono::steady_clock::now();
    if (!invalid_odom_since_) {
      invalid_odom_since_ = steady;
      RCLCPP_WARN(get_logger(), "%s; zero-hold awaiting valid measurement (steady limit .5s)",
        diagnostic.str().c_str());
    }
    const double elapsed = std::chrono::duration<double>(steady - *invalid_odom_since_).count();
    if (SegmentCoordinator::inputWaitStatus(false, elapsed, .5) ==
      SegmentCoordinator::InputWaitStatus::TimedOut)
    {
      throw nav2_core::NoValidControl(diagnostic.str() + "; steady .5s measurement wait expired");
    }
    return false;  // Caller publishes only zero; invalid/default data NEVER counts as stopped.
  }
  invalid_odom_since_.reset();
  measured = velocity;
  if (original_stamp_ns) {*original_stamp_ns=stamp.nanoseconds();}
  return true;
}

void SegmentControllerServer::resetFinalGoalStop()
{
  final_goal_stop_dwell_.reset();final_stop_zero_this_cycle_=false;
}

std::int64_t SegmentControllerServer::publishFinalGoalStop(
  const geometry_msgs::msg::PoseStamped & actual)
{
  auto * map=costmap_ros_->getCostmap();
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    sweep::ScopedObstacleIndex index(*map);
    const double horizon=get_parameter("segments.command_sweep_horizon").as_double();
    if (horizon<.5 || !sweep::commandSweepFree(*map,costmap_ros_->getRobotFootprint(),
      actual.pose.position.x,actual.pose.position.y,SegmentCoordinator::yaw(actual.pose),0.,0.,horizon))
    {throw nav2_core::NoValidControl("Filled original-goal final zero sweep blocked");}
  }
  if (!rclcpp::ok() || !vel_publisher_ || !vel_publisher_->is_activated() ||
    vel_publisher_->get_subscription_count()==0)
  {throw nav2_core::NoValidControl("Original-goal final zero publisher unavailable");}
  geometry_msgs::msg::TwistStamped zero;
  zero.header.frame_id=costmap_ros_->getBaseFrameID();zero.header.stamp=now();
  publishVelocity(zero);last_valid_cmd_time_=now();
  final_stop_zero_this_cycle_=true;
  return rclcpp::Time(zero.header.stamp,get_clock()->get_clock_type()).nanoseconds();
}

void SegmentControllerServer::publishSegmentStop()
{
  geometry_msgs::msg::TwistStamped zero;
  zero.header.frame_id = costmap_ros_->getBaseFrameID();
  zero.header.stamp = now();
  publishVelocity(zero);  // Same owner/publisher; do not reset RPP at every braking cycle.
}

void SegmentControllerServer::validateTransformFreshness()
{
  const auto timeout = coordinator_.config().odom_timeout;
  for (const auto & frame : {costmap_ros_->getGlobalFrameID(), current_path_.header.frame_id}) {
    try {
      const auto transform = costmap_ros_->getTfBuffer()->lookupTransform(
        frame, costmap_ros_->getBaseFrameID(), tf2::TimePointZero);
      const rclcpp::Time stamp(transform.header.stamp, get_clock()->get_clock_type());
      const double age = (now() - stamp).seconds();
      auto & history = transform_history_[frame];
      const auto steady = std::chrono::steady_clock::now();
      if (stamp.nanoseconds() <= 0 || age < -0.05 || age > timeout ||
        stamp.nanoseconds() < history.stamp)
      {
        throw nav2_core::ControllerTFError("Stale, future or regressed actual TF chain: " + frame);
      }
      if (stamp.nanoseconds() > history.stamp) {
        history.stamp = stamp.nanoseconds(); history.advanced = steady;
      }
      if (std::chrono::duration<double>(steady - history.advanced).count() > timeout) {
        throw nav2_core::ControllerTFError("Actual TF stamp stopped advancing: " + frame);
      }
    } catch (const tf2::TransformException & e) {
      throw nav2_core::ControllerTFError(e.what());
    }
  }
}

bool SegmentControllerServer::rotationSweepFree(
  const geometry_msgs::msg::PoseStamped & actual_pose,
  const geometry_msgs::msg::PoseStamped & requested_target)
{
  // Transform only a derived endpoint copy at the actual pose time. The source path stamp
  // and frame remain intact. Sweep is about the ACTUAL body origin, not a snapped grid center.
  auto target = requested_target;
  target.header.stamp = actual_pose.header.stamp;
  geometry_msgs::msg::PoseStamped transformed;
  auto tolerance = rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance());
  if (!nav_2d_utils::transformPose(costmap_ros_->getTfBuffer(),
    costmap_ros_->getGlobalFrameID(), target, transformed, tolerance))
  {
    throw nav2_core::ControllerTFError("Failed to transform rotation target");
  }
  auto * map = costmap_ros_->getCostmap();
  const auto footprint = costmap_ros_->getRobotFootprint();
  const double x = actual_pose.pose.position.x, y = actual_pose.pose.position.y;
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
  sweep::ScopedObstacleIndex index(*map);
  return sweep::sweepFree(*map, footprint, x, y, SegmentCoordinator::yaw(actual_pose.pose),
    x, y, SegmentCoordinator::yaw(transformed.pose));
}

void SegmentControllerServer::enforceRotationSweep(
  const geometry_msgs::msg::PoseStamped & actual_pose)
{
  if (!rotationSweepFree(actual_pose, terminal_rotation_active_ ? *original_goal_ : end_pose_)) {
    throw nav2_core::NoValidControl("Full actual-pose rotation footprint sweep is blocked");
  }
}

void SegmentControllerServer::activateTerminalRotation(
  const geometry_msgs::msg::PoseStamped & path_pose)
{
  resetFinalGoalStop();
  // This ephemeral RPP path is never installed in the ordered coordinator. Its original
  // full path/goal remain the authority for compatibility, pending progress and success.
  nav_msgs::msg::Path path;
  path.header = path_pose.header;
  path.poses.push_back(path_pose);
  terminal_target_ = path_pose;
  terminal_target_.pose.orientation = original_goal_->pose.orientation;
  path.poses.push_back(terminal_target_);
  current_path_ = path;
  controllers_[current_controller_]->reset();
  controllers_[current_controller_]->setPlan(path);
  goal_checkers_[current_goal_checker_]->reset();
  terminal_rotation_active_ = true;
  terminal_braking_ = segment_braking_ = false;
  // The original unstarted-anchor wait survives this ephemeral shortcut.
  rotation_anchor_waiting_ = false;
  RCLCPP_INFO(get_logger(), "Goal-tolerance FINAL normal rotation at actual body origin; "
    "original goal (%.4f, %.4f, %.4f) retained for final checking",
    original_goal_->pose.position.x, original_goal_->pose.position.y,
    SegmentCoordinator::yaw(original_goal_->pose));
}

void SegmentControllerServer::restoreOrderedSegment()
{
  // Must precede any queued-path install; that older snapshot cannot rearm FINAL.
  terminal_retry_blocked_ = true;
  resetFinalGoalStop();
  terminal_rotation_active_ = terminal_braking_ = false;
  segment_braking_ = false;
  if (queued_path_) {
    const auto latest = *queued_path_;
    setPlannerPath(latest);
  } else {
    coordinator_.requireEntryStop();
    installActiveSegment();
  }
  segment_braking_ = true;  // Restore-cycle zero until next tick revalidates the original anchor.
  publishSegmentStop();
}

void SegmentControllerServer::computeAndPublishVelocity()
{
  if (final_stop_zero_this_cycle_) {return;}  // The checked final zero was already published.
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    throw nav2_core::ControllerTFError("Failed to obtain robot pose");
  }

  geometry_msgs::msg::Twist measured;
  if (!measuredVelocity(measured) || segment_braking_ || rotation_anchor_waiting_ ||
    !coordinator_.executionReady(measured))
  {
    publishSegmentStop();
    last_valid_cmd_time_ = now();
    return;
  }

  if (coordinator_.rotating() || terminal_rotation_active_) {
    enforceRotationSweep(pose);  // At entry and every command cycle, full remaining turn.
    // RPP must rotate to the ordered yaw, never toward a displaced old XY carrot.
    // This derived two-pose control path does not replace current_path_, end_pose_,
    // the coordinator's original anchor/progress, or the external original goal.
    controllers_[current_controller_]->setPlan(SegmentCoordinator::actualOriginRotationPath(
      toPathPose(pose),terminal_rotation_active_ ? *original_goal_ : end_pose_));
  } else if (!coordinator_.finalSegment()) {
    // Use the SAME actual pose passed to RPP. A fresh TF update between the goal/phase
    // check and this call must not let RPP start an implicit end-of-TRACK yaw correction.
    const auto actual = toPathPose(pose);
    coordinator_.observe(actual.pose);
    if (coordinator_.geometryReached(actual.pose, originalGoalTolerances().first)) {
      segment_braking_ = true;
      publishSegmentStop();
      return;
    }
  }

  geometry_msgs::msg::TwistStamped cmd_vel_2d;

  try {
    cmd_vel_2d =
      controllers_[current_controller_]->computeVelocityCommands(
      pose,
      measured,
      (terminal_rotation_active_ || coordinator_.finalSegment()) ?
      goal_checkers_[current_goal_checker_].get() :
      intermediate_goal_checker_.get());
    // Limit BEFORE both the filled command sweep and publication. Validate raw
    // finite/forward/axis invariants first; never convert invalid output to motion.
    if (!boundPlanarAngularVelocity(cmd_vel_2d.twist,
      get_parameter("segments.max_angular_velocity").as_double()))
    {
      throw nav2_core::NoValidControl("RPP produced an invalid forward planar command or angular limit");
    }
    if (coordinator_.rotating() || terminal_rotation_active_) {
      if (std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y) > 1e-6) {
        throw nav2_core::NoValidControl("RPP attempted translation inside an explicit rotation");
      }
      const double remaining = SegmentCoordinator::angle(
        SegmentCoordinator::yaw(terminal_rotation_active_ ? original_goal_->pose : end_pose_.pose) -
        SegmentCoordinator::yaw(toPathPose(pose).pose));
      const double settling_yaw_tolerance = terminal_rotation_active_ ?
        originalGoalTolerances().second : coordinator_.config().yaw_tolerance;
      const auto decision = SegmentCoordinator::rotationCommandDecision(
        remaining, cmd_vel_2d.twist.angular.z, measured.angular.z, settling_yaw_tolerance);
      if (decision != SegmentCoordinator::RotationCommandDecision::Allow) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "RPP rotation command %s: remaining=%.9f cmd_wz=%.9f measured_wz=%.9f",
          decision == SegmentCoordinator::RotationCommandDecision::HoldZero ?
          "zero-held for endpoint settling or measured deceleration" : "rejected",
          remaining, cmd_vel_2d.twist.angular.z, measured.angular.z);
        if (decision == SegmentCoordinator::RotationCommandDecision::HoldZero) {
          publishSegmentStop();
          last_valid_cmd_time_ = now();
          return;
        }
        throw nav2_core::NoValidControl("RPP rotation direction disagrees with the active segment");
      }
    } else if (std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y) < 1e-6 &&
      std::abs(cmd_vel_2d.twist.angular.z) > 1e-6)
    {
      throw nav2_core::NoValidControl("TRACK cannot introduce an unplanned in-place rotation");
    }
    {
      auto * map = costmap_ros_->getCostmap();
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
      sweep::ScopedObstacleIndex index(*map);
      const auto & v = cmd_vel_2d.twist;
      const double horizon = get_parameter("segments.command_sweep_horizon").as_double();
      if (horizon < 0.5 || v.linear.y != 0.0 || v.linear.z != 0.0 ||
        v.angular.x != 0.0 || v.angular.y != 0.0 ||
        !sweep::commandSweepFree(*map, costmap_ros_->getRobotFootprint(),
        pose.pose.position.x, pose.pose.position.y, SegmentCoordinator::yaw(pose.pose),
        v.linear.x, v.angular.z, horizon))
      {
        throw nav2_core::NoValidControl("Filled actual command sweep blocked or invalid");
      }
    }
    last_valid_cmd_time_ = now();
    cmd_vel_2d.header.frame_id = costmap_ros_->getBaseFrameID();
    cmd_vel_2d.header.stamp = last_valid_cmd_time_;
    // Only no valid control exception types are valid to attempt to have control patience, as
    // other types will not be resolved with more attempts
  } catch (nav2_core::NoValidControl & e) {
    if (failure_tolerance_ > 0 || failure_tolerance_ == -1.0) {
      RCLCPP_WARN(this->get_logger(), "%s", e.what());
      cmd_vel_2d.twist.angular.x = 0;
      cmd_vel_2d.twist.angular.y = 0;
      cmd_vel_2d.twist.angular.z = 0;
      cmd_vel_2d.twist.linear.x = 0;
      cmd_vel_2d.twist.linear.y = 0;
      cmd_vel_2d.twist.linear.z = 0;
      cmd_vel_2d.header.frame_id = costmap_ros_->getBaseFrameID();
      cmd_vel_2d.header.stamp = now();
      if ((now() - last_valid_cmd_time_).seconds() > failure_tolerance_ &&
        failure_tolerance_ != -1.0)
      {
        throw nav2_core::PatienceExceeded("Controller patience exceeded");
      }
    } else {
      throw nav2_core::NoValidControl(e.what());
    }
  }

  std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
  feedback->speed = std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y);

  if (terminal_rotation_active_) {
    const auto actual = actualPathPose();
    feedback->distance_to_goal = std::hypot(actual.pose.position.x - original_goal_->pose.position.x,
      actual.pose.position.y - original_goal_->pose.position.y);
  } else {
    feedback->distance_to_goal = coordinator_.remainingDistance();
  }
  action_server_->publish_feedback(feedback);

  RCLCPP_DEBUG(get_logger(), "Publishing velocity at time %.2f", now().seconds());
  publishVelocity(cmd_vel_2d);
}

void SegmentControllerServer::updateGlobalPath()
{
  if (!action_server_->is_preempt_requested()) {return;}
  auto goal = action_server_->accept_pending_goal();
  if (!goal) {throw nav2_core::InvalidPath("Missing pending FollowPath goal");}
  installPendingPlannerGoal(*goal);
}

void SegmentControllerServer::installPendingPlannerGoal(const Action::Goal & goal)
{
  resetFinalGoalStop();  // Includes compatible replans queued during a turn.
  std::string controller, goal_checker, progress_checker;
  if (!findControllerId(goal.controller_id, controller)) {
    throw nav2_core::InvalidController("Invalid controller in preempting path");
  }
  if (!findGoalCheckerId(goal.goal_checker_id, goal_checker) ||
    !findProgressCheckerId(goal.progress_checker_id, progress_checker))
  {
    throw nav2_core::ControllerException("Invalid checker in preempting path");
  }
  // FollowPath does not carry the parent navigation task UUID. Compatibility means only
  // same final pose/frame and same plugins within this worker; do not claim task identity.
  const bool compatible = controller == current_controller_ &&
    goal_checker == current_goal_checker_ && progress_checker == current_progress_checker_ &&
    SegmentCoordinator::compatibleGoal(current_wire_path_, goal.path);
  SegmentCoordinator validated(coordinator_.config());
  try {validated.setPath(executionInput(goal.path));} catch (const std::invalid_argument & e) {
    throw nav2_core::InvalidPath(e.what());
  }
  if (compatible && ((coordinator_.rotating() && !coordinator_.entryStopPending()) ||
    terminal_rotation_active_ || terminal_braking_))
  {
    queued_path_ = goal.path;  // Preserve the unfinished angular phase and its measured progress.
    RCLCPP_DEBUG(get_logger(), "Queued compatible replan until the current rotation stops");
    return;
  }
  // A new compatible path is not proof of repair or ordered progress.
  const bool changed_progress_checker = progress_checker != current_progress_checker_;
  const bool require_stop = !compatible || coordinator_.entryStopPending() || validated.rotating();
  if (require_stop) {publishSegmentStop();}
  current_controller_ = controller;
  current_goal_checker_ = goal_checker;
  current_progress_checker_ = progress_checker;
  setPlannerPath(goal.path, require_stop);
  if (changed_progress_checker && !current_progress_checker_.empty()) {
    progress_checkers_[current_progress_checker_]->reset();
  }
}

void SegmentControllerServer::publishVelocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>(velocity);
  if (rclcpp::ok() && vel_publisher_ && vel_publisher_->is_activated() &&
    vel_publisher_->get_subscription_count() > 0)
  {
    vel_publisher_->publish(std::move(cmd_vel));
  }
}

void SegmentControllerServer::publishZeroVelocity()
{
  if (get_parameter("publish_zero_velocity").as_bool()) {
    geometry_msgs::msg::TwistStamped velocity;
    velocity.twist.angular.x = 0;
    velocity.twist.angular.y = 0;
    velocity.twist.angular.z = 0;
    velocity.twist.linear.x = 0;
    velocity.twist.linear.y = 0;
    velocity.twist.linear.z = 0;
    velocity.header.frame_id = costmap_ros_->getBaseFrameID();
    velocity.header.stamp = now();
    publishVelocity(velocity);
  }

  // Reset the state of the controllers after the task has ended
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->reset();
  }
}

std::pair<double, double> SegmentControllerServer::originalGoalTolerances()
{
  geometry_msgs::msg::Pose tolerance_pose;
  geometry_msgs::msg::Twist tolerance_velocity;
  if (!goal_checkers_[current_goal_checker_]->getTolerances(tolerance_pose, tolerance_velocity)) {
    throw nav2_core::ControllerException("Final checker must expose its tolerances");
  }
  const double xy_tolerance = tolerance_pose.position.x;
  const double yaw_tolerance = std::abs(SegmentCoordinator::yaw(tolerance_pose));
  if (!std::isfinite(xy_tolerance) || xy_tolerance <= 0.0 || !std::isfinite(yaw_tolerance)) {
    throw nav2_core::ControllerException("Invalid original goal checker tolerances");
  }
  return {xy_tolerance, yaw_tolerance};
}

bool SegmentControllerServer::isGoalReached()
{
  final_stop_zero_this_cycle_=false;
  if (coordinator_.empty() || !original_goal_) {resetFinalGoalStop();return false;}
  geometry_msgs::msg::PoseStamped actual;
  if (!getRobotPose(actual)) {
    resetFinalGoalStop();throw nav2_core::ControllerTFError("Failed to obtain final observation pose");
  }
  const auto path_pose=toPathPose(actual);
  coordinator_.observe(path_pose.pose);
  geometry_msgs::msg::Twist measured;std::int64_t original_odom_stamp=0;
  if (!measuredVelocity(measured,&original_odom_stamp)) {
    resetFinalGoalStop();
    segment_braking_ = true;  // Do not bypass this cycle's phase/goal checks if DDS catches up.
    publishSegmentStop();
    return false;
  }
  const auto [xy_tolerance, yaw_tolerance] = originalGoalTolerances();

  // A repeated XY location near the original goal is not route completion. Check
  // remaining ordered progress before consulting even a stateful final checker.
  bool terminal_eligible = coordinator_.terminalWindowEligible(
    path_pose.pose, original_goal_->pose, xy_tolerance);
  if ((terminal_rotation_active_ || terminal_braking_) && queued_path_) {
    SegmentCoordinator pending(coordinator_.config());
    pending.setPath(executionInput(*queued_path_));
    pending.observe(path_pose.pose);
    terminal_eligible = terminal_eligible && pending.terminalWindowEligible(
      path_pose.pose, original_goal_->pose, xy_tolerance);
  }

  // Single NavigateToPose acceptance is the original user goal, not exhaustion of an
  // exact sub-cell adapter. Recheck BOTH current geometry components before the stateful
  // checker so an earlier XY latch cannot accept a robot that subsequently moved away.
  if (terminal_eligible && SegmentCoordinator::originalGoalGeometryReached(
    path_pose.pose, original_goal_->pose, xy_tolerance, yaw_tolerance))
  {
    // Only final original-goal acceptance changes. The coordinator's mandatory
    // prefix/segment stop remains authoritative and unchanged.
    segment_braking_=true;
    const auto zero_command_stamp=publishFinalGoalStop(actual);
    const bool measured_stopped=OriginalGoalStopDwell::measuredStopped(
      measured.linear.x,measured.linear.y,measured.angular.z);
    const bool checker_reached=goal_checkers_[current_goal_checker_]->isGoalReached(
      path_pose.pose,original_goal_->pose,measured);
    const auto steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    const bool reached = final_goal_stop_dwell_.observe(
      steady_ns,original_odom_stamp,zero_command_stamp,true,
      measured_stopped && checker_reached,true);
    // The same original goal gets at most .5s priority from its first valid
    // checked stopped zero, across every compatible replan/dwell reset.
    bool progress_priority = false;
    if (measured_stopped && checker_reached && zero_command_stamp > 0) {
      const auto priority_now = std::chrono::steady_clock::now();
      if (!final_stop_progress_priority_since_) {final_stop_progress_priority_since_ = priority_now;}
      progress_priority = std::chrono::duration<double>(
        priority_now - *final_stop_progress_priority_since_).count() <= .5;
    }
    if (!progress_priority && !current_progress_checker_.empty() &&
      !progress_checkers_[current_progress_checker_]->check(actual))
    {throw nav2_core::FailedToMakeProgress("Failed to make progress");}
    return reached;  // Success still needs unchanged .3s original/steady dwell.
  }
  // Every other healthy zero/phase branch retains the SAME goal progress budget.
  // Geometry/ordered-eligibility loss ends priority, but NEVER skips this check.
  final_stop_progress_priority_since_.reset();
  if (!current_progress_checker_.empty() &&
    !progress_checkers_[current_progress_checker_]->check(actual))
  {throw nav2_core::FailedToMakeProgress("Failed to make progress");}
  resetFinalGoalStop();  // XY/yaw/mandatory eligibility loss breaks the continuous dwell.
  if (!terminal_braking_) {segment_braking_ = false;}

  bool sweep_free = false;
  if (terminal_eligible) {
    geometry_msgs::msg::PoseStamped actual;
    if (!getRobotPose(actual)) {
      throw nav2_core::ControllerTFError("Failed to obtain actual pose for terminal sweep");
    }
    sweep_free = rotationSweepFree(actual, *original_goal_);
  }
  if (terminal_rotation_active_) {
    if (!terminal_eligible || !sweep_free) {
      RCLCPP_WARN(get_logger(), "FINAL normal phase no longer feasible; retaining ordered path");
      restoreOrderedSegment();
      return false;
    }
    return false;  // Original current geometry was checked above, before any new command.
  }
  // Test the full shortest sweep BEFORE braking. A blocked short turn must not repeatedly
  // interrupt an already valid longer ordered route. After measured stop, test it again.
  if (terminal_eligible && sweep_free && !terminal_retry_blocked_) {
    terminal_braking_ = segment_braking_ = true;
    if (coordinator_.stopped(measured)) {
      coordinator_.executionReady(measured);
      activateTerminalRotation(path_pose);
      return false;
    }
    return false;
  }
  if (terminal_braking_) {
    terminal_retry_blocked_ = true;
    terminal_braking_ = segment_braking_ = false;
  }
  if (coordinator_.rotating() && coordinator_.entryStopPending()) {
    const bool ready = coordinator_.stopped(measured) &&
      coordinator_.rotationAnchorReached(path_pose.pose) &&
      (!incoming_rotation_snapshot_ || coordinator_.incomingRotationStartReached(path_pose.pose));
    if (!ready) {
      const auto steady = std::chrono::steady_clock::now();
      if (!rotation_anchor_wait_since_) {
        rotation_anchor_wait_since_ = steady;
        const auto & anchor = coordinator_.active().path.poses.front().pose;
        RCLCPP_WARN(get_logger(), "Zero-hold before unstarted ROTATE: actual=(%.4f, %.4f, %.4f), "
          "planned anchor=(%.4f, %.4f, %.4f), incoming_snapshot=%d, speed=(%.4f, %.4f, %.4f); "
          "await measured stop and a fresh compatible plan within %.2fs steady",
          path_pose.pose.position.x, path_pose.pose.position.y, SegmentCoordinator::yaw(path_pose.pose),
          anchor.position.x, anchor.position.y, SegmentCoordinator::yaw(anchor),
          static_cast<int>(incoming_rotation_snapshot_),
          measured.linear.x, measured.linear.y, measured.angular.z,
          coordinator_.config().rotation_anchor_wait_timeout);
      }
      rotation_anchor_waiting_ = segment_braking_ = true;
      const double elapsed = std::chrono::duration<double>(
        steady - *rotation_anchor_wait_since_).count();
      if (SegmentCoordinator::inputWaitStatus(false, elapsed,
        coordinator_.config().rotation_anchor_wait_timeout) ==
        SegmentCoordinator::InputWaitStatus::TimedOut)
      {
        throw nav2_core::NoValidControl("Unstarted ROTATE measured-stop/fresh-anchor wait expired");
      }
      return false;  // Preserve entry-pending; same-goal replans may replace this unstarted phase.
    }
    rotation_anchor_wait_since_.reset();
    rotation_anchor_waiting_ = segment_braking_ = false;
    incoming_rotation_snapshot_ = false;
  }
  if (!coordinator_.executionReady(measured)) {return false;}
  if (coordinator_.stoppedRotationNeedsReplan(path_pose.pose,measured)) {
    // The angular checkpoint is observed, but its original XY anchor is not.
    // Preserve that failure: no advance/success. The bounded BT recovery may
    // compute a new full path to the SAME external goal from this stopped body.
    publishSegmentStop();
    throw nav2_core::NoValidControl("Ordered rotation stopped outside its original XY anchor; fresh original-goal replan required");
  }
  if (!coordinator_.finalSegment()) {
    segment_braking_ = coordinator_.geometryReached(path_pose.pose, xy_tolerance);
    if (segment_braking_ && coordinator_.stopped(measured)) {
      if (queued_path_) {
        // This snapshot was generated DURING the completed turn. Its original start
        // yaw is stale now; replaying it can alternate between adjacent heading bins.
        // Keep the checked remaining route, accept the next fresh compatible replan.
        queued_path_.reset();
        RCLCPP_INFO(get_logger(), "Discarded compatible replan captured during completed ROTATE");
      }
      coordinator_.advance();
      terminal_retry_blocked_ = false;  // Certified ordered progress can permit one new shortcut.
      // This is certified ordered geometry + measured stop, not a phase reinstall.
      rotation_anchor_wait_since_.reset();
      rotation_anchor_waiting_ = false;
      incoming_rotation_snapshot_ = false;  // An existing route's next turn is not a new snapshot.
      installActiveSegment();
    }
    return false;
  }
  return false;  // Reaching the ordered tail alone never replaces original goal geometry.
}

bool SegmentControllerServer::getRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!costmap_ros_->getRobotPose(current_pose)) {
    return false;
  }
  pose = current_pose;
  return true;
}

void SegmentControllerServer::speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg)
{
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->setSpeedLimit(msg->speed_limit, msg->percentage);
  }
}

rcl_interfaces::msg::SetParametersResult
SegmentControllerServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    // If we are trying to change the parameter of a plugin we can just skip it at this point
    // as they handle parameter changes themselves and don't need to lock the mutex
    if (name.find('.') != std::string::npos) {
      continue;
    }

    if (!dynamic_params_lock_.try_lock()) {
      RCLCPP_WARN(
        get_logger(),
        "Unable to dynamically change Parameters while the controller is currently running");
      result.successful = false;
      result.reason =
        "Unable to dynamically change Parameters while the controller is currently running";
      return result;
    }

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "controller_frequency") {
        controller_frequency_ = parameter.as_double();
      } else if (name == "min_x_velocity_threshold") {
        min_x_velocity_threshold_ = parameter.as_double();
      } else if (name == "min_y_velocity_threshold") {
        min_y_velocity_threshold_ = parameter.as_double();
      } else if (name == "min_theta_velocity_threshold") {
        min_theta_velocity_threshold_ = parameter.as_double();
      } else if (name == "failure_tolerance") {
        failure_tolerance_ = parameter.as_double();
      }
    }

    dynamic_params_lock_.unlock();
  }

  result.successful = true;
  return result;
}

}  // namespace robot_feasible_navigation

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robot_feasible_navigation::SegmentControllerServer)

