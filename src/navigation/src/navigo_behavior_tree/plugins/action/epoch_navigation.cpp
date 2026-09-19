// Copyright 2026 Zsibot navigation contributors
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <map>
#include <optional>
#include <set>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "navigo_behavior_tree/bt_conversions.hpp"
#include "navigo_epoch_msgs/action/follow_path_epoch.hpp"
#include "navigo_epoch_msgs/msg/epoch_path.hpp"
#include "navigo_epoch_msgs/msg/localization_epoch.hpp"
#include "navigo_epoch_msgs/msg/navigation_intent.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

namespace navigo_behavior_tree::epoch
{
using State = navigo_epoch_msgs::msg::LocalizationEpoch;
using Identity = navigo_epoch_msgs::msg::LocalizationIdentity;
using Token = navigo_epoch_msgs::msg::NavigationToken;
using Path = navigo_epoch_msgs::msg::EpochPath;
using Intent = navigo_epoch_msgs::msg::NavigationIntent;
using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;

static std::string readId(const char * filename)
{
  std::ifstream stream(filename);
  std::string result;
  stream >> result;
  if (result.empty()) {throw std::runtime_error(std::string("Missing identity: ") + filename);}
  return result;
}
static uint64_t steadyNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    Clock::now().time_since_epoch()).count();
}
static double age(Time time)
{
  return time == Time{} ? INFINITY : std::chrono::duration<double>(Clock::now() - time).count();
}
static int64_t stampNs(const builtin_interfaces::msg::Time & stamp)
{
  return int64_t(stamp.sec) * 1000000000LL + stamp.nanosec;
}
static bool finitePose(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  return !pose.header.frame_id.empty() && stampNs(pose.header.stamp) > 0 &&
         std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(norm) && std::abs(norm - 1.0) < 1e-3;
}

// Shared only by one BT blackboard. Callbacks are pumped on the BT thread;
// no callback mutates a pending action's captured request or token.
class Context
{
public:
  explicit Context(rclcpp::Node::SharedPtr node) : node_(std::move(node)),
    session_(readId("/proc/sys/kernel/random/uuid")),
    boot_(readId("/proc/sys/kernel/random/boot_id"))
  {
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    state_sub_ = node_->create_subscription<State>("/lightning/localization_epoch", 1,
      [this](State::ConstSharedPtr value) {observe(*value);}, options);
    gate_sub_ = node_->create_subscription<std_msgs::msg::String>("/nav_epoch/gate_session", 1,
      [this](std_msgs::msg::String::ConstSharedPtr value) {
        if (value->data.empty() || retired_gates_.count(value->data)) {return;}
        if (!gate_.empty() && gate_ != value->data) {retired_gates_.insert(gate_);}
        gate_ = value->data;
        gate_received_ = Clock::now();
      }, options);
    intent_pub_ = node_->create_publisher<Intent>("/nav_epoch/intent", 1);
  }
  void pump()
  {
    executor_.spin_some();
    // Child actions are not ticked while Guard waits for localization. Their
    // late ACK/result callbacks still need service to cancel retired requests.
    for (const auto & item : action_pumps_) {item.second();}
  }
  void attach(void * owner, std::function<void()> pump) {action_pumps_[owner] = std::move(pump);}
  void detach(void * owner) {action_pumps_.erase(owner);}
  bool ready() const
  {
    return state_.schema_version == 1 && state_.lifecycle_enabled && state_.health == State::NORMAL &&
           state_.fusion_ready && state_.output_ready && finitePose(state_.output_pose) &&
           state_.identity.epoch > 0 && state_.identity.commits > 0 && !state_.identity.process_session_id.empty() &&
           !state_.identity.map_loaded_instance.empty() && !source_reversed_ &&
           age(state_received_) < 0.4 && age(output_advanced_) < 0.4 &&
           !gate_.empty() && age(gate_received_) < 0.4 &&
           (map_binding_.empty() || map_binding_ == state_.identity.map_loaded_instance);
  }
  void beginTask()
  {
    revoke();
    ++task_;
    task_identity_ = state_.identity;
    task_gate_ = gate_;
    if (map_binding_.empty()) {map_binding_ = state_.identity.map_loaded_instance;}
  }
  bool currentTask() const
  {
    return ready() && task_ > 0 && task_identity_ == state_.identity && task_gate_ == gate_;
  }
  bool current(const Token & token) const
  {
    return currentTask() && token.localization == task_identity_ &&
           token.navigation_session_id == session_ && token.task_sequence == task_ &&
           token.gate_session_id == task_gate_ && token.plan_sequence > 0;
  }
  Path request()
  {
    Path path;
    if (!currentTask()) {return path;}
    path.token.localization = task_identity_;
    path.token.navigation_session_id = session_;
    path.token.task_sequence = task_;
    path.token.plan_sequence = ++plan_;
    path.token.gate_session_id = task_gate_;
    path.planning_start = state_.output_pose;
    path.planning_heartbeat_sequence = state_.heartbeat_sequence;
    return path;
  }
  bool latest(const Token & token) const {return current(token) && token.plan_sequence == plan_;}
  void authorize(const Token & token)
  {
    if (!current(token)) {revoke(); return;}
    intent_.token = token;
    intent_.active = true;
    publishIntent();
  }
  void renew()
  {
    if (intent_.active && !current(intent_.token)) {intent_.active = false;}
    publishIntent();
  }
  void revoke() {intent_.active = false; publishIntent();}
  const State & state() const {return state_;}

private:
  void observe(const State & value)
  {
    if (value.identity.process_session_id.empty() ||
      retired_sessions_.count(value.identity.process_session_id)) {return;}
    const bool new_session = state_.identity.process_session_id != value.identity.process_session_id;
    if (!new_session && value.heartbeat_sequence <= state_.heartbeat_sequence) {return;}
    if (new_session && !state_.identity.process_session_id.empty()) {
      retired_sessions_.insert(state_.identity.process_session_id);
    }
    if (!new_session && (value.identity.epoch < state_.identity.epoch ||
      value.identity.commits < state_.identity.commits)) {
      if (!value.lifecycle_enabled || value.health != State::NORMAL || !value.output_ready) {
        // Reload can explicitly revoke with an empty identity. Consume the
        // newer revocation without lowering the identity/source watermarks.
        state_.heartbeat_sequence = value.heartbeat_sequence;
        state_.health = value.health; state_.output_ready = false; state_.fusion_ready = false;
        state_received_ = Clock::now();
      }
      return;
    }
    const bool new_identity = state_.identity != value.identity;
    const int64_t stamp = stampNs(value.output_pose.header.stamp);
    if (new_identity) {
      source_reversed_ = false; output_advanced_ = Time{}; output_highwater_ = 0;
    }
    // Not-ready snapshots may contain an empty pose. They must neither erase
    // the last positive source watermark nor masquerade as a clock rollback.
    if (stamp > output_highwater_) {
      output_highwater_ = stamp; output_advanced_ = Clock::now();
    } else if (stamp > 0 && stamp < output_highwater_) {source_reversed_ = true;}
    state_ = value;
    state_received_ = Clock::now();
  }
  void publishIntent()
  {
    intent_.heartbeat_sequence = ++intent_sequence_;
    intent_.source_steady_time_ns = steadyNs();
    intent_.boot_id = boot_;
    intent_pub_->publish(intent_);
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<State>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gate_sub_;
  rclcpp::Publisher<Intent>::SharedPtr intent_pub_;
  State state_;
  Intent intent_;
  Identity task_identity_;
  std::string session_, boot_, gate_, task_gate_, map_binding_;
  std::set<std::string> retired_sessions_, retired_gates_;
  std::map<void *, std::function<void()>> action_pumps_;
  uint64_t task_{0}, plan_{0}, intent_sequence_{0};
  int64_t output_highwater_{0};
  Time state_received_{}, gate_received_{}, output_advanced_{};
  bool source_reversed_{false};
};

static std::shared_ptr<Context> context(const BT::NodeConfiguration & config)
{
  std::shared_ptr<Context> value;
  if (!config.blackboard->get("epoch_context", value)) {
    value = std::make_shared<Context>(config.blackboard->get<rclcpp::Node::SharedPtr>("node"));
    config.blackboard->set("epoch_context", value);
  }
  return value;
}

class Guard : public BT::DecoratorNode
{
public:
  Guard(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config), context_(context(config)) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("recovery_wait_sec", 60.0, "Steady duration limit in seconds"),
      BT::InputPort<double>("mission_timeout_sec", 600.0, "Steady duration limit in seconds")};
  }
  BT::NodeStatus tick() override
  {
    context_->pump();
    if (status() == BT::NodeStatus::IDLE) {
      started_ = Clock::now(); waiting_ = Clock::now(); running_ = false;
    }
    setStatus(BT::NodeStatus::RUNNING);
    double wait_limit = 60.0, mission_limit = 600.0;
    getInput("recovery_wait_sec", wait_limit); getInput("mission_timeout_sec", mission_limit);
    if (!(std::isfinite(wait_limit) && wait_limit > 0 && std::isfinite(mission_limit) && mission_limit > 0)) {
      halt(); return BT::NodeStatus::FAILURE;
    }
    uint64_t revision = 0;
    config().blackboard->get("epoch_goal_revision", revision);
    if (revision != revision_) {
      if (running_) {invalidate();}
      revision_ = revision; started_ = Clock::now(); waiting_ = Clock::now();
    }
    if (running_ && !context_->currentTask()) {invalidate();}
    if (age(started_) >= mission_limit || (!running_ && age(waiting_) >= wait_limit)) {
      invalidate(); return BT::NodeStatus::FAILURE;
    }
    if (!running_) {
      context_->revoke();
      if (!context_->ready()) {return BT::NodeStatus::RUNNING;}
      context_->beginTask(); running_ = true; revision_ = revision;
    }
    context_->renew();
    const auto result = child_node_->executeTick();
    if (result != BT::NodeStatus::RUNNING) {context_->revoke(); running_ = false;}
    return result;
  }
  void halt() override {invalidate(); BT::DecoratorNode::halt();}
private:
  void invalidate()
  {
    context_->revoke();
    if (child_node_ && child_node_->status() != BT::NodeStatus::IDLE) {haltChild();}
    config().blackboard->set("epoch_raw_path", Path{});
    config().blackboard->set("epoch_path", Path{});
    config().blackboard->set("path", nav_msgs::msg::Path{});
    running_ = false; waiting_ = Clock::now();
  }
  std::shared_ptr<Context> context_;
  Time started_{}, waiting_{};
  uint64_t revision_{0};
  bool running_{false};
};

// Unlike the legacy adapter, late goal acknowledgements are explicitly
// canceled after halt, callbacks never dereference a cleared goal_handle,
// and all deadlines use steady time. Each callback owns its request record.
template<class Action>
class ActionNode : public BT::ActionNodeBase
{
public:
  using Handle = rclcpp_action::ClientGoalHandle<Action>;
  using Client = rclcpp_action::Client<Action>;
  struct Request
  {
    bool valid{true};
    typename Handle::SharedPtr handle;
    std::optional<typename Handle::WrappedResult> result;
    bool rejected{false};
    Path path;
    Time sent;
  };
  ActionNode(const std::string & name, const BT::NodeConfiguration & config, const std::string & endpoint)
  : BT::ActionNodeBase(name, config), context_(context(config))
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(group_, node_->get_node_base_interface());
    std::string action_name = endpoint;
    getInput("server_name", action_name);
    client_ = rclcpp_action::create_client<Action>(node_, action_name, group_);
    context_->attach(this, [this] {executor_.spin_some();});
  }
  ~ActionNode() override {cancel(); context_->detach(this);}
  static BT::PortsList basicPorts()
  {
    return {BT::InputPort<std::string>("server_name"),
      BT::InputPort<double>("action_timeout_sec", 5.0, "Steady duration limit in seconds")};
  }
  BT::NodeStatus tick() override
  {
    executor_.spin_some();
    if (!context_->currentTask()) {halt(); return BT::NodeStatus::FAILURE;}
    // A newer path uses action preemption. Sending a cancellation immediately
    // before its replacement can cancel the server's pending new goal as well.
    if (request_ && shouldReplace()) {cancel(false);}
    if (!request_) {
      if (!client_->action_server_is_ready()) {
        if (server_wait_ == Time{}) {server_wait_ = Clock::now();}
        setStatus(BT::NodeStatus::RUNNING);
        return age(server_wait_) < 2.0 ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
      }
      server_wait_ = Time{};
      typename Action::Goal goal;
      Path lineage;
      if (!makeGoal(goal, lineage)) {return BT::NodeStatus::FAILURE;}
      send(goal, lineage);
    }
    setStatus(BT::NodeStatus::RUNNING);
    if (!context_->current(request_->path.token) || request_->rejected) {
      halt(); return BT::NodeStatus::FAILURE;
    }
    double timeout = 5.0;
    getInput("action_timeout_sec", timeout);
    if (!std::isfinite(timeout) || timeout <= 0 ||
      (!request_->handle && age(request_->sent) > 2.0) ||
      (boundedDuration() && age(request_->sent) > timeout)) {
      halt(); return BT::NodeStatus::FAILURE;
    }
    whileRunning();
    if (!request_->result || !request_->handle) {return BT::NodeStatus::RUNNING;}
    const auto result = *request_->result;
    if (result.goal_id != request_->handle->get_goal_id()) {halt(); return BT::NodeStatus::FAILURE;}
    BT::NodeStatus outcome = BT::NodeStatus::FAILURE;
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result && result.result->error_code == 0) {
      outcome = acceptResult(*result.result, request_->path);
    }
    request_->valid = false;
    request_.reset();
    finished();
    return outcome;
  }
  void halt() override {cancel(); finished(); server_wait_ = Time{}; setStatus(BT::NodeStatus::IDLE);}
protected:
  virtual bool makeGoal(typename Action::Goal &, Path &) = 0;
  virtual BT::NodeStatus acceptResult(const typename Action::Result &, const Path &) = 0;
  virtual bool boundedDuration() const {return true;}
  virtual bool shouldReplace() {return false;}
  virtual void whileRunning() {}
  virtual void finished() {}
  void cancel(bool cancel_goal = true)
  {
    if (!request_) {return;}
    request_->valid = false;
    if (cancel_goal && request_->handle && !request_->result) {client_->async_cancel_goal(request_->handle);}
    request_.reset();
  }
  std::shared_ptr<Context> context_;
  std::shared_ptr<Request> request_;
private:
  void send(const typename Action::Goal & goal, const Path & lineage)
  {
    auto request = std::make_shared<Request>();
    request->path = lineage; request->sent = Clock::now();
    typename Client::SendGoalOptions options;
    std::weak_ptr<Client> client(client_);
    options.goal_response_callback = [request, client](typename Handle::SharedPtr handle) {
      request->handle = handle; request->rejected = !handle;
      if (!request->valid && handle) {
        if (auto owner = client.lock()) {owner->async_cancel_goal(handle);}
      }
    };
    options.result_callback = [request](const typename Handle::WrappedResult & result) {
      if (request->valid) {request->result = result;}
    };
    client_->async_send_goal(goal, options);
    request_ = request;
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  typename Client::SharedPtr client_;
  Time server_wait_{};
};

template<class Action, bool Through>
class Compute : public ActionNode<Action>
{
  using Base = ActionNode<Action>;
public:
  Compute(const std::string & name, const BT::NodeConfiguration & config)
  : Base(name, config, Through ? "compute_path_through_poses" : "compute_path_to_pose") {}
  static BT::PortsList providedPorts()
  {
    auto ports = Base::basicPorts();
    ports.insert(BT::OutputPort<Path>("epoch_path"));
    ports.insert(BT::InputPort<std::string>("planner_id", "GridBased", "Mapped plugin identifier"));
    if constexpr (Through) {
      ports.insert(BT::BidirectionalPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"));
      ports.insert(BT::InputPort<double>("passed_goal_radius", 0.2, "Intermediate waypoint XY radius in meters"));
    }
    else {ports.insert(BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"));}
    return ports;
  }
protected:
  bool makeGoal(typename Action::Goal & goal, Path & lineage) override
  {
    lineage = this->context_->request();
    if (!this->context_->current(lineage.token)) {return false;}
    this->getInput("planner_id", goal.planner_id);
    if constexpr (Through) {
      if (!this->getInput("goals", goal.goals) || goal.goals.empty()) {return false;}
      double radius = 0.2;
      this->getInput("passed_goal_radius", radius);
      if (!std::isfinite(radius) || radius <= 0) {return false;}
      // Prune in order using the same authoritative pose captured for planning.
      // Never consume the last goal: FollowPath must complete its yaw/stop checks.
      // Strict multi-pose requests use the localization map frame. Mixing frames
      // here would compare coordinates without a source-bound transformation.
      for (const auto & pose : goal.goals) {
        if (pose.header.frame_id != lineage.planning_start.header.frame_id ||
          !std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y)) {return false;}
      }
      const auto & start = lineage.planning_start.pose.position;
      while (goal.goals.size() > 1 &&
        std::hypot(goal.goals.front().pose.position.x - start.x,
        goal.goals.front().pose.position.y - start.y) <= radius) {
        goal.goals.erase(goal.goals.begin());
      }
      this->setOutput("goals", goal.goals);
    } else if (!this->getInput("goal", goal.goal)) {return false;}
    goal.use_start = true; goal.start = lineage.planning_start;
    this->setOutput("epoch_path", Path{});
    return true;
  }
  BT::NodeStatus acceptResult(const typename Action::Result & result, const Path & lineage) override
  {
    if (!this->context_->latest(lineage.token) || result.path.poses.empty() ||
      result.path.header.frame_id != lineage.planning_start.header.frame_id) {return BT::NodeStatus::FAILURE;}
    auto path = lineage; path.path = result.path;
    this->setOutput("epoch_path", path);
    return BT::NodeStatus::SUCCESS;
  }
};

class Smooth : public ActionNode<nav2_msgs::action::SmoothPath>
{
  using Action = nav2_msgs::action::SmoothPath;
public:
  Smooth(const std::string & name, const BT::NodeConfiguration & config)
  : ActionNode(name, config, "smooth_path") {}
  static BT::PortsList providedPorts()
  {
    auto ports = basicPorts();
    ports.insert(BT::InputPort<Path>("input_path")); ports.insert(BT::OutputPort<Path>("epoch_path"));
    ports.insert(BT::OutputPort<nav_msgs::msg::Path>("path"));
    ports.insert(BT::InputPort<std::string>("smoother_id", "simple_smoother", "Mapped plugin identifier"));
    ports.insert(BT::InputPort<double>("max_smoothing_duration", 0.2, "Steady duration limit in seconds"));
    return ports;
  }
protected:
  bool makeGoal(Action::Goal & goal, Path & lineage) override
  {
    if (!getInput("input_path", lineage) || !context_->latest(lineage.token) || lineage.path.poses.empty()) {return false;}
    goal.path = lineage.path; getInput("smoother_id", goal.smoother_id);
    double seconds = 0.2; getInput("max_smoothing_duration", seconds);
    if (!std::isfinite(seconds) || seconds <= 0) {return false;}
    goal.max_smoothing_duration = rclcpp::Duration::from_seconds(seconds);
    goal.check_for_collisions = true;
    return true;
  }
  BT::NodeStatus acceptResult(const Action::Result & result, const Path & lineage) override
  {
    if (!context_->latest(lineage.token) || result.path.poses.empty() ||
      result.path.header.frame_id != lineage.planning_start.header.frame_id) {return BT::NodeStatus::FAILURE;}
    auto path = lineage; path.path = result.path;
    setOutput("epoch_path", path); setOutput("path", result.path);
    return BT::NodeStatus::SUCCESS;
  }
};

class Follow : public ActionNode<navigo_epoch_msgs::action::FollowPathEpoch>
{
  using Action = navigo_epoch_msgs::action::FollowPathEpoch;
public:
  Follow(const std::string & name, const BT::NodeConfiguration & config)
  : ActionNode(name, config, "follow_path_epoch") {}
  static BT::PortsList providedPorts()
  {
    auto ports = basicPorts(); ports.insert(BT::InputPort<Path>("epoch_path"));
    ports.insert(BT::InputPort<std::string>("controller_id", "FollowPath", "Mapped plugin identifier"));
    ports.insert(BT::InputPort<std::string>("goal_checker_id", "", "Mapped plugin identifier"));
    ports.insert(BT::InputPort<std::string>("progress_checker_id", "", "Mapped plugin identifier"));
    return ports;
  }
protected:
  bool makeGoal(Action::Goal & goal, Path & lineage) override
  {
    if (!getInput("epoch_path", lineage) || !context_->current(lineage.token) || lineage.path.poses.empty()) {return false;}
    goal.epoch_path = lineage;
    getInput("controller_id", goal.controller_id); getInput("goal_checker_id", goal.goal_checker_id);
    getInput("progress_checker_id", goal.progress_checker_id);
    context_->authorize(lineage.token);
    return true;
  }
  bool boundedDuration() const override {return false;}
  bool shouldReplace() override
  {
    Path next;
    return getInput("epoch_path", next) && context_->current(next.token) &&
           next.token.plan_sequence > request_->path.token.plan_sequence;
  }
  void whileRunning() override {context_->authorize(request_->path.token);}
  void finished() override {context_->revoke();}
  BT::NodeStatus acceptResult(const Action::Result &, const Path & lineage) override
  {
    return context_->current(lineage.token) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};
}  // namespace navigo_behavior_tree::epoch

BT_REGISTER_NODES(factory)
{
  using namespace navigo_behavior_tree::epoch;
  factory.registerNodeType<Guard>("EpochGuard");
  factory.registerNodeType<Compute<nav2_msgs::action::ComputePathToPose, false>>("EpochComputePathToPose");
  factory.registerNodeType<Compute<nav2_msgs::action::ComputePathThroughPoses, true>>("EpochComputePathThroughPoses");
  factory.registerNodeType<Smooth>("EpochSmoothPath");
  factory.registerNodeType<Follow>("EpochFollowPath");
}
