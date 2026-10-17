#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_debug_msgs/msg/build_info.hpp>
#include <robot_debug_msgs/msg/checker_sample.hpp>
#include <robot_debug_msgs/msg/status_observation.hpp>

namespace robot_debug
{
// Per-producer state and code: no cross-DSO registry or weak worker binding.
#if defined(__GNUC__) || defined(__clang__)
class __attribute__((visibility("hidden"))) Recorder
#else
class Recorder
#endif
{
  using Observation = robot_debug_msgs::msg::StatusObservation;
  struct State
  {
    std::mutex mutex;
    std::deque<Observation> pending;
    std::atomic<uint64_t> dropped{0};
    rclcpp::Clock::SharedPtr clock;
    rclcpp::Publisher<Observation>::SharedPtr output;
    rclcpp::Publisher<robot_debug_msgs::msg::BuildInfo>::SharedPtr metadata;
    robot_debug_msgs::msg::BuildInfo info;
    bool enabled{true};
    bool metadata_only{false};
    std::atomic<bool> stopping{false};
    std::mutex worker_mutex;
    std::condition_variable worker_condition;
    uint64_t ticks{0};
  };

public:
  template<class NodeT>
  Recorder(NodeT & node, const std::string & component,
    const robot_debug_msgs::msg::BuildStamp & build,
    const std::vector<std::string> & endpoints, bool metadata_only = false)
  : state_(std::make_shared<State>())
  {
    state_->clock = node.get_clock();
    state_->metadata_only = metadata_only;
    if (!node.has_parameter("enable_debug_observations")) {
      node.declare_parameter("enable_debug_observations", true);
    }
    state_->enabled = node.get_parameter("enable_debug_observations").as_bool();
    if (!node.has_parameter("enable_debug_build_info")) {
      node.declare_parameter("enable_debug_build_info", true);
    }
    const bool metadata_enabled = node.get_parameter("enable_debug_build_info").as_bool();
    if (!metadata_enabled && (metadata_only || !state_->enabled)) {
      // A diagnostic-only OFF control: no publisher, worker or context callback.
      state_->enabled = false;
      return;
    }
    if (!metadata_only && state_->enabled) {
      state_->output = rclcpp::create_publisher<Observation>(
        node, "/debug/observations", rclcpp::QoS(64).reliable());
    }
    if (metadata_enabled) {
      state_->metadata = rclcpp::create_publisher<robot_debug_msgs::msg::BuildInfo>(
        node, "/debug/build_info", rclcpp::QoS(16).reliable().transient_local());
    }
    state_->info.build = build;
    state_->info.component = component;
    state_->info.node_fqn = node.get_fully_qualified_name();
    state_->info.endpoints = endpoints;
    std::random_device random;
    std::ostringstream id;
    id << std::hex << random() << random() << random() << random();
    state_->info.source_instance = id.str();
    auto state = state_;
    worker_ = std::thread([state]() {
      while (!state->stopping.load(std::memory_order_relaxed)) {
        flush(state);
        std::unique_lock<std::mutex> lock(state->worker_mutex);
        state->worker_condition.wait_for(lock,
          std::chrono::milliseconds(state->metadata_only ? 2000 : 100),
          [state]() {return state->stopping.load(std::memory_order_relaxed);});
      }
    });
  }

  ~Recorder() noexcept
  {
    state_->stopping.store(true, std::memory_order_relaxed);
    state_->worker_condition.notify_all();
    if (worker_.joinable()) {
      // A lifecycle component may unload its DSO after this destructor. Never
      // detach code that could still execute from that DSO after unload.
      worker_.join();
    }
  }

  bool enabled() const noexcept {return state_->enabled && !state_->metadata_only;}

  void branch(const std::string & site, const std::string & branch_name,
    const std::string & error = "", bool value_valid = false, bool value = false,
    const std::array<uint8_t, 16> & goal_id = {}, const std::string & endpoint = "") noexcept
  {
    if (!state_->enabled || state_->metadata_only) {return;}
    try {
      auto msg = sample(goal_id);
      if (!endpoint.empty()) {msg.endpoint = endpoint;}
      robot_debug_msgs::msg::BranchSample branch;
      branch.site = site;
      branch.branch = branch_name;
      branch.original_error = error.substr(0, 512);
      branch.value_valid = value_valid;
      branch.value = value;
      msg.branch.push_back(std::move(branch));
      enqueue(std::move(msg));
    } catch (...) {state_->dropped.fetch_add(1, std::memory_order_relaxed);}
  }

  void checker(const robot_debug_msgs::msg::CheckerSample & checker_sample,
    const std::array<uint8_t, 16> & goal_id, uint64_t path_revision) noexcept
  {
    if (!state_->enabled || state_->metadata_only) {return;}
    try {
      auto msg = sample(goal_id);
      msg.path_revision = path_revision;
      msg.checker.push_back(checker_sample);
      enqueue(std::move(msg));
    } catch (...) {state_->dropped.fetch_add(1, std::memory_order_relaxed);}
  }

  void path_installed(const std::array<uint8_t, 16> & goal_id, uint64_t revision) noexcept
  {
    if (!state_->enabled || state_->metadata_only) {return;}
    try {
      auto msg = sample(goal_id);
      msg.path_revision = revision;
      robot_debug_msgs::msg::BranchSample branch;
      branch.site = "path_update";
      branch.branch = "installed";
      msg.branch.push_back(std::move(branch));
      enqueue(std::move(msg));
    } catch (...) {state_->dropped.fetch_add(1, std::memory_order_relaxed);}
  }

private:
  Observation sample(const std::array<uint8_t, 16> & goal_id)
  {
    Observation msg;
    msg.header.stamp = state_->clock->now();
    msg.source_build = state_->info.build;
    msg.reporter_build = state_->info.build;
    msg.source_instance = state_->info.source_instance;
    msg.component = state_->info.component;
    if (!state_->info.endpoints.empty()) {msg.endpoint = state_->info.endpoints.front();}
    msg.goal_id.uuid = goal_id;
    msg.goal_id_valid = std::any_of(goal_id.begin(), goal_id.end(), [](uint8_t v) {return v != 0;});
    return msg;
  }

  void enqueue(Observation && msg) noexcept
  {
    std::unique_lock<std::mutex> lock(state_->mutex, std::try_to_lock);
    if (!lock.owns_lock() || state_->pending.size() >= 128) {
      state_->dropped.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    try {state_->pending.emplace_back(std::move(msg));}
    catch (...) {state_->dropped.fetch_add(1, std::memory_order_relaxed);}
  }

  static void flush(const std::shared_ptr<State> & state) noexcept
  {
    try {
      std::deque<Observation> batch;
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        batch.swap(state->pending);
      }
      for (auto & msg : batch) {
        if (state->stopping.load(std::memory_order_relaxed)) {break;}
        try {
          msg.dropped_records = state->dropped.load(std::memory_order_relaxed);
          state->output->publish(msg);
        } catch (...) {
          state->dropped.fetch_add(1, std::memory_order_relaxed);
        }
      }
      if (state->metadata && (state->metadata_only || (state->ticks++ % 20) == 0)) {
        state->info.header.stamp = state->clock->now();
        state->metadata->publish(state->info);
      }
    } catch (...) {state->dropped.fetch_add(1, std::memory_order_relaxed);}
  }

  std::shared_ptr<State> state_;
  std::thread worker_;
};
}  // namespace robot_debug
