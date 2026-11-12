// Copyright (c) 2026 The project contributors
// SPDX-License-Identifier: Apache-2.0
#include "nav2_lifecycle_manager/bounded_transition.hpp"

#include <algorithm>
#include <future>
#include <memory>
#include <optional>

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace nav2_lifecycle_manager
{
using namespace std::chrono_literals;
using Change = lifecycle_msgs::srv::ChangeState;
using Get = lifecycle_msgs::srv::GetState;

namespace
{
// Remove unanswered requests on every return/exception, including reconciliation.
struct PendingRequests
{
  rclcpp::Client<Change>::SharedPtr change;
  rclcpp::Client<Get>::SharedPtr get;
  std::optional<rclcpp::Client<Change>::FutureAndRequestId> change_future;
  std::optional<rclcpp::Client<Get>::FutureAndRequestId> get_future;
  ~PendingRequests()
  {
    if (change_future) {change->remove_pending_request(change_future->request_id);}
    if (get_future) {get->remove_pending_request(get_future->request_id);}
  }
};
}  // namespace

TransitionResult request_transition(
  rclcpp::Node & manager, const std::string & target, std::uint8_t transition,
  std::uint8_t expected, std::chrono::milliseconds budget,
  const std::atomic_bool & stopping)
{
  const auto started = std::chrono::steady_clock::now();
  const auto deadline = started + budget;
  const auto context = manager.get_node_base_interface()->get_context();
  std::uint8_t actual = 0;
  const auto result = [&](TransitionOutcome outcome, const std::string & reason) {
      return TransitionResult{outcome, reason, actual,
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count()};
    };
  const auto running = [&]() {
      return !stopping.load(std::memory_order_relaxed) && rclcpp::ok(context);
    };
  if (!running()) {return result(TransitionOutcome::Failed, "manager stopping");}
  bool sent = false;

  try {
    // Never attach the manager Node to a second executor. Its private client
    // group is excluded from the container executor, just like upstream clients.
    auto group = manager.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::ExecutorOptions options;
    options.context = context;
    rclcpp::executors::SingleThreadedExecutor executor(options);
    executor.add_callback_group(group, manager.get_node_base_interface());
    PendingRequests pending;
    pending.change = manager.create_client<Change>(target + "/change_state", rclcpp::ServicesQoS(),
        group);
    pending.get = manager.create_client<Get>(target + "/get_state", rclcpp::ServicesQoS(), group);
    const bool may_reconcile =
      transition == lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE ||
      transition == lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    bool acknowledged = false;
    auto state_sent = started;
    auto next_state_request = started;
    auto state_received = started - 1s;
    auto transition_sent = started;

    const auto take_change_reply = [&]() {
        if (pending.change_future &&
          pending.change_future->wait_for(0s) == std::future_status::ready)
        {
          const auto response = pending.change_future->get();
          pending.change_future.reset();
          acknowledged = response->success;
          return response->success ? 1 : -1;
        }
        return 0;
      };

    while (running() && std::chrono::steady_clock::now() < deadline) {
      const auto now = std::chrono::steady_clock::now();
      if (!sent && pending.change->service_is_ready()) {
        auto request = std::make_shared<Change::Request>();
        request->transition.id = transition;
        pending.change_future.emplace(pending.change->async_send_request(request));
        transition_sent = now;
        sent = true;  // Never retry a state-changing request.
      }
      if (take_change_reply() < 0) {
        return result(TransitionOutcome::Failed, "transition explicitly rejected");
      }
      if (pending.get_future &&
        pending.get_future->wait_for(0s) == std::future_status::ready)
      {
        actual = pending.get_future->get()->current_state.id;
        pending.get_future.reset();
        state_received = now;
        next_state_request = now + 200ms;
      } else if (pending.get_future && now - state_sent >= 1s) {
        pending.get->remove_pending_request(pending.get_future->request_id);
        pending.get_future.reset();
        state_received = started - 1s;  // An older state cannot authorize continuation.
        next_state_request = now;
      }

      if (sent && actual == expected && now - state_received <= 400ms) {
        // Prefer any explicit rejection over a simultaneously ready state reply.
        if (take_change_reply() < 0) {
          return result(TransitionOutcome::Failed, "transition explicitly rejected");
        }
        if (!running()) {return result(TransitionOutcome::Failed, "manager stopping");}
        if (std::chrono::steady_clock::now() >= deadline) {break;}
        if (acknowledged) {
          return result(TransitionOutcome::Confirmed, "reply and state confirmed");
        }
        if (may_reconcile && now - transition_sent >= 1s) {
          return result(TransitionOutcome::Reconciled,
              "RPC reply unconfirmed; fresh target state verified");
        }
      }
      if (sent && !pending.get_future && now >= next_state_request &&
        pending.get->service_is_ready())
      {
        pending.get_future.emplace(pending.get->async_send_request(
            std::make_shared<Get::Request>()));
        state_sent = now;
      }
      const auto remaining = deadline - std::chrono::steady_clock::now();
      if (remaining > std::chrono::steady_clock::duration::zero()) {
        executor.spin_once(std::min(
            std::chrono::duration_cast<std::chrono::nanoseconds>(remaining),
            std::chrono::duration_cast<std::chrono::nanoseconds>(50ms)));
      }
    }
  } catch (const std::exception & error) {
    return result(TransitionOutcome::Failed,
      running() ? std::string("lifecycle RPC error: ") + error.what() : "manager stopping");
  }
  return result(TransitionOutcome::Failed,
    running() ?
           (sent ? "transition/state confirmation deadline exceeded" :
           "transition service unavailable") :
    "manager stopping");
}
}  // namespace nav2_lifecycle_manager
