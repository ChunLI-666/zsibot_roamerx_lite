// Copyright (c) 2026 The project contributors
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
// Child-process regression for the real manager worker and an unanswered
// lifecycle service, with no navigation algorithm or actuator process.
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using Manage = nav2_msgs::srv::ManageLifecycleNodes;
using Change = lifecycle_msgs::srv::ChangeState;
using Get = lifecycle_msgs::srv::GetState;

namespace
{
void require(bool value, const char * message)
{
  if (!value) {throw std::runtime_error(message);}
}

struct ExecutorThread
{
  explicit ExecutorThread(rclcpp::Executor & exec)
  : executor(exec), thread([&exec]() {exec.spin();}) {}
  ~ExecutorThread()
  {
    executor.cancel();
    thread.join();
  }
  rclcpp::Executor & executor;
  std::thread thread;
};

struct FixtureBondPreshutdown
{
  rclcpp::Context::SharedPtr context;
  rclcpp::PreShutdownCallbackHandle handle;
  bool registered{false};
  ~FixtureBondPreshutdown()
  {
    if (registered) {context->remove_pre_shutdown_callback(handle);}
  }
};

int exercise(const std::string & mode)
{
  // nav2_util 1.3.13's nested client executors use the global default Context.
  // Use that real Context, not a custom Context which would mask this wait cycle.
  auto context = rclcpp::contexts::get_global_default_context();
  const auto initial_callbacks = context->get_pre_shutdown_callbacks().size();
  auto fixture = std::make_shared<rclcpp::Node>("unanswered_lifecycle_fixture");
  auto requester = std::make_shared<rclcpp::Node>("manager_shutdown_requester");
  std::atomic<bool> request_taken{false};
  std::atomic<uint8_t> target_state{lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED};
  const bool bond_recovery = mode == "bond_recovery_shutdown";
  const bool bond_active = mode == "bond_active" || bond_recovery;
  const bool bond_release = mode == "bond_owner_release";
  std::atomic<bool> reset_request_taken{false};
  std::atomic<bool> recovery_during_pre{false};
  std::shared_ptr<rmw_request_id_t> activation_header;
  auto counterpart = bond_active ?
    std::make_shared<bond::Bond>("bond", "shutdown_target", fixture) : nullptr;
  FixtureBondPreshutdown counterpart_cleanup;
  counterpart_cleanup.context = context;
  if (bond_active) {
    // Match a real LifecycleNode's pre-shutdown bond teardown while Context is
    // valid. A plain rclcpp fixture otherwise introduces its own late-Bond bug.
    counterpart_cleanup.handle = context->add_pre_shutdown_callback(
      [&, counterpart]() {
        counterpart->breakBond();
        if (bond_recovery) {
          // A real lifecycle node breaks its bond during pre-shutdown. Keep
          // Context valid until the manager has begun its normal hard reset,
          // then let Context shutdown interrupt the held DEACTIVATE response.
          const auto deadline = std::chrono::steady_clock::now() + 5s;
          while (!reset_request_taken.load() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(1ms);
          }
          recovery_during_pre.store(reset_request_taken.load());
          std::cout << "bond_recovery_shutdown: hard reset entered during pre=" <<
            recovery_during_pre.load() << std::endl;
        }
      });
    counterpart_cleanup.registered = true;
  }
  auto state_service = fixture->create_service<Get>(
    "shutdown_target/get_state",
    [&target_state](const std::shared_ptr<Get::Request>, std::shared_ptr<Get::Response> response) {
      response->current_state.id = target_state.load();
    });
  // Deferred-response signature deliberately never sends a reply. No sleep,
  // discovery race, RMW override or production timeout modification is involved.
  auto change_service = fixture->create_service<Change>(
    "shutdown_target/change_state",
    [&](
      const std::shared_ptr<rclcpp::Service<Change>> service,
      const std::shared_ptr<rmw_request_id_t> header,
      const std::shared_ptr<Change::Request> request) {
      if (bond_recovery &&
      request->transition.id == lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)
      {
        reset_request_taken.store(true);
        return;
      }
      if (bond_active || bond_release) {
        if (request->transition.id == lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE) {
          target_state.store(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
        } else {
          target_state.store(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
          if (bond_release) {
            activation_header = header;
            request_taken.store(true);
            return;
          }
          counterpart->start();
        }
        auto response = std::make_shared<Change::Response>();
        response->success = true;
        service->send_response(*header, *response);
        return;
      }
      request_taken.store(true);
    });
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("node_names", std::vector<std::string>{"shutdown_target"}),
      rclcpp::Parameter("autostart", false),
      rclcpp::Parameter("bond_timeout", bond_active || bond_release ? 4.0 : 0.0)});
  auto manager = std::make_shared<nav2_lifecycle_manager::LifecycleManager>(options);
  std::weak_ptr<nav2_lifecycle_manager::LifecycleManager> weak_manager = manager;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(fixture);
  executor.add_node(requester);
  executor.add_node(manager);
  ExecutorThread spinning(executor);

  auto ready_client = requester->create_client<std_srvs::srv::Trigger>(
    "lifecycle_manager/is_active");
  require(ready_client->wait_for_service(5s), "manager is_active service not discovered");
  auto ready = ready_client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  // The callback runs on the manager's actual worker: ready proves that its
  // initialization timer has created all clients and the service thread.
  require(ready.wait_for(5s) == std::future_status::ready, "manager worker not ready");

  if (mode == "ownership") {
    executor.remove_node(manager);
    manager.reset();
    const bool released = weak_manager.expired();
    if (!released) {
      // Keep the old-cycle node alive through cleanup so the control fails with
      // an ownership assertion, rather than self-destroying inside its callback.
      manager = weak_manager.lock();
    }
    rclcpp::shutdown();
    manager.reset();
    require(released, "client map retained an owning reference to its parent manager");
  } else {
    auto client = requester->create_client<Manage>("lifecycle_manager/manage_nodes");
    require(client->wait_for_service(5s), "manager command service not discovered");
    auto command = std::make_shared<Manage::Request>();
    command->command = mode == "shutdown" ? Manage::Request::SHUTDOWN : Manage::Request::STARTUP;
    auto pending = client->async_send_request(command);
    if (bond_active) {
      require(pending.wait_for(8s) == std::future_status::ready,
          "bond activation did not complete");
      require(pending.get()->success, "real bond handshake failed");
      require(context->shutdown("active real bond regression"), "Context shutdown failed");
      executor.remove_node(manager);
      manager.reset();
      if (bond_recovery) {
        require(recovery_during_pre.load(), "bond recovery did not begin during pre-shutdown");
      }
      require(weak_manager.expired(), "active bonded manager retained itself");
      std::cout << mode << ": PASS (real activation and bond handshake)" << std::endl;
      return 0;
    }
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!request_taken.load() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(1ms);
    }
    require(request_taken.load(), "deferred lifecycle request was not taken");
    require(pending.wait_for(0ms) != std::future_status::ready, "unexpected manager response");
    if (bond_release) {
      // A finite activation reply is released only after destruction has begun.
      // This exercises the final worker's Bond construction without requiring
      // support for a permanently unanswered RPC under a still-valid Context.
      executor.remove_node(manager);
      std::thread release_owner([owned = std::move(manager)]() mutable {owned.reset();});
      const auto destruction_deadline = std::chrono::steady_clock::now() + 5s;
      while (context->get_pre_shutdown_callbacks().size() != initial_callbacks &&
        std::chrono::steady_clock::now() < destruction_deadline)
      {
        std::this_thread::sleep_for(1ms);
      }
      const bool destroying = context->get_pre_shutdown_callbacks().size() == initial_callbacks;
      std::cout << "bond_owner_release: destructor entered=" << destroying <<
        "; releasing finite ACTIVATE reply" << std::endl;
      auto response = std::make_shared<Change::Response>();
      response->success = true;
      change_service->send_response(*activation_header, *response);
      release_owner.join();
      if (!destroying) {manager = weak_manager.lock();}
      context->shutdown("finite activation owner release regression");
      manager.reset();
      require(destroying, "manager destructor was retained by a parent ownership cycle");
      require(weak_manager.expired(), "manager did not complete destruction");
      std::cout << "bond_owner_release: PASS (finite reply, original bond timeout, worker join)" <<
        std::endl;
      return 0;
    }
    std::cout << mode << ": request taken, response deliberately absent; shutdown Context" <<
      std::endl;
    // The regression is this synchronous call. CTest's child-process deadline
    // catches the old pre-shutdown join deadlock without hiding it in a thread.
    require(context->shutdown("blocked lifecycle request regression"), "Context shutdown failed");
    executor.remove_node(manager);
    manager.reset();
    require(weak_manager.expired(), "manager did not destruct after Context shutdown");
  }
  require(context->get_pre_shutdown_callbacks().size() == initial_callbacks,
    "manager callback registration leaked");
  std::cout << mode << ": PASS (Context shutdown, worker join and manager destruction)" <<
    std::endl;
  return 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {return 2;}
  const std::string mode = argv[1];
  if (mode != "ownership" && mode != "startup" && mode != "shutdown" &&
    mode != "bond_active" && mode != "bond_owner_release" &&
    mode != "bond_recovery_shutdown") {return 2;}
  rclcpp::init(0, nullptr);
  try {
    return exercise(mode);
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
