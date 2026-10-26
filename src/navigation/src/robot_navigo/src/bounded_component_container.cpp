// Preserve isolated component executors while giving each pool an explicit budget.
#include <charconv>
#include <chrono>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl/error_handling.h>
#include <rcl/service.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_components/component_manager_isolated.hpp>

namespace
{
// Set once before loading components. ComponentManagerIsolated constructs its
// ExecutorT without arguments, so the default constructor reads this startup budget.
unsigned int threads_per_component = 2;

class BoundedExecutor : public rclcpp::executors::MultiThreadedExecutor
{
public:
  BoundedExecutor()
  : MultiThreadedExecutor(rclcpp::ExecutorOptions(), threads_per_component)
  {
  }
};


// Called only before the node is added to an executor. Preserve the one RMW
// service object, its queues/QoS/GUIDs; release the old C++ callback owner first.
template<typename ServiceT>
void rebind_service_callback(
  rclcpp::Node & node,
  typename rclcpp::Service<ServiceT>::SharedPtr & owner,
  rclcpp::AnyServiceCallback<ServiceT> callback)
{
  const auto handle = owner->get_service_handle();
  const std::weak_ptr<rclcpp::Service<ServiceT>> old_wrapper = owner;
  owner->clear_on_new_request_callback();
  owner.reset();
  if (!old_wrapper.expired()) {
    throw std::logic_error("Old LoadNode service wrapper still has a strong owner");
  }
  auto replacement = std::make_shared<rclcpp::Service<ServiceT>>(
    node.get_node_base_interface()->get_shared_rcl_node_handle(), handle, callback);
  if (replacement->get_service_handle().get() != handle.get()) {
    throw std::logic_error("LoadNode RMW service handle changed during callback rebind");
  }
  node.get_node_services_interface()->add_service(
    replacement, node.get_node_base_interface()->get_default_callback_group());
  owner = std::move(replacement);
}

// Reuse the original RMW service; replace only its C++ response callback before
// the manager is added to the outer single-threaded executor.
class RetryingLoadComponentManager
  : public rclcpp_components::ComponentManagerIsolated<BoundedExecutor>
{
  friend struct LoadReplyTestAccess;
  using Base = rclcpp_components::ComponentManagerIsolated<BoundedExecutor>;
  using LoadNode = rclcpp_components::ComponentManager::LoadNode;
  using Clock = std::chrono::steady_clock;

  struct PendingResponse
  {
    rmw_request_id_t request_id;
    std::shared_ptr<LoadNode::Response> response;
    Clock::time_point deadline;
    Clock::time_point next_due;
    unsigned int attempts = 0;
  };

public:
  explicit RetryingLoadComponentManager(std::weak_ptr<rclcpp::Executor> executor)
  : Base(executor)
  {
    rclcpp::AnyServiceCallback<LoadNode> callback;
    std::function<void(
      std::shared_ptr<rmw_request_id_t>, std::shared_ptr<LoadNode::Request>)> deferred =
      [this](std::shared_ptr<rmw_request_id_t> header,
        std::shared_ptr<LoadNode::Request> request)
      {
        auto response = std::make_shared<LoadNode::Response>();
        // This is the only construction call, including when reply delivery fails.
        Base::on_load_node(header, request, response);
        const auto begin = Clock::now();
        PendingResponse pending{*header, std::move(response),
          begin + std::chrono::seconds(2), begin, 0};
        if (send_response(pending) != RCL_RET_TIMEOUT) {
          return;
        }
        pending.next_due = Clock::now() + std::chrono::milliseconds(200);
        if (pending_responses_.size() >= 32) {
          throw std::runtime_error("LoadNode reply retry queue full; no request is replayed");
        }
        const bool was_empty = pending_responses_.empty();
        pending_responses_.push_back(std::move(pending));
        if (was_empty) {
          retry_timer_->reset();
        }
      };
    callback.set(deferred);
    rebind_service_callback<LoadNode>(*this, loadNode_srv_, callback);
    const auto group = get_node_base_interface()->get_default_callback_group();
    retry_timer_ = create_wall_timer(std::chrono::milliseconds(200),
      [this]() {retry_pending_responses();}, group, false);
  }

  ~RetryingLoadComponentManager() override
  {
    // main() stops the outer executor before destroying the manager and only
    // shuts down the context afterwards. No detached thread owns this object.
    if (retry_timer_) {
      try {
        retry_timer_->cancel();
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Cancel LoadNode reply retry timer: %s", error.what());
      }
    }
    pending_responses_.clear();
  }

private:
  rcl_ret_t send_response(PendingResponse & pending)
  {
    ++pending.attempts;
    const rcl_ret_t result = rcl_send_response(loadNode_srv_->get_service_handle().get(),
      &pending.request_id, pending.response.get());
    if (result == RCL_RET_OK) {
      if (pending.attempts > 1) {
        RCLCPP_INFO(get_logger(),
          "LoadNode response send returned OK seq=%lld id=%llu attempt=%u (not client ACK)",
          static_cast<long long>(pending.request_id.sequence_number),
          static_cast<unsigned long long>(pending.response->unique_id), pending.attempts);
      }
      return result;
    }
    const std::string detail = rcl_get_error_string().str;
    if (result == RCL_RET_TIMEOUT) {
      rcl_reset_error();
      RCLCPP_WARN(get_logger(), "LoadNode response timeout seq=%lld id=%llu attempt=%u: %s",
        static_cast<long long>(pending.request_id.sequence_number),
        static_cast<unsigned long long>(pending.response->unique_id), pending.attempts,
        detail.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "LoadNode response stopped seq=%lld return=%d: %s",
        static_cast<long long>(pending.request_id.sequence_number), static_cast<int>(result),
        detail.c_str());
      // Preserve the original rclcpp hard-failure contract for both the first
      // send and a timer retry. throw_from_rcl_error retains the original detail.
      rclcpp::exceptions::throw_from_rcl_error(result, "failed to send LoadNode response");
    }
    return result;
  }

  void retry_pending_responses()
  {
    // The timer and LoadNode service share the default mutually-exclusive group,
    // and the manager retains the original outer SingleThreadedExecutor.
    if (!rclcpp::ok(get_node_base_interface()->get_context())) {
      pending_responses_.clear();
      retry_timer_->cancel();
      return;
    }
    for (auto it = pending_responses_.begin(); it != pending_responses_.end();) {
      const auto now = Clock::now();
      // Never begin another attempt after the deadline/cap. An in-flight RMW
      // send can itself return after that deadline; this is not a hard 2s bound.
      if (now >= it->deadline || it->attempts >= 10) {
        RCLCPP_ERROR(get_logger(), "LoadNode response retry expired seq=%lld attempts=%u",
          static_cast<long long>(it->request_id.sequence_number), it->attempts);
        it = pending_responses_.erase(it);
        continue;
      }
      if (now < it->next_due) {
        ++it;
        continue;
      }
      if (send_response(*it) == RCL_RET_TIMEOUT) {
        it->next_due = Clock::now() + std::chrono::milliseconds(200);
        ++it;
      } else {
        it = pending_responses_.erase(it);
      }
    }
    if (pending_responses_.empty()) {
      retry_timer_->cancel();
    }
  }

  std::deque<PendingResponse> pending_responses_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};

unsigned int parse_threads(const std::string & value)
{
  unsigned int result = 0;
  const char * begin = value.data();
  const char * end = begin + value.size();
  const auto parsed = std::from_chars(begin, end, result);
  if (parsed.ec != std::errc{} || parsed.ptr != end || result > 256) {
    throw std::invalid_argument("--executor-threads requires 0 (auto) or an integer in [1, 256]");
  }
  return result;
}

bool parse_arguments(const std::vector<std::string> & arguments)
{
  for (size_t i = 1; i < arguments.size(); ++i) {
    const auto & argument = arguments[i];
    if (argument == "--help" || argument == "-h") {
      std::cout << "Usage: bounded_component_container [--executor-threads N] [--ros-args ...]\n"
                << "N is the worker count per isolated component; default 2, 0 uses CPU auto detection.\n";
      return false;
    }
    if (argument == "--use_multi_threaded_executor") {
      continue;  // Accept the existing isolated-container launch flag.
    }
    if (argument == "--executor-threads") {
      if (++i == arguments.size()) {
        throw std::invalid_argument("Missing value for --executor-threads");
      }
      threads_per_component = parse_threads(arguments[i]);
    } else if (argument.rfind("--executor-threads=", 0) == 0) {
      threads_per_component = parse_threads(argument.substr(19));
    } else {
      throw std::invalid_argument("Unknown argument: " + argument);
    }
  }
  return true;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (!parse_arguments(rclcpp::remove_ros_arguments(argc, argv))) {
      return 0;
    }
    rclcpp::init(argc, argv);
    {
      auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      auto manager = std::make_shared<
        RetryingLoadComponentManager>(executor);
      RCLCPP_INFO(
        manager->get_logger(), "Isolated component executor threads: %u (0 means CPU auto)",
        threads_per_component);
      executor->add_node(manager);
      executor->spin();
    }
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "bounded_component_container: " << error.what() << '\n';
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 2;
  }
}
