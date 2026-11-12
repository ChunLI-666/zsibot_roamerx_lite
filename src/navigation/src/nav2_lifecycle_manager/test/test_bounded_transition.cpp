// Copyright (c) 2026 The project contributors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_lifecycle_manager/bounded_transition.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
using namespace std::chrono_literals;
using ChangeState = lifecycle_msgs::srv::ChangeState;
using GetState = lifecycle_msgs::srv::GetState;
using State = lifecycle_msgs::msg::State;
using Transition = lifecycle_msgs::msg::Transition;
using nav2_lifecycle_manager::TransitionOutcome;
using nav2_lifecycle_manager::TransitionResult;

enum class ReplyMode {Positive, Negative, Hold};

class BoundedTransitionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // A dedicated Context catches accidental use of the global Context in the
    // helper. Only the service node is spun externally; the manager client is
    // left for request_transition's own callback-group executor.
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context_).use_global_arguments(false);
    static std::atomic<unsigned int> sequence{0};
    const std::string suffix = std::to_string(sequence.fetch_add(1));
    target_ = "bounded_transition_target_" + suffix;
    server_ = std::make_shared<rclcpp::Node>("bounded_transition_server_" + suffix, options);
    manager_ = std::make_shared<rclcpp::Node>("bounded_transition_manager_" + suffix, options);

    change_service_ = server_->create_service<ChangeState>(
      target_ + "/change_state",
      [this](
        const std::shared_ptr<rclcpp::Service<ChangeState>> service,
        const std::shared_ptr<rmw_request_id_t> header,
        const std::shared_ptr<ChangeState::Request> request)
      {
        last_transition_.store(request->transition.id);
        target_state_.store(state_after_change_.load());
        const auto mode = change_reply_.load();
        if (mode == ReplyMode::Hold) {
          // Deferred-response signature: taking a request does not synthesize
          // a response. Keep its header alive without ever replaying the RPC.
          held_change_headers_.push_back(header);
        }
        change_requests_.fetch_add(1);
        if (mode != ReplyMode::Hold) {
          auto response = std::make_shared<ChangeState::Response>();
          response->success = mode == ReplyMode::Positive;
          service->send_response(*header, *response);
        }
      });
    state_service_ = server_->create_service<GetState>(
      target_ + "/get_state",
      [this](
        const std::shared_ptr<rclcpp::Service<GetState>> service,
        const std::shared_ptr<rmw_request_id_t> header,
        const std::shared_ptr<GetState::Request>/*request*/)
      {
        const bool hold = hold_state_reply_.load();
        if (hold) {
          held_state_headers_.push_back(header);
        }
        state_requests_.fetch_add(1);
        if (!hold) {
          auto response = std::make_shared<GetState::Response>();
          response->current_state.id = target_state_.load();
          response->current_state.label = "fixture_state";
          service->send_response(*header, *response);
        }
      });

    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    service_executor_ =
      std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    service_executor_->add_node(server_);
    service_thread_ = std::thread([this]() {
          try {
            service_executor_->spin();
          } catch (...) {
            executor_exception_ = std::current_exception();
          }
      });

    // Discovery readiness only. These clients never send requests and are
    // destroyed before the helper under test creates its actual clients.
    auto change_ready = manager_->create_client<ChangeState>(target_ + "/change_state");
    auto state_ready = manager_->create_client<GetState>(target_ + "/get_state");
    ASSERT_TRUE(change_ready->wait_for_service(1s));
    ASSERT_TRUE(state_ready->wait_for_service(1s));
  }

  void TearDown() override
  {
    // Cancel/join and destroy the external executor while its Context is still
    // valid. The Context-stop test performs this same ordering before shutdown.
    // No callback, future worker or deferred-header holder outlives its nodes.
    stop_service_executor();
    EXPECT_FALSE(static_cast<bool>(executor_exception_));
    change_service_.reset();
    state_service_.reset();
    held_change_headers_.clear();
    held_state_headers_.clear();
    manager_.reset();
    server_.reset();
    if (context_ && context_->is_valid()) {
      context_->shutdown("bounded transition fixture complete");
    }
    context_.reset();
  }

  void stop_service_executor()
  {
    if (!service_executor_) {
      return;
    }
    if (context_->is_valid()) {
      service_executor_->cancel();
    }
    if (service_thread_.joinable()) {
      service_thread_.join();
    }
    if (context_->is_valid()) {
      service_executor_->remove_node(server_);
    }
    service_executor_.reset();
  }

  bool wait_for_change_request()
  {
    const auto deadline = std::chrono::steady_clock::now() + 1500ms;
    while (change_requests_.load() == 0 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(1ms);
    }
    return change_requests_.load() != 0;
  }

  TransitionResult activate(std::chrono::milliseconds budget = 2s)
  {
    return invoke(Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE, budget);
  }

  TransitionResult invoke(
    std::uint8_t transition, std::uint8_t expected, std::chrono::milliseconds budget)
  {
    const auto started = std::chrono::steady_clock::now();
    auto result = nav2_lifecycle_manager::request_transition(
      *manager_, target_, transition, expected, budget, stopping_);
    measured_elapsed_seconds_ =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    return result;
  }

  void expect_one_request(const TransitionResult & result)
  {
    EXPECT_EQ(1u, change_requests_.load()) << "A missing reply must not replay ChangeState";
    EXPECT_GE(result.elapsed_seconds, 0.0);
    // All supplied budgets are at most 2s; allow a small scheduler margin.
    EXPECT_LE(result.elapsed_seconds, 2.2);
    EXPECT_LE(measured_elapsed_seconds_, 2.2);
  }

  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::SharedPtr server_;
  rclcpp::Node::SharedPtr manager_;
  std::string target_;
  rclcpp::Service<ChangeState>::SharedPtr change_service_;
  rclcpp::Service<GetState>::SharedPtr state_service_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> service_executor_;
  std::thread service_thread_;
  std::exception_ptr executor_exception_;
  std::atomic_bool stopping_{false};
  std::atomic<ReplyMode> change_reply_{ReplyMode::Positive};
  std::atomic_bool hold_state_reply_{false};
  std::atomic<std::uint8_t> target_state_{State::PRIMARY_STATE_INACTIVE};
  std::atomic<std::uint8_t> state_after_change_{State::PRIMARY_STATE_ACTIVE};
  std::atomic<std::uint8_t> last_transition_{0};
  std::atomic<unsigned int> change_requests_{0};
  std::atomic<unsigned int> state_requests_{0};
  double measured_elapsed_seconds_{0.0};
  // Only the single service executor writes these; teardown joins it first.
  std::vector<std::shared_ptr<rmw_request_id_t>> held_change_headers_;
  std::vector<std::shared_ptr<rmw_request_id_t>> held_state_headers_;
};

TEST_F(BoundedTransitionTest, PositiveReplyAndFreshStateAreConfirmed)
{
  const auto result = activate(1500ms);
  EXPECT_EQ(TransitionOutcome::Confirmed, result.outcome) << result.reason;
  EXPECT_EQ(State::PRIMARY_STATE_ACTIVE, result.state);
  EXPECT_GE(state_requests_.load(), 1u);
  EXPECT_EQ(Transition::TRANSITION_ACTIVATE, last_transition_.load());
  expect_one_request(result);
}

TEST_F(BoundedTransitionTest, MissingReplyReconcilesFreshExpectedStateAfterGrace)
{
  change_reply_.store(ReplyMode::Hold);
  const auto result = activate();
  EXPECT_EQ(TransitionOutcome::Reconciled, result.outcome) << result.reason;
  EXPECT_EQ(State::PRIMARY_STATE_ACTIVE, result.state);
  EXPECT_GE(state_requests_.load(), 1u);
  EXPECT_GE(result.elapsed_seconds, 1.0) << "An unanswered reply must retain its 1s grace";
  EXPECT_GE(measured_elapsed_seconds_, 1.0);
  expect_one_request(result);
}

TEST_F(BoundedTransitionTest, ExplicitNegativeReplyFailsEvenWhenStateIsExpected)
{
  change_reply_.store(ReplyMode::Negative);
  state_after_change_.store(State::PRIMARY_STATE_ACTIVE);
  const auto result = activate(1500ms);
  EXPECT_EQ(TransitionOutcome::Failed, result.outcome) << result.reason;
  EXPECT_EQ(State::PRIMARY_STATE_ACTIVE, target_state_.load());
  expect_one_request(result);
}

TEST_F(BoundedTransitionTest, MissingReplyAndWrongStateFailWithinBudget)
{
  change_reply_.store(ReplyMode::Hold);
  state_after_change_.store(State::PRIMARY_STATE_INACTIVE);
  const auto result = activate();
  EXPECT_EQ(TransitionOutcome::Failed, result.outcome) << result.reason;
  EXPECT_GE(state_requests_.load(), 1u);
  expect_one_request(result);
}

TEST_F(BoundedTransitionTest, PositiveReplyWithoutFreshGetStateReplyFails)
{
  hold_state_reply_.store(true);
  const auto result = activate(1500ms);
  EXPECT_EQ(TransitionOutcome::Failed, result.outcome) << result.reason;
  EXPECT_GE(state_requests_.load(), 1u);
  expect_one_request(result);
}

TEST_F(BoundedTransitionTest, MissingDeactivateReplyCannotBeReconciled)
{
  change_reply_.store(ReplyMode::Hold);
  target_state_.store(State::PRIMARY_STATE_ACTIVE);
  state_after_change_.store(State::PRIMARY_STATE_INACTIVE);
  const auto result = invoke(
    Transition::TRANSITION_DEACTIVATE, State::PRIMARY_STATE_INACTIVE, 1500ms);
  EXPECT_EQ(TransitionOutcome::Failed, result.outcome) << result.reason;
  EXPECT_EQ(State::PRIMARY_STATE_INACTIVE, target_state_.load());
  EXPECT_EQ(Transition::TRANSITION_DEACTIVATE, last_transition_.load());
  expect_one_request(result);
}

TEST_F(BoundedTransitionTest, StoppingFlagInterruptsOutstandingReply)
{
  change_reply_.store(ReplyMode::Hold);
  std::atomic_bool triggered{false};
  std::thread stopper([this, &triggered]() {
      if (wait_for_change_request()) {
        stopping_.store(true);
        triggered.store(true);
      }
    });
  TransitionResult result{TransitionOutcome::Failed, "helper threw", 0, 0.0};
  EXPECT_NO_THROW(result = activate());
  stopper.join();
  EXPECT_TRUE(triggered.load());
  EXPECT_EQ(TransitionOutcome::Failed, result.outcome) << result.reason;
  EXPECT_LT(result.elapsed_seconds, 1.0) << "Stopping must not wait for the reply grace";
  EXPECT_LT(measured_elapsed_seconds_, 1.0);
  expect_one_request(result);
}

TEST_F(BoundedTransitionTest, ContextShutdownInterruptsOutstandingReply)
{
  change_reply_.store(ReplyMode::Hold);
  std::atomic_bool triggered{false};
  std::atomic_bool stop_error{false};
  std::thread stopper([this, &triggered, &stop_error]() {
      if (wait_for_change_request()) {
        try {
          // Destroy our external executor before invalidating its guard
          // conditions; request_transition still owns its pending client.
          stop_service_executor();
          triggered.store(context_->shutdown("interrupt unanswered transition"));
        } catch (...) {
          stop_error.store(true);
          stopping_.store(true);
        }
      }
    });
  TransitionResult result{TransitionOutcome::Failed, "helper threw", 0, 0.0};
  EXPECT_NO_THROW(result = activate());
  stopper.join();
  EXPECT_TRUE(triggered.load());
  EXPECT_FALSE(stop_error.load());
  EXPECT_FALSE(context_->is_valid());
  EXPECT_EQ(TransitionOutcome::Failed, result.outcome) << result.reason;
  EXPECT_LT(result.elapsed_seconds, 1.0) << "Context stop must not wait for the reply grace";
  EXPECT_LT(measured_elapsed_seconds_, 1.0);
  expect_one_request(result);
}
}  // namespace
