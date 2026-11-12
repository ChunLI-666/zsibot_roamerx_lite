// Copyright (c) 2026 The project contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_LIFECYCLE_MANAGER__BOUNDED_TRANSITION_HPP_
#define NAV2_LIFECYCLE_MANAGER__BOUNDED_TRANSITION_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace nav2_lifecycle_manager
{
enum class TransitionOutcome {Confirmed, Reconciled, Failed};

struct TransitionResult
{
  TransitionOutcome outcome;
  std::string reason;
  std::uint8_t state;
  double elapsed_seconds;
};

// One transition request, finite steady-clock observation, fresh state checks.
// Only CONFIGURE/ACTIVATE may reconcile an unconfirmed reply. The caller must
// still perform its normal bond handshake before accepting activation.
TransitionResult request_transition(
  rclcpp::Node & manager, const std::string & target, std::uint8_t transition,
  std::uint8_t expected, std::chrono::milliseconds budget,
  const std::atomic_bool & stopping);
}  // namespace nav2_lifecycle_manager
#endif  // NAV2_LIFECYCLE_MANAGER__BOUNDED_TRANSITION_HPP_
