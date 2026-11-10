// SPDX-License-Identifier: Apache-2.0
#pragma once

namespace matrix_recovery_bt {
// Project-owned uint16 action results, separate from Nav2's 100-900 ranges.
// Keep the mission classifier and normal BT readback contract in sync.
enum RecoveryError : int {
  None = 0,
  PlanStartInvalidated = 60001,
  InputHealthTimeout = 60002,
  OwnershipUnknown = 60003,
  InvalidPlanContract = 60004,
  CollisionWaitTimeout = 60005,
};
enum class Certificate {
  Valid, InputWait, PlanInvalidated, InvalidPath, Blocked
};
}  // namespace matrix_recovery_bt
