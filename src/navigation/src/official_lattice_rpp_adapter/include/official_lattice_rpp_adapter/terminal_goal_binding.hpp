// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <functional>
#include <cstdint>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>
#include "official_lattice_rpp_adapter/planner_goal_contract.hpp"

namespace official_lattice_rpp_adapter::goal_region {
inline constexpr double kHeadingRoundoff=8.*std::numeric_limits<float>::epsilon()*M_PI;
struct CellProof {
  bool valid=false;
  bool native_singleton=false;
  double error=0.;
  double budget=0.;
};
CellProof proveGoalCell(const nav_msgs::msg::Path & raw,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,const nav2_costmap_2d::Costmap2D & map);
double consumerBudget(double resolution,double global_extent,const geometry_msgs::msg::Pose & goal);

struct Binding {
  std::uint64_t generation=0;
  std::uint64_t geometric_digest=0;
  nav_msgs::msg::Path executable;
  geometry_msgs::msg::PoseStamped original_goal;
  double error=0.;
  std::size_t suffix_begin=0;
  std::size_t rotation_begin=0;
  std::vector<double> terminal_headings;
};
// Explicit opt-in protocol only: the final standard goal pose is separated from
// the mandatory executable prefix. Plain FollowPath must not call this parser.
Binding splitBinding(const nav_msgs::msg::Path & path,double resolution,double global_extent);

// Completion is permission to check the current goal geometry, never a cached
// geometry result. A token also binds that check to one immutable plan/cycle.
class CompletionGate {
public:
  struct Snapshot {
    std::uint64_t epoch=0,plan_generation=0;
    bool ready=false;
    std::shared_ptr<const Binding> binding;
    std::int64_t source_valid_until_ns=0;
    std::int64_t source_valid_from_ns=0;
  };
  void invalidate(); // GoalChecker reset-after-setPlan preserves the new binding.
  void clearPlan();
  std::uint64_t bindPlan();
  void installBinding(std::shared_ptr<const Binding> binding);
  std::uint64_t beginCycle(std::uint64_t plan=0);
  void finishCycle(std::uint64_t epoch,bool complete,
    std::chrono::steady_clock::time_point valid_until=std::chrono::steady_clock::time_point::max(),
    std::int64_t source_valid_until_ns=0,std::int64_t source_valid_from_ns=0);
  enum class TerminalProgress {Ordinary, Healthy, Failed};
  void updateTerminalProgress(std::uint64_t plan,std::int64_t deadline_ns,
    std::chrono::steady_clock::time_point valid_until,std::int64_t source_from,
    std::int64_t source_until,std::int64_t source_now);
  struct ProgressSnapshot {TerminalProgress state;std::uint64_t plan;std::int64_t deadline_ns;std::string reason;};
  ProgressSnapshot terminalProgress(std::int64_t source_now);
  void revokeTerminalProgress();
  Snapshot snapshot();
  // The provider only reads the source clock; it runs under the gate lock.
  bool stillEligible(const Snapshot & token,std::function<std::int64_t()> source_clock={});
private:
  bool eligibleLocked() const;
  std::mutex mutex_;
  std::uint64_t epoch_=0,plan_generation_=0;
  bool ready_=false;
  std::chrono::steady_clock::time_point updated_;
  std::chrono::steady_clock::time_point valid_until_;
  std::int64_t source_valid_until_ns_=0;
  std::int64_t source_valid_from_ns_=0;
  std::shared_ptr<const Binding> binding_;
  bool terminal_progress_active_=false,terminal_progress_failed_=false;
  std::uint64_t terminal_progress_plan_=0;
  std::int64_t terminal_deadline_ns_=0,terminal_source_from_=0,terminal_source_until_=0,terminal_last_source_=0;
  std::chrono::steady_clock::time_point terminal_health_until_;
  geometry_msgs::msg::PoseStamped terminal_progress_goal_;
  std::string terminal_progress_reason_;
};

class HeadingCursor {
public:
  void reset(std::vector<double> headings) {headings_=std::move(headings);next_=0;}
  void advance(double actual,double tolerance);
  bool complete() const {return next_==headings_.size();}
  double target() const;
  std::size_t next() const {return next_;}
  const std::vector<double> & headings() const {return headings_;}
private:
  std::vector<double> headings_;
  std::size_t next_=0;
};
}
