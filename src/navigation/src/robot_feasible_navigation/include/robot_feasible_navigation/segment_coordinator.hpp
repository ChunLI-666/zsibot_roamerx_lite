// Copyright 2026. Licensed under the Apache License, Version 2.0.
#ifndef ROBOT_FEASIBLE_NAVIGATION__SEGMENT_COORDINATOR_HPP_
#define ROBOT_FEASIBLE_NAVIGATION__SEGMENT_COORDINATOR_HPP_

#include <cstddef>
#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot_feasible_navigation
{

// Ordered geometry only: no node, publisher, task result, collision claim or worker thread.
// Input contract is forward straight motion plus explicit same-XY rotations. It deliberately
// does not infer rotations from XY corners or search a full path for a later equal-XY pose.
class SegmentCoordinator
{
public:
  struct Config
  {
    double xy_tolerance{0.015};
    double yaw_tolerance{0.03};
    double stopped_linear_velocity{0.01};
    double stopped_angular_velocity{0.01};
    double same_xy_epsilon{0.0001};
    double progress_search_distance{0.25};
    double odom_timeout{0.5};
    double rotation_anchor_wait_timeout{15.0};
  };
  enum class Kind {Translation, Rotation};
  enum class CostmapWaitStatus {Ready, Waiting, Stopped, Cancelled, TimedOut};
  enum class InputWaitStatus {Ready, Waiting, TimedOut};
  enum class RotationCommandDecision {Allow, HoldZero, Reject};
  struct Segment
  {
    Kind kind{Kind::Translation};
    nav_msgs::msg::Path path;
    std::vector<double> distance;
    double signed_rotation{0.0};
  };

  SegmentCoordinator();
  explicit SegmentCoordinator(const Config & config);
  void clear();
  void setPath(const nav_msgs::msg::Path & path, bool require_entry_stop = true);
  bool empty() const {return segments_.empty();}
  const Segment & active() const;
  std::size_t index() const {return index_;}
  std::size_t size() const {return segments_.size();}
  bool finalSegment() const {return !empty() && index_ + 1 == segments_.size();}
  bool rotating() const {return !empty() && active().kind == Kind::Rotation;}
  const Config & config() const {return config_;}
  const nav_msgs::msg::Path & fullPath() const {return full_path_;}

  // Pose must already be in path.header.frame_id. Repeated observations are idempotent.
  void observe(const geometry_msgs::msg::Pose & pose);
  bool geometryReached(const geometry_msgs::msg::Pose & pose) const;
  bool orderedProgressReached(double xy_tolerance, double yaw_tolerance) const;
  bool stopped(const geometry_msgs::msg::Twist & measured_velocity) const;
  // Initial task/new-goal/segment entry must first stop. Latches once per entry; an active
  // rotation's nonzero measured angular velocity must not re-enter braking every cycle.
  bool executionReady(const geometry_msgs::msg::Twist & measured_velocity);
  bool entryStopPending() const {return entry_stop_pending_;}
  void requireEntryStop() {entry_stop_pending_ = true;}
  bool rotationAnchorReached(const geometry_msgs::msg::Pose & actual_pose) const;
  // Only for the first phase of a newly received snapshot. The planner retains the
  // actual unquantized request start; an old yaw must not replay an obsolete turn.
  bool incomingRotationStartReached(const geometry_msgs::msg::Pose & actual_pose) const;
  bool validOdometry(double sample_time, double current_time,
    const geometry_msgs::msg::Twist & velocity) const;
  bool terminalWindowEligible(const geometry_msgs::msg::Pose & actual_pose,
    const geometry_msgs::msg::Pose & original_goal, double original_xy_tolerance) const;
  double remainingDistance() const;
  // Caller must first check pose + measured stop. Advancing never selects a nearest segment.
  bool advance();
  // A queued compatible replan may start with an already-finished turn. Skip only reached
  // leading rotations or a sub-tolerance start connector wholly under the current body
  // origin. No nearest-point search or macroscopic translation may be skipped.
  void skipReachedLeadingRotations(const geometry_msgs::msg::Pose & actual_pose);
  static bool compatibleGoal(const nav_msgs::msg::Path & a, const nav_msgs::msg::Path & b);
  static double yaw(const geometry_msgs::msg::Pose & pose);
  static double angle(double value);
  static bool originalGoalGeometryReached(const geometry_msgs::msg::Pose & actual_pose,
    const geometry_msgs::msg::Pose & original_goal, double xy_tolerance, double yaw_tolerance);
  static CostmapWaitStatus costmapWaitStatus(bool running, bool active, bool cancel,
    bool current, double steady_elapsed, double timeout);
  static InputWaitStatus inputWaitStatus(bool ready, double steady_elapsed, double timeout);
  // Direction policy only; the caller must independently certify the remaining sweep.
  // An opposite-direction deceleration requests zero hold, never permission to publish it.
  static RotationCommandDecision rotationCommandDecision(
    double remaining_angle, double commanded_angular_velocity, double measured_angular_velocity);

private:
  void resetObservation();
  void append(Kind kind, const nav_msgs::msg::Path & source, std::size_t first,
    std::size_t last);
  Config config_;
  nav_msgs::msg::Path full_path_;
  std::vector<Segment> segments_;
  std::size_t index_{0};
  std::size_t cursor_{0};
  bool yaw_observed_{false};
  double previous_yaw_{0.0};
  double observed_rotation_{0.0};
  bool entry_stop_pending_{true};
};

}  // namespace robot_feasible_navigation
#endif  // ROBOT_FEASIBLE_NAVIGATION__SEGMENT_COORDINATOR_HPP_
