// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "robot_feasible_navigation/segment_coordinator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace robot_feasible_navigation
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
double distance(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
}
bool finitePose(const geometry_msgs::msg::Pose & p)
{
  const auto & q = p.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
         std::isfinite(p.position.z) && std::isfinite(norm) && std::abs(norm - 1.0) < 0.001;
}
}  // namespace

SegmentCoordinator::SegmentCoordinator() : SegmentCoordinator(Config{}) {}
SegmentCoordinator::SegmentCoordinator(const Config & config) : config_(config)
{
  const double values[] = {config.xy_tolerance, config.yaw_tolerance,
    config.stopped_linear_velocity, config.stopped_angular_velocity,
    config.same_xy_epsilon, config.progress_search_distance, config.odom_timeout,
    config.rotation_anchor_wait_timeout};
  for (double value : values) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("Segment tolerances and search distance must be positive");
    }
  }
  if (config.progress_search_distance < config.xy_tolerance ||
    config.yaw_tolerance >= kPi / 2.0)
  {
    throw std::invalid_argument("Segment search window/tolerances are inconsistent");
  }
}

double SegmentCoordinator::angle(double value) {return std::atan2(std::sin(value), std::cos(value));}
SegmentCoordinator::RotationCommandDecision SegmentCoordinator::rotationCommandDecision(
  double remaining_angle, double commanded_angular_velocity, double measured_angular_velocity)
{
  if (!std::isfinite(remaining_angle) || !std::isfinite(commanded_angular_velocity) ||
    !std::isfinite(measured_angular_velocity))
  {
    return RotationCommandDecision::Reject;
  }
  if (commanded_angular_velocity == 0.0 ||
    (remaining_angle > 0.0 && commanded_angular_velocity > 0.0) ||
    (remaining_angle < 0.0 && commanded_angular_velocity < 0.0))
  {
    return RotationCommandDecision::Allow;
  }
  const bool same_as_measured =
    (commanded_angular_velocity > 0.0 && measured_angular_velocity > 0.0) ||
    (commanded_angular_velocity < 0.0 && measured_angular_velocity < 0.0);
  if (same_as_measured &&
    std::abs(commanded_angular_velocity) <= std::abs(measured_angular_velocity))
  {
    // RPP's acceleration clamp can retain the measured direction while braking.
    // That direction was not certified by the actual-to-target sweep: publish
    // zero instead, retaining the active phase and the existing progress limit.
    return RotationCommandDecision::HoldZero;
  }
  return RotationCommandDecision::Reject;
}
double SegmentCoordinator::yaw(const geometry_msgs::msg::Pose & p)
{
  const auto & q = p.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
void SegmentCoordinator::resetObservation()
{
  cursor_ = 0;
  yaw_observed_ = false;
  previous_yaw_ = 0.0;
  observed_rotation_ = 0.0;
  entry_stop_pending_ = true;
}
void SegmentCoordinator::clear()
{
  segments_.clear();
  full_path_ = nav_msgs::msg::Path{};
  index_ = 0;
  resetObservation();
}
const SegmentCoordinator::Segment & SegmentCoordinator::active() const
{
  if (empty() || index_ >= segments_.size()) {
    throw std::logic_error("No active ordered path segment");
  }
  return segments_[index_];
}

void SegmentCoordinator::append(
  Kind kind, const nav_msgs::msg::Path & source, std::size_t first, std::size_t last)
{
  Segment segment;
  segment.kind = kind;
  segment.path.header = source.header;
  segment.path.poses.assign(source.poses.begin() + first, source.poses.begin() + last + 1);
  segment.distance.push_back(0.0);
  double unsupported_translation = 0.0;
  double translated_yaw_change = 0.0;
  for (std::size_t i = 1; i < segment.path.poses.size(); ++i) {
    const auto & a = segment.path.poses[i - 1].pose;
    const auto & b = segment.path.poses[i].pose;
    const double ds = distance(a, b);
    segment.distance.push_back(segment.distance.back() + ds);
    segment.signed_rotation += angle(yaw(b) - yaw(a));
    if (kind == Kind::Rotation &&
      distance(segment.path.poses.front().pose, b) > config_.same_xy_epsilon * 2.0)
    {
      throw std::invalid_argument("A rotation phase must retain one body origin");
    }
    if (kind == Kind::Translation && ds > config_.same_xy_epsilon) {
      if (ds > config_.progress_search_distance) {
        throw std::invalid_argument("Translation samples exceed the ordered progress window");
      }
      const double projection = (b.position.x - a.position.x) * std::cos(yaw(a)) +
        (b.position.y - a.position.y) * std::sin(yaw(a));
      if (projection < 0.5 * ds) {
        unsupported_translation += ds;
      }
      translated_yaw_change += std::abs(angle(yaw(b) - yaw(a)));
      // A translating primitive must retain heading. Endpoint connector displacement may be
      // sub-tolerance, but a long sideways/reverse path must never slip through dense samples.
      if (std::abs(angle(yaw(b) - yaw(a))) > 0.03 && ds > config_.xy_tolerance) {
        throw std::invalid_argument("Translated yaw change needs an explicit same-XY rotation");
      }
    }
  }
  if (unsupported_translation > config_.xy_tolerance) {
    throw std::invalid_argument("Only forward translation and explicit rotations are supported");
  }
  if (translated_yaw_change > config_.yaw_tolerance) {
    throw std::invalid_argument("Curved translation is outside the forward/in-place path contract");
  }
  segments_.push_back(std::move(segment));
}

void SegmentCoordinator::setPath(const nav_msgs::msg::Path & input, bool require_entry_stop)
{
  if (input.header.frame_id.empty() || input.poses.empty()) {
    throw std::invalid_argument("Ordered path needs a frame and at least one pose");
  }
  for (const auto & pose : input.poses) {
    if (!finitePose(pose.pose) ||
      (!pose.header.frame_id.empty() && pose.header.frame_id != input.header.frame_id))
    {
      throw std::invalid_argument("Invalid pose or mixed frames in ordered path");
    }
  }
  // Build transactionally: an invalid replan cannot partially replace active progress.
  SegmentCoordinator next(config_);
  next.full_path_ = input;
  nav_msgs::msg::Path path;
  path.header = input.header;
  for (const auto & pose : input.poses) {
    if (path.poses.empty() || distance(pose.pose, path.poses.back().pose) > 1e-9 ||
      std::abs(angle(yaw(pose.pose) - yaw(path.poses.back().pose))) > 1e-9)
    {path.poses.push_back(pose);}
  }
  auto edgeKind = [&](std::size_t i) {
      return distance(path.poses[i - 1].pose, path.poses[i].pose) < config_.same_xy_epsilon &&
             std::abs(angle(yaw(path.poses[i].pose) - yaw(path.poses[i - 1].pose))) > 1e-7 ?
             Kind::Rotation : Kind::Translation;
    };
  if (path.poses.size() == 1) {
    next.append(Kind::Translation, path, 0, 0);
  } else {
    std::size_t first = 0;
    Kind kind = edgeKind(1);
    double rotation = 0.0;
    for (std::size_t i = 1; i < path.poses.size(); ++i) {
      const Kind current = edgeKind(i);
      const double dyaw = angle(yaw(path.poses[i].pose) - yaw(path.poses[i - 1].pose));
      // Split before a sign reversal or a >90deg accumulated turn. No shortcut across pi.
      const bool split_rotation = kind == Kind::Rotation && current == kind &&
        (std::abs(rotation + dyaw) > kPi / 2.0 + 1e-6 || rotation * dyaw < -1e-12);
      if (current != kind || split_rotation) {
        if (i - 1 > first) {
          next.append(kind, path, first, i - 1);
        }
        first = i - 1;
        kind = current;
        rotation = 0.0;
      }
      if (kind == Kind::Rotation) {
        if (std::abs(dyaw) > kPi / 2.0 + 1e-6) {
          throw std::invalid_argument("Rotation input must sample each turn at <=90 degrees");
        }
        rotation += dyaw;
      }
    }
    next.append(kind, path, first, path.poses.size() - 1);
  }
  next.entry_stop_pending_ = require_entry_stop;
  *this = std::move(next);
}

void SegmentCoordinator::observe(const geometry_msgs::msg::Pose & pose)
{
  if (!finitePose(pose)) {
    throw std::invalid_argument("Non-finite robot pose");
  }
  const auto & segment = active();
  if (segment.kind == Kind::Rotation) {
    const double current_yaw = yaw(pose);
    if (!yaw_observed_) {
      // Count actual offset from the planned start, bounded to this <=90deg chunk.
      observed_rotation_ = angle(current_yaw - yaw(segment.path.poses.front().pose));
      yaw_observed_ = true;
    } else {
      observed_rotation_ += angle(current_yaw - previous_yaw_);
    }
    previous_yaw_ = current_yaw;
    return;
  }
  const double limit = segment.distance[cursor_] + config_.progress_search_distance;
  std::size_t best = cursor_;
  double best_distance = distance(pose, segment.path.poses[cursor_].pose);
  for (std::size_t i = cursor_ + 1; i < segment.path.poses.size(); ++i) {
    if (segment.distance[i] > limit) {break;}
    const double d = distance(pose, segment.path.poses[i].pose);
    // Strict < deliberately retains the EARLIEST equal-XY branch.
    if (d < best_distance - 1e-10) {best = i; best_distance = d;}
  }
  cursor_ = best;
}

bool SegmentCoordinator::orderedProgressReached(double xy_tolerance, double yaw_tolerance) const
{
  const auto & segment = active();
  if (segment.kind == Kind::Rotation) {
    return yaw_observed_ &&
           std::abs(segment.signed_rotation - observed_rotation_) <= yaw_tolerance;
  }
  return segment.distance.back() - segment.distance[cursor_] <= xy_tolerance;
}
bool SegmentCoordinator::geometryReached(const geometry_msgs::msg::Pose & pose) const
{
  return geometryReached(pose, config_.xy_tolerance);
}
bool SegmentCoordinator::geometryReached(const geometry_msgs::msg::Pose & pose,
  double maximum_track_xy_tolerance) const
{
  if (!std::isfinite(maximum_track_xy_tolerance) || maximum_track_xy_tolerance <= 0.0) {
    return false;
  }
  const double xy_tolerance = rotating() ? config_.xy_tolerance :
    std::min(config_.xy_tolerance, maximum_track_xy_tolerance);
  const auto & target = active().path.poses.back().pose;
  return distance(pose, target) <= xy_tolerance &&
         (!rotating() || std::abs(angle(yaw(target) - yaw(pose))) <= config_.yaw_tolerance) &&
         orderedProgressReached(xy_tolerance * 2.0, config_.yaw_tolerance);
}
bool SegmentCoordinator::stopped(const geometry_msgs::msg::Twist & velocity) const
{
  return std::isfinite(velocity.linear.x) && std::isfinite(velocity.linear.y) &&
         std::isfinite(velocity.angular.z) &&
         std::hypot(velocity.linear.x, velocity.linear.y) <= config_.stopped_linear_velocity &&
         std::abs(velocity.angular.z) <= config_.stopped_angular_velocity;
}
bool SegmentCoordinator::executionReady(const geometry_msgs::msg::Twist & velocity)
{
  if (entry_stop_pending_ && stopped(velocity)) {entry_stop_pending_ = false;}
  return !entry_stop_pending_;
}
bool SegmentCoordinator::rotationAnchorReached(const geometry_msgs::msg::Pose & actual_pose) const
{
  return !rotating() || (finitePose(actual_pose) &&
         distance(actual_pose, active().path.poses.front().pose) <= config_.xy_tolerance);
}
bool SegmentCoordinator::incomingRotationStartReached(
  const geometry_msgs::msg::Pose & actual_pose) const
{
  return !rotating() || (rotationAnchorReached(actual_pose) &&
         std::abs(angle(yaw(full_path_.poses.front().pose) - yaw(actual_pose))) <=
         config_.yaw_tolerance);
}
bool SegmentCoordinator::validOdometry(double sample_time, double current_time,
  const geometry_msgs::msg::Twist & velocity) const
{
  const double age = current_time - sample_time;
  return std::isfinite(sample_time) && std::isfinite(current_time) && sample_time > 0.0 &&
         age >= 0.0 && age <= config_.odom_timeout && std::isfinite(velocity.linear.x) &&
         std::isfinite(velocity.linear.y) && std::isfinite(velocity.linear.z) &&
         std::isfinite(velocity.angular.x) && std::isfinite(velocity.angular.y) &&
         std::isfinite(velocity.angular.z);
}
bool SegmentCoordinator::terminalWindowEligible(const geometry_msgs::msg::Pose & actual_pose,
  const geometry_msgs::msg::Pose & original_goal, double tolerance) const
{
  if (empty() || !finitePose(actual_pose) || !finitePose(original_goal) ||
    !std::isfinite(tolerance) || tolerance <= 0.0 ||
    distance(actual_pose, original_goal) > tolerance)
  {return false;}
  // Project only onto the current TRACK edge. The nearest discrete sample may be
  // slightly behind the body; adding that distance to the remaining route would
  // count already completed travel twice and deadlock at the final XY tolerance.
  // No later segment or equal-XY occurrence is searched here.
  const auto & active_poses = active().path.poses;
  auto prefix = active_poses[cursor_].pose;
  double completed_edge_distance = 0.0;
  std::size_t remaining_first = cursor_;
  if (!rotating() && cursor_ + 1 < active_poses.size()) {
    const auto & next = active_poses[cursor_ + 1].pose;
    const double dx = next.position.x - prefix.position.x;
    const double dy = next.position.y - prefix.position.y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared > 0.0) {
      const double fraction = std::clamp(((actual_pose.position.x - prefix.position.x) * dx +
        (actual_pose.position.y - prefix.position.y) * dy) / length_squared, 0.0, 1.0);
      prefix.position.x += fraction * dx;
      prefix.position.y += fraction * dy;
      completed_edge_distance = fraction * std::sqrt(length_squared);
      if (fraction > 0.0) {remaining_first = cursor_ + 1;}
    }
  }
  // Route progress and current goal geometry are separate tests. Lateral
  // tracking error is already bounded by the Euclidean goal check above;
  // adding it here creates a stricter taxicab threshold that can prevent the
  // explicit final rotation even though the original XY tolerance is met.
  const double remaining = remainingDistance() - completed_edge_distance;
  if (remaining > tolerance) {return false;}
  if (distance(prefix, original_goal) > tolerance) {return false;}
  for (std::size_t phase = index_; phase < segments_.size(); ++phase) {
    const auto & poses = segments_[phase].path.poses;
    const std::size_t first = phase == index_ ? remaining_first : 0;
    for (std::size_t i = first; i < poses.size(); ++i) {
      if (distance(poses[i].pose, original_goal) > tolerance) {return false;}
    }
  }
  return true;
}
bool SegmentCoordinator::originalGoalGeometryReached(const geometry_msgs::msg::Pose & actual,
  const geometry_msgs::msg::Pose & goal, double xy_tolerance, double yaw_tolerance)
{
  return finitePose(actual) && finitePose(goal) && std::isfinite(xy_tolerance) &&
         std::isfinite(yaw_tolerance) && xy_tolerance > 0.0 && yaw_tolerance >= 0.0 &&
         distance(actual, goal) <= xy_tolerance &&
         std::abs(angle(yaw(goal) - yaw(actual))) <= yaw_tolerance;
}
double SegmentCoordinator::remainingDistance() const
{
  double remaining = active().distance.back() - active().distance[cursor_];
  for (std::size_t i = index_ + 1; i < segments_.size(); ++i) {
    remaining += segments_[i].distance.back();
  }
  return remaining;
}
bool SegmentCoordinator::advance()
{
  if (finalSegment()) {return false;}
  ++index_;
  resetObservation();
  return true;
}
void SegmentCoordinator::skipReachedLeadingRotations(const geometry_msgs::msg::Pose & actual_pose)
{
  while (!empty() && !finalSegment()) {
    const auto & target = active().path.poses.back().pose;
    if (!rotating()) {
      // Replanning may retain a sub-cell start connector with an old heading. Its entire
      // translation must already fit inside the strict XY tolerance before skipping it.
      if (active().distance.back() > config_.xy_tolerance) {break;}
      bool wholly_reached = true;
      for (const auto & pose : active().path.poses) {
        wholly_reached = wholly_reached && distance(actual_pose, pose.pose) <= config_.xy_tolerance;
      }
      if (!wholly_reached) {break;}
      advance();
      continue;
    }
    if (distance(actual_pose, target) > config_.xy_tolerance ||
      std::abs(angle(yaw(target) - yaw(actual_pose))) > config_.yaw_tolerance)
    {break;}
    advance();
  }
}
bool SegmentCoordinator::compatibleGoal(const nav_msgs::msg::Path & a, const nav_msgs::msg::Path & b)
{
  return !a.poses.empty() && !b.poses.empty() && a.header.frame_id == b.header.frame_id &&
         distance(a.poses.back().pose, b.poses.back().pose) <= 1e-4 &&
         std::abs(angle(yaw(a.poses.back().pose) - yaw(b.poses.back().pose))) <= 1e-4;
}
SegmentCoordinator::CostmapWaitStatus SegmentCoordinator::costmapWaitStatus(
  bool running, bool active, bool cancel, bool current, double steady_elapsed, double timeout)
{
  if (!running || !active) {return CostmapWaitStatus::Stopped;}
  if (cancel) {return CostmapWaitStatus::Cancelled;}
  if (current) {return CostmapWaitStatus::Ready;}
  if (!std::isfinite(steady_elapsed) || !std::isfinite(timeout) || timeout < 0.0 ||
    steady_elapsed >= timeout)
  {return CostmapWaitStatus::TimedOut;}
  return CostmapWaitStatus::Waiting;
}
SegmentCoordinator::InputWaitStatus SegmentCoordinator::inputWaitStatus(
  bool ready, double steady_elapsed, double timeout)
{
  if (ready) {return InputWaitStatus::Ready;}
  if (!std::isfinite(steady_elapsed) || !std::isfinite(timeout) || timeout < 0.0 ||
    steady_elapsed >= timeout)
  {return InputWaitStatus::TimedOut;}
  return InputWaitStatus::Waiting;
}
}  // namespace robot_feasible_navigation
