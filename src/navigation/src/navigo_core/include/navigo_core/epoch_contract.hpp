#ifndef NAVIGO_CORE__EPOCH_CONTRACT_HPP_
#define NAVIGO_CORE__EPOCH_CONTRACT_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <set>
#include <stdexcept>
#include <string>
#include <time.h>
#include "navigo_epoch_msgs/msg/localization_epoch.hpp"
#include "navigo_epoch_msgs/msg/navigation_intent.hpp"
#include "navigo_epoch_msgs/msg/epoch_command.hpp"

namespace navigo_core::epoch
{
inline uint64_t steadyNow()
{
  timespec stamp{};
  if (clock_gettime(CLOCK_MONOTONIC, &stamp) != 0) {throw std::runtime_error("CLOCK_MONOTONIC unavailable");}
  return static_cast<uint64_t>(stamp.tv_sec) * 1000000000ULL + stamp.tv_nsec;
}
inline std::string readId(const std::string & path)
{
  std::ifstream input(path);
  std::string id;
  std::getline(input, id);
  if (id.empty()) {throw std::runtime_error("Cannot read Linux identity: " + path);}
  return id;
}
inline std::string bootId() {return readId("/proc/sys/kernel/random/boot_id");}
inline std::string sessionId() {return readId("/proc/sys/kernel/random/uuid");}
inline bool fresh(uint64_t source, uint64_t now, uint64_t ttl)
{return source > 0 && source <= now && now - source <= ttl;}
inline bool finitePose(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  return !pose.header.frame_id.empty() && pose.header.stamp.sec >= 0 && pose.header.stamp.nanosec < 1000000000 &&
    (pose.header.stamp.sec > 0 || pose.header.stamp.nanosec > 0) &&
    std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(norm) &&
    std::abs(norm - 1.) < 1e-3;
}
inline bool motionTokenValid(const navigo_epoch_msgs::msg::NavigationToken & t)
{
  return (t.execution_kind == t.TRACK && t.backup_max_speed == 0. && t.execution_deadline_ns == 0) ||
    (t.execution_kind == t.BACKUP && std::isfinite(t.backup_max_speed) && t.backup_max_speed >= .05 &&
    t.backup_max_speed <= .1 && t.execution_deadline_ns > 0);
}
inline bool motionCommandValid(const navigo_epoch_msgs::msg::NavigationToken & t,
  const geometry_msgs::msg::Twist & v, uint64_t now)
{
  if (!motionTokenValid(t)) {return false;}
  if (t.execution_kind == t.TRACK) {return std::isfinite(v.linear.x) && v.linear.x >= 0.;}
  return now < t.execution_deadline_ns && std::isfinite(v.linear.x) &&
    v.linear.x <= 0. && v.linear.x >= -t.backup_max_speed &&
    v.linear.y == 0. && v.linear.z == 0. && v.angular.x == 0. && v.angular.y == 0. && v.angular.z == 0.;
}
inline bool validToken(const navigo_epoch_msgs::msg::NavigationToken & t)
{
  return motionTokenValid(t) && !t.localization.process_session_id.empty() && !t.localization.map_loaded_instance.empty() &&
    t.localization.epoch > 0 && t.localization.commits > 0 && !t.navigation_session_id.empty() &&
    t.task_sequence > 0 && t.plan_sequence > 0 && !t.gate_session_id.empty();
}
inline bool commandFresh(const navigo_epoch_msgs::msg::EpochCommand & command,
  const std::string & boot, uint64_t now, uint64_t ttl)
{
  return command.boot_id == boot && command.command_sequence > 0 && !command.controller_session_id.empty() &&
    command.max_age_ms > 0 && fresh(command.source_steady_time_ns, now,
      std::min<uint64_t>(ttl, static_cast<uint64_t>(command.max_age_ms) * 1000000ULL));
}
// Caller owns synchronization. High-water marks survive expiry/invalidation; an
// old retained heartbeat must never renew authority or revive a retired process.
class Authority
{
public:
  std::string boot = bootId();
  uint64_t localization_ttl_ns = 400000000;
  uint64_t intent_ttl_ns = 400000000;
  uint64_t gate_ttl_ns = 400000000;
  navigo_epoch_msgs::msg::LocalizationEpoch localization;
  navigo_epoch_msgs::msg::NavigationIntent intent;
  std::string gate_session;
  std::string loaded_map;
  bool map_changed = false;

  void updateGate(const std::string & value, uint64_t now)
  {
    if (value.empty() || retired_gates.count(value)) {return;}
    if (!gate_session.empty() && value != gate_session) {retired_gates.insert(gate_session);}
    gate_session = value;
    gate_received = now;
  }
  void updateLocalization(const navigo_epoch_msgs::msg::LocalizationEpoch & value, uint64_t now)
  {
    const auto & session = value.identity.process_session_id;
    if (session.empty() || retired_localizations.count(session)) {return;}
    if (session == localization.identity.process_session_id) {
      if (value.heartbeat_sequence <= localization.heartbeat_sequence) {return;}
      const bool ready_claim = value.output_ready && value.fusion_ready && value.lifecycle_enabled && value.health == 1;
      if (ready_claim && (value.identity.epoch < epoch_highwater || value.identity.commits < commits_highwater)) {return;}
    } else if (!localization.identity.process_session_id.empty()) {
      retired_localizations.insert(localization.identity.process_session_id);
      epoch_highwater = commits_highwater = 0;
    }
    if (!value.identity.map_loaded_instance.empty()) {
      if (loaded_map.empty()) {loaded_map = value.identity.map_loaded_instance;}
      if (loaded_map != value.identity.map_loaded_instance) {map_changed = true;}
    }
    epoch_highwater = std::max(epoch_highwater, value.identity.epoch);
    commits_highwater = std::max(commits_highwater, value.identity.commits);
    const int64_t stamp = static_cast<int64_t>(value.output_pose.header.stamp.sec) * 1000000000LL + value.output_pose.header.stamp.nanosec;
    if (!value.identity.map_loaded_instance.empty() && value.identity.epoch > 0 && value.identity.commits > 0 &&
      value.identity.epoch >= epoch_highwater && value.identity.commits >= commits_highwater) {
      if (value.identity != output_identity) {
        output_identity = value.identity;
        output_regressed = false; output_stamp = std::max<int64_t>(0, stamp); output_advanced = stamp > 0 ? now : 0;
      } else if (stamp > 0 && stamp < output_stamp) {
        output_regressed = true;
      } else if (stamp > output_stamp) {
        output_stamp = stamp; output_advanced = now;
      }
    }
    localization = value;
    localization_received = now;
  }
  void updateIntent(const navigo_epoch_msgs::msg::NavigationIntent & value, uint64_t now)
  {
    const auto & session = value.token.navigation_session_id;
    if (!validToken(value.token) || value.boot_id != boot || retired_navigations.count(session) ||
      value.token.localization != localization.identity || value.token.gate_session_id != gate_session ||
      !fresh(value.source_steady_time_ns, now, intent_ttl_ns)) {return;}
    if (session == intent.token.navigation_session_id) {
      if ((value.token.task_sequence == intent.token.task_sequence && value.token.plan_sequence == intent.token.plan_sequence && value.token != intent.token) ||
        value.heartbeat_sequence <= intent.heartbeat_sequence ||
        value.token.task_sequence < intent.token.task_sequence ||
        (value.token.task_sequence == intent.token.task_sequence && value.token.plan_sequence < intent.token.plan_sequence)) {return;}
    } else if (!intent.token.navigation_session_id.empty()) {
      retired_navigations.insert(intent.token.navigation_session_id);
    }
    intent = value;
    intent_received = now;
  }
  bool localizationReady(uint64_t now) const
  {
    return !map_changed && !output_regressed && fresh(output_advanced, now, localization_ttl_ns) && localization.schema_version == 1 && localization.heartbeat_sequence > 0 &&
      localization.lifecycle_enabled && localization.output_ready && localization.fusion_ready &&
      localization.health == navigo_epoch_msgs::msg::LocalizationEpoch::NORMAL &&
      !localization.base_frame_id.empty() && finitePose(localization.output_pose) &&
      fresh(localization_received, now, localization_ttl_ns) && fresh(gate_received, now, gate_ttl_ns);
  }
  bool permits(const navigo_epoch_msgs::msg::NavigationToken & token, uint64_t now) const
  {
    return validToken(token) && (token.execution_kind != token.BACKUP || now < token.execution_deadline_ns) && localizationReady(now) && token.localization == localization.identity &&
      token.gate_session_id == gate_session && intent.active && token == intent.token &&
      fresh(intent_received, now, intent_ttl_ns) && fresh(intent.source_steady_time_ns, now, intent_ttl_ns);
  }
private:
  uint64_t localization_received = 0, intent_received = 0, gate_received = 0, output_advanced = 0;
  int64_t output_stamp = 0;
  bool output_regressed = false;
  uint64_t epoch_highwater = 0, commits_highwater = 0;
  navigo_epoch_msgs::msg::LocalizationIdentity output_identity;
  std::set<std::string> retired_gates, retired_localizations, retired_navigations;
};
}  // namespace navigo_core::epoch
#endif
