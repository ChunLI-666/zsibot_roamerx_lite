// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace matrix_recovery_bt {
// Diagnostic counters only: check returns the original predicate unchanged.
enum class Reject : std::size_t {
  FollowActive,OdomMissing,OdomFrame,OdomStamp,OdomReceive,Localization,PoseValid,
  LocalizationReceive,ValidReceive,ScanMissing,ScanStamp,ScanReceive,SafeCommand,
  MapMissing,MapReceive,MapStamp,MapMetadata,MapOrientation,BodyCollision,
  PathMissing,PathFrame,BodyDrift,PathStart,PathSweep,Exception,StopDwell,Count
};
inline constexpr std::array<const char *,static_cast<std::size_t>(Reject::Count)> reject_names={
  "follow_active","odom_missing","odom_frame","odom_stamp","odom_receive","localization",
  "pose_valid","localization_receive","valid_receive","scan_missing","scan_stamp","scan_receive",
  "safe_command","map_missing","map_receive","map_stamp","map_metadata","map_orientation",
  "body_collision","path_missing","path_frame","body_drift","path_start","path_sweep","exception","stop_dwell"};
struct WaitCounters {
  std::array<std::uint64_t,static_cast<std::size_t>(Reject::Count)> rejected{};
  const char * last="none";
  bool check(bool value,Reject reason) noexcept {
    if (!value) {++rejected[static_cast<std::size_t>(reason)];last=reject_names[static_cast<std::size_t>(reason)];}
    return value;
  }
};
inline std::string jsonText(const std::string & text) {
  std::ostringstream out;out<<'"';
  for (unsigned char c:text) {
    if (c=='"'||c=='\\') {out<<'\\'<<c;}
    else if (c<32) {out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<static_cast<unsigned int>(c)<<std::dec;}
    else {out<<c;}
  }
  out<<'"';return out.str();
}
inline std::string jsonNumber(double value) {
  if (!std::isfinite(value)) {return "null";}
  std::ostringstream out;out<<std::setprecision(17)<<value;return out.str();
}
}
