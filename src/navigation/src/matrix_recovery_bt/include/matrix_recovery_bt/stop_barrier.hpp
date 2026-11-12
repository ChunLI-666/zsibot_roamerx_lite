// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <cstdint>

namespace matrix_recovery_bt {
// Feed ORIGINAL samples once per callback; tick() only checks their freshness.
// Sampling at 500 Hz does not invalidate a stationary pose or restart the dwell.
class StopBarrier {
public:
  static constexpr std::int64_t dwell_ns = 300000000;
  void arm(std::int64_t steady, std::int64_t source) {
    reset(); armed_ = true; zero_steady_ = steady; zero_source_ = source;
  }
  void reset() {armed_=qualified_=false; first_source_=first_steady_=last_source_=last_steady_=0;}
  void invalidate() {qualified_=false; first_source_=first_steady_=0;}
  bool sample(std::int64_t source, std::int64_t steady, bool healthy,
    double vx, double vy, double wz) {
    const bool sequence = source>last_source_ && steady>=last_steady_ &&
      (!last_source_ || source-last_source_<=dwell_ns) &&
      (!last_steady_ || steady-last_steady_<=dwell_ns);
    // Read-only observations; never participate in the stop predicate.
    diagnostic_sequence_=sequence;diagnostic_healthy_=healthy;++diagnostic_samples_;
    last_source_=source; last_steady_=steady;
    if (!armed_ || !sequence || !healthy || source<=zero_source_ || steady<=zero_steady_ ||
      !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz) ||
      std::hypot(vx,vy)>.03 || std::abs(wz)>.05) {invalidate();return false;}
    if (!qualified_) {qualified_=true;first_source_=source;first_steady_=steady;}
    return ready(steady);
  }
  bool ready(std::int64_t steady) const {
    return armed_ && qualified_ && steady>=last_steady_ && steady-last_steady_<=dwell_ns &&
      last_source_-first_source_>=dwell_ns && last_steady_-first_steady_>=dwell_ns;
  }
  struct Diagnostic {
    bool armed,qualified,advancing_and_bounded,healthy;
    std::int64_t zero_source,zero_steady,first_source,first_steady,last_source,last_steady;
    std::uint64_t samples;
  };
  Diagnostic diagnostic() const noexcept {
    return {armed_,qualified_,diagnostic_sequence_,diagnostic_healthy_,zero_source_,zero_steady_,
      first_source_,first_steady_,last_source_,last_steady_,diagnostic_samples_};
  }
private:
  bool diagnostic_sequence_=false,diagnostic_healthy_=false;
  std::uint64_t diagnostic_samples_=0;
  bool armed_=false, qualified_=false;
  std::int64_t zero_steady_=0,zero_source_=0,first_source_=0,first_steady_=0,last_source_=0,last_steady_=0;
};

// Caller serializes all publications and transitions with the same mutex.
class ZeroEpoch {
public:
  enum class Mode {Closed, Hold, Controller};
  std::uint64_t acquire() {mode=Mode::Hold; return ++epoch;}
  bool release(std::uint64_t token) {
    if (!token || token!=epoch || mode==Mode::Closed) {return false;}
    mode=Mode::Controller;return true; // Idempotent response recovery.
  }
  bool close(std::uint64_t token) {
    // A late successful BT node halt must never zero a newer FollowPath.
    if (!token || token!=epoch || mode!=Mode::Hold) {return false;}
    mode=Mode::Closed;return true;
  }
  bool zeroAllowed(std::uint64_t token) const {return token==epoch && mode!=Mode::Controller;}
  Mode mode=Mode::Closed;
  std::uint64_t epoch=0;
};
}
