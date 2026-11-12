// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace official_lattice_rpp_adapter::goal_region {
// Measurement history, not a motion model: a covered interval says nothing
// about future tail motion. Callers serialize this object with the Odom cache.
class TerminalYawStability {
public:
  struct Proof {
    bool covered=false;
    std::uint64_t revision=0;
    std::int64_t first_stamp=0,last_stamp=0,first_steady=0,last_steady=0;
    double minimum=0.,maximum=0.;
  };
  void configure(std::int64_t window_ns,double maximum_span) {
    if (window_ns<0||window_ns>2000000000LL||!std::isfinite(maximum_span)||
      maximum_span<0.||maximum_span>.08||((window_ns==0)!=(maximum_span==0.))) {
      throw std::invalid_argument("invalid paired terminal yaw stability window/span");
    }
    window_=window_ns;span_=maximum_span;reset();
  }
  bool enabled() const {return window_>0;}
  void clearCertificate() {certified_=false;}
  void reset() {count_=next_=0;++revision_;clearCertificate();}
  bool hasCertificate() const {return certified_;}
  void observe(std::int64_t stamp,std::int64_t steady,double heading) {
    if (!enabled()) {return;}
    if (stamp<=0||steady<=0||!std::isfinite(heading)) {reset();return;}
    if (count_) {
      const auto & last=samples_[(next_+kCapacity-1)%kCapacity];
      if (stamp<=last.stamp||steady<=last.steady||
        stamp-last.stamp>300000000LL||steady-last.steady>300000000LL) {reset();}
    }
    double unwrapped=heading;
    if (count_) {
      const auto & last=samples_[(next_+kCapacity-1)%kCapacity];
      unwrapped=last.heading+std::atan2(std::sin(heading-last_wrapped_),std::cos(heading-last_wrapped_));
    }
    samples_[next_]={stamp,steady,unwrapped};last_wrapped_=heading;
    next_=(next_+1)%kCapacity;count_=std::min(count_+1,kCapacity);++revision_;
    // O(1) revocation between control ticks. Conservatively retain the old
    // certificate's extrema until the next locked full-window measurement.
    if (certified_) {
      certified_min_=std::min(certified_min_,unwrapped);
      certified_max_=std::max(certified_max_,unwrapped);
      if (certified_max_-certified_min_>span_) {clearCertificate();}
    }
  }
  Proof measure() const {
    Proof proof;proof.revision=revision_;
    if (!enabled()||!count_) {return proof;}
    const auto & newest=samples_[(next_+kCapacity-1)%kCapacity];
    proof.last_stamp=newest.stamp;proof.last_steady=newest.steady;
    proof.minimum=proof.maximum=newest.heading;
    for (std::size_t i=0;i<count_;++i) {
      const auto & item=samples_[(next_+kCapacity-1-i)%kCapacity];
      proof.first_stamp=item.stamp;proof.first_steady=item.steady;
      proof.minimum=std::min(proof.minimum,item.heading);
      proof.maximum=std::max(proof.maximum,item.heading);
      // Include the boundary predecessor. Selecting only samples newer than
      // newest-window would usually cover LESS than one complete window.
      if (newest.stamp-item.stamp>=window_&&newest.steady-item.steady>=window_) {
        proof.covered=true;break;
      }
    }
    return proof;
  }
  bool qualifies(const Proof & proof,std::int64_t zero_stamp,std::int64_t zero_steady) const {
    return !enabled()||(proof.covered&&zero_stamp>0&&zero_steady>0&&
      proof.first_stamp>zero_stamp&&proof.first_steady>zero_steady&&
      proof.maximum-proof.minimum<=span_);
  }
  bool certify(const Proof & proof,std::int64_t zero_stamp,std::int64_t zero_steady) {
    if (!enabled()||proof.revision!=revision_||!qualifies(proof,zero_stamp,zero_steady)) {
      clearCertificate();return false;
    }
    certified_min_=proof.minimum;certified_max_=proof.maximum;certified_=true;return true;
  }
private:
  struct Sample {std::int64_t stamp=0,steady=0;double heading=0.;};
  inline static constexpr std::size_t kCapacity=1024;
  std::array<Sample,kCapacity> samples_{};
  std::size_t count_=0,next_=0;
  std::uint64_t revision_=0;
  std::int64_t window_=0;
  double span_=0.,last_wrapped_=0.,certified_min_=0.,certified_max_=0.;
  bool certified_=false;
};
}
