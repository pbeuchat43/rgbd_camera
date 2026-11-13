// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#pragma once
#include "rgbd_camera/provider.hpp"

namespace rgbd_camera {
// One acquisition owner calls inspect before publishing any part of a poll.
// Never remaps timestamps or retains image/IMU payloads. A shared discontinuity
// holds all outputs until every enabled stream has settled for two seconds.
class ClockRecovery {
 public:
  explicit ClockRecovery(const Config& config, int64_t imu_future_tolerance_ns);
  bool inspect(const Products& products, int64_t now_ns);  // true: discard this poll
  bool recovering() const { return recovering_; }
  uint64_t discontinuities() const { return discontinuities_; }
  int64_t last_step_ns() const { return last_step_ns_; }
  int64_t since_ns() const { return since_ns_; }
  static constexpr int64_t jump_limit_ns = 5000000;
  static constexpr int64_t settle_ns = 2000000000;
 private:
  struct Observation {
    uint64_t sequence = 0;
    int64_t raw = 0, offset = 0, mapped = 0, received = 0;
    bool seen = false, valid = false;
  };
  void observe(size_t slot, uint64_t sequence, int64_t mapped, int64_t received,
               const ClockReading& clock, int64_t now, bool& jumped, bool& invalid);
  std::array<Observation, 6> previous_{};
  std::array<bool, 6> active_{};
  bool recover_, recovering_ = false;
  int64_t future_tolerance_, max_age_, stable_since_ = 0, since_ns_ = 0, last_step_ns_ = 0;
  uint64_t discontinuities_ = 0;
};
}
