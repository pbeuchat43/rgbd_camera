// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#include "rgbd_camera/clock_recovery.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rgbd_camera {
ClockRecovery::ClockRecovery(const Config& config, int64_t future_tolerance)
    : active_{true, true, config.enable_imu, config.enable_imu, config.publish_stereo, config.publish_stereo},
      recover_(config.timestamp_policy == "recover"), future_tolerance_(future_tolerance),
      max_age_(int64_t(config.max_frame_age_ms * 1e6)) {}

void ClockRecovery::observe(size_t slot, uint64_t sequence, int64_t mapped, int64_t received,
                            const ClockReading& clock, int64_t now, bool& jumped, bool& invalid) {
  if (!active_[slot]) throw std::runtime_error("clock observation from disabled stream");
  if (received <= 0 || received > now) throw std::runtime_error("invalid host receipt clock");
  auto& p = previous_[slot];
  const bool motion = slot == 2 || slot == 3;
  // DepthAI repeats independent reports in paired IMU packets. A repeated raw
  // report is not a second observation, nor a clock correction.
  if (clock.device_ns > 0 && p.raw > 0) {
    if (motion && sequence == p.sequence && clock.device_ns == p.raw) return;
    const bool counter_wrap = motion && p.sequence <= UINT32_MAX && sequence <= UINT32_MAX &&
      p.sequence > 0xf0000000ULL && sequence < 0x10000000ULL;
    if (clock.device_ns <= p.raw || (sequence <= p.sequence && !counter_wrap))
      throw std::runtime_error("raw device clock/sequence reset; new device session required");
    const auto step = (clock.sdk_ns-clock.device_ns)-p.offset;
    if (std::abs(step) > jump_limit_ns) { jumped = true; last_step_ns_ = step; }
  }
  const auto tolerance = motion ? future_tolerance_ : 0;
  const bool valid = mapped > 0 && mapped <= received+tolerance && now-mapped <= max_age_ &&
    (!p.seen || mapped > p.mapped || (motion && sequence == p.sequence && mapped == p.mapped));
  invalid |= !valid;
  p = {sequence, clock.device_ns, clock.sdk_ns-clock.device_ns, mapped, received, true, valid};
}

bool ClockRecovery::inspect(const Products& products, int64_t now) {
  bool jumped = false, invalid = false;
  for (const auto& m : products.motion)
    observe(m.kind == MotionKind::Accelerometer ? 2 : 3, m.sequence, m.acquisition_ns,
            m.received_ns, m.clock, now, jumped, invalid);
  if (products.frame) {
    const auto& f = *products.frame;
    observe(0, f.sequence, std::max(f.acquisition_ns, f.exposure_midpoint_ns), f.received_ns, f.clocks[0], now, jumped, invalid);
    observe(1, f.depth_sequence, std::max(f.depth_acquisition_ns, f.depth_exposure_midpoint_ns), f.received_ns, f.clocks[1], now, jumped, invalid);
  }
  if (products.stereo) for (size_t i = 0; i < 2; ++i) {
    const auto& s = *products.stereo;
    observe(4+i, s.sequence[i], std::max(s.event_ns[i], s.midpoint_ns[i]), s.received_ns, s.clocks[i], now, jumped, invalid);
  }
  if (jumped) {
    if (!recover_) throw std::runtime_error("SDK clock discontinuity: offset_step_ns=" + std::to_string(last_step_ns_));
    if (!recovering_) { ++discontinuities_; since_ns_ = now; recovering_ = true; }
    stable_since_ = 0;
  }
  if (!recovering_) return false;
  bool fresh = !invalid && !jumped;
  for (size_t i = 0; i < active_.size(); ++i) if (active_[i]) {
    const auto& p = previous_[i];
    fresh &= p.seen && p.valid && p.received >= since_ns_ && now-p.received <= max_age_;
  }
  if (!fresh) stable_since_ = 0;
  else if (!stable_since_) stable_since_ = now;
  else if (now-stable_since_ >= settle_ns) recovering_ = false;
  // Also discard the settling poll: no partial batch straddles the boundary.
  return true;
}
}
