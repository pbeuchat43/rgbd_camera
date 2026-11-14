// Copyright (c) 2026 Paul N. Beuchat. MIT License.
// Experimental offline prototype; not used by the camera provider.
#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace clock_experiment {
enum class ClockSource { Sdk, Local };
enum class FaultPolicy { Stop, Drop };

// One instance per camera generation, used by its sole polling owner. Observe
// only one consistent receipt family (here: IMU provider dequeue receipts).
// The owner already has those receipts; no callback handoff or new queue is needed.
// Receipt is an upper bound including transport delay, NOT a clock reference.
class Mapper {
 public:
  static constexpr int64_t bin_ns = 250000000;
  static constexpr int64_t startup_ns = 1000000000;
  static constexpr int64_t margin_ns = 5000000;
  static constexpr int64_t slew_ppb = 5000000;  // +/-5000 ppm, prototype only.
  static constexpr int64_t nominal_ppb = 1000000000;

  void observe(int64_t device, int64_t receipt) {
    if (device <= 0 || receipt <= 0) throw std::invalid_argument("invalid observation");
    if (!first_receipt_) first_receipt_ = receipt;
    if (receipt < last_receipt_) throw std::invalid_argument("receipt clock regressed");
    last_receipt_ = receipt;
    const auto index = (receipt - first_receipt_) / bin_ns;
    auto& bin = bins_[size_t(index) % bins_.size()];
    if (bin.index != index) bin = {index, receipt - device};
    else bin.minimum = std::min(bin.minimum, receipt - device);
    if (receipt - first_receipt_ < startup_ns) return;
    if (size_ && (index == last_update_bin_ || device <= newest().device)) return;
    int64_t offset = std::numeric_limits<int64_t>::max();
    for (const auto& candidate : bins_)
      if (candidate.index >= 0 && index - candidate.index < int64_t(bins_.size()))
        offset = std::min(offset, candidate.minimum);
    const auto target = device + offset - margin_ns;
    // Join at the old map's value. Published history is never moved or clamped.
    const auto anchor = size_ ? evaluate(newest(), device) : target;
    const auto rate = nominal_ppb + std::clamp(target - anchor, -slew_ppb, slew_ppb);
    append({device, anchor, rate});
    min_rate_ = std::min(min_rate_, rate);
    max_rate_ = std::max(max_rate_, rate);
    last_update_bin_ = index;
  }

  std::optional<int64_t> map(int64_t device) const {
    for (size_t n = 0; n < size_; ++n) {
      const auto& segment = segments_[(head_ + segments_.size() - 1 - n) % segments_.size()];
      if (device >= segment.device) return evaluate(segment, device);
    }
    return {};  // Startup or an event older than the retained coefficient history.
  }
  bool ready() const { return size_ != 0; }
  int64_t min_rate_ppb() const { return min_rate_; }
  int64_t max_rate_ppb() const { return max_rate_; }

 private:
  struct Bin { int64_t index = -1, minimum = 0; };
  struct Segment { int64_t device = 0, host = 0, rate = nominal_ppb; };
  static int64_t evaluate(const Segment& segment, int64_t device) {
    // Integer local origins avoid floating-point precision loss on boot epochs.
    return segment.host + int64_t((__int128(device - segment.device) * segment.rate) / nominal_ppb);
  }
  const Segment& newest() const { return segments_[(head_ + segments_.size() - 1) % segments_.size()]; }
  void append(Segment segment) {
    segments_[head_] = segment;
    head_ = (head_ + 1) % segments_.size();
    size_ = std::min(size_ + 1, segments_.size());
  }
  std::array<Bin, 8> bins_{};              // Two seconds of receipt minima.
  std::array<Segment, 64> segments_{};     // Normally sixteen seconds of history.
  size_t head_ = 0, size_ = 0;
  int64_t first_receipt_ = 0, last_receipt_ = 0, last_update_bin_ = -1;
  int64_t min_rate_ = nominal_ppb, max_rate_ = nominal_ppb;
};

// Raw source reset is an epoch fault; neither clock choice may conceal it.
class RawClockGuard {
 public:
  void observe(size_t sensor, int64_t device, uint32_t sequence) {
    const auto delta = uint32_t(sequence - last_sequence_[sensor]);
    if (device <= 0 || (seen_[sensor] &&
        (device < last_device_[sensor] || delta > 0x80000000u)))
      throw std::runtime_error("raw device clock/sequence reset: new generation required");
    seen_[sensor] = true;
    last_device_[sensor] = device;
    last_sequence_[sensor] = sequence;
  }
 private:
  std::array<bool, 2> seen_{};
  std::array<int64_t, 2> last_device_{};
  std::array<uint32_t, 2> last_sequence_{};
};
}  // namespace clock_experiment
