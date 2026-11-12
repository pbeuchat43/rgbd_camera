// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#pragma once
#include "rgbd_camera/provider.hpp"
#include <deque>

namespace rgbd_camera {
struct CombinedMotion {
  Motion gyro;
  std::array<double, 3> acceleration;
  int64_t before_ns, after_ns;
};
// One instance per provider generation; caller first applies MotionGate.
class ImuCombiner {
 public:
  explicit ImuCombiner(int64_t max_gap_ns);
  void push(const Motion& sample);
  std::vector<CombinedMotion> drain(int64_t now_ns, int64_t max_age_ns);
  uint64_t rejected() const { return rejected_; }
  uint64_t overflows() const { return overflows_; }
 private:
  static constexpr size_t capacity_ = 200;
  int64_t max_gap_ns_;
  std::deque<Motion> accel_, gyro_;
  std::array<int64_t, 2> last_ns_{};
  uint64_t rejected_ = 0, overflows_ = 0;
};
}
