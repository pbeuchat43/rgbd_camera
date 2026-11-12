// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#include "rgbd_camera/imu_combiner.hpp"
#include <cmath>
#include <stdexcept>

namespace rgbd_camera {
ImuCombiner::ImuCombiner(int64_t max_gap_ns) : max_gap_ns_(max_gap_ns) {
  if (max_gap_ns <= 0 || max_gap_ns > 50000000) throw std::invalid_argument("invalid acceleration gap limit");
}
void ImuCombiner::push(const Motion& sample) {
  const size_t i = sample.kind == MotionKind::Accelerometer ? 0 : 1;
  if (sample.acquisition_ns <= last_ns_[i]) throw std::invalid_argument("combined IMU input regressed or repeated");
  for (double x : sample.xyz) if (!std::isfinite(x)) throw std::invalid_argument("nonfinite combined IMU input");
  last_ns_[i] = sample.acquisition_ns;
  auto& queue = i == 0 ? accel_ : gyro_;
  if (queue.size() == capacity_) { queue.pop_front(); ++overflows_; }
  queue.push_back(sample);
}
std::vector<CombinedMotion> ImuCombiner::drain(int64_t now, int64_t max_age) {
  std::vector<CombinedMotion> result;
  while (!gyro_.empty()) {
    const auto g = gyro_.front();
    if (now - g.acquisition_ns > max_age) { gyro_.pop_front(); ++rejected_; continue; }
    if (accel_.size() < 2 || g.acquisition_ns > accel_.back().acquisition_ns) break;
    gyro_.pop_front();
    if (g.acquisition_ns < accel_.front().acquisition_ns) { ++rejected_; continue; }
    while (accel_.size() > 2 && accel_[1].acquisition_ns <= g.acquisition_ns) accel_.pop_front();
    const auto& a = accel_[0]; const auto& b = accel_[1];
    if (g.acquisition_ns > b.acquisition_ns || b.acquisition_ns - a.acquisition_ns > max_gap_ns_ ||
        now - a.acquisition_ns > max_age) { ++rejected_; continue; }
    const double f = double(g.acquisition_ns - a.acquisition_ns) / double(b.acquisition_ns - a.acquisition_ns);
    std::array<double, 3> acceleration{};
    for (size_t i = 0; i < 3; ++i) acceleration[i] = a.xyz[i] + f*(b.xyz[i] - a.xyz[i]);
    result.push_back({g, acceleration, a.acquisition_ns, b.acquisition_ns});
  }
  // Retain at most the latest old acceleration as a possible lower bracket.
  while (accel_.size() > 1 && now - accel_[1].acquisition_ns > max_age) accel_.pop_front();
  return result;
}
}
