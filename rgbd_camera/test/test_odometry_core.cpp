// Copyright (c) 2026 Paul N. Beuchat. MIT License.
// Synthetic timing/geometry checks; no physical calibration evidence.
#include "rgbd_camera/provider.hpp"
#include "rgbd_camera/imu_combiner.hpp"
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
using namespace rgbd_camera;
void check(bool value) { if (!value) throw std::runtime_error("odometry input assertion failed"); }
void rejects(const std::function<void()>& fn) {
  try { fn(); } catch (const std::exception&) { return; }
  throw std::runtime_error("expected rejection");
}
StereoFrame stereo() {
  StereoFrame p; p.sequence = {4,5}; p.event_ns = {1000000,1000100};
  p.midpoint_ns = {1001000,1001100}; p.exposure_ns = {2000,2000}; p.received_ns = 2000000;
  p.calibration_id = "synthetic";
  for (size_t i = 0; i < 2; ++i) { p.cameras[i] = {640,400,{500,0,320,0,500,200,0,0,1},std::vector<double>(5,0)}; p.pixels[i].resize(640*400); }
  return p;
}
int main() {
  Config config; config.fps = 30; config.width = 640; config.height = 360; config.publish_stereo = true;
  config.validate(); config.combine_imu = true; rejects([&] { config.validate(); });
  config.enable_imu = true; config.imu_rate_hz = 200; config.validate();
  config.backend = "realsense"; config.imu_rate_hz = 0; rejects([&] { config.validate(); });
  config.depth_width = 640; config.depth_height = 480; config.validate();
  InternalExtrinsics extrinsics; extrinsics.validate();
  extrinsics.rgb_from_camera[0][0] = -1; rejects([&] { extrinsics.validate(); });
  auto pair = stereo(); StereoGate gate;
  check(!gate.admit(pair,3000000,5000000)); rejects([&] { gate.admit(pair,3000000,5000000); });
  StereoGate stale; check(stale.admit(pair,10000000,5000000) == "stale_stereo");
  pair.midpoint_ns[1] += 1000001; pair.received_ns += 2000000;
  StereoGate skew; check(skew.admit(pair,5000000,5000000) == "stereo_skew");
  pair = stereo(); pair.pixels[0].pop_back(); StereoGate storage;
  rejects([&] { storage.admit(pair,3000000,5000000); });
  const auto a = [](int64_t t, double x) { return Motion{MotionKind::Accelerometer,uint64_t(t),t,t,{x,0,9.81}}; };
  const auto g = [](int64_t t) { return Motion{MotionKind::Gyroscope,uint64_t(t),t,t,{1,2,3}}; };
  ImuCombiner combine(10000);
  combine.push(a(1000,0)); combine.push(g(2500)); check(combine.drain(3000,100000).empty()); // Never extrapolate.
  combine.push(a(4000,6)); const auto result = combine.drain(5000,100000);
  check(result.size() == 1 && result[0].gyro.acquisition_ns == 2500 && result[0].acceleration[0] == 3 &&
        result[0].before_ns == 1000 && result[0].after_ns == 4000);
  rejects([&] { combine.push(a(3000,0)); });
  combine.push(g(7000)); combine.push(a(20000,0)); check(combine.drain(21000,100000).empty());
  check(combine.rejected() == 1); // Excessive bracket stays visible.
  ImuCombiner startup(10000); startup.push(g(1000)); startup.push(a(2000,0)); startup.push(a(3000,0));
  check(startup.drain(4000,100000).empty() && startup.rejected() == 1);
  ImuCombiner bounded(10000);
  for (int64_t i = 1; i <= 201; ++i) bounded.push(g(i));
  check(bounded.overflows() == 1 && bounded.drain(1000000,100).empty() && bounded.rejected() == 200);
  ImuCombiner new_generation(10000); new_generation.push(a(1,0)); // A new session starts independent history.
  std::cout << "stereo and combined IMU contracts passed (synthetic)\n";
}
