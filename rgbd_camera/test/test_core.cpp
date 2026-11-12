// Synthetic contract checks. No camera, ROS participant, or hardware evidence.
#include "rgbd_camera/provider.hpp"
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace rgbd_camera;
void check(bool value) { if (!value) throw std::runtime_error("assertion failed"); }
void rejects(const std::function<void()>& action) {
  try { action(); } catch (const std::runtime_error&) { return; }
  throw std::runtime_error("expected rejection");
}
Frame frame(uint64_t sequence = 1, int64_t stamp = 1000000000) {
  Frame f;
  f.sequence = sequence; f.depth_sequence = sequence + 7; // IDs need not be equal
  f.acquisition_ns = stamp; f.depth_acquisition_ns = stamp + 1000000;
  f.received_ns = stamp + 10000000;
  f.geometry.k = {500, 0, 320, 0, 500, 320, 0, 0, 1};
  f.geometry.calibration_id = "calibration-a";
  f.bgr.resize(640 * 640 * 3); f.depth_le.resize(640 * 640 * 2);
  return f;
}
int main() {
  Config config;
  config.validate();
  check(config.output_width() == 640 && config.output_height() == 640);
  Config footage; footage.width = 1280; footage.height = 720; footage.fps = 30; footage.validate();
  footage.backend = "realsense"; footage.validate();
  footage.fps = 31; rejects([&] { footage.validate(); });
  footage.fps = 30; footage.height = 0; rejects([&] { footage.validate(); });
  footage.width = 0; footage.validate();
  check(footage.output_width() == 640 && footage.output_height() == 480);
  auto wide = frame(); wide.geometry.width = 1280; wide.geometry.height = 720;
  wide.bgr.resize(1280*720*3); wide.depth_le.resize(1280*720*2);
  PairGate wide_gate(config); check(!wide_gate.admit(wide, wide.received_ns));
  Config high_rate; high_rate.fps = 60; high_rate.validate();
  high_rate.backend = "realsense"; high_rate.validate();
  high_rate.pairing_tolerance_ms = 15;
  rejects([&] { high_rate.validate(); }); // greater than half the output period
  for (double rate : {0.0, 60.001, std::numeric_limits<double>::quiet_NaN()}) {
    Config invalid; invalid.fps = rate;
    rejects([&] { invalid.validate(); });
  }
  for (double tolerance : {0.0, 50.0, std::numeric_limits<double>::quiet_NaN()}) {
    Config invalid; invalid.pairing_tolerance_ms = tolerance;
    rejects([&] { invalid.validate(); });
  }
  PairGate gate(config);
  auto first = frame(); check(!gate.admit(first, first.received_ns));
  auto next = frame(3, 1100000000); check(!gate.admit(next, next.received_ns));
  check(gate.gaps() == 1);
  rejects([&] { gate.admit(next, next.received_ns); });
  auto changed = frame(4, 1200000000); changed.geometry.calibration_id = "b";
  rejects([&] { gate.admit(changed, changed.received_ns); });
  PairGate skew(config);
  auto jitter = frame(); jitter.depth_acquisition_ns += 5000000;
  check(skew.admit(jitter, jitter.received_ns) == "pairing_skew");
  auto recovered = frame(2, 1100000000); check(!skew.admit(recovered, recovered.received_ns));
  Config debug; debug.pairing_tolerance_ms = 10;
  PairGate relaxed(debug); check(!relaxed.admit(jitter, jitter.received_ns));
  PairGate stale(config); check(stale.admit(first, first.received_ns + 600000000) == "stale_pair");
  PairGate bad(config); auto malformed = frame(); malformed.bgr.pop_back();
  rejects([&] { bad.admit(malformed, malformed.received_ns); });
  malformed = frame(); malformed.geometry.k[0] = 0;
  rejects([&] { bad.admit(malformed, malformed.received_ns); });
  malformed = frame(); malformed.acquisition_ns = malformed.received_ns + 1;
  rejects([&] { bad.admit(malformed, malformed.received_ns); });
  PairGate restart(config); check(!restart.admit(first, first.received_ns));
  auto timed = frame(); timed.exposure_midpoint_ns = timed.acquisition_ns + 2000000;
  timed.exposure_duration_ns = 4000000;
  timed.depth_exposure_midpoint_ns = timed.depth_acquisition_ns;
  timed.depth_exposure_duration_ns = 1000000;
  PairGate exposure(config); check(!exposure.admit(timed, timed.received_ns));
  for (int variant = 0; variant < 3; ++variant) {
    auto invalid = timed;
    if (variant == 0) invalid.exposure_midpoint_ns = invalid.received_ns + 1;
    if (variant == 1) invalid.exposure_duration_ns = -1;
    if (variant == 2) invalid.depth_exposure_midpoint_ns = 0;
    PairGate invalid_gate(config); rejects([&] { invalid_gate.admit(invalid, invalid.received_ns); });
  }
  MotionGate motion;
  Motion a{MotionKind::Accelerometer, 1, 1000, 1100, {0, 0, 9.81}};
  check(motion.admit(a, 1100, 1000)); check(!motion.admit(a, 1100, 1000));
  a.acquisition_ns = 999; rejects([&] { motion.admit(a, 1100, 1000); });
  a.kind = MotionKind::Gyroscope; a.xyz = {0, 0, 0};
  check(motion.admit(a, 1100, 1000));
  a.acquisition_ns = 1200; a.received_ns = 1300; a.sequence = 3;
  check(!motion.admit(a, 3000, 1000)); check(motion.gaps() == 1);
  Motion future{MotionKind::Accelerometer, 1, 1000600000, 1000000000, {0, 0, 9.81}};
  MotionGate strict;
  rejects([&] { strict.admit(future, future.received_ns, 500000000); });
  MotionGate estimated(5000000);
  check(estimated.admit(future, future.received_ns, 500000000));
  check(future.acquisition_ns == 1000600000); // acceptance never rewrites time
  future.sequence = 2; future.acquisition_ns = future.received_ns + 5000000;
  check(estimated.admit(future, future.received_ns, 500000000));
  future.sequence = 3; ++future.acquisition_ns;
  rejects([&] { estimated.admit(future, future.received_ns, 500000000); });
  future.acquisition_ns = 1000000001;
  rejects([&] { estimated.admit(future, future.received_ns, 500000000); });
  rejects([] { MotionGate invalid(-1); });
  rejects([] { MotionGate invalid(5000001); });
  check(sha256("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  std::cout << "camera contract checks passed (synthetic)\n";
}
