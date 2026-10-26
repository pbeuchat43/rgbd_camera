// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#include "rgbd_camera/provider.hpp"
#include <openssl/evp.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace rgbd_camera {
namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
bool between(double value, double low, double high) {
  return std::isfinite(value) && value >= low && value <= high;
}
}

void Config::validate() const {
  require(backend == "oak" || backend == "realsense", "backend must be oak or realsense");
  require(between(fps, 1, 60), "fps must be finite and within [1, 60]");
  require(between(pairing_tolerance_ms, 0.1, 50) && pairing_tolerance_ms < 500.0 / fps,
          "pairing_tolerance_ms must be in [0.1, 50] and below half a frame period");
  require(between(max_frame_age_ms, 10, 2000), "max_frame_age_ms must be in [10, 2000]");
  require(between(startup_timeout_s, 5, 120), "startup_timeout_s must be in [5, 120]");
  require(between(stall_timeout_s, 1, 30) && stall_timeout_s > 1.0 / fps,
          "stall_timeout_s must be in [1, 30] and exceed a frame period");
  require(device_id.empty() || std::regex_match(device_id, std::regex("[A-Za-z0-9_-]{1,128}")),
          "device_id must be an SDK device identity, not a path");
  const std::regex frame_pattern("[A-Za-z][A-Za-z0-9_/]{0,127}");
  require(std::regex_match(optical_frame, frame_pattern), "invalid optical_frame");
  require(std::regex_match(imu_frame, frame_pattern) && imu_frame != optical_frame,
          "imu_frame must be valid and distinct from optical_frame");
}

void Geometry::validate() const {
  require(width == 640 && (height == 640 || height == 480), "unsupported geometry dimensions");
  for (auto value : k) require(std::isfinite(value), "nonfinite intrinsics");
  require(k[0] > 0 && k[4] > 0 && k[2] >= 0 && k[2] < width && k[5] >= 0 && k[5] < height,
          "invalid focal length or principal point");
  require(std::abs(k[1]) < 1e-6 && std::abs(k[3]) < 1e-6 && std::abs(k[6]) < 1e-6 &&
          std::abs(k[7]) < 1e-6 && std::abs(k[8] - 1) < 1e-6, "unsupported projection matrix");
  require(!calibration_id.empty(), "missing calibration identity");
}

int64_t monotonic_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::string read_identifier(const std::string& path) {
  std::ifstream input(path);
  std::string value;
  require(bool(std::getline(input, value)) && !value.empty(), "cannot read session/boot identity");
  return value;
}
std::string sha256(const std::string& bytes) {
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int size = 0;
  require(EVP_Digest(bytes.data(), bytes.size(), digest, &size, EVP_sha256(), nullptr) == 1,
          "cannot calculate calibration hash");
  std::ostringstream out;
  for (unsigned int i = 0; i < size; ++i) out << std::hex << std::setw(2) << std::setfill('0') << int(digest[i]);
  return out.str();
}

PairGate::PairGate(Config config) : config_(std::move(config)) { config_.validate(); }
std::optional<std::string> PairGate::admit(const Frame& frame, int64_t now_ns) {
  frame.geometry.validate();
  const size_t pixels = size_t(frame.geometry.width) * frame.geometry.height;
  require(frame.bgr.size() == pixels * 3 && frame.depth_le.size() == pixels * 2,
          "invalid image buffer size");
  if (geometry_) require(geometry_->width == frame.geometry.width && geometry_->height == frame.geometry.height &&
                        geometry_->k == frame.geometry.k &&
                        geometry_->calibration_id == frame.geometry.calibration_id,
                        "geometry changed; explicit new session required");
  else geometry_ = frame.geometry;
  require(frame.acquisition_ns > 0 && frame.depth_acquisition_ns > 0 &&
          frame.acquisition_ns <= frame.received_ns && frame.depth_acquisition_ns <= frame.received_ns &&
          frame.received_ns <= now_ns, "invalid acquisition clock mapping");
  require(frame.acquisition_ns > last_rgb_ns_ && frame.depth_acquisition_ns > last_depth_ns_,
          "acquisition clock regressed or repeated");
  if (last_sequence_) {
    require(frame.sequence > *last_sequence_ && frame.depth_sequence > *last_depth_sequence_,
            "image sequence regressed or repeated");
    gaps_ += frame.sequence - *last_sequence_ - 1;
  }
  last_sequence_ = frame.sequence; last_depth_sequence_ = frame.depth_sequence;
  last_rgb_ns_ = frame.acquisition_ns; last_depth_ns_ = frame.depth_acquisition_ns;
  if (std::abs(frame.depth_acquisition_ns - frame.acquisition_ns) > config_.pairing_tolerance_ms * 1e6)
    return "pairing_skew";
  if (now_ns - std::min(frame.acquisition_ns, frame.depth_acquisition_ns) > config_.max_frame_age_ms * 1e6)
    return "stale_pair";
  return std::nullopt;
}

MotionGate::MotionGate(int64_t future_tolerance_ns) : future_tolerance_ns_(future_tolerance_ns) {
  require(future_tolerance_ns >= 0 && future_tolerance_ns <= 5000000, "invalid IMU future tolerance");
}
bool MotionGate::admit(const Motion& sample, int64_t now_ns, int64_t max_age_ns) {
  const auto index = sample.kind == MotionKind::Accelerometer ? 0 : 1;
  for (auto value : sample.xyz) require(std::isfinite(value), "nonfinite IMU sample");
  if (!(sample.acquisition_ns > 0 && sample.received_ns > 0 &&
        sample.acquisition_ns - sample.received_ns <= future_tolerance_ns_ &&
        sample.received_ns <= now_ns)) {
    std::ostringstream detail;
    detail << "invalid IMU clock mapping: sensor=" << (index == 0 ? "accel" : "gyro")
           << " sequence=" << sample.sequence << " acquisition_ns=" << sample.acquisition_ns
           << " received_ns=" << sample.received_ns << " now_ns=" << now_ns
           << " future_tolerance_ns=" << future_tolerance_ns_
           << " previous_acquisition_ns=" << last_ns_[index];
    throw std::runtime_error(detail.str());
  }
  require(sample.acquisition_ns >= last_ns_[index], "IMU acquisition clock regressed");
  if (sample.acquisition_ns == last_ns_[index]) return false;
  if (last_sequence_[index] && sample.sequence > *last_sequence_[index])
    gaps_ += sample.sequence - *last_sequence_[index] - 1;
  last_ns_[index] = sample.acquisition_ns;
  last_sequence_[index] = sample.sequence;
  return now_ns - sample.acquisition_ns <= max_age_ns;
}
}  // namespace rgbd_camera
