// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#include "rgbd_camera/provider.hpp"
#include <openssl/evp.h>
#include <algorithm>
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
  require(timestamp_policy == "strict" || timestamp_policy == "recover", "timestamp_policy must be strict or recover");
  require(backend == "oak" || backend == "realsense", "backend must be oak or realsense");
  require(between(fps, 1, 60), "fps must be finite and within [1, 60]");
  require((width == 0 && height == 0) ||
          (width == 640 && height == (backend == "oak" ? 640 : 480)) ||
          (width == 640 && height == 360 && fps <= 30) ||
          (width == 1280 && height == 720 && fps <= 30),
          "choose legacy dimensions, 640x360 or 1280x720 (wide profiles at <=30 Hz)");
  require((depth_width == 0 && depth_height == 0) ||
          (backend == "realsense" && depth_width == 640 && depth_height == 480 && fps <= 30),
          "explicit native depth is D455 640x480 at <=30 Hz");
  require(backend != "realsense" || output_height() != 360 || depth_width == 640,
          "D455 360p requires explicit 640x480 native depth");
  require(!publish_stereo || (fps == 30 && (backend == "oak" ||
          (native_depth_width() == 640 && native_depth_height() == 480))),
          "stereo requires 30 Hz and D455 native depth/IR 640x480");
  require(!combine_imu || (enable_imu && publish_stereo), "combined IMU requires IMU and stereo enabled");
  require(!combine_imu || backend != "oak" || imu_rate_hz == 200, "OAK combined IMU requires the explicit 200 Hz preset");
  require(imu_rate_hz == 0 || (enable_imu && backend == "oak" && (imu_rate_hz == 100 || imu_rate_hz == 200)),
          "explicit IMU rate is OAK 100 or 200 Hz; zero retains backend selection");
  require((mono_exposure_us == 0 && mono_iso == 0) ||
          (backend == "oak" && publish_stereo && mono_exposure_us >= 1 && mono_exposure_us <= 30000 &&
           mono_iso >= 100 && mono_iso <= 1600), "manual mono exposure requires OAK stereo, 1..30000 us and ISO 100..1600");
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
  require(std::regex_match(left_frame, frame_pattern) && std::regex_match(right_frame, frame_pattern) &&
          left_frame != right_frame && left_frame != optical_frame && right_frame != optical_frame &&
          left_frame != imu_frame && right_frame != imu_frame, "stereo frames must be valid and distinct");
}

void Geometry::validate() const {
  require((width == 640 && (height == 640 || height == 480 || height == 360)) || (width == 1280 && height == 720),
          "unsupported geometry dimensions");
  for (auto value : k) require(std::isfinite(value), "nonfinite intrinsics");
  require(k[0] > 0 && k[4] > 0 && k[2] >= 0 && k[2] < width && k[5] >= 0 && k[5] < height,
          "invalid focal length or principal point");
  require(std::abs(k[1]) < 1e-6 && std::abs(k[3]) < 1e-6 && std::abs(k[6]) < 1e-6 &&
          std::abs(k[7]) < 1e-6 && std::abs(k[8] - 1) < 1e-6, "unsupported projection matrix");
  require(!calibration_id.empty(), "missing calibration identity");
}

void CameraModel::validate() const {
  require(width == 640 && (height == 400 || height == 480), "unsupported stereo grid");
  for (double x : k) require(std::isfinite(x), "nonfinite stereo intrinsics");
  require(k[0] > 0 && k[4] > 0 && k[2] >= 0 && k[2] < width && k[5] >= 0 && k[5] < height &&
          std::abs(k[1]) < 1e-6 && std::abs(k[3]) < 1e-6 && std::abs(k[6]) < 1e-6 &&
          std::abs(k[7]) < 1e-6 && std::abs(k[8] - 1) < 1e-6, "invalid stereo intrinsics");
  require(d.size() == 5 || d.size() == 8, "unsupported stereo distortion model");
  for (double x : d) require(std::isfinite(x), "nonfinite stereo distortion");
}
void InternalExtrinsics::validate() const {
  const auto valid = [](const RigidTransform& m) {
    for (double x : m) require(std::isfinite(x), "nonfinite internal transform");
    require(m[12] == 0 && m[13] == 0 && m[14] == 0 && m[15] == 1, "invalid homogeneous transform");
    for (size_t r = 0; r < 3; ++r) for (size_t c = 0; c < 3; ++c) {
      double dot = 0;
      for (size_t k = 0; k < 3; ++k) dot += m[4*r+k] * m[4*c+k];
      require(std::abs(dot - (r == c ? 1.0 : 0.0)) < 1e-3, "non-orthonormal internal rotation");
    }
    const double det = m[0]*(m[5]*m[10]-m[6]*m[9]) - m[1]*(m[4]*m[10]-m[6]*m[8]) + m[2]*(m[4]*m[9]-m[5]*m[8]);
    require(std::abs(det - 1) < 1e-3, "internal rotation is not right handed");
  };
  for (const auto& m : rgb_from_camera) valid(m);
  if (has_imu) valid(rgb_from_imu);
}

std::optional<std::string> StereoGate::admit(const StereoFrame& pair, int64_t now, int64_t max_age) {
  require(!pair.calibration_id.empty(), "missing stereo calibration identity");
  require(calibration_.empty() || pair.calibration_id == calibration_, "stereo calibration changed within session");
  require(pair.received_ns > 0 && pair.received_ns <= now, "invalid stereo receipt clock");
  for (size_t i = 0; i < 2; ++i) {
    pair.cameras[i].validate();
    require(pair.pixels[i].size() == size_t(pair.cameras[i].width)*pair.cameras[i].height, "invalid stereo image buffer");
    require(pair.exposure_ns[i] >= 0 && pair.exposure_ns[i] <= 1000000000, "invalid stereo exposure duration");
    require(!last_sequence_[i] || pair.sequence[i] > *last_sequence_[i], "stereo sequence regressed or repeated");
    const bool clock_valid = pair.event_ns[i] > 0 && pair.event_ns[i] <= pair.received_ns && pair.midpoint_ns[i] > 0 &&
            pair.midpoint_ns[i] <= pair.received_ns && pair.exposure_ns[i] >= 0 &&
            pair.exposure_ns[i] <= 1000000000 && pair.midpoint_ns[i] > last_ns_[i];
    if (!clock_valid && recover_) { clock_rejected_ += 2; return "invalid_stereo_clock"; }
    require(clock_valid, "invalid stereo exposure timing or clock regression");
  }
  for (size_t i = 0; i < 2; ++i) {
    if (last_sequence_[i]) gaps_ += pair.sequence[i] - *last_sequence_[i] - 1;
    last_ns_[i] = pair.midpoint_ns[i]; last_sequence_[i] = pair.sequence[i];
  }
  calibration_ = pair.calibration_id;
  if (std::abs(pair.midpoint_ns[0] - pair.midpoint_ns[1]) > 1000000) return "stereo_skew";
  if (now - std::min(pair.midpoint_ns[0], pair.midpoint_ns[1]) > max_age) return "stale_stereo";
  return std::nullopt;
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
  require(frame.received_ns > 0 && frame.received_ns <= now_ns, "invalid host receipt clock");
  const bool clock_valid = frame.acquisition_ns > 0 && frame.depth_acquisition_ns > 0 &&
          frame.acquisition_ns <= frame.received_ns && frame.depth_acquisition_ns <= frame.received_ns &&
          frame.acquisition_ns > last_rgb_ns_ && frame.depth_acquisition_ns > last_depth_ns_;
  const auto exposure_valid = [&](int64_t midpoint, int64_t duration) {
    return midpoint >= 0 && midpoint <= frame.received_ns && duration >= 0 &&
        duration <= 1000000000 && (duration == 0 || midpoint > 0);
  };
  require(frame.exposure_duration_ns >= 0 && frame.exposure_duration_ns <= 1000000000 &&
          frame.depth_exposure_duration_ns >= 0 && frame.depth_exposure_duration_ns <= 1000000000,
          "invalid exposure duration");
  if (last_sequence_) {
    require(frame.sequence > *last_sequence_ && frame.depth_sequence > *last_depth_sequence_,
            "image sequence regressed or repeated");
  }
  const bool exposure_clock_valid = exposure_valid(frame.exposure_midpoint_ns, frame.exposure_duration_ns) &&
    exposure_valid(frame.depth_exposure_midpoint_ns, frame.depth_exposure_duration_ns);
  if ((!clock_valid || !exposure_clock_valid) && config_.timestamp_policy == "recover") {
    clock_rejected_ += 2; return "invalid_pair_clock";
  }
  require(clock_valid && exposure_clock_valid, "invalid acquisition/exposure clock mapping or regression");
  if (last_sequence_) gaps_ += frame.sequence - *last_sequence_ - 1;
  last_sequence_ = frame.sequence; last_depth_sequence_ = frame.depth_sequence;
  last_rgb_ns_ = frame.acquisition_ns; last_depth_ns_ = frame.depth_acquisition_ns;
  if (std::abs(frame.depth_acquisition_ns - frame.acquisition_ns) > config_.pairing_tolerance_ms * 1e6)
    return "pairing_skew";
  if (now_ns - std::min(frame.acquisition_ns, frame.depth_acquisition_ns) > config_.max_frame_age_ms * 1e6)
    return "stale_pair";
  return std::nullopt;
}

MotionGate::MotionGate(int64_t future_tolerance_ns, bool recover) : recover_(recover), future_tolerance_ns_(future_tolerance_ns) {
  require(future_tolerance_ns >= 0 && future_tolerance_ns <= 5000000, "invalid IMU future tolerance");
}
bool MotionGate::admit(const Motion& sample, int64_t now_ns, int64_t max_age_ns) {
  const auto index = sample.kind == MotionKind::Accelerometer ? 0 : 1;
  for (auto value : sample.xyz) require(std::isfinite(value), "nonfinite IMU sample");
  require(sample.received_ns > 0 && sample.received_ns <= now_ns, "invalid host receipt clock");
  if (recover_ && (sample.acquisition_ns <= 0 || sample.acquisition_ns-sample.received_ns > future_tolerance_ns_ ||
                   sample.acquisition_ns < last_ns_[index])) { ++clock_rejected_; return false; }
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
