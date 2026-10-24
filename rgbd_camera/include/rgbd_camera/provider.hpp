// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace rgbd_camera {

struct Config {
  std::string backend = "oak";
  std::string device_id;
  double fps = 10.0;
  double pairing_tolerance_ms = 5.0;
  double max_frame_age_ms = 500.0;
  double startup_timeout_s = 30.0;
  double stall_timeout_s = 2.0;
  bool enable_imu = false;
  bool publish_images = false;
  std::string optical_frame = "oak_rgb_optical_frame";
  std::string imu_frame = "oak_imu_native_frame";
  void validate() const;
};

struct Geometry {
  uint32_t width = 640, height = 640;
  std::array<double, 9> k{};
  // v1 supports aligned depth in a rectified virtual RGB camera only.
  std::string calibration_id;
  void validate() const;
};

struct Identity {
  std::string source_id, generation, clock_domain;
  std::string backend = "depthai-3.9.0";
  std::string usb_speed, imu_sensor;
  std::string imu_axes = "sensor_native";
  int64_t unix_offset_ns = 0;
  // Admission policy, not a measured clock-error bound. Images remain strict.
  int64_t imu_future_tolerance_ns = 0;
};

struct Frame {
  uint64_t sequence = 0, depth_sequence = 0;
  int64_t acquisition_ns = 0, depth_acquisition_ns = 0, received_ns = 0;
  Geometry geometry;
  // Owned buffers: SDK storage is never retained/reused underneath a consumer.
  std::vector<uint8_t> bgr, depth_le;
};

enum class MotionKind { Accelerometer, Gyroscope };
struct Motion {
  MotionKind kind;
  uint64_t sequence;
  int64_t acquisition_ns, received_ns;
  std::array<double, 3> xyz;
};

struct Products {
  std::unique_ptr<Frame> frame;
  std::vector<Motion> motion;  // at most 20 samples per poll
  std::string diagnostic;  // bounded provider admission note, exposed in status
};

class Provider {
 public:
  virtual ~Provider() = default;
  virtual Identity start(const Config& config) = 0;
  // Nonblocking, bounded: one latest image pair and a small IMU batch.
  virtual Products poll() = 0;
  virtual void stop() = 0;
};

struct DeviceDescriptor { std::string id, state, protocol; };
std::vector<DeviceDescriptor> discover_oak();  // never opens a device
std::unique_ptr<Provider> make_oak_provider();
std::vector<DeviceDescriptor> discover_realsense();  // probes controls, never starts capture
std::unique_ptr<Provider> make_realsense_provider();
std::vector<DeviceDescriptor> discover(const std::string& backend);
std::unique_ptr<Provider> make_provider(const std::string& backend);

int64_t monotonic_ns();
std::string read_identifier(const std::string& path);
std::string sha256(const std::string& bytes);

// Throws on contract/clock faults, returns a reason for recoverable pair rejection.
class PairGate {
 public:
  explicit PairGate(Config config);
  std::optional<std::string> admit(const Frame& frame, int64_t now_ns);
  uint64_t gaps() const { return gaps_; }
 private:
  Config config_;
  std::optional<uint64_t> last_sequence_, last_depth_sequence_;
  int64_t last_rgb_ns_ = 0, last_depth_ns_ = 0;
  std::optional<Geometry> geometry_;
  uint64_t gaps_ = 0;
};

// Deduplicates repeated IMU reports; each sensor maintains independent progress.
class MotionGate {
 public:
  explicit MotionGate(int64_t future_tolerance_ns = 0);
  bool admit(const Motion& sample, int64_t now_ns, int64_t max_age_ns);
  uint64_t gaps() const { return gaps_; }
 private:
  int64_t future_tolerance_ns_;
  std::array<int64_t, 2> last_ns_{};
  std::array<std::optional<uint64_t>, 2> last_sequence_;
  uint64_t gaps_ = 0;
};
}  // namespace rgbd_camera
