// Copyright (c) 2026 Paul N. Beuchat. MIT License.
// Pipeline/geometry design informed by UMelb's oakd_gpu_cone_detector (NOTICE.md).
#include "rgbd_camera/provider.hpp"
#include <depthai/depthai.hpp>
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>

namespace rgbd_camera {
namespace {
template<class T> int64_t nanos(T time) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}
template<class T> std::string label(T value) { std::ostringstream out; out << value; return out.str(); }
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

class DeviceLock {
 public:
  ~DeviceLock() { close(); }
  void acquire(const std::string& serial) {
    const auto directory = "/tmp/rgbd-camera-" + std::to_string(getuid());
    if (::mkdir(directory.c_str(), 0700) != 0 && errno != EEXIST)
      throw std::runtime_error("cannot create camera lock directory");
    struct stat info{};
    require(lstat(directory.c_str(), &info) == 0 && S_ISDIR(info.st_mode) &&
            info.st_uid == getuid() && (info.st_mode & 077) == 0, "unsafe camera lock directory");
    const auto path = directory + "/" + sha256(serial) + ".lock";
    fd_ = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    require(fd_ >= 0, "cannot open camera ownership lock");
    if (flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      close(); throw std::runtime_error("camera already owned by another rgbd_camera session");
    }
  }
  void close() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
 private:
  int fd_ = -1;
};

std::array<double, 9> intrinsics(dai::ImgFrame& image) {
  auto& transform = image.getTransformation();
  require(transform.getSize() == std::make_pair(size_t(640), size_t(640)),
          "unexpected SDK transformation size");
  const auto matrix = transform.getIntrinsicMatrix();
  std::array<double, 9> result{};
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col) result[3 * row + col] = matrix[row][col];
  return result;
}

std::vector<uint8_t> copy_image(dai::ImgFrame& image, unsigned int row_bytes) {
  require(image.getWidth() == 640 && image.getHeight() == 640, "unexpected SDK image size");
  const auto stride = image.getStride();
  const auto data = image.getData();
  require(stride >= row_bytes && data.size() >= size_t(stride) * 640,
          "unexpected SDK buffer stride/size");
  std::vector<uint8_t> output(size_t(row_bytes) * 640);
  for (size_t row = 0; row < 640; ++row)
    std::copy_n(data.begin() + row * stride, row_bytes, output.begin() + row * row_bytes);
  return output;
}

class OakProvider final : public Provider {
 public:
  ~OakProvider() override { try { stop(); } catch (...) {} }
  Identity start(const Config& config) override {
    require(!device_, "provider already started");
    config.validate();
    auto devices = dai::Device::getAllAvailableDevices();
    if (!config.device_id.empty()) {
      devices.erase(std::remove_if(devices.begin(), devices.end(), [&](const auto& info) {
        return info.getDeviceId() != config.device_id;
      }), devices.end());
    }
    require(devices.size() == 1, "select exactly one available OAK; use device_id for explicit selection");
    lock_.acquire(devices.front().getDeviceId());
    device_ = std::make_shared<dai::Device>(devices.front());
    device_->setMaxReconnectionAttempts(0);
    require(device_->getPlatform() == dai::Platform::RVC2, "this provider requires OAK RVC2");
    std::set<dai::CameraBoardSocket> sockets;
    for (const auto& feature : device_->getConnectedCameraFeatures()) sockets.insert(feature.socket);
    require(sockets.count(dai::CameraBoardSocket::CAM_A) && sockets.count(dai::CameraBoardSocket::CAM_B) &&
            sockets.count(dai::CameraBoardSocket::CAM_C), "CAM_A/B/C required");
    bool calibrated_stereo = false;
    for (const auto& pair : device_->getAvailableStereoPairs())
      if (pair.left == dai::CameraBoardSocket::CAM_B && pair.right == dai::CameraBoardSocket::CAM_C)
        calibrated_stereo = true;
    require(calibrated_stereo, "calibrated CAM_B/C stereo pair required");
    calibration_json_ = device_->readCalibration().eepromToJson().dump();
    Identity identity;
    identity.source_id = "oak:" + device_->getDeviceId();
    identity.generation = read_identifier("/proc/sys/kernel/random/uuid");
    identity.clock_domain = "linux_monotonic:" + read_identifier("/proc/sys/kernel/random/boot_id");
    identity.unix_offset_ns = nanos(std::chrono::system_clock::now()) - monotonic_ns();
    identity.usb_speed = label(device_->getUsbSpeed());
    identity.imu_sensor = device_->getConnectedIMU();
    if (config.enable_imu)
      require(!identity.imu_sensor.empty() && identity.imu_sensor != "NONE",
              "IMU requested but this camera reports no integrated IMU");

    pipeline_ = std::make_unique<dai::Pipeline>(device_);
    auto rgb_camera = pipeline_->create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto left = pipeline_->create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto right = pipeline_->create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto rgb = rgb_camera->requestOutput({640, 640}, dai::ImgFrame::Type::BGR888i,
                                        dai::ImgResizeMode::STRETCH, float(config.fps), true);
    auto stereo = pipeline_->create<dai::node::StereoDepth>();
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::DENSITY);
    stereo->setSubpixel(true);
    left->requestOutput({640, 400}, dai::ImgFrame::Type::RAW8, dai::ImgResizeMode::STRETCH,
                        float(config.fps))->link(stereo->left);
    right->requestOutput({640, 400}, dai::ImgFrame::Type::RAW8, dai::ImgResizeMode::STRETCH,
                         float(config.fps))->link(stereo->right);
    rgb->link(stereo->inputAlignTo);
    auto sync = pipeline_->create<dai::node::Sync>();
    sync->setSyncThreshold(std::chrono::nanoseconds(int64_t(config.pairing_tolerance_ms * 1e6)));
    // A finite attempt count exposes failed groups for host rejection/counters.
    // -1 would hide all progress when skew exceeds tolerance.
    sync->setSyncAttempts(1);
    sync->inputs["rgb"].setMaxSize(1); sync->inputs["rgb"].setBlocking(false);
    sync->inputs["depth"].setMaxSize(1); sync->inputs["depth"].setBlocking(false);
    rgb->link(sync->inputs["rgb"]); stereo->depth.link(sync->inputs["depth"]);
    frames_ = sync->out.createOutputQueue(1, false);
    if (config.enable_imu) {
      auto imu = pipeline_->create<dai::node::IMU>();
      imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 100);
      imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 100);
      imu->setBatchReportThreshold(1); imu->setMaxBatchReports(10);
      motion_ = imu->out.createOutputQueue(4, false);
    }
    pipeline_->start();
    return identity;
  }

  Products poll() override {
    require(device_ && !device_->isClosed() && pipeline_->isRunning(), "camera connection/pipeline unavailable");
    Products result;
    if (motion_) {
      if (auto data = motion_->tryGet<dai::IMUData>()) {
        require(data->packets.size() <= 10, "SDK exceeded IMU batch limit");
        const auto received = monotonic_ns();
        for (const auto& packet : data->packets) {
          const auto& a = packet.acceleroMeter;
          const auto& g = packet.gyroscope;
          // Initial SDK batches may not yet contain both requested reports.
          if (nanos(a.getTimestamp()) != 0)
            result.motion.push_back({MotionKind::Accelerometer, uint32_t(a.sequence), nanos(a.getTimestamp()),
                                     received, {a.x, a.y, a.z}});
          if (nanos(g.getTimestamp()) != 0)
            result.motion.push_back({MotionKind::Gyroscope, uint32_t(g.sequence), nanos(g.getTimestamp()),
                                     received, {g.x, g.y, g.z}});
        }
      }
    }
    if (auto group = frames_->tryGet<dai::MessageGroup>()) {
      const auto received = monotonic_ns();
      auto rgb = std::dynamic_pointer_cast<dai::ImgFrame>((*group)["rgb"]);
      auto depth = std::dynamic_pointer_cast<dai::ImgFrame>((*group)["depth"]);
      require(rgb && depth, "incomplete SDK frame group");
      require(rgb->getType() == dai::ImgFrame::Type::BGR888i && depth->getType() == dai::ImgFrame::Type::RAW16,
              "unexpected SDK image encoding");
      auto frame = std::make_unique<Frame>();
      require(rgb->getSequenceNum() >= 0 && depth->getSequenceNum() >= 0, "negative SDK image identity");
      frame->sequence = rgb->getSequenceNum(); frame->depth_sequence = depth->getSequenceNum();
      frame->acquisition_ns = nanos(rgb->getTimestamp());
      frame->depth_acquisition_ns = nanos(depth->getTimestamp()); frame->received_ns = received;
      frame->geometry.k = intrinsics(*rgb);
      const auto depth_k = intrinsics(*depth);
      for (size_t i = 0; i < 9; ++i)
        require(std::abs(frame->geometry.k[i] - depth_k[i]) < 1e-4, "RGB/depth virtual camera mismatch");
      std::ostringstream calibration;
      calibration << calibration_json_ << ":640x640:undistorted:aligned_rgb:optical_z:mm:" << std::setprecision(17);
      for (auto value : frame->geometry.k) calibration << value << ',';
      frame->geometry.calibration_id = sha256(calibration.str());
      frame->bgr = copy_image(*rgb, 640 * 3); frame->depth_le = copy_image(*depth, 640 * 2);
      result.frame = std::move(frame);
    }
    return result;
  }

  void stop() override {
    std::exception_ptr failure;
    if (pipeline_) {
      try { pipeline_->stop(); pipeline_->wait(); } catch (...) { failure = std::current_exception(); }
    }
    motion_.reset(); frames_.reset(); pipeline_.reset();
    if (device_) {
      try { device_->close(); } catch (...) { if (!failure) failure = std::current_exception(); }
      device_.reset();
    }
    lock_.close();
    if (failure) std::rethrow_exception(failure);
  }
 private:
  DeviceLock lock_;
  std::string calibration_json_;
  std::shared_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::MessageQueue> frames_, motion_;
};
}  // namespace

std::vector<DeviceDescriptor> discover_oak() {
  std::vector<DeviceDescriptor> result;
  for (const auto& info : dai::Device::getAllAvailableDevices())
    result.push_back({info.getDeviceId(), label(info.state), label(info.protocol)});
  return result;
}
std::unique_ptr<Provider> make_oak_provider() { return std::make_unique<OakProvider>(); }
}  // namespace rgbd_camera
