// Copyright (c) 2026 Paul N. Beuchat. MIT License.
// Pipeline/geometry design informed by UMelb's oakd_gpu_cone_detector (NOTICE.md).
#include "rgbd_camera/provider.hpp"
#include "rgbd_camera/device_lock.hpp"
#include <depthai/depthai.hpp>
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

std::array<double, 9> intrinsics(dai::ImgFrame& image, int width, int height) {
  auto& transform = image.getTransformation();
  require(transform.getSize() == std::make_pair(size_t(width), size_t(height)),
          "unexpected SDK transformation size");
  const auto matrix = transform.getIntrinsicMatrix();
  std::array<double, 9> result{};
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col) result[3 * row + col] = matrix[row][col];
  return result;
}

std::vector<uint8_t> copy_image(dai::ImgFrame& image, int width, int height, unsigned int pixel_bytes) {
  require(image.getWidth() == uint32_t(width) && image.getHeight() == uint32_t(height), "unexpected SDK image size");
  const auto row_bytes = width * pixel_bytes;
  const auto stride = image.getStride();
  const auto data = image.getData();
  require(stride >= row_bytes && data.size() >= size_t(stride) * height,
          "unexpected SDK buffer stride/size");
  std::vector<uint8_t> output(size_t(row_bytes) * height);
  for (size_t row = 0; row < size_t(height); ++row)
    std::copy_n(data.begin() + row * stride, row_bytes, output.begin() + row * row_bytes);
  return output;
}

class OakProvider final : public Provider {
 public:
  ~OakProvider() override { try { stop(); } catch (...) {} }
  Identity start(const Config& config) override {
    require(!device_, "provider already started");
    config.validate();
    width_ = config.output_width(); height_ = config.output_height();
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
    const auto calibration = device_->readCalibration();
    calibration_json_ = calibration.eepromToJson().dump();
    Identity identity;
    if (config.publish_stereo) {
      auto extrinsics = std::make_shared<InternalExtrinsics>();
      const auto convert = [](const auto& matrix) {
        auto result = identity_transform();
        for (size_t r = 0; r < 3; ++r) for (size_t c = 0; c < 4; ++c)
          result[4*r+c] = matrix[r][c] * (c == 3 ? 0.01 : 1.0); // EEPROM centimetres -> metres.
        return result;
      };
      extrinsics->rgb_from_camera[0] = convert(calibration.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_A, false));
      extrinsics->rgb_from_camera[1] = convert(calibration.getCameraExtrinsics(dai::CameraBoardSocket::CAM_C, dai::CameraBoardSocket::CAM_A, false));
      extrinsics->has_imu = config.enable_imu;
      if (config.enable_imu) extrinsics->rgb_from_imu = convert(calibration.getImuToCameraExtrinsics(dai::CameraBoardSocket::CAM_A));
      extrinsics->validate(); identity.extrinsics = std::move(extrinsics);
    }
    identity.source_id = "oak:" + device_->getDeviceId();
    identity.generation = read_identifier("/proc/sys/kernel/random/uuid");
    identity.clock_domain = "linux_monotonic:" + read_identifier("/proc/sys/kernel/random/boot_id");
    identity.unix_offset_ns = nanos(std::chrono::system_clock::now()) - monotonic_ns();
    // SDK host synchronization is an estimate; fast IMU delivery can precede it.
    // Preserve acquisition time and declare a bounded admission allowance.
    identity.imu_future_tolerance_ns = 5000000;
    identity.usb_speed = label(device_->getUsbSpeed());
    identity.imu_sensor = device_->getConnectedIMU();
    if (config.enable_imu)
      require(!identity.imu_sensor.empty() && identity.imu_sensor != "NONE",
              "IMU requested but this camera reports no integrated IMU");

    pipeline_ = std::make_unique<dai::Pipeline>(device_);
    pipeline_->setXLinkChunkSize(0);
    auto rgb_camera = pipeline_->create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto left = pipeline_->create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto right = pipeline_->create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    if (config.mono_exposure_us) {
      left->initialControl.setManualExposure(config.mono_exposure_us, config.mono_iso);
      right->initialControl.setManualExposure(config.mono_exposure_us, config.mono_iso);
    }
    // Keep the legacy square projection unchanged. The footage profile uses an
    // aspect-preserving centre crop, with the delivered SDK intrinsics below.
    auto rgb = rgb_camera->requestOutput({uint32_t(width_), uint32_t(height_)}, dai::ImgFrame::Type::BGR888i,
        width_ == height_ ? dai::ImgResizeMode::STRETCH : dai::ImgResizeMode::CROP, float(config.fps), true);
    auto stereo = pipeline_->create<dai::node::StereoDepth>();
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::DENSITY);
    stereo->setSubpixel(true);
    auto left_out = left->requestOutput({640, 400}, dai::ImgFrame::Type::RAW8, dai::ImgResizeMode::STRETCH, float(config.fps));
    auto right_out = right->requestOutput({640, 400}, dai::ImgFrame::Type::RAW8, dai::ImgResizeMode::STRETCH, float(config.fps));
    left_out->link(stereo->left); right_out->link(stereo->right);
    if (config.publish_stereo) {
      auto pair = pipeline_->create<dai::node::Sync>();
      pair->setSyncThreshold(std::chrono::microseconds(100)); pair->setSyncAttempts(1);
      for (const auto* name : {"left", "right"}) { pair->inputs[name].setMaxSize(2); pair->inputs[name].setBlocking(false); }
      left_out->link(pair->inputs["left"]); right_out->link(pair->inputs["right"]);
      stereo_frames_ = pair->out.createOutputQueue(1, false);
    }
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
      const auto rate = config.imu_rate_hz ? config.imu_rate_hz : 100;
      imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, rate);
      imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, rate);
      imu->setBatchReportThreshold(1); imu->setMaxBatchReports(10);
      motion_ = imu->out.createOutputQueue(16, false);
    }
    pipeline_->start();
    return identity;
  }

  Products poll() override {
    require(device_ && !device_->isClosed() && pipeline_->isRunning(), "camera connection/pipeline unavailable");
    Products result;
    if (motion_) {
      const auto batch = motion_->tryGetAll<dai::IMUData>(); // One atomic bounded snapshot.
      require(batch.size() <= 16, "SDK exceeded IMU host queue bound");
      for (const auto& data : batch) {
        require(data->packets.size() <= 10, "SDK exceeded IMU batch limit");
        const auto received = monotonic_ns();
        for (const auto& packet : data->packets) {
          const auto& a = packet.acceleroMeter;
          const auto& g = packet.gyroscope;
          // Initial SDK batches may not yet contain both requested reports.
          if (nanos(a.getTimestamp()) != 0)
            result.motion.push_back({MotionKind::Accelerometer, uint32_t(a.sequence), nanos(a.getTimestamp()),
                                     received, {a.x, a.y, a.z}, {nanos(a.getTimestampDevice()), nanos(a.getTimestamp())}});
          if (nanos(g.getTimestamp()) != 0)
            result.motion.push_back({MotionKind::Gyroscope, uint32_t(g.sequence), nanos(g.getTimestamp()),
                                     received, {g.x, g.y, g.z}, {nanos(g.getTimestampDevice()), nanos(g.getTimestamp())}});
        }
      }
    }
    if (stereo_frames_) if (auto group = stereo_frames_->tryGet<dai::MessageGroup>()) {
      auto pair = std::make_unique<StereoFrame>(); pair->received_ns = monotonic_ns();
      std::ostringstream calibration;
      calibration << calibration_json_ << ":raw_stereo_640x400:" << std::setprecision(17);
      for (size_t i = 0; i < 2; ++i) {
        auto image = std::dynamic_pointer_cast<dai::ImgFrame>((*group)[i == 0 ? "left" : "right"]);
        require(image && image->getType() == dai::ImgFrame::Type::RAW8 && image->getSequenceNum() >= 0, "invalid OAK stereo image");
        pair->sequence[i] = image->getSequenceNum(); pair->event_ns[i] = nanos(image->getTimestamp());
        pair->clocks[i] = {nanos(image->getTimestampDevice()), pair->event_ns[i]};
        pair->midpoint_ns[i] = nanos(image->getTimestamp(dai::CameraExposureOffset::MIDDLE));
        pair->exposure_ns[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(image->getExposureTime()).count();
        auto& model = pair->cameras[i]; model.width = 640; model.height = 400;
        model.k = intrinsics(*image, 640, 400);
        const auto distortion = image->getTransformation().getDistortionCoefficients();
        require(distortion.size() == 5 || distortion.size() >= 8, "unsupported OAK stereo distortion");
        const size_t count = distortion.size() == 5 ? 5 : 8;
        for (size_t k = count; k < distortion.size(); ++k) require(std::abs(distortion[k]) < 1e-8, "unsupported higher-order stereo distortion");
        model.d.assign(distortion.begin(), distortion.begin() + count); model.validate();
        for (double x : model.k) calibration << x << ',';
        for (double x : model.d) calibration << x << ',';
        pair->pixels[i] = copy_image(*image, 640, 400, 1);
      }
      pair->calibration_id = sha256(calibration.str());
      result.stereo = std::move(pair);
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
      frame->exposure_midpoint_ns = nanos(rgb->getTimestamp(dai::CameraExposureOffset::MIDDLE));
      frame->exposure_duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(rgb->getExposureTime()).count();
      frame->depth_acquisition_ns = nanos(depth->getTimestamp()); frame->received_ns = received;
      frame->clocks = {{{nanos(rgb->getTimestampDevice()), frame->acquisition_ns},
                        {nanos(depth->getTimestampDevice()), frame->depth_acquisition_ns}}};
      frame->geometry.width = width_; frame->geometry.height = height_;
      frame->geometry.k = intrinsics(*rgb, width_, height_);
      const auto depth_k = intrinsics(*depth, width_, height_);
      for (size_t i = 0; i < 9; ++i)
        require(std::abs(frame->geometry.k[i] - depth_k[i]) < 1e-4, "RGB/depth virtual camera mismatch");
      std::ostringstream calibration;
      calibration << calibration_json_ << ':' << width_ << 'x' << height_
                  << ":undistorted:aligned_rgb:optical_z:mm:" << std::setprecision(17);
      for (auto value : frame->geometry.k) calibration << value << ',';
      frame->geometry.calibration_id = sha256(calibration.str());
      frame->bgr = copy_image(*rgb, width_, height_, 3); frame->depth_le = copy_image(*depth, width_, height_, 2);
      result.frame = std::move(frame);
    }
    return result;
  }

  void stop() override {
    std::exception_ptr failure;
    if (pipeline_) {
      try { pipeline_->stop(); pipeline_->wait(); } catch (...) { failure = std::current_exception(); }
    }
    motion_.reset(); frames_.reset(); stereo_frames_.reset(); pipeline_.reset();
    if (device_) {
      try { device_->close(); } catch (...) { if (!failure) failure = std::current_exception(); }
      device_.reset();
    }
    lock_.close();
    if (failure) std::rethrow_exception(failure);
  }
 private:
  DeviceLock lock_;
  int width_ = 640, height_ = 640;
  std::string calibration_json_;
  std::shared_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::MessageQueue> frames_, motion_, stereo_frames_;
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
