// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#include "rgbd_camera/provider.hpp"
#include "rgbd_camera/device_lock.hpp"
#include "rgbd_camera/realsense_geometry.hpp"
#include <librealsense2/rs.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>

namespace rgbd_camera {
namespace {
void require(bool condition, const char* reason) { if (!condition) throw std::runtime_error(reason); }
int64_t epoch_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
std::string info(const rs2::device& device, rs2_camera_info field) {
  return device.supports(field) ? device.get_info(field) : "unknown";
}
struct VideoSample {
  rs2::video_frame frame;
  int64_t acquired = 0, received = 0;
  ClockReading clock;
};
struct SensorSession {
  rs2::sensor sensor;
  std::vector<rs2::stream_profile> profiles;
  bool opened = false, started = false, motion = false;
};
struct Selection { size_t sensor; rs2::stream_profile profile; };

class RealsenseProvider final : public Provider {
 public:
  ~RealsenseProvider() override { try { stop(); } catch (...) {} }
  Identity start(const Config& config) override {
    require(!context_, "provider already started"); config.validate(); config_ = config;
    identity_ = Identity{};
    native_fps_ = config.fps > 30 ? 60 : 30;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      error_.clear(); last_acquired_.fill(0); future_motion_time_ = 0;
      last_sequence_.fill(std::nullopt); next_output_ns_ = 0;
      raw_previous_.fill(std::nullopt); raw_unwrapped_.fill(0);
      clock_samples_valid_.fill(false); clock_ready_ = false; stable_since_ns_ = 0;
      received_samples_.fill(0); last_received_.fill(0); domains_.fill(-1); invalid_clocks_.fill(0);
      colors_.clear(); depths_.clear(); motion_.clear(); infrared_ = {}; active_.fill(false);
    }
    rs2_error* version_error = nullptr;
    const int version = rs2_get_api_version(&version_error);
    if (version_error) {
      const std::string error = rs2_get_error_message(version_error); rs2_free_error(version_error);
      throw std::runtime_error(error);
    }
    require(version == RS2_API_VERSION, "librealsense headers and loaded SDK differ");
    context_ = std::make_unique<rs2::context>();
    std::vector<rs2::device> candidates;
    for (auto device : context_->query_devices())
      if (config.device_id.empty() || info(device,RS2_CAMERA_INFO_SERIAL_NUMBER) == config.device_id)
        candidates.push_back(device);
    require(candidates.size() == 1, "select exactly one RealSense device; use device_id for explicit selection");
    device_ = candidates.front();
    require(info(device_,RS2_CAMERA_INFO_PRODUCT_ID) == "0B5C", "this RealSense profile requires D455 (PID 0B5C)");
    require(info(device_,RS2_CAMERA_INFO_FIRMWARE_VERSION) == "5.17.0.10",
            "this D455 profile requires firmware 5.17.0.10 with librealsense 2.56.5; firmware changes are explicit");
    const auto serial = info(device_,RS2_CAMERA_INFO_SERIAL_NUMBER);
    lock_.acquire("realsense:" + serial);
    const auto sensors = device_.query_sensors();
    std::array<std::optional<Selection>,6> selected;
    for (size_t index = 0; index < sensors.size(); ++index) {
      for (auto profile : sensors[index].get_stream_profiles()) {
        const auto stream = profile.stream_type();
        if (auto video = profile.as<rs2::video_stream_profile>()) {
          if (profile.fps() == native_fps_) {
            if (stream == RS2_STREAM_COLOR && profile.format() == RS2_FORMAT_BGR8 &&
                video.width() == config.output_width() && video.height() == config.output_height()) selected[0] = Selection{index,profile};
            if (stream == RS2_STREAM_DEPTH && profile.format() == RS2_FORMAT_Z16 &&
                video.width() == config.native_depth_width() && video.height() == config.native_depth_height()) selected[1] = Selection{index,profile};
            if (config.publish_stereo && stream == RS2_STREAM_INFRARED && profile.format() == RS2_FORMAT_Y8 &&
                video.width() == 640 && video.height() == 480 && (profile.stream_index() == 1 || profile.stream_index() == 2))
              selected[3+profile.stream_index()] = Selection{index,profile};
          }
        } else if (config.enable_imu && profile.format() == RS2_FORMAT_MOTION_XYZ32F && profile.fps() > 0 && profile.fps() <= 400) {
          const int slot = stream == RS2_STREAM_ACCEL ? 2 : stream == RS2_STREAM_GYRO ? 3 : -1;
          if (slot >= 0 && (!selected[slot] || profile.fps() < selected[slot]->profile.fps()))
            selected[slot] = Selection{index,profile};
        }
      }
    }
    require(selected[0] && selected[1], "D455 requires supported BGR8/Z16 profiles at the requested dimensions and native rate");
    require(!config.enable_imu || (selected[2] && selected[3]), "requested RealSense accel/gyro streams unavailable");
    require(!config.publish_stereo || (selected[4] && selected[5]), "D455 concurrent IR 640x480 profiles unavailable");
    for (size_t i = 0; i < selected.size(); ++i) active_[i] = bool(selected[i]);
    RealsenseCalibration calibration;
    const auto color_profile = selected[0]->profile.as<rs2::video_stream_profile>();
    const auto depth_profile = selected[1]->profile.as<rs2::video_stream_profile>();
    calibration.color = color_profile.get_intrinsics(); calibration.depth = depth_profile.get_intrinsics();
    calibration.depth_to_color = depth_profile.get_extrinsics_to(color_profile);
    calibration.depth_unit_m = sensors[selected[1]->sensor].as<rs2::depth_sensor>().get_depth_scale();
    calibration.profile_identity = serial + ":" + info(device_,RS2_CAMERA_INFO_FIRMWARE_VERSION) +
      ":" + std::to_string(config.output_width()) + "x" + std::to_string(config.output_height()) +
      ":depth_" + std::to_string(config.native_depth_width()) + "x" + std::to_string(config.native_depth_height()) +
      "@" + std::to_string(native_fps_) + ":bgr8:z16";
    geometry_ = std::make_unique<RealsenseGeometry>(calibration);
    if (config.publish_stereo) {
      auto extrinsics = std::make_shared<InternalExtrinsics>();
      const auto convert = [](const rs2_extrinsics& e) {
        auto m = identity_transform();
        for (size_t r = 0; r < 3; ++r) {
          for (size_t c = 0; c < 3; ++c) m[4*r+c] = e.rotation[3*c+r];
          m[4*r+3] = e.translation[r];
        }
        return m;
      };
      for (size_t i = 0; i < 2; ++i) {
        const auto profile = selected[4+i]->profile.as<rs2::video_stream_profile>();
        const auto in = profile.get_intrinsics();
        auto& model = stereo_models_[i]; model.width = in.width; model.height = in.height;
        model.k = {in.fx,0,in.ppx,0,in.fy,in.ppy,0,0,1}; model.d.assign(in.coeffs, in.coeffs+5);
        require(in.model == RS2_DISTORTION_NONE || in.model == RS2_DISTORTION_BROWN_CONRADY, "unsupported IR distortion convention");
        model.validate();
        for (double x : model.d) require(std::abs(x) < 1e-8, "D455 rectified IR requires zero distortion");
        extrinsics->rgb_from_camera[i] = convert(profile.get_extrinsics_to(color_profile));
      }
      for (size_t i = 0; i < 9; ++i)
        require(std::abs(stereo_models_[0].k[i] - stereo_models_[1].k[i]) < 1e-5, "D455 IR intrinsics are not rectified together");
      const auto left_from_right = convert(selected[5]->profile.get_extrinsics_to(selected[4]->profile));
      const auto unit = identity_transform();
      for (size_t r = 0; r < 3; ++r) for (size_t c = 0; c < 3; ++c)
        require(std::abs(left_from_right[4*r+c] - unit[4*r+c]) < 1e-5, "D455 IR relative rotation is not rectified");
      require(left_from_right[3] > 0.001 && left_from_right[3] < 1 &&
              std::abs(left_from_right[7]) < 1e-5 && std::abs(left_from_right[11]) < 1e-5,
              "D455 IR baseline is not horizontal left-to-right");
      extrinsics->rectified = true; extrinsics->has_imu = config.enable_imu;
      if (config.enable_imu) extrinsics->rgb_from_imu = convert(selected[2]->profile.get_extrinsics_to(color_profile));
      extrinsics->validate(); identity_.extrinsics = extrinsics;
      std::ostringstream fingerprint; fingerprint << geometry_->geometry().calibration_id << ":rectified_ir:" << std::setprecision(17);
      for (const auto& model : stereo_models_) { for (double x : model.k) fingerprint << x << ','; for (double x : model.d) fingerprint << x << ','; }
      for (const auto& m : extrinsics->rgb_from_camera) for (double x : m) fingerprint << x << ',';
      if (extrinsics->has_imu) for (double x : extrinsics->rgb_from_imu) fingerprint << x << ',';
      stereo_calibration_ = sha256(fingerprint.str());
    }
    identity_.source_id = "realsense:" + serial;
    identity_.generation = read_identifier("/proc/sys/kernel/random/uuid");
    identity_.clock_domain = "linux_monotonic:" + read_identifier("/proc/sys/kernel/random/boot_id");
    identity_.backend = std::string("librealsense-") + RS2_API_VERSION_STR + ":D455:firmware-5.17.0.10:native_" +
      std::to_string(native_fps_) + "hz:exposure_midpoint_global_time";
    identity_.usb_speed = info(device_,RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
    identity_.unix_offset_ns = epoch_ns()-monotonic_ns();
    identity_.imu_future_tolerance_ns = 5000000;
    identity_.imu_axes = "sdk_depth_oriented_at_imu_origin";
    if (config.enable_imu) {
      const auto& sensor = sensors[selected[2]->sensor];
      identity_.imu_sensor = sensor.supports(RS2_CAMERA_INFO_NAME) ? sensor.get_info(RS2_CAMERA_INFO_NAME) : "RealSense motion";
    }
    for (size_t index = 0; index < sensors.size(); ++index) {
      SensorSession session{sensors[index],{}};
      for (size_t slot = 0; slot < selected.size(); ++slot)
        if (selected[slot] && selected[slot]->sensor == index) {
          session.profiles.push_back(selected[slot]->profile); session.motion |= slot == 2 || slot == 3;
        }
      if (!session.profiles.empty()) sessions_.push_back(std::move(session));
    }
    stopping_ = false;
    for (auto& session : sessions_) {
      require(session.sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED), "RealSense sensor lacks global-time mapping");
      session.sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED,1.0F);
      if (session.sensor.supports(RS2_OPTION_FRAMES_QUEUE_SIZE))
        session.sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE,session.motion ? 16.0F : 4.0F);
      session.sensor.open(session.profiles); session.opened = true;
      session.sensor.start([this](rs2::frame frame) { receive(std::move(frame)); }); session.started = true;
    }
    return identity_;
  }

  Products poll() override {
    require(bool(context_), "RealSense provider not started");
    require(std::abs((epoch_ns()-monotonic_ns())-identity_.unix_offset_ns) <= 5000000,
            "host epoch-to-monotonic offset changed by more than 5 ms; restart required");
    Products products;
    std::optional<VideoSample> color, depth, left, right;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (!error_.empty()) throw std::runtime_error(error_);
      if (future_motion_time_)
        products.diagnostic = "realsense_imu_small_future_estimates=" + std::to_string(future_motion_time_);
      if (!clock_ready_) {
        products.diagnostic = "realsense_clock_starting stable_ms=" +
          std::to_string(stable_since_ns_ ? (monotonic_ns()-stable_since_ns_)/1000000 : 0);
        for (size_t slot = 0; slot < active_.size(); ++slot) if (active_[slot])
          products.diagnostic += " stream" + std::to_string(slot) + "{samples=" + std::to_string(received_samples_[slot]) +
            ",domain=" + std::to_string(domains_[slot]) + ",invalid=" + std::to_string(invalid_clocks_[slot]) +
            ",age_us=" + std::to_string((last_received_[slot]-last_acquired_[slot])/1000) + "}";
      }
      products.motion.assign(motion_.begin(),motion_.end()); motion_.clear();
      while (!infrared_[0].empty() && !infrared_[1].empty()) {
        const auto skew = infrared_[1].front().acquired - infrared_[0].front().acquired;
        if (skew > 1000000) { infrared_[0].pop_front(); continue; }
        if (skew < -1000000) { infrared_[1].pop_front(); continue; }
        left = std::move(infrared_[0].front()); infrared_[0].pop_front();
        right = std::move(infrared_[1].front()); infrared_[1].pop_front(); break;
      }
      while (!colors_.empty() && !depths_.empty()) {
        const auto skew = depths_.front().acquired-colors_.front().acquired;
        // Select the nearest native-rate pair without reusing a frame. Expose pairs
        // above the user tolerance to PairGate, so rejection remains observable.
        const int64_t half_period_ns = (500000000 + native_fps_ - 1) / native_fps_;
        if (skew > half_period_ns) { colors_.pop_front(); continue; }
        if (skew < -half_period_ns) { depths_.pop_front(); continue; }
        color = std::move(colors_.front()); colors_.pop_front();
        depth = std::move(depths_.front()); depths_.pop_front();
        if (color->acquired < next_output_ns_) { color.reset(); depth.reset(); continue; }
        const auto period = int64_t(std::llround(1e9/config_.fps));
        if (!next_output_ns_) next_output_ns_ = color->acquired;
        next_output_ns_ += ((color->acquired-next_output_ns_)/period+1)*period;
        break;
      }
    }
    if (left && right) {
      auto pair = std::make_unique<StereoFrame>(); pair->cameras = stereo_models_;
      pair->rectified = true; pair->calibration_id = stereo_calibration_;
      pair->received_ns = std::max(left->received, right->received);
      const VideoSample* samples[] = {&*left, &*right};
      for (size_t i = 0; i < 2; ++i) {
        const auto& s = *samples[i]; pair->sequence[i] = s.frame.get_frame_number(); pair->midpoint_ns[i] = s.acquired;
        pair->clocks[i] = s.clock;
        pair->event_ns[i] = s.acquired; // Preserve the provider's D455 midpoint convention.
        if (s.frame.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE))
          pair->exposure_ns[i] = s.frame.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)*1000;
        const auto stride = s.frame.get_stride_in_bytes();
        require(stride >= 640 && s.frame.get_data_size() >= size_t(stride)*480, "invalid IR storage");
        const auto* data = static_cast<const uint8_t*>(s.frame.get_data());
        pair->pixels[i].resize(640*480);
        for (size_t row = 0; row < 480; ++row) std::copy_n(data+row*stride, 640, pair->pixels[i].begin()+row*640);
      }
      products.stereo = std::move(pair);
    }
    if (color && depth) {
      auto output = std::make_unique<Frame>();
      output->sequence = color->frame.get_frame_number(); output->depth_sequence = depth->frame.get_frame_number();
      output->acquisition_ns = color->acquired; output->depth_acquisition_ns = depth->acquired;
      output->clocks = {color->clock, depth->clock};
      output->exposure_midpoint_ns = color->acquired;
      output->depth_exposure_midpoint_ns = depth->acquired;
      if (color->frame.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE))
        output->exposure_duration_ns = color->frame.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE) * 1000;
      if (depth->frame.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE))
        output->depth_exposure_duration_ns = depth->frame.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE) * 1000;
      output->received_ns = std::max(color->received,depth->received);
      output->geometry = geometry_->geometry();
      output->bgr = geometry_->rectify_bgr(static_cast<const uint8_t*>(color->frame.get_data()),
                                         color->frame.get_data_size(),color->frame.get_stride_in_bytes());
      output->depth_le = geometry_->align_depth(static_cast<const uint8_t*>(depth->frame.get_data()),
                                               depth->frame.get_data_size(),depth->frame.get_stride_in_bytes());
      products.frame = std::move(output);
    }
    return products;
  }
  void stop() override {
    { std::lock_guard<std::mutex> guard(mutex_); stopping_ = true; }
    std::exception_ptr failure;
    for (auto it = sessions_.rbegin(); it != sessions_.rend(); ++it) {
      if (it->started) { try { it->sensor.stop(); } catch (...) { if (!failure) failure = std::current_exception(); } }
      if (it->opened) { try { it->sensor.close(); } catch (...) { if (!failure) failure = std::current_exception(); } }
    }
    sessions_.clear();
    { std::lock_guard<std::mutex> guard(mutex_); colors_.clear(); depths_.clear(); motion_.clear(); infrared_ = {}; }
    geometry_.reset(); device_ = rs2::device(); context_.reset(); lock_.close();
    if (failure) std::rethrow_exception(failure);
  }
 private:
  void receive(rs2::frame frame) noexcept {
    try {
      const auto received = monotonic_ns();
      std::lock_guard<std::mutex> guard(mutex_);
      if (stopping_ || !error_.empty()) return;
      const auto stream = frame.get_profile().stream_type();
      const int slot = stream == RS2_STREAM_COLOR ? 0 : stream == RS2_STREAM_DEPTH ? 1 :
                       stream == RS2_STREAM_ACCEL ? 2 : stream == RS2_STREAM_GYRO ? 3 :
                       stream == RS2_STREAM_INFRARED && (frame.get_profile().stream_index() == 1 || frame.get_profile().stream_index() == 2)
                         ? 3 + frame.get_profile().stream_index() : -1;
      require(slot >= 0 && active_[slot], "unexpected RealSense stream");
      const bool is_motion = slot == 2 || slot == 3;
      ++received_samples_[slot]; last_received_[slot] = received;
      domains_[slot] = frame.get_frame_timestamp_domain();
      if (domains_[slot] != RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME) {
        require(!clock_ready_, "RealSense timestamp domain changed during capture");
        clock_samples_valid_[slot] = false; stable_since_ns_ = 0;
        return;
      }
      int64_t acquired;
      ClockReading clock;
      if (!is_motion) {
        require(frame.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP) &&
                frame.supports_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP), "RealSense exposure timestamp metadata unavailable");
        const auto raw = uint32_t(frame.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP));
        if (raw_previous_[slot]) {
          const uint32_t delta = raw-*raw_previous_[slot];
          require(delta > 0 && delta < 0x80000000U, "RealSense raw device clock regressed or repeated");
          raw_unwrapped_[slot] += int64_t(delta)*1000;
        } else raw_unwrapped_[slot] = int64_t(raw)*1000;
        raw_previous_[slot] = raw;
        clock = {raw_unwrapped_[slot], realsense_motion_time_ns(frame.get_timestamp(),identity_.unix_offset_ns)};
        acquired = realsense_video_time_ns(frame.get_timestamp(),identity_.unix_offset_ns, raw,
          uint32_t(frame.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP)));
      } else acquired = realsense_motion_time_ns(frame.get_timestamp(),identity_.unix_offset_ns);
      const auto sequence = frame.get_frame_number();
      // SDK frame numbering can restart while metadata/counter sources settle.
      // The unpublished startup gate already restarts its two-second window on
      // non-increasing samples. Once streaming, a reset remains terminal.
      require(!clock_ready_ || !last_sequence_[slot] || sequence > *last_sequence_[slot],
              "RealSense raw sequence regressed or repeated");
      // An individual callback may arrive slightly before the SDK clock estimate.
      // Video startup stays strict; after startup allow the same small mapping
      // error as IMU. PairGate still requires both image timestamps to precede
      // the complete bundle's receipt (the later of its two image receipts).
      const auto future_tolerance = is_motion || clock_ready_ ? identity_.imu_future_tolerance_ns : 0;
      const bool increasing = acquired > last_acquired_[slot] &&
                              (!last_sequence_[slot] || sequence > *last_sequence_[slot]);
      const bool within_tolerance = acquired <= received+future_tolerance;
      const bool valid_clock = increasing && within_tolerance;
      if (!valid_clock) ++invalid_clocks_[slot];
      if (clock_ready_ && config_.timestamp_policy == "strict" && (!increasing || !within_tolerance))
        throw std::runtime_error("RealSense clock/sequence fault on " + std::string(rs2_stream_to_string(stream)) +
          ": acquisition_minus_receipt_ns=" + std::to_string(acquired-received) +
          ", acquisition_delta_ns=" + std::to_string(acquired-last_acquired_[slot]) +
          ", sequence=" + std::to_string(sequence) + ", previous_sequence=" +
          (last_sequence_[slot] ? std::to_string(*last_sequence_[slot]) : "none"));
      last_acquired_[slot] = acquired; last_sequence_[slot] = sequence;
      if (!clock_ready_) {
        // GLOBAL_TIME may be reported before the SDK regression has settled.
        // Require two continuous seconds of valid samples on every requested
        // stream. Never clamp to receipt or publish the initial invalid mapping.
        clock_samples_valid_[slot] = valid_clock;
        for (size_t i = 0; i < active_.size(); ++i)
          if (active_[i] && !clock_samples_valid_[i]) { stable_since_ns_ = 0; return; }
        if (!stable_since_ns_) stable_since_ns_ = received;
        if (received-stable_since_ns_ < 2000000000) return;
        clock_ready_ = true;
      }
      // Recovery admission happens in the shared gate after dequeue; rejected
      // mapped timestamps cannot poison its accepted watermark.
      if (is_motion && within_tolerance && acquired > received) ++future_motion_time_;
      if (!is_motion) {
        auto video = frame.as<rs2::video_frame>();
        const auto width = slot == 0 ? config_.output_width() : slot == 1 ? config_.native_depth_width() : 640;
        const auto height = slot == 0 ? config_.output_height() : slot == 1 ? config_.native_depth_height() : 480;
        const auto format = slot == 0 ? RS2_FORMAT_BGR8 : slot == 1 ? RS2_FORMAT_Z16 : RS2_FORMAT_Y8;
        require(video && video.get_width() == width && video.get_height() == height &&
                frame.get_profile().format() == format && frame.get_profile().fps() == native_fps_, "RealSense video profile changed");
        auto& queue = slot == 0 ? colors_ : slot == 1 ? depths_ : infrared_[slot-4];
        if (queue.size() == 2) queue.pop_front();
        queue.push_back({video,acquired,received,clock});
      } else {
        const auto xyz = frame.as<rs2::motion_frame>().get_motion_data();
        if (motion_.size() == 20) motion_.pop_front();
        motion_.push_back({slot == 2 ? MotionKind::Accelerometer : MotionKind::Gyroscope,
                           sequence,acquired,received,{xyz.x,xyz.y,xyz.z}});
      }
    } catch (const std::exception& error) {
      std::lock_guard<std::mutex> guard(mutex_);
      if (error_.empty()) error_ = std::string(error.what()).substr(0,1024);
    }
  }
  Config config_;
  int native_fps_ = 30;
  Identity identity_;
  DeviceLock lock_;
  std::unique_ptr<rs2::context> context_;
  rs2::device device_;
  std::vector<SensorSession> sessions_;
  std::unique_ptr<RealsenseGeometry> geometry_;
  std::mutex mutex_;
  bool stopping_ = false;
  std::string error_;
  uint64_t future_motion_time_ = 0;
  std::array<bool,6> clock_samples_valid_{}, active_{};
  bool clock_ready_ = false;
  std::array<int64_t,6> last_acquired_{};
  std::array<int64_t,6> last_received_{};
  std::array<uint64_t,6> received_samples_{}, invalid_clocks_{};
  std::array<int,6> domains_{};
  std::array<std::optional<uint64_t>,6> last_sequence_;
  std::array<std::optional<uint32_t>,6> raw_previous_;
  std::array<int64_t,6> raw_unwrapped_{};
  int64_t stable_since_ns_ = 0, next_output_ns_ = 0;
  std::deque<VideoSample> colors_, depths_;
  std::array<std::deque<VideoSample>, 2> infrared_;
  std::array<CameraModel, 2> stereo_models_;
  std::string stereo_calibration_;
  std::deque<Motion> motion_;
};
}

std::vector<DeviceDescriptor> discover_realsense() {
  rs2::context context;
  std::vector<DeviceDescriptor> result;
  for (auto device : context.query_devices())
    result.push_back({info(device,RS2_CAMERA_INFO_SERIAL_NUMBER),info(device,RS2_CAMERA_INFO_NAME) +
                      " firmware " + info(device,RS2_CAMERA_INFO_FIRMWARE_VERSION),
                      "USB " + info(device,RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)});
  return result;
}
std::unique_ptr<Provider> make_realsense_provider() { return std::make_unique<RealsenseProvider>(); }
}  // namespace rgbd_camera
