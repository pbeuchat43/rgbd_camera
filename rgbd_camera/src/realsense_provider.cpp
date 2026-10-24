// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#include "rgbd_camera/provider.hpp"
#include "rgbd_camera/device_lock.hpp"
#include "rgbd_camera/realsense_geometry.hpp"
#include <librealsense2/rs.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>
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
    {
      std::lock_guard<std::mutex> guard(mutex_);
      error_.clear(); last_acquired_.fill(0); future_motion_time_ = rejected_motion_time_ = 0;
      last_sequence_.fill(std::nullopt); next_output_ns_ = 0;
      clock_samples_valid_.fill(false); clock_ready_ = false; stable_since_ns_ = 0;
      received_samples_.fill(0); last_received_.fill(0); domains_.fill(-1); invalid_clocks_.fill(0);
      colors_.clear(); depths_.clear(); motion_.clear();
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
    require(info(device_,RS2_CAMERA_INFO_FIRMWARE_VERSION) == "5.16.0.1",
            "this D455 profile requires firmware 5.16.0.1 with librealsense 2.56.3; firmware changes are explicit");
    const auto serial = info(device_,RS2_CAMERA_INFO_SERIAL_NUMBER);
    lock_.acquire("realsense:" + serial);
    const auto sensors = device_.query_sensors();
    std::array<std::optional<Selection>,4> selected;
    for (size_t index = 0; index < sensors.size(); ++index) {
      for (auto profile : sensors[index].get_stream_profiles()) {
        const auto stream = profile.stream_type();
        if (auto video = profile.as<rs2::video_stream_profile>()) {
          if (video.width() == 640 && video.height() == 480 && profile.fps() == 30) {
            if (stream == RS2_STREAM_COLOR && profile.format() == RS2_FORMAT_BGR8) selected[0] = Selection{index,profile};
            if (stream == RS2_STREAM_DEPTH && profile.format() == RS2_FORMAT_Z16) selected[1] = Selection{index,profile};
          }
        } else if (config.enable_imu && profile.format() == RS2_FORMAT_MOTION_XYZ32F && profile.fps() > 0 && profile.fps() <= 400) {
          const int slot = stream == RS2_STREAM_ACCEL ? 2 : stream == RS2_STREAM_GYRO ? 3 : -1;
          if (slot >= 0 && (!selected[slot] || profile.fps() < selected[slot]->profile.fps()))
            selected[slot] = Selection{index,profile};
        }
      }
    }
    require(selected[0] && selected[1], "D455 requires supported 640x480 BGR8/Z16 streams at 30 Hz");
    require(!config.enable_imu || (selected[2] && selected[3]), "requested RealSense accel/gyro streams unavailable");
    RealsenseCalibration calibration;
    const auto color_profile = selected[0]->profile.as<rs2::video_stream_profile>();
    const auto depth_profile = selected[1]->profile.as<rs2::video_stream_profile>();
    calibration.color = color_profile.get_intrinsics(); calibration.depth = depth_profile.get_intrinsics();
    calibration.depth_to_color = depth_profile.get_extrinsics_to(color_profile);
    calibration.depth_unit_m = sensors[selected[1]->sensor].as<rs2::depth_sensor>().get_depth_scale();
    calibration.profile_identity = serial + ":" + info(device_,RS2_CAMERA_INFO_FIRMWARE_VERSION) + ":640x480@30:bgr8:z16";
    geometry_ = std::make_unique<RealsenseGeometry>(calibration);
    identity_.source_id = "realsense:" + serial;
    identity_.generation = read_identifier("/proc/sys/kernel/random/uuid");
    identity_.clock_domain = "linux_monotonic:" + read_identifier("/proc/sys/kernel/random/boot_id");
    identity_.backend = std::string("librealsense-") + RS2_API_VERSION_STR + ":D455:firmware-5.16.0.1:exposure_midpoint_global_time";
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
          session.profiles.push_back(selected[slot]->profile); session.motion |= slot >= 2;
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
    std::optional<VideoSample> color, depth;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (!error_.empty()) throw std::runtime_error(error_);
      if (future_motion_time_ || rejected_motion_time_)
        products.diagnostic = "realsense_imu_future_timestamps_admitted=" + std::to_string(future_motion_time_) +
          ", rejected_over_tolerance=" + std::to_string(rejected_motion_time_);
      if (!clock_ready_) {
        products.diagnostic = "realsense_clock_starting stable_ms=" +
          std::to_string(stable_since_ns_ ? (monotonic_ns()-stable_since_ns_)/1000000 : 0);
        for (size_t slot = 0; slot < (config_.enable_imu ? 4u : 2u); ++slot)
          products.diagnostic += " stream" + std::to_string(slot) + "{samples=" + std::to_string(received_samples_[slot]) +
            ",domain=" + std::to_string(domains_[slot]) + ",invalid=" + std::to_string(invalid_clocks_[slot]) +
            ",age_us=" + std::to_string((last_received_[slot]-last_acquired_[slot])/1000) + "}";
      }
      products.motion.assign(motion_.begin(),motion_.end()); motion_.clear();
      while (!colors_.empty() && !depths_.empty()) {
        const auto skew = depths_.front().acquired-colors_.front().acquired;
        // Select the nearest 30 Hz pair without reusing a frame. Expose pairs
        // above the user tolerance to PairGate, so rejection remains observable.
        if (skew > 16666667) { colors_.pop_front(); continue; }
        if (skew < -16666667) { depths_.pop_front(); continue; }
        color = std::move(colors_.front()); colors_.pop_front();
        depth = std::move(depths_.front()); depths_.pop_front();
        if (color->acquired < next_output_ns_) { color.reset(); depth.reset(); continue; }
        const auto period = int64_t(std::llround(1e9/config_.fps));
        if (!next_output_ns_) next_output_ns_ = color->acquired;
        next_output_ns_ += ((color->acquired-next_output_ns_)/period+1)*period;
        break;
      }
    }
    if (color && depth) {
      auto output = std::make_unique<Frame>();
      output->sequence = color->frame.get_frame_number(); output->depth_sequence = depth->frame.get_frame_number();
      output->acquisition_ns = color->acquired; output->depth_acquisition_ns = depth->acquired;
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
    { std::lock_guard<std::mutex> guard(mutex_); colors_.clear(); depths_.clear(); motion_.clear(); }
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
                       stream == RS2_STREAM_ACCEL ? 2 : stream == RS2_STREAM_GYRO ? 3 : -1;
      require(slot >= 0, "unexpected RealSense stream");
      ++received_samples_[slot]; last_received_[slot] = received;
      domains_[slot] = frame.get_frame_timestamp_domain();
      if (domains_[slot] != RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME) {
        require(!clock_ready_, "RealSense timestamp domain changed during capture");
        clock_samples_valid_[slot] = false; stable_since_ns_ = 0;
        return;
      }
      int64_t acquired;
      if (slot < 2) {
        require(frame.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP) &&
                frame.supports_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP), "RealSense exposure timestamp metadata unavailable");
        acquired = realsense_video_time_ns(frame.get_timestamp(),identity_.unix_offset_ns,
          uint32_t(frame.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP)),
          uint32_t(frame.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP)));
      } else acquired = realsense_motion_time_ns(frame.get_timestamp(),identity_.unix_offset_ns);
      const auto sequence = frame.get_frame_number();
      // An individual callback may arrive slightly before the SDK clock estimate.
      // Video startup stays strict; after startup allow the same small mapping
      // error as IMU. PairGate still requires both image timestamps to precede
      // the complete bundle's receipt (the later of its two image receipts).
      const auto future_tolerance = slot >= 2 || clock_ready_ ? identity_.imu_future_tolerance_ns : 0;
      const bool increasing = acquired > last_acquired_[slot] &&
                              (!last_sequence_[slot] || sequence > *last_sequence_[slot]);
      const bool within_tolerance = acquired <= received+future_tolerance;
      const bool valid_clock = increasing && within_tolerance;
      if (!valid_clock) ++invalid_clocks_[slot];
      if (clock_ready_ && (!increasing || (slot < 2 && !within_tolerance)))
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
        const size_t streams = config_.enable_imu ? 4 : 2;
        if (!std::all_of(clock_samples_valid_.begin(),clock_samples_valid_.begin()+streams,
                         [](bool valid) { return valid; })) { stable_since_ns_ = 0; return; }
        if (!stable_since_ns_) stable_since_ns_ = received;
        if (received-stable_since_ns_ < 2000000000) return;
        clock_ready_ = true;
      }
      // A monotonic IMU estimate outside its declared clock budget is omitted;
      // it does not invalidate otherwise admissible RGB/depth. The adapter's
      // independent IMU freshness status expires if valid samples stop.
      if (slot >= 2 && !within_tolerance) { ++rejected_motion_time_; return; }
      if (slot >= 2 && acquired > received) ++future_motion_time_;
      if (slot < 2) {
        auto video = frame.as<rs2::video_frame>();
        require(video && video.get_width() == 640 && video.get_height() == 480 &&
                frame.get_profile().format() == (slot == 0 ? RS2_FORMAT_BGR8 : RS2_FORMAT_Z16), "RealSense video profile changed");
        auto& queue = slot == 0 ? colors_ : depths_;
        if (queue.size() == 2) queue.pop_front();
        queue.push_back({video,acquired,received});
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
  Identity identity_;
  DeviceLock lock_;
  std::unique_ptr<rs2::context> context_;
  rs2::device device_;
  std::vector<SensorSession> sessions_;
  std::unique_ptr<RealsenseGeometry> geometry_;
  std::mutex mutex_;
  bool stopping_ = false;
  std::string error_;
  uint64_t future_motion_time_ = 0, rejected_motion_time_ = 0;
  std::array<bool,4> clock_samples_valid_{};
  bool clock_ready_ = false;
  std::array<int64_t,4> last_acquired_{};
  std::array<int64_t,4> last_received_{};
  std::array<uint64_t,4> received_samples_{}, invalid_clocks_{};
  std::array<int,4> domains_{};
  std::array<std::optional<uint64_t>,4> last_sequence_;
  int64_t stable_since_ns_ = 0, next_output_ns_ = 0;
  std::deque<VideoSample> colors_, depths_;
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
