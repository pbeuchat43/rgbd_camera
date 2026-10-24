// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#include "rgbd_camera/camera_node.hpp"
#include <rgbd_camera_interfaces/msg/frame.hpp>
#include <rgbd_camera_interfaces/msg/motion.hpp>
#include <rgbd_camera_interfaces/msg/status.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

namespace rgbd_camera {
namespace {
using FrameMessage = rgbd_camera_interfaces::msg::Frame;
using MotionMessage = rgbd_camera_interfaces::msg::Motion;
using StatusMessage = rgbd_camera_interfaces::msg::Status;
using CaptureReference = rgbd_camera_interfaces::msg::CaptureReference;

template<class T> T setting(rclcpp::Node& node, const char* name, T value, const char* description) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true; descriptor.description = description;
  return node.declare_parameter<T>(name, value, descriptor);
}
sensor_msgs::msg::CameraInfo camera_info(const Geometry& geometry, const std_msgs::msg::Header& header) {
  sensor_msgs::msg::CameraInfo info;
  info.header = header; info.width = geometry.width; info.height = geometry.height;
  info.k = geometry.k; info.distortion_model = "plumb_bob"; info.d.assign(5, 0.0);
  info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  info.p = {geometry.k[0], 0, geometry.k[2], 0, 0, geometry.k[4], geometry.k[5], 0, 0, 0, 1, 0};
  return info;
}
}

class CameraNode::Implementation {
 public:
  Implementation(CameraNode& node, std::unique_ptr<Provider> provider)
      : node_(node), provider_(std::move(provider)) {
    if (node.get_parameter("use_sim_time").as_bool())
      throw std::runtime_error("physical acquisition cannot use simulated ROS time");
    config_.backend = setting<std::string>(node, "backend", "oak", "Explicit camera family: oak or realsense");
    config_.device_id = setting<std::string>(node, "device_id", "", "Exact device ID within the selected backend, or require exactly one device");
    config_.fps = setting<double>(node, "fps", 10.0, "RGB/depth rate request, 1..30 Hz");
    config_.pairing_tolerance_ms = setting<double>(node, "pairing_tolerance_ms", 5.0,
        "Maximum accepted RGB/depth skew, 0.1..50 ms and below half a frame period");
    config_.max_frame_age_ms = setting<double>(node, "max_frame_age_ms", 500.0,
        "Maximum accepted acquisition age, 10..2000 ms");
    config_.startup_timeout_s = setting<double>(node, "startup_timeout_s", 30.0,
        "First raw pair deadline, 5..120 seconds; SDK opening itself is synchronous");
    config_.stall_timeout_s = setting<double>(node, "stall_timeout_s", 2.0,
        "Raw acquisition silence deadline, 1..30 seconds; does not count rejected pairs as silence");
    config_.enable_imu = setting<bool>(node, "enable_imu", false, "Enable raw accel and gyro from the same camera session");
    config_.publish_images = setting<bool>(node, "publish_images", false, "Also publish standard RGB/depth topics (copies)");
    config_.optical_frame = setting<std::string>(node, "optical_frame", config_.backend + "_rgb_optical_frame", "Rectified RGB optical frame");
    config_.imu_frame = setting<std::string>(node, "imu_frame", config_.backend == "oak" ? "oak_imu_native_frame" : "realsense_imu_frame",
        "Backend-declared IMU axes and origin; no mounting transform implied");
    config_.validate();
    if (!provider_) provider_ = make_provider(config_.backend);
    const auto image_qos = rclcpp::SensorDataQoS().keep_last(1);
    frames_ = node.create_publisher<FrameMessage>("~/frames", image_qos);
    status_publisher_ = node.create_publisher<StatusMessage>("~/status", rclcpp::QoS(1).best_effort());
    if (config_.publish_images) {
      rgb_ = node.create_publisher<sensor_msgs::msg::Image>("~/rgb/image_rect", image_qos);
      depth_ = node.create_publisher<sensor_msgs::msg::Image>("~/depth/image_rect", image_qos);
      rgb_info_ = node.create_publisher<sensor_msgs::msg::CameraInfo>("~/rgb/camera_info", image_qos);
      depth_info_ = node.create_publisher<sensor_msgs::msg::CameraInfo>("~/depth/camera_info", image_qos);
    }
    if (config_.enable_imu) {
      accel_ = node.create_publisher<MotionMessage>("~/imu/accel_sample", rclcpp::SensorDataQoS().keep_last(4));
      gyro_ = node.create_publisher<MotionMessage>("~/imu/gyro_sample", rclcpp::SensorDataQoS().keep_last(4));
      accel_standard_ = node.create_publisher<sensor_msgs::msg::Imu>("~/imu/accel", rclcpp::SensorDataQoS().keep_last(4));
      gyro_standard_ = node.create_publisher<sensor_msgs::msg::Imu>("~/imu/gyro", rclcpp::SensorDataQoS().keep_last(4));
    }
    status_.state = "starting"; status_.imu_enabled = config_.enable_imu;
    status_.imu_state = config_.enable_imu ? "starting" : "disabled";
    status_.pairing_tolerance_ns = int64_t(config_.pairing_tolerance_ms * 1e6);
    status_.max_frame_age_ns = int64_t(config_.max_frame_age_ms * 1e6);
    timer_ = node.create_wall_timer(std::chrono::milliseconds(200), [this] { publish_status(); });
    worker_ = std::thread([this] { run(); });
  }
  ~Implementation() {
    timer_->cancel();
    stopping_ = true;
    if (worker_.joinable()) worker_.join();
  }

 private:
  CaptureReference reference(const std::string& stream, uint64_t sequence, int64_t acquired, int64_t received) const {
    CaptureReference result;
    result.source_id = identity_.source_id; result.generation = identity_.generation;
    result.stream_id = stream;
    result.sequence = sequence; result.acquisition_ns = acquired; result.received_ns = received;
    result.clock_domain = identity_.clock_domain; result.unix_offset_ns = identity_.unix_offset_ns;
    return result;
  }
  std_msgs::msg::Header header(int64_t acquired, const std::string& frame) const {
    std_msgs::msg::Header result;
    result.frame_id = frame; result.stamp = rclcpp::Time(acquired + identity_.unix_offset_ns);
    return result;
  }
  void publish_frame(std::unique_ptr<Frame> input) {
    auto output = std::make_unique<FrameMessage>();
    output->capture = reference("rgbd", input->sequence, input->acquisition_ns, input->received_ns);
    output->depth_sequence = input->depth_sequence; output->depth_acquisition_ns = input->depth_acquisition_ns;
    output->skew_ns = input->depth_acquisition_ns - input->acquisition_ns;
    output->pairing_tolerance_ns = int64_t(config_.pairing_tolerance_ms * 1e6);
    output->calibration_id = input->geometry.calibration_id; output->backend = identity_.backend;
    output->alignment = "aligned_to_rgb"; output->depth_convention = "optical_z";
    output->depth_unit_m = 0.001; output->invalid_depth_value = 0;
    output->rgb.header = header(input->acquisition_ns, config_.optical_frame);
    output->depth.header = header(input->depth_acquisition_ns, config_.optical_frame);
    output->rgb.width = output->depth.width = input->geometry.width;
    output->rgb.height = output->depth.height = input->geometry.height;
    output->rgb.encoding = "bgr8"; output->depth.encoding = "16UC1";
    output->rgb.step = input->geometry.width * 3; output->depth.step = input->geometry.width * 2;
    output->rgb.data = std::move(input->bgr); output->depth.data = std::move(input->depth_le);
    output->rgb_info = camera_info(input->geometry, output->rgb.header);
    output->depth_info = camera_info(input->geometry, output->depth.header);
    if (rgb_) {
      rgb_->publish(output->rgb); depth_->publish(output->depth);
      rgb_info_->publish(output->rgb_info); depth_info_->publish(output->depth_info);
    }
    frames_->publish(std::move(output));
  }
  void publish_motion(const Motion& input) {
    auto output = std::make_unique<MotionMessage>();
    const bool accelerometer = input.kind == MotionKind::Accelerometer;
    const auto sequence = accelerometer ? ++accel_sequence_ : ++gyro_sequence_;
    output->capture = reference(accelerometer ? "accel" : "gyro", sequence, input.acquisition_ns, input.received_ns);
    output->sensor_sequence = input.sequence;
    output->future_tolerance_ns = identity_.imu_future_tolerance_ns;
    output->sensor = identity_.imu_sensor; output->axes = identity_.imu_axes;
    output->sample.header = header(input.acquisition_ns, config_.imu_frame);
    output->sample.orientation_covariance[0] = -1;
    output->sample.angular_velocity_covariance[0] = -1;
    output->sample.linear_acceleration_covariance[0] = -1;
    if (input.kind == MotionKind::Accelerometer) {
      output->kind = "accelerometer";
      output->sample.linear_acceleration.x = input.xyz[0];
      output->sample.linear_acceleration.y = input.xyz[1];
      output->sample.linear_acceleration.z = input.xyz[2];
      output->sample.linear_acceleration_covariance.fill(0); // covariance unknown
      accel_standard_->publish(output->sample); accel_->publish(std::move(output));
    } else {
      output->kind = "gyroscope";
      output->sample.angular_velocity.x = input.xyz[0];
      output->sample.angular_velocity.y = input.xyz[1];
      output->sample.angular_velocity.z = input.xyz[2];
      output->sample.angular_velocity_covariance.fill(0);
      gyro_standard_->publish(output->sample); gyro_->publish(std::move(output));
    }
  }
  void publish_status() {
    StatusMessage output;
    { std::lock_guard<std::mutex> guard(mutex_); output = status_; }
    output.header.stamp = node_.now(); output.observed_ns = monotonic_ns();
    output.expires_after_ns = 1000000000;
    if (output.state == "streaming" && output.observed_ns - output.last_acquisition_ns > output.max_frame_age_ns) {
      output.state = "unavailable";
      if (output.detail.empty()) output.detail = "no_recent_accepted_pair";
    }
    if (output.imu_enabled && output.state != "error") {
      output.imu_state = (output.last_accel_ns > 0 && output.last_gyro_ns > 0 &&
        output.observed_ns - std::min(output.last_accel_ns, output.last_gyro_ns) <= output.max_frame_age_ns)
        ? "streaming" : "unavailable";
    }
    status_publisher_->publish(output);
  }
  void run() {
    try {
      const auto started = monotonic_ns();
      identity_ = provider_->start(config_);
      {
        std::lock_guard<std::mutex> guard(mutex_);
        status_.source_id = identity_.source_id; status_.generation = identity_.generation;
        status_.clock_domain = identity_.clock_domain; status_.backend = identity_.backend;
        status_.usb_speed = identity_.usb_speed; status_.imu_sensor = identity_.imu_sensor;
      }
      PairGate pairs(config_); MotionGate motion(identity_.imu_future_tolerance_ns);
      int64_t last_raw = 0;
      while (!stopping_ && rclcpp::ok(node_.get_node_base_interface()->get_context())) {
        auto products = provider_->poll();
        const auto now = monotonic_ns();
        if (products.motion.size() > 20) throw std::runtime_error("provider exceeded motion batch bound");
        if (!products.frame && !last_raw && !products.diagnostic.empty()) {
          std::lock_guard<std::mutex> guard(mutex_);
          status_.detail = products.diagnostic.substr(0,1024);
        }
        if (products.frame) {
          last_raw = now;
          const auto reason = pairs.admit(*products.frame, now);
          {
            std::lock_guard<std::mutex> guard(mutex_);
            ++status_.pairs_received; status_.sequence_gaps = pairs.gaps();
            status_.last_pair_received_ns = products.frame->received_ns;
            status_.last_skew_ns = products.frame->depth_acquisition_ns - products.frame->acquisition_ns;
            status_.detail = reason.value_or(products.diagnostic.substr(0,1024));
            if (reason) { ++status_.rejected_pairs; if (status_.frames == 0) status_.state = "unavailable"; }
            else { ++status_.frames; status_.last_acquisition_ns = products.frame->acquisition_ns; status_.state = "streaming"; }
          }
          if (!reason) publish_frame(std::move(products.frame));
        }
        for (const auto& sample : products.motion) {
          if (!config_.enable_imu) throw std::runtime_error("provider emitted disabled IMU output");
          if (!motion.admit(sample, monotonic_ns(), int64_t(config_.max_frame_age_ms * 1e6))) continue;
          publish_motion(sample);
          std::lock_guard<std::mutex> guard(mutex_);
          status_.imu_sequence_gaps = motion.gaps();
          if (sample.kind == MotionKind::Accelerometer) { ++status_.accel_samples; status_.last_accel_ns = sample.acquisition_ns; }
          else { ++status_.gyro_samples; status_.last_gyro_ns = sample.acquisition_ns; }
        }
        if ((!last_raw && now - started > config_.startup_timeout_s * 1e9) ||
            (last_raw && now - last_raw > config_.stall_timeout_s * 1e9))
          throw std::runtime_error(last_raw ? "raw_acquisition_stalled" : "first_pair_timeout: " + products.diagnostic.substr(0,1024));
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    } catch (const std::exception& error) {
      std::lock_guard<std::mutex> guard(mutex_);
      status_.state = "error"; status_.imu_state = config_.enable_imu ? "unavailable" : "disabled";
      status_.detail = error.what();
      RCLCPP_ERROR(node_.get_logger(), "%s", error.what());
    }
    try { provider_->stop(); }
    catch (const std::exception& error) {
      std::lock_guard<std::mutex> guard(mutex_);
      status_.state = "error"; status_.detail += std::string("; cleanup: ") + error.what();
      RCLCPP_ERROR(node_.get_logger(), "cleanup: %s", error.what());
    }
  }

  CameraNode& node_;
  Config config_;
  Identity identity_;
  uint64_t accel_sequence_ = 0, gyro_sequence_ = 0;
  std::unique_ptr<Provider> provider_;
  std::atomic<bool> stopping_{false};
  std::thread worker_;
  std::mutex mutex_;
  StatusMessage status_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<FrameMessage>::SharedPtr frames_;
  rclcpp::Publisher<StatusMessage>::SharedPtr status_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_, depth_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_, depth_info_;
  rclcpp::Publisher<MotionMessage>::SharedPtr accel_, gyro_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr accel_standard_, gyro_standard_;
};

CameraNode::CameraNode(const rclcpp::NodeOptions& options) : CameraNode(options, nullptr) {}
CameraNode::CameraNode(const rclcpp::NodeOptions& options, std::unique_ptr<Provider> provider)
    : Node("rgbd_camera", options), impl_(std::make_unique<Implementation>(*this, std::move(provider))) {}
CameraNode::~CameraNode() = default;
}  // namespace rgbd_camera
RCLCPP_COMPONENTS_REGISTER_NODE(rgbd_camera::CameraNode)
