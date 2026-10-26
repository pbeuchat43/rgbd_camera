// ROS adapter checks with a synthetic provider. Does not open hardware.
#include "rgbd_camera/camera_node.hpp"
#include <rgbd_camera_interfaces/msg/frame.hpp>
#include <rgbd_camera_interfaces/msg/motion.hpp>
#include <rgbd_camera_interfaces/msg/status.hpp>
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
using namespace rgbd_camera;
using namespace std::chrono_literals;
void check(bool result, const char* message) { if (!result) throw std::runtime_error(message); }
class Fixture : public Provider {
 public:
  explicit Fixture(std::shared_ptr<std::atomic<bool>> stopped) : stopped_(stopped) {}
  Identity start(const Config& config) override {
    config_ = config;
    Identity identity;
    identity.source_id = "synthetic:test"; identity.generation = "generation-test";
    identity.clock_domain = "synthetic:monotonic"; identity.imu_sensor = "synthetic";
    identity.unix_offset_ns = 1700000000000000000LL;
    identity.imu_future_tolerance_ns = 5000000;
    return identity;
  }
  Products poll() override {
    Products result;
    const auto now = monotonic_ns();
    if (now - last_ < 20000000) return result;
    last_ = now;
    if (++sequence_ == 100) throw std::runtime_error("synthetic_device_loss");
    auto frame = std::make_unique<Frame>();
    frame->sequence = sequence_; frame->depth_sequence = sequence_ + 10;
    frame->acquisition_ns = now - 20000000;
    frame->depth_acquisition_ns = frame->acquisition_ns + (sequence_ >= 10 && sequence_ < 50 ? 6000000 : 1000000);
    frame->received_ns = now;
    frame->geometry.k = {500, 0, 320, 0, 500, 320, 0, 0, 1};
    frame->geometry.calibration_id = "fixture";
    frame->bgr.resize(640 * 640 * 3); frame->depth_le.resize(640 * 640 * 2);
    // Simulate withheld IMU estimates while valid images continue; readiness
    // must expire independently and recover when admissible samples resume.
    if (config_.enable_imu && (sequence_ < 55 || sequence_ >= 75)) {
      // Replay a small SDK clock-estimate lead without changing its timestamps.
      result.motion.push_back({MotionKind::Accelerometer, sequence_,
          sequence_ == 5 ? now + 500000 : now - 2000000, now, {0, 0, 9.81}});
      result.motion.push_back({MotionKind::Gyroscope, sequence_, now - 1000000, now, {0, 0, 0}});
    }
    result.frame = std::move(frame);
    return result;
  }
  void stop() override { *stopped_ = true; }
 private:
  Config config_;
  std::shared_ptr<std::atomic<bool>> stopped_;
  uint64_t sequence_ = 0;
  int64_t last_ = 0;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto stopped = std::make_shared<std::atomic<bool>>(false);
    auto options = rclcpp::NodeOptions().use_intra_process_comms(true)
      .parameter_overrides({{"enable_imu", true}, {"max_frame_age_ms", 100.0}});
    auto camera = std::make_shared<CameraNode>(options, std::make_unique<Fixture>(stopped));
    auto observer = std::make_shared<rclcpp::Node>("contract_observer", rclcpp::NodeOptions().use_intra_process_comms(true));
    uint64_t frames = 0, accel = 0, gyro = 0;
    bool recovered = false, unavailable = false, error = false;
    bool imu_expired_with_images = false, imu_recovered = false;
    bool future_estimate_preserved = false;
    auto frame_sub = observer->create_subscription<rgbd_camera_interfaces::msg::Frame>(
      "/rgbd_camera/frames", rclcpp::SensorDataQoS().keep_last(1),
      [&](rgbd_camera_interfaces::msg::Frame::ConstSharedPtr frame) {
        ++frames; check(frame->capture.generation == "generation-test", "generation lost");
        check(frame->capture.stream_id == "rgbd", "image stream identity lost");
        check(frame->rgb.data.size() == 640 * 640 * 3, "RGB data missing");
        check(frame->depth.data.size() == 640 * 640 * 2, "depth data missing");
        check(frame->rgb_info.header == frame->rgb.header, "RGB geometry binding lost");
        check(frame->depth_info.header == frame->depth.header, "depth geometry binding lost");
        check(frame->capture.sequence < 10 || frame->capture.sequence >= 50, "rejected skew leaked");
        check(frame->skew_ns == 1000000 && frame->pairing_tolerance_ns == 5000000, "wrong pairing metadata");
        recovered |= frame->capture.sequence >= 50;
      });
    auto motion_check = [&](rgbd_camera_interfaces::msg::Motion::ConstSharedPtr sample) {
      check(sample->sample.orientation_covariance[0] == -1, "invented orientation");
      check(sample->axes == "sensor_native", "IMU frame mislabeled");
      check(sample->future_tolerance_ns == 5000000, "IMU clock allowance lost");
      if (sample->kind == "accelerometer") {
        if (sample->sensor_sequence == 5) {
          check(sample->capture.acquisition_ns - sample->capture.received_ns == 500000,
                "SDK acquisition timestamp clamped to receipt");
          const auto stamp_ns = int64_t(sample->sample.header.stamp.sec) * 1000000000 + sample->sample.header.stamp.nanosec;
          check(stamp_ns == sample->capture.acquisition_ns + sample->capture.unix_offset_ns,
                "standard IMU header changed acquisition time");
          future_estimate_preserved = true;
        }
        check(sample->capture.stream_id == "accel", "accel stream identity lost");
        ++accel; check(sample->sample.angular_velocity_covariance[0] == -1, "invented gyro");
        check(sample->sample.linear_acceleration_covariance[0] == 0, "incorrect accel covariance");
      } else {
        check(sample->capture.stream_id == "gyro", "gyro stream identity lost");
        ++gyro; check(sample->sample.linear_acceleration_covariance[0] == -1, "invented accel");
      }
    };
    auto a = observer->create_subscription<rgbd_camera_interfaces::msg::Motion>(
      "/rgbd_camera/imu/accel_sample", rclcpp::SensorDataQoS().keep_last(4), motion_check);
    auto g = observer->create_subscription<rgbd_camera_interfaces::msg::Motion>(
      "/rgbd_camera/imu/gyro_sample", rclcpp::SensorDataQoS().keep_last(4), motion_check);
    auto s = observer->create_subscription<rgbd_camera_interfaces::msg::Status>(
      "/rgbd_camera/status", rclcpp::QoS(1).best_effort(),
      [&](rgbd_camera_interfaces::msg::Status::ConstSharedPtr status) {
        unavailable |= status->state == "unavailable" && status->rejected_pairs > 0;
        imu_expired_with_images |= status->state == "streaming" && status->imu_state == "unavailable";
        imu_recovered |= imu_expired_with_images && status->state == "streaming" && status->imu_state == "streaming";
        error |= status->state == "error" && status->detail == "synthetic_device_loss";
      });
    check(!camera->set_parameters({rclcpp::Parameter("pairing_tolerance_ms", 10.0)})[0].successful,
          "shared camera was reconfigured live");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(camera); executor.add_node(observer);
    const auto deadline = monotonic_ns() + 5000000000;
    while (!error && monotonic_ns() < deadline) {
      executor.spin_some(); std::this_thread::sleep_for(1ms);
    }
    check(frames > 10 && accel > frames && gyro > frames, "missing independent streams");
    check(recovered && unavailable && error && stopped->load(), "fault/recovery policy failed");
    check(imu_expired_with_images && imu_recovered, "IMU freshness did not expire/recover independently of images");
    check(future_estimate_preserved, "bounded future IMU estimate was not delivered");
    executor.remove_node(camera); camera.reset();
    rclcpp::shutdown();
    std::cout << "camera adapter checks passed (synthetic provider, composed ROS)\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; rclcpp::shutdown(); return 1;
  }
}
