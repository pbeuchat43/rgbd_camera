// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#include "rgbd_camera/odometry_output.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <limits>

namespace rgbd_camera {
OdometryOutput::OdometryOutput(rclcpp::Node& node, const Config& config) : node_(node), config_(config) {
  if (config.publish_stereo) {
    const auto qos = rclcpp::SensorDataQoS().keep_last(4);
    for (size_t i = 0; i < 2; ++i) {
      const std::string prefix = i == 0 ? "~/stereo/left/" : "~/stereo/right/";
      images_[i] = node.create_publisher<sensor_msgs::msg::Image>(prefix+"image", qos);
      info_[i] = node.create_publisher<sensor_msgs::msg::CameraInfo>(prefix+"camera_info", qos);
    }
    capture_ = node.create_publisher<rgbd_camera_interfaces::msg::StereoCapture>("~/stereo/capture", rclcpp::SensorDataQoS().keep_last(32));
    transforms_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
  }
  if (config.combine_imu) imu_ = node.create_publisher<sensor_msgs::msg::Imu>("~/imu/combined", rclcpp::SensorDataQoS().keep_last(50));
}
void OdometryOutput::start(const Identity& identity) {
  identity_ = identity;
  if (!config_.publish_stereo) return;
  if (!identity.extrinsics || (config_.enable_imu && !identity.extrinsics->has_imu))
    throw std::runtime_error("requested stereo/IMU internal calibration is unavailable");
  identity.extrinsics->validate();
  std::vector<geometry_msgs::msg::TransformStamped> output;
  const auto add = [&](const std::string& child, const RigidTransform& m) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = config_.optical_frame; transform.header.stamp = node_.now(); transform.child_frame_id = child;
    transform.transform.translation.x = m[3]; transform.transform.translation.y = m[7]; transform.transform.translation.z = m[11];
    tf2::Quaternion q; tf2::Matrix3x3(m[0],m[1],m[2],m[4],m[5],m[6],m[8],m[9],m[10]).getRotation(q); q.normalize();
    transform.transform.rotation.x = q.x(); transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z(); transform.transform.rotation.w = q.w(); output.push_back(transform);
  };
  add(config_.left_frame, identity.extrinsics->rgb_from_camera[0]);
  add(config_.right_frame, identity.extrinsics->rgb_from_camera[1]);
  if (config_.enable_imu) add(config_.imu_frame, identity.extrinsics->rgb_from_imu);
  transforms_->sendTransform(output);
}
std_msgs::msg::Header OdometryOutput::header(int64_t time, const std::string& frame) const {
  if (time <= 0 || identity_.unix_offset_ns < -time || identity_.unix_offset_ns > std::numeric_limits<int64_t>::max()-time)
    throw std::runtime_error("invalid stereo/IMU ROS clock mapping");
  std_msgs::msg::Header result; result.frame_id = frame; result.stamp = rclcpp::Time(time+identity_.unix_offset_ns); return result;
}
void OdometryOutput::stereo(std::unique_ptr<StereoFrame> pair) {
  if (!capture_ || pair->rectified != identity_.extrinsics->rectified) throw std::runtime_error("unexpected stereo convention");
  rgbd_camera_interfaces::msg::StereoCapture capture;
  capture.header = header(std::max(pair->midpoint_ns[0],pair->midpoint_ns[1]), config_.left_frame);
  capture.calibration_id = pair->calibration_id; capture.rectified = pair->rectified; capture.pairing_tolerance_ns = 1000000;
  for (size_t i = 0; i < 2; ++i) {
    auto& reference = i == 0 ? capture.left : capture.right;
    reference.source_id = identity_.source_id; reference.generation = identity_.generation;
    reference.clock_domain = identity_.clock_domain; reference.unix_offset_ns = identity_.unix_offset_ns;
    reference.stream_id = i == 0 ? "stereo_left" : "stereo_right"; reference.sequence = pair->sequence[i];
    reference.acquisition_ns = pair->event_ns[i]; reference.received_ns = pair->received_ns;
    reference.exposure_midpoint_ns = pair->midpoint_ns[i]; reference.exposure_duration_ns = pair->exposure_ns[i];
  }
  // Metadata may arrive before or after images. Consumers join exactly, never by arrival.
  capture_->publish(capture);
  for (size_t i = 0; i < 2; ++i) {
    const auto& c = pair->cameras[i]; sensor_msgs::msg::CameraInfo info;
    info.header = header(pair->midpoint_ns[i], i == 0 ? config_.left_frame : config_.right_frame);
    info.width = c.width; info.height = c.height; info.k = c.k; info.d = c.d;
    info.distortion_model = c.d.size() == 8 ? "rational_polynomial" : "plumb_bob";
    info.r = {1,0,0,0,1,0,0,0,1}; info.p = {c.k[0],0,c.k[2],0,0,c.k[4],c.k[5],0,0,0,1,0};
    info_[i]->publish(info);
    auto image = std::make_unique<sensor_msgs::msg::Image>(); image->header = info.header;
    image->width = c.width; image->height = c.height; image->step = c.width; image->encoding = "mono8";
    image->data = std::move(pair->pixels[i]); images_[i]->publish(std::move(image));
  }
}
void OdometryOutput::motion(const CombinedMotion& sample) {
  if (!imu_) throw std::runtime_error("unexpected combined IMU output");
  sensor_msgs::msg::Imu output; output.header = header(sample.gyro.acquisition_ns, config_.imu_frame);
  output.orientation_covariance[0] = -1; // Orientation unavailable; other all-zero covariances mean unknown.
  output.angular_velocity.x = sample.gyro.xyz[0]; output.angular_velocity.y = sample.gyro.xyz[1]; output.angular_velocity.z = sample.gyro.xyz[2];
  output.linear_acceleration.x = sample.acceleration[0]; output.linear_acceleration.y = sample.acceleration[1]; output.linear_acceleration.z = sample.acceleration[2];
  imu_->publish(output);
}
}
