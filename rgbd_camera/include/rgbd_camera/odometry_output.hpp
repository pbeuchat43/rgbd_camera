// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#pragma once
#include "rgbd_camera/provider.hpp"
#include "rgbd_camera/imu_combiner.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rgbd_camera_interfaces/msg/stereo_capture.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace rgbd_camera {
// Thin ROS output for camera-owned products. No estimator or device ownership.
class OdometryOutput {
 public:
  OdometryOutput(rclcpp::Node& node, const Config& config);
  void start(const Identity& identity);
  void stereo(std::unique_ptr<StereoFrame> pair);
  void motion(const CombinedMotion& sample);
 private:
  std_msgs::msg::Header header(int64_t time, const std::string& frame) const;
  rclcpp::Node& node_;
  Config config_;
  Identity identity_;
  std::array<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr, 2> images_;
  std::array<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr, 2> info_;
  rclcpp::Publisher<rgbd_camera_interfaces::msg::StereoCapture>::SharedPtr capture_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> transforms_;
};
}
