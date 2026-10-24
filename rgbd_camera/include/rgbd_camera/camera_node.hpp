// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#pragma once
#include "rgbd_camera/provider.hpp"
#include <rclcpp/rclcpp.hpp>

namespace rgbd_camera {
class CameraNode : public rclcpp::Node {
 public:
  explicit CameraNode(const rclcpp::NodeOptions& options);
  // Provider injection keeps failure/admission checks independent of hardware.
  CameraNode(const rclcpp::NodeOptions& options, std::unique_ptr<Provider> provider);
  ~CameraNode() override;
 private:
  class Implementation;
  std::unique_ptr<Implementation> impl_;
};
}  // namespace rgbd_camera
