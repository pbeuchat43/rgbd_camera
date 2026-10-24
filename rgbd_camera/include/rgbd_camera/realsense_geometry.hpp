// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#pragma once
#include "rgbd_camera/provider.hpp"
#include <librealsense2/h/rs_sensor.h>

namespace rgbd_camera {
struct RealsenseCalibration {
  rs2_intrinsics depth{}, color{};
  rs2_extrinsics depth_to_color{};
  double depth_unit_m = 0;
  std::string profile_identity;
};

// Virtual undistorted RGB camera. Nearest-point reprojection with a nearest-Z
// buffer; holes stay invalid. Stored depth is target-camera Z, not source Z.
class RealsenseGeometry {
 public:
  explicit RealsenseGeometry(const RealsenseCalibration& calibration);
  ~RealsenseGeometry();
  const Geometry& geometry() const;
  std::vector<uint8_t> rectify_bgr(const uint8_t* data, size_t bytes, size_t stride) const;
  std::vector<uint8_t> align_depth(const uint8_t* data, size_t bytes, size_t stride) const;
 private:
  class Implementation;
  std::unique_ptr<Implementation> impl_;
};

// GLOBAL_TIME is epoch milliseconds. Video correction moves the device readout
// timestamp to exposure midpoint; metadata's microsecond clock wraps at 2^32.
int64_t realsense_video_time_ns(double global_ms, int64_t epoch_minus_monotonic_ns,
                               uint32_t readout_us, uint32_t exposure_midpoint_us);
int64_t realsense_motion_time_ns(double global_ms, int64_t epoch_minus_monotonic_ns);
}  // namespace rgbd_camera
