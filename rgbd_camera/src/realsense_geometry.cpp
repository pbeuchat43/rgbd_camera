// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#include "rgbd_camera/realsense_geometry.hpp"
#include <librealsense2/rsutil.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace rgbd_camera {
namespace {
void require(bool condition, const char* reason) { if (!condition) throw std::runtime_error(reason); }
void validate_intrinsics(const rs2_intrinsics& intrinsics, bool color) {
  require(intrinsics.width == 640 && intrinsics.height == 480, "RealSense requires the 640x480 profile");
  require(std::isfinite(intrinsics.fx) && std::isfinite(intrinsics.fy) && intrinsics.fx > 0 && intrinsics.fy > 0 &&
          std::isfinite(intrinsics.ppx) && std::isfinite(intrinsics.ppy) && intrinsics.ppx >= 0 &&
          intrinsics.ppx < intrinsics.width && intrinsics.ppy >= 0 && intrinsics.ppy < intrinsics.height,
          "invalid RealSense intrinsics");
  for (float value : intrinsics.coeffs) require(std::isfinite(value), "nonfinite RealSense distortion");
  require(intrinsics.model == RS2_DISTORTION_NONE || intrinsics.model == RS2_DISTORTION_BROWN_CONRADY ||
          intrinsics.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY ||
          (color && intrinsics.model == RS2_DISTORTION_MODIFIED_BROWN_CONRADY), "unsupported RealSense distortion model");
}
void fingerprint(std::ostream& out, const rs2_intrinsics& intrinsics) {
  out << intrinsics.width << ',' << intrinsics.height << ',' << intrinsics.fx << ',' << intrinsics.fy << ','
      << intrinsics.ppx << ',' << intrinsics.ppy << ',' << int(intrinsics.model) << ':';
  for (auto coefficient : intrinsics.coeffs) out << coefficient << ',';
}
void validate_storage(const uint8_t* data, size_t bytes, size_t stride, size_t pixel_bytes) {
  require(data && stride >= 640 * pixel_bytes && stride <= 640 * pixel_bytes + 4096 && bytes == stride * 480,
          "unexpected RealSense image storage");
}
}

class RealsenseGeometry::Implementation {
 public:
  explicit Implementation(const RealsenseCalibration& calibration) : calibration_(calibration) {
    validate_intrinsics(calibration.depth, false); validate_intrinsics(calibration.color, true);
    require(std::isfinite(calibration.depth_unit_m) && calibration.depth_unit_m >= 1e-9 && calibration.depth_unit_m <= 1,
            "invalid RealSense depth unit");
    const auto& transform = calibration.depth_to_color;
    for (float value : transform.rotation) require(std::isfinite(value), "nonfinite RealSense rotation");
    for (float value : transform.translation) require(std::isfinite(value) && std::abs(value) < 1,
                                                    "invalid RealSense translation");
    // SDK rotations are column-major. Reject scale, shear and reflections.
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
      double dot = 0;
      for (int row = 0; row < 3; ++row) dot += transform.rotation[3*i+row] * transform.rotation[3*j+row];
      require(std::abs(dot - (i == j ? 1 : 0)) < 1e-3, "nonorthogonal RealSense extrinsics");
    }
    const auto* r = transform.rotation;
    const double determinant = r[0]*(r[4]*r[8]-r[7]*r[5])-r[3]*(r[1]*r[8]-r[7]*r[2])+r[6]*(r[1]*r[5]-r[4]*r[2]);
    require(std::abs(determinant-1) < 1e-3, "reflected RealSense extrinsics");
    geometry_.width = 640; geometry_.height = 480;
    geometry_.k = {calibration.color.fx,0,calibration.color.ppx,0,calibration.color.fy,calibration.color.ppy,0,0,1};
    std::ostringstream calibration_text;
    calibration_text << std::setprecision(17) << "realsense_rectified_rgb_nearest_point_target_z_mm_v1:"
                     << calibration.profile_identity << ':' << calibration.depth_unit_m << ':';
    fingerprint(calibration_text, calibration.depth); fingerprint(calibration_text, calibration.color);
    for (float value : transform.rotation) calibration_text << value << ',';
    for (float value : transform.translation) calibration_text << value << ',';
    geometry_.calibration_id = sha256(calibration_text.str());
    geometry_.validate();
    map_x_.create(480,640,CV_32F); map_y_.create(480,640,CV_32F);
    rotated_rays_.resize(640*480);
    for (int y = 0; y < 480; ++y) for (int x = 0; x < 640; ++x) {
      const float pixel[2]{float(x),float(y)};
      float ray[3]; rs2_deproject_pixel_to_point(ray, &calibration.depth, pixel, 1.0F);
      auto rotation_only = transform;
      std::fill(std::begin(rotation_only.translation), std::end(rotation_only.translation), 0.0F);
      rs2_transform_point_to_point(rotated_rays_[y*640+x].data(), &rotation_only, ray);
      const float color_ray[3]{(x-calibration.color.ppx)/calibration.color.fx,
                               (y-calibration.color.ppy)/calibration.color.fy, 1};
      float source_pixel[2]; rs2_project_point_to_pixel(source_pixel, &calibration.color, color_ray);
      require(std::isfinite(source_pixel[0]) && std::isfinite(source_pixel[1]), "nonfinite RGB rectification map");
      map_x_.at<float>(y,x) = source_pixel[0]; map_y_.at<float>(y,x) = source_pixel[1];
      for (float value : rotated_rays_[y*640+x]) require(std::isfinite(value), "nonfinite depth projection ray");
    }
  }
  RealsenseCalibration calibration_;
  Geometry geometry_;
  cv::Mat map_x_, map_y_;
  std::vector<std::array<float,3>> rotated_rays_;
};

RealsenseGeometry::RealsenseGeometry(const RealsenseCalibration& calibration)
    : impl_(std::make_unique<Implementation>(calibration)) {}
RealsenseGeometry::~RealsenseGeometry() = default;
const Geometry& RealsenseGeometry::geometry() const { return impl_->geometry_; }
std::vector<uint8_t> RealsenseGeometry::rectify_bgr(const uint8_t* data, size_t bytes, size_t stride) const {
  validate_storage(data, bytes, stride, 3);
  std::vector<uint8_t> output(640*480*3);
  const cv::Mat source(480,640,CV_8UC3,const_cast<uint8_t*>(data),stride);
  cv::Mat destination(480,640,CV_8UC3,output.data());
  cv::remap(source,destination,impl_->map_x_,impl_->map_y_,cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(0,0,0));
  return output;
}
std::vector<uint8_t> RealsenseGeometry::align_depth(const uint8_t* data, size_t bytes, size_t stride) const {
  validate_storage(data, bytes, stride, 2);
  std::vector<uint8_t> output(640*480*2, 0);
  const auto& calibration = impl_->calibration_;
  const auto& intrinsics = calibration.color;
  for (int y = 0; y < 480; ++y) for (int x = 0; x < 640; ++x) {
    const auto* raw_pixel = data + y*stride + x*2;
    const uint16_t raw = uint16_t(raw_pixel[0]) | uint16_t(raw_pixel[1]) << 8;
    if (!raw) continue;
    const double source_z = raw * calibration.depth_unit_m;
    const auto& ray = impl_->rotated_rays_[y*640+x];
    const double target_z = ray[2]*source_z + calibration.depth_to_color.translation[2];
    if (target_z < 0.0005 || target_z >= 65.5355) continue;
    const double px = (ray[0]*source_z + calibration.depth_to_color.translation[0])*intrinsics.fx/target_z + intrinsics.ppx;
    const double py = (ray[1]*source_z + calibration.depth_to_color.translation[1])*intrinsics.fy/target_z + intrinsics.ppy;
    if (px < -0.5 || px >= 639.5 || py < -0.5 || py >= 479.5) continue;
    const auto offset = 2*(size_t(std::floor(py+0.5))*640 + size_t(std::floor(px+0.5)));
    const auto millimetres = uint16_t(std::llround(target_z*1000));
    const uint16_t old = uint16_t(output[offset]) | uint16_t(output[offset+1]) << 8;
    if (!old || millimetres < old) {
      output[offset] = millimetres & 255; output[offset+1] = millimetres >> 8;
    }
  }
  return output;
}

int64_t realsense_motion_time_ns(double global_ms, int64_t offset) {
  require(std::isfinite(global_ms) && global_ms > 0 && global_ms < 9e12,
          "invalid RealSense global timestamp");
  const auto epoch_ns = int64_t(std::llround(global_ms*1e6));
  require(offset >= 0 && epoch_ns > offset, "invalid RealSense epoch-to-monotonic mapping");
  return epoch_ns-offset;
}
int64_t realsense_video_time_ns(double global_ms, int64_t offset, uint32_t readout_us, uint32_t exposure_us) {
  const auto wrapped = uint32_t(exposure_us-readout_us);
  const int64_t correction_us = wrapped <= uint32_t(INT32_MAX) ? int64_t(wrapped) : int64_t(wrapped)-(int64_t(1)<<32);
  require(correction_us <= 0 && correction_us >= -100000, "invalid RealSense exposure midpoint correction");
  const auto timestamp = realsense_motion_time_ns(global_ms,offset) + correction_us*1000;
  require(timestamp > 0, "invalid RealSense exposure timestamp");
  return timestamp;
}
}  // namespace rgbd_camera
