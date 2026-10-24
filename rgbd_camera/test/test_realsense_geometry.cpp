// Synthetic calibrated geometry/time checks; no RealSense device is opened.
#include "rgbd_camera/realsense_geometry.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace rgbd_camera;
void check(bool condition, const char* detail) { if (!condition) throw std::runtime_error(detail); }
template<class F> void rejects(F action, const char* detail) {
  try { action(); } catch (const std::exception&) { return; }
  throw std::runtime_error(detail);
}
RealsenseCalibration calibration() {
  RealsenseCalibration value;
  value.color = {640,480,320,240,100,100,RS2_DISTORTION_NONE,{0,0,0,0,0}};
  value.depth = value.color;
  value.depth_to_color.rotation[0] = value.depth_to_color.rotation[4] = value.depth_to_color.rotation[8] = 1;
  value.depth_unit_m = .002;
  value.profile_identity = "fixture";
  return value;
}
void put(std::vector<uint8_t>& data, int x, int y, uint16_t value, size_t stride = 1280) {
  data[y*stride+x*2] = value & 255; data[y*stride+x*2+1] = value >> 8;
}
uint16_t get(const std::vector<uint8_t>& data, int x, int y) {
  const auto index = y*1280+x*2;
  return uint16_t(data[index]) | uint16_t(data[index+1]) << 8;
}
int main() {
  try {
    auto cal = calibration();
    RealsenseGeometry identity(cal);
    check(identity.geometry().width == 640 && identity.geometry().height == 480, "nonsquare geometry lost");
    std::vector<uint8_t> depth(1280*480,0);
    put(depth,320,240,1000);
    auto aligned = identity.align_depth(depth.data(),depth.size(),1280);
    check(get(aligned,320,240) == 2000 && get(aligned,319,240) == 0, "depth scale or invalid-hole semantics wrong");

    // A translation along Z proves target optical Z differs from source depth.
    cal.depth_to_color.translation[0] = .2F; cal.depth_to_color.translation[2] = .5F;
    RealsenseGeometry translated(cal);
    aligned = translated.align_depth(depth.data(),depth.size(),1280);
    check(get(aligned,328,240) == 2500 && get(aligned,320,240) == 0, "source Z was relabeled as RGB optical Z");
    check(identity.geometry().calibration_id != translated.geometry().calibration_id, "extrinsics missing from calibration identity");

    // Rotation changes both the target pixel and Z, not just the image offset.
    cal = calibration();
    const float sine = .6F, cosine = .8F;
    cal.depth_to_color.rotation[0] = cosine; cal.depth_to_color.rotation[2] = -sine;
    cal.depth_to_color.rotation[6] = sine; cal.depth_to_color.rotation[8] = cosine;
    RealsenseGeometry rotated(cal);
    aligned = rotated.align_depth(depth.data(),depth.size(),1280);
    check(get(aligned,395,240) == 1600, "column-major rotation/target Z wrong");

    // Multiple source pixels can map to one target pixel: keep nearest Z.
    cal = calibration(); cal.color.fx = 10;
    RealsenseGeometry compressed(cal);
    put(depth,321,240,500);
    aligned = compressed.align_depth(depth.data(),depth.size(),1280);
    check(get(aligned,320,240) == 1000, "occlusion buffer did not keep nearest surface");

    std::vector<uint8_t> bgr(1928*480,239);
    for (int y = 0; y < 480; ++y) for (int x = 0; x < 640; ++x) {
      bgr[y*1928+x*3] = uint8_t(x%251);
      bgr[y*1928+x*3+1] = uint8_t(y%251);
      bgr[y*1928+x*3+2] = 17;
    }
    const auto original = bgr;
    auto rectified = identity.rectify_bgr(bgr.data(),bgr.size(),1928);
    check(bgr == original && rectified.size() == 640*480*3 && rectified[240*1920+320*3] == 320%251 &&
          rectified[240*1920+320*3+1] == 240, "identity remap/stride/channel preservation wrong");
    cal = calibration(); cal.color.model = RS2_DISTORTION_BROWN_CONRADY; cal.color.coeffs[0] = .1F;
    RealsenseGeometry distorted(cal);
    rectified = distorted.rectify_bgr(bgr.data(),bgr.size(),1928);
    // At output (420,240), normalized radius=1; k1=.1 maps to input x=430.
    check(rectified[240*1920+420*3] == 430%251 && rectified[240*1920+420*3+2] == 17,
          "RGB Brown-Conrady rectification direction wrong");
    // The pinned SDK supports projection for the D455's inverse Brown model.
    cal.color.model = RS2_DISTORTION_INVERSE_BROWN_CONRADY;
    RealsenseGeometry inverse_distorted(cal);
    rectified = inverse_distorted.rectify_bgr(bgr.data(),bgr.size(),1928);
    check(rectified[240*1920+420*3] == 430%251 && rectified[240*1920+420*3+2] == 17,
          "RGB inverse Brown-Conrady rectification direction wrong");
    rejects([&] { identity.rectify_bgr(bgr.data(),bgr.size()-1,1928); }, "short RGB storage accepted");
    cal = calibration(); cal.depth_to_color.rotation[0] = 2;
    rejects([&] { RealsenseGeometry bad(cal); }, "nonrigid transform accepted");
    cal = calibration(); cal.color.model = RS2_DISTORTION_FTHETA;
    rejects([&] { RealsenseGeometry bad(cal); }, "unsupported distortion accepted");
    cal = calibration(); cal.depth_unit_m = NAN;
    rejects([&] { RealsenseGeometry bad(cal); }, "invalid depth units accepted");

    const int64_t offset = 1700000000000000000LL;
    const double global_ms = 1700000001000.0;
    check(realsense_motion_time_ns(global_ms,offset) == 1000000000LL, "epoch milliseconds mislabeled monotonic");
    check(realsense_video_time_ns(global_ms,offset,10000,5000) == 995000000LL, "exposure midpoint correction wrong");
    check(realsense_video_time_ns(global_ms,offset,1000,UINT32_MAX-999) == 998000000LL, "microsecond wrap correction wrong");
    rejects([&] { realsense_video_time_ns(global_ms,offset,1000,2000); }, "future exposure correction accepted");
    rejects([&] { realsense_motion_time_ns(NAN,offset); }, "nonfinite global timestamp accepted");
    std::cout << "RealSense geometry/time checks passed (synthetic)\n";
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
