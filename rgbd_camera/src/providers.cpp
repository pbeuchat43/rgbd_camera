// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#include "rgbd_camera/provider.hpp"
#include <stdexcept>

namespace rgbd_camera {
std::vector<DeviceDescriptor> discover(const std::string& backend) {
#ifdef RGBD_CAMERA_HAS_OAK
  if (backend == "oak") return discover_oak();
#endif
#ifdef RGBD_CAMERA_HAS_REALSENSE
  if (backend == "realsense") return discover_realsense();
#endif
  throw std::runtime_error("camera backend unavailable in this build: " + backend);
}
std::unique_ptr<Provider> make_provider(const std::string& backend) {
#ifdef RGBD_CAMERA_HAS_OAK
  if (backend == "oak") return make_oak_provider();
#endif
#ifdef RGBD_CAMERA_HAS_REALSENSE
  if (backend == "realsense") return make_realsense_provider();
#endif
  throw std::runtime_error("camera backend unavailable in this build: " + backend);
}
}  // namespace rgbd_camera
