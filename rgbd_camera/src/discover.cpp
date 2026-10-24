#include "rgbd_camera/provider.hpp"
#include <iostream>
int main(int argc, char** argv) {
  try {
    const std::string backend = argc == 1 ? "oak" :
      argc == 3 && std::string(argv[1]) == "--backend" ? argv[2] : "";
    if (backend.empty()) throw std::runtime_error("usage: camera_discover [--backend oak|realsense]");
    for (const auto& device : rgbd_camera::discover(backend))
      std::cout << device.id << '\t' << device.state << '\t' << device.protocol << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
