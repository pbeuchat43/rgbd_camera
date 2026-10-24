#include "rgbd_camera/provider.hpp"
#include <iostream>
int main() {
  try {
    for (const auto& device : rgbd_camera::discover_oak())
      std::cout << device.id << '\t' << device.state << '\t' << device.protocol << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
