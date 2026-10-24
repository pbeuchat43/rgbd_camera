cmake_minimum_required(VERSION 3.20)
project(rgbd_camera VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rgbd_camera_interfaces REQUIRED)
find_package(depthai 3.9.0 EXACT CONFIG REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)

add_library(rgbd_camera_core SHARED src/core.cpp)
target_include_directories(rgbd_camera_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(rgbd_camera_core PRIVATE OpenSSL::Crypto)

add_library(rgbd_camera_oak SHARED src/oak_provider.cpp)
target_link_libraries(rgbd_camera_oak PUBLIC rgbd_camera_core PRIVATE depthai::core Threads::Threads)

add_library(rgbd_camera_component SHARED src/camera_node.cpp)
target_link_libraries(rgbd_camera_component PRIVATE rgbd_camera_oak)
ament_target_dependencies(rgbd_camera_component PUBLIC rclcpp rclcpp_components
  rcl_interfaces sensor_msgs rgbd_camera_interfaces)
rclcpp_components_register_node(rgbd_camera_component
  PLUGIN "rgbd_camera::CameraNode" EXECUTABLE camera_node)

add_executable(camera_discover src/discover.cpp)
target_link_libraries(camera_discover PRIVATE rgbd_camera_oak)

install(TARGETS rgbd_camera_core rgbd_camera_oak rgbd_camera_component camera_discover
  EXPORT export_rgbd_camera ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  add_executable(test_core test/test_core.cpp)
  target_link_libraries(test_core PRIVATE rgbd_camera_core)
  add_test(NAME camera_contract COMMAND test_core)
  add_executable(test_node test/test_node.cpp)
  target_link_libraries(test_node PRIVATE rgbd_camera_component rgbd_camera_core)
  ament_target_dependencies(test_node PUBLIC rclcpp rgbd_camera_interfaces)
  add_test(NAME camera_adapter COMMAND test_node)
  set_tests_properties(camera_adapter PROPERTIES TIMEOUT 20)
endif()
ament_export_targets(export_rgbd_camera HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs rgbd_camera_interfaces)
ament_package()
