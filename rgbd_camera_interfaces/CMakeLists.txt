cmake_minimum_required(VERSION 3.16)
project(rgbd_camera_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CaptureReference.msg" "msg/Frame.msg" "msg/Motion.msg" "msg/Status.msg"
  DEPENDENCIES sensor_msgs std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
