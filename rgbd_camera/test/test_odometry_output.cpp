// Copyright (c) 2026 Paul N. Beuchat. MIT License.
// Real ROS delivery of synthetic products; no device or estimator.
#include "rgbd_camera/odometry_output.hpp"
#include <chrono>
#include <iostream>
#include <thread>
using namespace rgbd_camera;
void check(bool condition, const char* reason) { if (!condition) throw std::runtime_error(reason); }
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rclcpp::Node>("camera_output_fixture");
    Config config; config.fps = 30; config.publish_stereo = config.enable_imu = config.combine_imu = true; config.imu_rate_hz = 200;
    OdometryOutput output(*node, config);
    Identity identity; identity.source_id = "fixture"; identity.generation = "A"; identity.clock_domain = "synthetic";
    identity.unix_offset_ns = 1700000000000000000LL;
    auto extrinsics = std::make_shared<InternalExtrinsics>(); extrinsics->has_imu = true;
    extrinsics->rgb_from_camera[1][3] = .075; identity.extrinsics = extrinsics; output.start(identity);
    std::array<sensor_msgs::msg::Image::ConstSharedPtr, 2> images;
    rgbd_camera_interfaces::msg::StereoCapture::ConstSharedPtr capture;
    sensor_msgs::msg::Imu::ConstSharedPtr imu;
    const auto qos = rclcpp::SensorDataQoS().keep_last(4);
    auto left = node->create_subscription<sensor_msgs::msg::Image>("~/stereo/left/image",qos,[&](sensor_msgs::msg::Image::ConstSharedPtr m){images[0]=m;});
    auto right = node->create_subscription<sensor_msgs::msg::Image>("~/stereo/right/image",qos,[&](sensor_msgs::msg::Image::ConstSharedPtr m){images[1]=m;});
    auto metadata = node->create_subscription<rgbd_camera_interfaces::msg::StereoCapture>("~/stereo/capture",qos,[&](rgbd_camera_interfaces::msg::StereoCapture::ConstSharedPtr m){capture=m;});
    auto combined = node->create_subscription<sensor_msgs::msg::Imu>("~/imu/combined",qos,[&](sensor_msgs::msg::Imu::ConstSharedPtr m){imu=m;});
    const auto acquired = monotonic_ns()-10000000;
    rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
    const auto deadline = monotonic_ns()+3000000000;
    while ((!images[0] || !images[1] || !capture || !imu) && monotonic_ns()<deadline) {
      auto pair = std::make_unique<StereoFrame>(); pair->sequence={10,11}; pair->event_ns={acquired-1000,acquired-900};
      pair->midpoint_ns={acquired,acquired+100}; pair->exposure_ns={2000,2000}; pair->received_ns=acquired+1000000;
      pair->calibration_id="calibration";
      for (size_t i=0;i<2;++i) {pair->cameras[i]={640,400,{500,0,320,0,500,200,0,0,1},std::vector<double>(5,0)};pair->pixels[i].resize(640*400,uint8_t(i+1));}
      output.stereo(std::move(pair));
      output.motion({{MotionKind::Gyroscope,5,acquired,acquired+10,{1,2,3}},{0,0,9.81},acquired-100,acquired+100});
      executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    check(images[0] && images[1] && capture && imu,"missing stereo/IMU outputs");
    check(capture->left.generation=="A" && capture->right.generation=="A" && capture->left.sequence==10 && capture->right.sequence==11,"capture identity lost");
    check(rclcpp::Time(images[0]->header.stamp).nanoseconds()==acquired+identity.unix_offset_ns &&
          rclcpp::Time(images[1]->header.stamp).nanoseconds()==acquired+100+identity.unix_offset_ns &&
          capture->header.stamp==images[1]->header.stamp,"midpoints or later-stamp sidecar key changed");
    check(capture->left.acquisition_ns==acquired-1000 && capture->left.exposure_midpoint_ns==acquired &&
          images[0]->data.front()==1 && images[1]->data.front()==2,"legacy event or image association changed");
    check(imu->orientation_covariance[0]==-1 && imu->linear_acceleration.z==9.81 &&
          imu->angular_velocity.y==2 && rclcpp::Time(imu->header.stamp).nanoseconds()==acquired+identity.unix_offset_ns,"combined IMU semantics lost");
    rclcpp::shutdown(); std::cout << "odometry ROS outputs passed (synthetic)\n"; return 0;
  } catch (const std::exception& e) {std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
}
