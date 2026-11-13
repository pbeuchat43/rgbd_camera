// Copyright (c) 2026 Paul N. Beuchat. MIT License.
// Synthetic live continuation; no device or estimator accuracy claim.
#include "rgbd_camera/camera_node.hpp"
#include <rgbd_camera_interfaces/msg/frame.hpp>
#include <rgbd_camera_interfaces/msg/status.hpp>
#include <atomic>
#include <chrono>
#include <iostream>
#include <set>
#include <thread>
using namespace rgbd_camera;
void check(bool ok,const char* message) {if(!ok)throw std::runtime_error(message);}
class RecoveryFixture final : public Provider {
 public:
  explicit RecoveryFixture(std::shared_ptr<std::atomic<int>> lifecycle): lifecycle_(lifecycle) {}
  Identity start(const Config&) override {
    ++*lifecycle_; Identity i; i.source_id="synthetic:recovery";i.generation="initial";
    i.clock_domain="synthetic";i.unix_offset_ns=1700000000000000000LL;i.imu_future_tolerance_ns=5000000;return i;
  }
  Products poll() override {
    Products p; const auto now=monotonic_ns(); if(now-last_<20000000)return p;last_=now;++seq_;
    const auto raw=now-100000000;
    const auto sdk=raw+(seq_>=60?30000000:0);
    p.frame=std::make_unique<Frame>();auto& f=*p.frame;
    f.sequence=f.depth_sequence=seq_;f.acquisition_ns=f.depth_acquisition_ns=sdk;f.received_ns=now;
    f.clocks={{{raw,sdk},{raw,sdk}}};f.geometry.k={500,0,320,0,500,320,0,0,1};f.geometry.calibration_id="fixture";
    f.bgr.resize(640*640*3);f.depth_le.resize(640*640*2);
    // Same clock mapping, one report estimated 6 ms beyond its receipt.
    p.motion.push_back({MotionKind::Accelerometer,seq_,sdk,seq_==30?sdk-6000000:now,{0,0,9.81},{raw,sdk}});
    p.motion.push_back({MotionKind::Gyroscope,seq_,sdk,now,{0,0,0},{raw,sdk}});
    return p;
  }
  void stop() override {++*lifecycle_;}
 private:
  std::shared_ptr<std::atomic<int>> lifecycle_; uint64_t seq_=0;int64_t last_=0;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    auto lifecycle=std::make_shared<std::atomic<int>>(0);
    auto options=rclcpp::NodeOptions().use_intra_process_comms(true).parameter_overrides({{"enable_imu",true}});
    auto camera=std::make_shared<CameraNode>(options,std::make_unique<RecoveryFixture>(lifecycle));
    auto observer=std::make_shared<rclcpp::Node>("recovery_observer",rclcpp::NodeOptions().use_intra_process_comms(true));
    bool held=false,drop=false,resumed=false;uint64_t frames_after=0,last_drops=0;std::set<std::string> generations;
    auto status=observer->create_subscription<rgbd_camera_interfaces::msg::Status>("/rgbd_camera/status",rclcpp::SensorDataQoS(),
      [&](rgbd_camera_interfaces::msg::Status::ConstSharedPtr m) {
        check(m->state!="error","camera terminated during recoverable clock fault");
        check(m->imu_clock_rejected>=last_drops,"cumulative drop count reset");last_drops=m->imu_clock_rejected;
        drop|=m->imu_clock_rejected>0;
        held|=m->clock_state=="recovering" && m->state=="unavailable";
        resumed|=held && m->clock_state=="stable" && m->state=="streaming" && m->clock_discontinuities==1;
      });
    auto frames=observer->create_subscription<rgbd_camera_interfaces::msg::Frame>("/rgbd_camera/frames",rclcpp::SensorDataQoS(),
      [&](rgbd_camera_interfaces::msg::Frame::ConstSharedPtr m) {
        generations.insert(m->capture.generation);
        if(m->capture.generation!="initial")++frames_after;
        else check(frames_after==0,"old generation published after recovery");
      });
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(camera);executor.add_node(observer);
    const auto deadline=monotonic_ns()+8000000000LL;
    while(monotonic_ns()<deadline && !(resumed && frames_after>10)) {executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}
    check(drop && held && resumed && generations.size()==2 && frames_after>10,"missing drop/hold/new-generation recovery");
    check(*lifecycle==1,"provider reopened or stopped during recovery");
    executor.remove_node(camera);camera.reset();check(*lifecycle==2,"ordinary shutdown did not stop provider");
    rclcpp::shutdown();std::cout<<"camera recovery continuation passed\n";return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
}
