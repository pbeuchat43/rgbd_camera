// Copyright (c) 2026 Paul N. Beuchat. MIT License.
// Lightweight external observer: metadata logging only, no inference or image work.
#include <rclcpp/rclcpp.hpp>
#include <rgbd_camera_interfaces/msg/frame.hpp>
#include <rgbd_camera_interfaces/msg/status.hpp>
#include <rgbd_object_detector_interfaces/msg/batch.hpp>
#include <rgbd_object_detector_interfaces/msg/status.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <filesystem>

using Frame = rgbd_camera_interfaces::msg::Frame;
using CameraStatus = rgbd_camera_interfaces::msg::Status;
using Batch = rgbd_object_detector_interfaces::msg::Batch;
using DetectorStatus = rgbd_object_detector_interfaces::msg::Status;
int64_t now_ns() { return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count(); }
class Probe : public rclcpp::Node {
 public:
  Probe() : Node("rate_probe") {
    const auto directory = declare_parameter<std::string>("output", "/tmp/rgbd-rate-probe");
    const auto depth = declare_parameter<int>("depth", 1);
    if (std::filesystem::exists(directory)) throw std::runtime_error("probe output directory already exists");
    std::filesystem::create_directories(directory);
    frames_.open(directory+"/frames.csv"); batches_.open(directory+"/batches.csv");
    camera_.open(directory+"/camera.csv"); detector_.open(directory+"/detector.csv");
    if (!frames_ || !batches_ || !camera_ || !detector_) throw std::runtime_error("cannot open probe output files");
    frames_ << "arrival_ns,sequence,acquisition_ns,depth_acquisition_ns,received_ns,generation,bytes,skew_ns\n";
    batches_ << "arrival_ns,sequence,acquisition_ns,started_ns,completed_ns,source_age_ns,preprocess_ns,inference_ns,postprocess_ns,valid,invalid,generation\n";
    camera_ << "arrival_ns,observed_ns,frames,pairs,rejected,gaps,last_acquisition_ns,state\n";
    detector_ << "arrival_ns,observed_ns,received,processed,rejected,replaced,stale,superseded,failures,state\n";
    const auto qos = rclcpp::SensorDataQoS().keep_last(depth);
    frame_sub_ = create_subscription<Frame>("/sweep/camera/frames", qos, [this](Frame::ConstSharedPtr f) {
      const auto t = now_ns(); ++frame_count_;
      frames_ << t << ',' << f->capture.sequence << ',' << f->capture.acquisition_ns << ','
        << f->depth_acquisition_ns << ',' << f->capture.received_ns << ',' << f->capture.generation << ','
        << f->rgb.data.size()+f->depth.data.size() << ',' << f->skew_ns << '\n';
    });
    batch_sub_ = create_subscription<Batch>("/sweep/detector/batches", qos, [this](Batch::ConstSharedPtr b) {
      const auto t = now_ns(); ++batch_count_; size_t valid = 0;
      for (const auto& p : b->spatial) valid += p.valid;
      batches_ << t << ',' << b->capture.sequence << ',' << b->capture.acquisition_ns << ',' << b->started_ns << ','
        << b->completed_ns << ',' << b->source_age_ns << ',' << b->preprocess_ns << ',' << b->inference_ns << ','
        << b->postprocess_ns << ',' << valid << ',' << b->spatial.size()-valid << ',' << b->capture.generation << '\n';
    });
    camera_sub_ = create_subscription<CameraStatus>("/sweep/camera/status", qos, [this](CameraStatus::ConstSharedPtr s) {
      camera_ << now_ns() << ',' << s->observed_ns << ',' << s->frames << ',' << s->pairs_received << ','
        << s->rejected_pairs << ',' << s->sequence_gaps << ',' << s->last_acquisition_ns << ',' << s->state << '\n';
      if (s->state == "error") std::cerr << "CAMERA_ERROR " << s->detail << std::endl;
    });
    detector_sub_ = create_subscription<DetectorStatus>("/sweep/detector/status", qos, [this](DetectorStatus::ConstSharedPtr s) {
      detector_ << now_ns() << ',' << s->observed_ns << ',' << s->received << ',' << s->processed << ',' << s->rejected << ','
        << s->replaced << ',' << s->stale << ',' << s->superseded << ',' << s->failures << ',' << s->state << '\n';
      if (s->state == "error") std::cerr << "DETECTOR_ERROR " << s->detail << std::endl;
    });
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
      frames_.flush(); batches_.flush(); camera_.flush(); detector_.flush();
      std::cout << "frames=" << frame_count_ << " batches=" << batch_count_ << std::endl;
    });
  }
 private:
  uint64_t frame_count_ = 0, batch_count_ = 0;
  std::ofstream frames_, batches_, camera_, detector_;
  rclcpp::Subscription<Frame>::SharedPtr frame_sub_;
  rclcpp::Subscription<Batch>::SharedPtr batch_sub_;
  rclcpp::Subscription<CameraStatus>::SharedPtr camera_sub_;
  rclcpp::Subscription<DetectorStatus>::SharedPtr detector_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) { rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<Probe>()); rclcpp::shutdown(); }
