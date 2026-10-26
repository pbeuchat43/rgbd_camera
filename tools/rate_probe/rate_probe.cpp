// Copyright (c) 2026 Paul N. Beuchat. MIT License.
// Lightweight external observer: metadata logging only, no inference or image work.
#include <rclcpp/rclcpp.hpp>
#include <rgbd_camera_interfaces/msg/frame.hpp>
#include <rgbd_camera_interfaces/msg/status.hpp>
#include <rgbd_camera_interfaces/msg/motion.hpp>
#include <rgbd_object_detector_interfaces/msg/batch.hpp>
#include <rgbd_object_detector_interfaces/msg/status.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <filesystem>
#include <map>
#include <vector>

using Frame = rgbd_camera_interfaces::msg::Frame;
using CameraStatus = rgbd_camera_interfaces::msg::Status;
using Batch = rgbd_object_detector_interfaces::msg::Batch;
using DetectorStatus = rgbd_object_detector_interfaces::msg::Status;
using Motion = rgbd_camera_interfaces::msg::Motion;
int64_t now_ns() { return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count(); }
int64_t stamp_ns(const std_msgs::msg::Header& header) {
  return int64_t(header.stamp.sec) * 1000000000 + header.stamp.nanosec;
}
class Probe : public rclcpp::Node {
 public:
  Probe() : Node("rate_probe") {
    const auto directory = declare_parameter<std::string>("output", "/tmp/rgbd-rate-probe");
    const auto depth = declare_parameter<int>("depth", 1);
    if (depth < 1 || depth > 16) throw std::runtime_error("probe depth must be within [1, 16]");
    const auto images = declare_parameter<bool>("images", false);
    const auto imu = declare_parameter<bool>("imu", false);
    if (std::filesystem::exists(directory)) throw std::runtime_error("probe output directory already exists");
    std::filesystem::create_directories(directory);
    frames_.open(directory+"/frames.csv"); batches_.open(directory+"/batches.csv");
    camera_.open(directory+"/camera.csv"); detector_.open(directory+"/detector.csv");
    if (!frames_ || !batches_ || !camera_ || !detector_) throw std::runtime_error("cannot open probe output files");
    frames_ << "arrival_ns,sequence,acquisition_ns,depth_acquisition_ns,received_ns,generation,bytes,skew_ns,rgb_stamp_ns,depth_stamp_ns,width,height,rgb_bytes,depth_bytes\n";
    batches_ << "arrival_ns,sequence,acquisition_ns,started_ns,completed_ns,source_age_ns,preprocess_ns,inference_ns,postprocess_ns,valid,invalid,generation\n";
    camera_ << "arrival_ns,observed_ns,frames,pairs,rejected,gaps,last_acquisition_ns,state,imu_state,last_accel_ns,last_gyro_ns,accel_samples,gyro_samples,imu_sequence_gaps,last_skew_ns,last_pair_received_ns,detail\n";
    detector_ << "arrival_ns,observed_ns,received,processed,rejected,replaced,stale,superseded,failures,state,last_input_acquisition_ns,last_processed_acquisition_ns\n";
    observer_.open(directory+"/observer.csv");
    if (!observer_) throw std::runtime_error("cannot open observer timing file");
    observer_ << "started_ns,completed_ns,flush_duration_ns,frames,batches\n";
    const auto qos = rclcpp::SensorDataQoS().keep_last(depth);
    frame_sub_ = create_subscription<Frame>("/sweep/camera/frames", qos, [this](Frame::ConstSharedPtr f) {
      const auto t = now_ns(); ++frame_count_;
      frames_ << t << ',' << f->capture.sequence << ',' << f->capture.acquisition_ns << ','
        << f->depth_acquisition_ns << ',' << f->capture.received_ns << ',' << f->capture.generation << ','
        << f->rgb.data.size()+f->depth.data.size() << ',' << f->skew_ns << ','
        << stamp_ns(f->rgb.header) << ',' << stamp_ns(f->depth.header) << ','
        << f->rgb.width << ',' << f->rgb.height << ',' << f->rgb.data.size() << ',' << f->depth.data.size() << '\n';
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
        << s->rejected_pairs << ',' << s->sequence_gaps << ',' << s->last_acquisition_ns << ',' << s->state << ','
        << s->imu_state << ',' << s->last_accel_ns << ',' << s->last_gyro_ns << ','
        << s->accel_samples << ',' << s->gyro_samples << ',' << s->imu_sequence_gaps << ','
        << s->last_skew_ns << ',' << s->last_pair_received_ns << ',' << std::quoted(s->detail, '"', '"') << '\n';
      if (s->state == "error") std::cerr << "CAMERA_ERROR " << s->detail << std::endl;
    });
    detector_sub_ = create_subscription<DetectorStatus>("/sweep/detector/status", qos, [this](DetectorStatus::ConstSharedPtr s) {
      detector_ << now_ns() << ',' << s->observed_ns << ',' << s->received << ',' << s->processed << ',' << s->rejected << ','
        << s->replaced << ',' << s->stale << ',' << s->superseded << ',' << s->failures << ',' << s->state << ','
        << s->last_input_acquisition_ns << ',' << s->last_processed_acquisition_ns << '\n';
      if (s->state == "error") std::cerr << "DETECTOR_ERROR " << s->detail << std::endl;
    });
    const auto open_extra = [&](const std::string& name, const std::string& header) {
      auto& file = extra_files_.try_emplace(name, directory+"/"+name+".csv").first->second;
      if (!file) throw std::runtime_error("cannot open " + name + " output file");
      file << header << '\n';
    };
    if (images) {
      for (const std::string stream : {"rgb", "depth"}) {
        open_extra(stream, "arrival_ns,stamp_ns,width,height,step,bytes,encoding");
        image_subs_.push_back(create_subscription<sensor_msgs::msg::Image>(
          "/sweep/camera/"+stream+"/image_rect", qos,
          [this, stream](sensor_msgs::msg::Image::ConstSharedPtr m) {
            extra_files_.at(stream) << now_ns() << ',' << stamp_ns(m->header) << ','
              << m->width << ',' << m->height << ',' << m->step << ',' << m->data.size() << ',' << m->encoding << '\n';
          }));
        const auto info = stream+"_info";
        open_extra(info, "arrival_ns,stamp_ns,width,height");
        info_subs_.push_back(create_subscription<sensor_msgs::msg::CameraInfo>(
          "/sweep/camera/"+stream+"/camera_info", qos,
          [this, info](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
            extra_files_.at(info) << now_ns() << ',' << stamp_ns(m->header) << ',' << m->width << ',' << m->height << '\n';
          }));
      }
    }
    if (imu) {
      const auto motion_qos = rclcpp::SensorDataQoS().keep_last(4);
      for (const std::string sensor : {"accel", "gyro"}) {
        open_extra(sensor, "arrival_ns,sequence,acquisition_ns,received_ns,stamp_ns,sensor_sequence,future_tolerance_ns,generation");
        motion_subs_.push_back(create_subscription<Motion>(
          "/sweep/camera/imu/"+sensor+"_sample", motion_qos, [this, sensor](Motion::ConstSharedPtr m) {
            extra_files_.at(sensor) << now_ns() << ',' << m->capture.sequence << ',' << m->capture.acquisition_ns << ','
              << m->capture.received_ns << ',' << stamp_ns(m->sample.header) << ',' << m->sensor_sequence << ','
              << m->future_tolerance_ns << ',' << m->capture.generation << '\n';
          }));
        const auto standard = sensor+"_standard";
        open_extra(standard, "arrival_ns,stamp_ns");
        imu_subs_.push_back(create_subscription<sensor_msgs::msg::Imu>(
          "/sweep/camera/imu/"+sensor, motion_qos, [this, standard](sensor_msgs::msg::Imu::ConstSharedPtr m) {
            extra_files_.at(standard) << now_ns() << ',' << stamp_ns(m->header) << '\n';
          }));
      }
    }
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
      const auto started = now_ns();
      frames_.flush(); batches_.flush(); camera_.flush(); detector_.flush();
      for (auto& entry : extra_files_) entry.second.flush();
      const auto completed = now_ns();
      observer_ << started << ',' << completed << ',' << completed-started << ',' << frame_count_ << ',' << batch_count_ << '\n';
      std::cout << "frames=" << frame_count_ << " batches=" << batch_count_ << std::endl;
    });
  }
 private:
  uint64_t frame_count_ = 0, batch_count_ = 0;
  std::ofstream frames_, batches_, camera_, detector_, observer_;
  std::map<std::string, std::ofstream> extra_files_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> info_subs_;
  std::vector<rclcpp::Subscription<Motion>::SharedPtr> motion_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> imu_subs_;
  rclcpp::Subscription<Frame>::SharedPtr frame_sub_;
  rclcpp::Subscription<Batch>::SharedPtr batch_sub_;
  rclcpp::Subscription<CameraStatus>::SharedPtr camera_sub_;
  rclcpp::Subscription<DetectorStatus>::SharedPtr detector_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) { rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<Probe>()); rclcpp::shutdown(); }
