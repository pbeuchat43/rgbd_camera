#!/usr/bin/env python3
"""Bounded OAK acceptance. Requires exclusive, explicitly authorized hardware use."""
import argparse
import array
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rgbd_camera_interfaces.msg import Frame, Motion, Status
from sensor_msgs.msg import Image, CameraInfo


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def percentile(values, fraction):
    return sorted(values)[min(len(values) - 1, int(len(values) * fraction))]


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class Observer(Node):
    def __init__(self, name, camera, imu, tolerance_ms, images=False):
        super().__init__(name)
        self.camera, self.imu, self.tolerance_ms = camera, imu, tolerance_ms
        self.frames, self.age_ms, self.skew_ms = [], [], []
        self.accel, self.gyro, self.statuses = [], [], []
        self.depth_sample = None
        self.standard_counts = dict(rgb=0, depth=0, rgb_info=0, depth_info=0)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions_owned = [
            self.create_subscription(Frame, f"/{camera}/frames", self.frame, qos),
            self.create_subscription(Status, f"/{camera}/status", self.statuses.append, qos),
        ]
        if imu:
            for kind in ("accel", "gyro"):
                self.subscriptions_owned.append(self.create_subscription(
                    Motion, f"/{camera}/imu/{kind}_sample", self.motion,
                    QoSProfile(depth=4, reliability=ReliabilityPolicy.BEST_EFFORT)))
        if images:
            for stream in ("rgb", "depth"):
                self.subscriptions_owned.append(self.create_subscription(
                    Image, f"/{camera}/{stream}/image_rect", lambda msg, key=stream: self.standard(msg, key), qos))
                self.subscriptions_owned.append(self.create_subscription(
                    CameraInfo, f"/{camera}/{stream}/camera_info", lambda msg, key=stream + "_info": self.standard(msg, key), qos))

    def standard(self, message, key):
        require(message.width == message.height == 640 and message.header.frame_id == "oak_rgb_optical_frame", "bad standard image geometry")
        if key == "rgb":
            require(message.encoding == "bgr8" and len(message.data) == 640 * 640 * 3, "bad standard RGB")
        elif key == "depth":
            require(message.encoding == "16UC1" and len(message.data) == 640 * 640 * 2, "bad standard depth")
        else:
            require(message.k[0] > 0 and list(message.d) == [0.0] * 5, "bad standard CameraInfo")
        self.standard_counts[key] += 1

    def frame(self, message):
        capture = message.capture
        now = time.monotonic_ns()
        require(capture.schema_version == 1, "unsupported schema")
        require(capture.stream_id == "rgbd", "missing image stream identity")
        require(capture.source_id.startswith("oak:") and capture.generation, "missing device identity")
        require(capture.clock_domain.startswith("linux_monotonic:"), "wrong clock domain")
        require(message.rgb.width == message.rgb.height == 640, "wrong RGB dimensions")
        require(message.depth.width == message.depth.height == 640, "wrong depth dimensions")
        require(message.rgb.encoding == "bgr8" and message.depth.encoding == "16UC1", "wrong formats")
        require(len(message.rgb.data) == 640 * 640 * 3 and len(message.depth.data) == 640 * 640 * 2, "wrong buffer size")
        require(message.rgb_info.header == message.rgb.header and message.depth_info.header == message.depth.header, "geometry not bound")
        require(list(message.rgb_info.k) == list(message.depth_info.k) and message.rgb_info.k[0] > 0, "geometry mismatch")
        require(list(message.rgb_info.d) == [0.0] * 5 and list(message.depth_info.d) == [0.0] * 5, "nonrectified geometry")
        require(len(message.calibration_id) == 64, "missing calibration hash")
        require(message.depth_convention == "optical_z" and message.alignment == "aligned_to_rgb", "ambiguous depth")
        require(message.depth_unit_m == 0.001 and message.invalid_depth_value == 0, "wrong depth semantics")
        require(message.pairing_tolerance_ns == round(self.tolerance_ms * 1e6), "tolerance override lost")
        require(message.skew_ns == message.depth_acquisition_ns - capture.acquisition_ns, "skew mismatch")
        require(abs(message.skew_ns) <= message.pairing_tolerance_ns, "out-of-tolerance pair published")
        require(stamp_ns(message.rgb.header) == capture.acquisition_ns + capture.unix_offset_ns, "RGB timestamp changed")
        require(stamp_ns(message.depth.header) == message.depth_acquisition_ns + capture.unix_offset_ns, "depth timestamp changed")
        require(capture.acquisition_ns <= capture.received_ns <= now, "invalid clock ordering")
        if self.frames:
            previous = self.frames[-1]
            require(capture.generation == previous["generation"], "unexpected session change")
            require(capture.sequence > previous["sequence"] and capture.acquisition_ns > previous["acquisition_ns"], "capture did not advance")
            require(message.calibration_id == previous["calibration_id"], "calibration changed")
        self.frames.append(dict(sequence=capture.sequence, depth_sequence=message.depth_sequence,
                                acquisition_ns=capture.acquisition_ns, received_ns=capture.received_ns,
                                generation=capture.generation, source_id=capture.source_id,
                                calibration_id=message.calibration_id, k=list(message.rgb_info.k)))
        self.age_ms.append((now - capture.acquisition_ns) / 1e6)
        self.skew_ms.append(message.skew_ns / 1e6)
        if self.depth_sample is None or self.depth_sample["valid_fraction"] == 0:
            depth = array.array("H")
            depth.frombytes(bytes(message.depth.data))
            nonzero = [value for value in depth if value]
            self.depth_sample = dict(valid_fraction=len(nonzero) / len(depth),
                                     median_nonzero_m=statistics.median(nonzero) / 1000 if nonzero else None)

    def motion(self, message):
        capture, sample = message.capture, message.sample
        require(message.axes == "sensor_native" and sample.header.frame_id != "oak_rgb_optical_frame", "IMU axes relabeled")
        require(sample.orientation_covariance[0] == -1, "orientation fabricated")
        require(stamp_ns(sample.header) == capture.acquisition_ns + capture.unix_offset_ns, "IMU timestamp changed")
        require(capture.acquisition_ns <= capture.received_ns <= time.monotonic_ns(), "invalid IMU clock ordering")
        if message.kind == "accelerometer":
            require(capture.stream_id == "accel", "missing accel stream identity")
            values = sample.linear_acceleration
            require(sample.angular_velocity_covariance[0] == -1 and sample.linear_acceleration_covariance[0] == 0, "bad accel availability")
            target = self.accel
        else:
            require(message.kind == "gyroscope", "unknown motion kind")
            require(capture.stream_id == "gyro", "missing gyro stream identity")
            values = sample.angular_velocity
            require(sample.linear_acceleration_covariance[0] == -1 and sample.angular_velocity_covariance[0] == 0, "bad gyro availability")
            target = self.gyro
        require(all(math.isfinite(v) for v in (values.x, values.y, values.z)), "nonfinite IMU sample")
        if target:
            require(capture.acquisition_ns > target[-1][0], "IMU sample repeated/regressed")
        target.append((capture.acquisition_ns, capture.sequence, values.x, values.y, values.z, capture.generation))


def proc_sample(pid):
    text = Path(f"/proc/{pid}/stat").read_text().split(")", 1)[1].split()
    cpu, rss = int(text[11]) + int(text[12]), int(text[21]) * os.sysconf("SC_PAGE_SIZE")
    # Tegra's kernel may omit /proc/.../children. Ask ps for this parent's children.
    children = subprocess.run(["ps", "-o", "pid=", "--ppid", str(pid)],
                              capture_output=True, text=True, check=False).stdout.split()
    for child in children:
        try:
            child_cpu, child_rss = proc_sample(int(child))
            cpu += child_cpu
            rss += child_rss
        except FileNotFoundError:
            pass
    return cpu, rss


def stop(process, check_exit=True):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        # No implicit SIGKILL: surface timeout and preserve the process for inspection.
        process.wait(timeout=20)
    if check_exit:
        require(process.returncode in (0, -signal.SIGINT), f"process exited {process.returncode}")


def run_case(args, name, composed=False, imu=False, tolerance=5.0, count=120, slow=False):
    observer = Observer(f"observer_{name}", "camera", imu, tolerance, images=composed)
    executor = SingleThreadedExecutor()
    executor.add_node(observer)
    slow_sequences = []
    slow_process = None
    slow_log = None
    if slow:
        slow_log = (args.output / "slow-consumer.log").open("w")
        slow_process = subprocess.Popen(
            [sys.executable, str(Path(__file__).with_name("slow_consumer.py")),
             "--output", str(args.output / "slow-consumer.json")],
            stdout=slow_log, stderr=subprocess.STDOUT, start_new_session=True)
    command = (["ros2", "launch", "rgbd_camera", "camera.launch.py",
                f"device_id:={args.device}", f"enable_imu:={str(imu).lower()}",
                f"pairing_tolerance_ms:={tolerance}", "publish_images:=true"] if composed else
               [args.executable, "--ros-args", "-r", "__node:=camera", "-p", f"device_id:='{args.device}'",
                "-p", f"enable_imu:={str(imu).lower()}", "-p", f"pairing_tolerance_ms:={tolerance}"])
    log = (args.output / f"{name}.log").open("w")
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    baseline = None
    try:
        deadline = time.monotonic() + 60
        while len(observer.frames) < count and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            require(process.poll() is None, f"{name} process ended early; inspect log")
            if observer.statuses and observer.statuses[-1].state == "error":
                raise AssertionError(f"camera error: {observer.statuses[-1].detail}")
            if observer.frames and baseline is None:
                baseline = (time.monotonic(), proc_sample(process.pid))
        require(len(observer.frames) >= count, f"only {len(observer.frames)} frames received")
        require(observer.statuses and observer.statuses[-1].state == "streaming", "provider not ready")
        require(observer.depth_sample["valid_fraction"] > 0, "no observed valid depth")
        if imu:
            require(len(observer.accel) > count and len(observer.gyro) > count, "IMU not independent of RGB rate")
            generation = observer.frames[0]["generation"]
            require(all(s[-1] == generation for s in observer.accel + observer.gyro), "IMU session mismatch")
        else:
            require(not any(topic.startswith("/camera/imu/") for topic, _ in observer.get_topic_names_and_types()), "disabled IMU topics exist")
        if slow:
            stop(slow_process)
            slow_sequences = json.loads((args.output / "slow-consumer.json").read_text())["sequences"]
            require(len(observer.frames) > 2 * len(slow_sequences), "slow consumer stalled fast consumer")
            require(any(b - a > 1 for a, b in zip(slow_sequences, slow_sequences[1:])), "slow consumer did not drop frames")
        if composed:
            require(all(value >= 30 for value in observer.standard_counts.values()), "standard image branch incomplete")
        end_time, end_sample = time.monotonic(), proc_sample(process.pid)
        status = observer.statuses[-1]
        result = dict(name=name, layout="composed" if composed else "standalone", imu=imu,
                      tolerance_ms=tolerance, frames=len(observer.frames), first=observer.frames[0], last=observer.frames[-1],
                      frame_rate_hz=(len(observer.frames) - 1) * 1e9 / (observer.frames[-1]["acquisition_ns"] - observer.frames[0]["acquisition_ns"]),
                      source_sequence_rate_hz=(observer.frames[-1]["sequence"] - observer.frames[0]["sequence"]) * 1e9 / (observer.frames[-1]["acquisition_ns"] - observer.frames[0]["acquisition_ns"]),
                      subscriber_age_ms=dict(p50=statistics.median(observer.age_ms), p95=percentile(observer.age_ms, .95), max=max(observer.age_ms)),
                      skew_ms=dict(min=min(observer.skew_ms), max=max(observer.skew_ms)), depth_sample=observer.depth_sample,
                      slow_consumer_count=len(slow_sequences), slow_consumer_sequences=slow_sequences,
                      standard_topic_counts=observer.standard_counts,
                      accel_count=len(observer.accel), gyro_count=len(observer.gyro),
                      accel_rate_hz=(len(observer.accel) - 1) * 1e9 / (observer.accel[-1][0] - observer.accel[0][0]) if imu else None,
                      gyro_rate_hz=(len(observer.gyro) - 1) * 1e9 / (observer.gyro[-1][0] - observer.gyro[0][0]) if imu else None,
                      accel_first=observer.accel[0] if imu else None, gyro_first=observer.gyro[0] if imu else None,
                      state=dict(usb_speed=status.usb_speed, imu_sensor=status.imu_sensor, frames=status.frames,
                                 pairs_received=status.pairs_received, sequence_gaps=status.sequence_gaps,
                                 rejected_pairs=status.rejected_pairs, imu_sequence_gaps=status.imu_sequence_gaps,
                                 imu_state=status.imu_state),
                      process_sample=dict(cpu_percent_one_core=(end_sample[0] - baseline[1][0]) / os.sysconf("SC_CLK_TCK") / (end_time - baseline[0]) * 100,
                                          rss_bytes=end_sample[1], initial_rss_bytes=baseline[1][1],
                                          note="owned process tree, including launcher/container where present; excludes verification consumers"))
        # Same physical camera while the first owner is active; never stop the first.
        if name == "standalone":
            frames_before_conflict = len(observer.frames)
            conflict = Observer("conflict_observer", "conflict", False, 5.0)
            executor.add_node(conflict)
            with (args.output / "conflict.log").open("w") as conflict_log:
                contender = subprocess.Popen([args.executable, "--ros-args", "-r", "__node:=conflict", "-p", f"device_id:='{args.device}'"],
                                             stdout=conflict_log, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    until = time.monotonic() + 12
                    while not any(s.state == "error" for s in conflict.statuses) and time.monotonic() < until:
                        executor.spin_once(timeout_sec=0.05)
                    require(any(s.state == "error" for s in conflict.statuses) and not conflict.frames, "second owner did not fail")
                    result["owner_conflict"] = conflict.statuses[-1].detail
                    require(len(observer.frames) > frames_before_conflict and process.poll() is None, "contender disturbed first owner")
                finally:
                    stop(contender)
            executor.remove_node(conflict); conflict.destroy_node()
        return result
    finally:
        if sys.exc_info()[0] is not None:
            (args.output / f"{name}-partial.json").write_text(json.dumps(dict(
                frame_count=len(observer.frames), first=observer.frames[0] if observer.frames else None,
                last=observer.frames[-1] if observer.frames else None,
                age_ms=observer.age_ms, skew_ms=observer.skew_ms,
                accel_count=len(observer.accel), gyro_count=len(observer.gyro),
                status=[dict(state=s.state, detail=s.detail, frames=s.frames, rejected=s.rejected_pairs)
                        for s in observer.statuses[-10:]]), indent=2) + "\n")
        stop(process, check_exit=sys.exc_info()[0] is None)
        log.close()
        executor.shutdown(timeout_sec=5)
        observer.destroy_node()
        if slow_process:
            stop(slow_process, check_exit=sys.exc_info()[0] is None)
            slow_log.close()


def invalid_selection(args):
    observer = Observer("invalid_observer", "invalid_camera", False, 5.0)
    executor = SingleThreadedExecutor()
    executor.add_node(observer)
    with (args.output / "invalid-selection.log").open("w") as log:
        process = subprocess.Popen([args.executable, "--ros-args", "-r", "__node:=invalid_camera",
                                    "-p", "device_id:=not_a_real_camera"],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 12
            while not observer.statuses and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            while observer.statuses and observer.statuses[-1].state != "error" and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            require(observer.statuses and observer.statuses[-1].state == "error" and not observer.frames, "invalid selection did not fail closed")
            return observer.statuses[-1].detail
        finally:
            stop(process)
            executor.shutdown(timeout_sec=5)
            observer.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", required=True)
    parser.add_argument("--executable", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--case", choices=["all", "standalone", "imu", "composed"], default="all")
    args = parser.parse_args()
    require(os.environ.get("ROS_DOMAIN_ID") == "229", "use isolated ROS domain 229")
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    results = []
    try:
        if args.case in ("all", "standalone"):
            results.append(run_case(args, "standalone", slow=True))
        if args.case in ("all", "imu"):
            results.append(run_case(args, "imu", imu=True, tolerance=8.0))
        if args.case in ("all", "composed"):
            results.append(run_case(args, "composed", composed=True, imu=True))
        require(len({r["first"]["generation"] for r in results}) == len(results), "restart reused session identity")
        failure_detail = invalid_selection(args)
        (args.output / "report.json").write_text(json.dumps(dict(passed=True, cases=results, invalid_selection=failure_detail), indent=2) + "\n")
        print(json.dumps(dict(passed=True, cases=[r["name"] for r in results])))
    except BaseException as error:
        (args.output / "report.json").write_text(json.dumps(dict(passed=False, error=str(error), completed=results), indent=2) + "\n")
        raise
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
