#!/usr/bin/env python3
"""Independent slow ROS reader for bounded fan-out acceptance; never opens hardware."""
import argparse
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rgbd_camera_interfaces.msg import Frame


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node("independent_slow_observer")
    sequences, ages_ms = [], []
    def consume(message):
        sequences.append(message.capture.sequence)
        ages_ms.append((time.monotonic_ns() - message.capture.acquisition_ns) / 1e6)
        time.sleep(0.35)
    node.create_subscription(Frame, "/camera/frames", consume,
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        args.output.write_text(json.dumps(dict(sequences=sequences, age_ms=ages_ms), indent=2) + "\n")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
