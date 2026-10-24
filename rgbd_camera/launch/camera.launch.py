"""Explicit camera start; composition enables intra-process delivery."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {"backend": "oak", "device_id": "", "fps": "10.0", "enable_imu": "false",
                "publish_images": "false", "pairing_tolerance_ms": "5.0"}
    types = {"backend": str, "device_id": str, "fps": float, "enable_imu": bool,
             "publish_images": bool, "pairing_tolerance_ms": float}
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        ComposableNodeContainer(
            name="perception", namespace="", package="rclcpp_components",
            executable="component_container", output="screen",
            composable_node_descriptions=[ComposableNode(
                package="rgbd_camera", plugin="rgbd_camera::CameraNode", name="camera",
                parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=types[name])
                             for name in defaults}],
                extra_arguments=[{"use_intra_process_comms": True}],
            )],
        ),
    ])
