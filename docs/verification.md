# Build and verification

Execution checks run on the designated reserved host; edit/Git operations may run locally.
Use an isolated source/build/install prefix and a private ROS domain. Do not
replace an existing deployment, source its application overlays, alter power settings,
or start unrelated workloads. Hold the shared verification reservation while
using the camera. RealSense and vehicle hardware are outside these checks.

## Dependencies and build

Required: Ubuntu 24.04 arm64, ROS 2 Jazzy (`rclcpp`, `rclcpp_components`,
`sensor_msgs`, `rosidl_default_generators`, `ament_cmake`), C++17, CMake ≥3.20,
OpenSSL development headers, colcon, and DepthAI **3.9.0 exactly**.
No Python detector, TensorRT, CUDA or model weights are needed.

The standalone interface package builds with only ROS dependencies:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rgbd_camera_interfaces
```

Build DepthAI at `ac928141ef7750bf3082b5e1c319a115618660e7` in a separate prefix.
`tools/build_sdk.sh PREFIX` records the minimal SDK configuration used by this
slice. It fetches dependencies into its build directory and installs only to
PREFIX. It does not install apt packages or change the system SDK. The upstream
build may use the ordinary user's vcpkg download/binary cache.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --executor sequential --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/absolute/sdk/prefix
source install/setup.bash
export LD_LIBRARY_PATH=/absolute/sdk/prefix/lib:${LD_LIBRARY_PATH:-}
export ROS_DOMAIN_ID=229 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
colcon test --packages-select rgbd_camera --return-code-on-test-failure --event-handlers console_direct+
colcon test-result --verbose
```

The core tests exercise malformed geometry/encoding bounds, stale and skewed
pairs, recovery after rejection, restart-local gate state, independent IMU timing,
deduplication and calibration identity. The adapter test uses an injected synthetic
provider in a real intra-process ROS graph; it checks read-only parameters,
association fields, IMU unavailable fields, freshness, skew recovery, and simulated
device loss/cleanup. It does not establish hardware behavior.

## Running explicitly

```bash
ros2 run rgbd_camera camera_discover
ros2 run rgbd_camera camera_node --ros-args -r __node:=camera \
  -p "device_id:='YOUR_DEVICE_ID'" -p enable_imu:=true
```

Or explicitly load the component with intra-process communication enabled:

```bash
ros2 launch rgbd_camera camera.launch.py device_id:=YOUR_DEVICE_ID enable_imu:=true
```

For a debugging run, pass a permitted `pairing_tolerance_ms` startup override.
Record it with actual skew and rejected counts. A larger tolerance is not evidence
of accurate RGB/depth association. Preserve comparison runs at the default.

`tools/verify_hardware.py` drives bounded standalone/composed acceptance using
existing installed binaries. It writes a JSON report, preserves camera generations,
checks owner conflicts and graceful stops, and observes optional IMU output.
Run only with explicit hardware scope and no other camera owner. Inspect reports
and node logs before claiming acceptance. Set `ROS_LOG_DIR` to the isolated test
directory. The slow reader runs in a separate process to avoid shared Python
executor interference. Full-rate lossless fan-out is not promised by this profile.

The [24 October verification summary](evidence/2026-10-24-oak-camera/README.md)
describes the checks and limitations. Detailed host/device identities, installed
artifact hashes, measurements, and preservation audits remain in the local
development archive; they are not part of this public repository.
