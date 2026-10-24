# Build and verification

Execution checks run on the designated reserved host; edit/Git operations may run locally.
Use an isolated source/build/install prefix and a private ROS domain. Do not
replace an existing deployment, source its application overlays, alter power settings,
or start unrelated workloads. Hold the shared verification reservation while
using the selected camera. Vehicle actuation is outside these checks.

## Dependencies and build

Required: Ubuntu 24.04 arm64, ROS 2 Jazzy (`rclcpp`, `rclcpp_components`,
`sensor_msgs`, `rosidl_default_generators`, `ament_cmake`), C++17, CMake ≥3.20,
OpenSSL development headers and colcon. OAK builds require DepthAI **3.9.0
exactly**. RealSense builds require librealsense **2.56.3 exactly** plus OpenCV
core/imgproc. At least one backend must be enabled.
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

`RGBD_CAMERA_ENABLE_OAK=ON` and `RGBD_CAMERA_ENABLE_REALSENSE=OFF` are the
defaults. For D455, `tools/build_realsense_sdk.sh PREFIX` builds official commit
`05d6eb7ec8efeed4d69b4025d9ff149e7132acd5` using the userspace RSUSB backend,
without CUDA, DDS, firmware download, system installation or kernel changes.
An existing libusb development package and Ninja are required. Camera USB access
must already be granted to the execution account. The build does not install
udev rules or upgrade camera firmware.

The D455 profile explicitly requires firmware **5.16.0.1**. This is the
[Isaac ROS 4.6 prescribed SDK/firmware pair](https://nvidia-isaac-ros.github.io/v/release-4.6/getting_started/sensors/realsense_setup.html).
It does not establish Isaac ROS deployment compatibility on this host. Other
firmware versions are rejected before streaming; upgrades are separate device
operations and are never performed by the provider or SDK build script.

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

Enable both backends with the two independent SDK prefixes:

```bash
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DRGBD_CAMERA_ENABLE_REALSENSE=ON \
  '-DCMAKE_PREFIX_PATH=/absolute/depthai/prefix;/absolute/realsense/prefix'
export LD_LIBRARY_PATH=/absolute/depthai/prefix/lib:/absolute/realsense/prefix/lib:${LD_LIBRARY_PATH:-}
```

A RealSense-only build also sets `-DRGBD_CAMERA_ENABLE_OAK=OFF` and needs only
the RealSense SDK prefix. Use a fresh build/install directory when verifying SDK
independence. The interface package remains independent of either SDK.

The core tests exercise malformed geometry/encoding bounds, stale and skewed
pairs, recovery after rejection, restart-local gate state, independent IMU timing,
deduplication and calibration identity. The adapter test uses an injected synthetic
provider in a real intra-process ROS graph; it checks read-only parameters,
association fields, IMU unavailable fields, freshness, skew recovery, and simulated
device loss/cleanup. It does not establish hardware behavior.
The optional RealSense geometry test uses synthetic nonidentity rotation and
translation, depth units, occlusion collisions, rectification with nonzero
distortion, malformed calibration, epoch conversion and microsecond timestamp
wrap. It establishes arithmetic behavior, not physical calibration accuracy.

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

For D455, pass `--backend realsense` to discovery, `-p backend:=realsense` to
the executable, or `backend:=realsense` to the launch file. Select its serial
number with `device_id`. Native RGB/depth capture is 30 Hz; `fps` controls
timestamp-based output admission (10 Hz by default). Standard image dimensions
are 640×480. The unchanged default pairing tolerance is 5 ms.

Verified D455 sessions measured approximately 1–16 ms exposure skew. A default
comparison rejected 40 consecutive pairs at about 15.9 ms skew and reported
`unavailable: pairing_skew`; an explicit comparison override is:

```bash
ros2 launch rgbd_camera camera.launch.py backend:=realsense \
  device_id:=YOUR_SERIAL pairing_tolerance_ms:=16.6 enable_imu:=true
```

This wider tolerance admits the measured asynchronous pair; it does not establish
motion compensation or simultaneous exposure. Record actual skew for each scene.
IMU SDK estimates up to 5 ms after receipt are admitted, declared in each motion
message and counted in streaming status. Larger monotonic IMU estimates are
omitted and counted; loss of valid IMU samples expires its readiness independently
of images. Regressions and domain changes remain faults. No published sample is
retimestamped to hide mapping error.

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
Pass `--backend realsense` for D455 checks. The observer checks that backend's
dimensions, source identity, optical frame and IMU axes. It retains actual skew,
delivered rate, native sequence rate, age, CPU/RSS and rejection counters.
Add `--tolerance-ms 16.6` to use the explicit D455 comparison setting.

The D455 acceptance and exact detector-association runs used an explicit
same-host Fast DDS profile, with a 32 MiB shared-memory segment per participant
and a 4 MiB maximum transport message. Apply it to every participating process:

```bash
export FASTDDS_DEFAULT_PROFILES_FILE=/absolute/rgbd_camera/tools/fastdds_local_frames.xml
```

This opts into shared-memory-only communication and is unsuitable for remote
consumers. It changes no system settings and does not change best-effort QoS.
External readers still dropped frames in measured runs; the profile is a recorded
verification condition, not a lossless-delivery guarantee. See the
[Fast DDS 2.14.6 transport documentation](https://fast-dds.docs.eprosima.com/en/v2.14.6/fastdds/transport/shared_memory/shared_memory.html).

The [OAK summary](evidence/2026-10-24-oak-camera/README.md) and
[D455 summary](evidence/2026-10-25-d455/README.md) describe the checks and
limitations. Detailed host/device identities, installed
artifact hashes, measurements, and preservation audits remain in the local
development archive; they are not part of this public repository.
