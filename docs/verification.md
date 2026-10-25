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
exactly**. RealSense builds require librealsense **2.56.5 exactly** plus OpenCV
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
`38a41441971387197193ad3aeae3cefe6a11f2cb` using the userspace RSUSB backend,
without CUDA, DDS, firmware download, system installation or kernel changes.
An existing libusb development package and Ninja are required. Camera USB access
must already be granted to the execution account. The build does not install
udev rules or upgrade camera firmware.

The D455 profile explicitly requires firmware **5.17.0.10**, paired with SDK
2.56.5 in the [official compatibility table](https://dev.realsenseai.com/docs/firmware-releases-d400/).
This supersedes the earlier Isaac ROS 4.6 pair (5.16.0.1 / 2.56.3): repeated raw
captures on that pair reproduced unstable depth auto-exposure with RGB enabled.
The [2.56.4 release](https://github.com/realsenseai/librealsense/releases/tag/v2.56.4)
lists a fix for that behavior; [2.56.5](https://github.com/realsenseai/librealsense/releases/tag/v2.56.5)
adds the D455 60 Hz depth/IR corruption fix. This profile does not establish
Isaac ROS deployment compatibility. Other firmware versions are rejected before
streaming; upgrades are separately approved device operations, never performed
by the provider or SDK build script.

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
number with `device_id`. Native RGB/depth capture is 30 Hz through a requested
30 Hz, or 60 Hz for higher requests up to 60 Hz; `fps` controls
timestamp-based output admission (10 Hz by default). Standard image dimensions
are 640×480. The unchanged default pairing tolerance is 5 ms.

Historical D455 sessions measured approximately 1–16 ms exposure skew. A default
comparison on firmware 5.16.0.1 rejected 40 consecutive pairs at about 15.9 ms skew and reported
`unavailable: pairing_skew`; an explicit comparison override is:

```bash
ros2 launch rgbd_camera camera.launch.py backend:=realsense \
  device_id:=YOUR_SERIAL pairing_tolerance_ms:=16.6 enable_imu:=true
```

This wider tolerance admits the measured asynchronous pair; it does not establish
motion compensation or simultaneous exposure. Record actual skew for each scene.
The subsequent 5.17.0.10 sweeps measured approximately 12 ms depth-before-RGB
skew at native 30 Hz and 7.17–8.18 ms at native 60 Hz, also exceeding the default.
For the D455 60 Hz native mode, the comparison uses 8.3 ms, below half its native
frame period. Output selection at rates that do not divide the native rate
(for example 35, 40, 45, 50 or 55 Hz from 60 Hz) has nonuniform exposure intervals.
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

For large atomic frames on the same host, use the Fast DDS profile with a 32 MiB
shared-memory segment per participant and a 4 MiB maximum transport message.
Apply it to every participating process:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/absolute/rgbd_camera/tools/fastdds_local_frames.xml
```

Or prefix each command with `tools/with_local_frames.sh`, which selects
`rmw_fastrtps_cpp` and this profile for that process and its children.
Fast DDS 2.14.6 uses **FASTRTPS**, not **FASTDDS**, in this variable name.
The earlier D455 acceptance set the latter name, so the profile was silently
ignored. Those historical frame-loss measurements describe the default transport.
See the [version-specific environment-variable reference](https://fast-dds.docs.eprosima.com/en/v2.14.6/fastdds/env_vars/env_vars.html).

This configures shared memory for same-host delivery and is intended for local
consumers. It changes no system settings and does not change best-effort QoS.
Bounded best-effort queues still permit drops if a consumer cannot keep up; this
is not an unconditional lossless-delivery guarantee. See the
[Fast DDS 2.14.6 transport documentation](https://fast-dds.docs.eprosima.com/en/v2.14.6/fastdds/transport/shared_memory/shared_memory.html).

The [OAK summary](evidence/2026-10-24-oak-camera/README.md) and
[D455 summary](evidence/2026-10-25-d455/README.md) describe the checks and
limitations. The later [delivery and rate investigation](evidence/2026-10-25-frame-delivery-rates/README.md)
records the corrected transport setup, updated firmware/SDK pair and rate limits.
The subsequent [all-output campaign](evidence/2026-10-26-all-output-rates/README.md)
measures IMU and standard images alongside the cone pipeline, including successful
native D455 runs and the observed OAK delivery and clock-mapping failures.
Detailed host/device identities, installed
artifact hashes, measurements, and preservation audits remain in the local
development archive; they are not part of this public repository.

## Rate measurements with an external reader

The optional `tools/rate_probe` executable consumes the actual atomic frame in a
separate process and records only metadata. It also records detector batches and
status counters. The benchmark controller runs separately, so resource sampling
does not block the frame callback. This consumer is excluded from ordinary
colcon discovery and is not a provider dependency.

Source the installed camera and detector overlays before building it:

```bash
cmake -S tools/rate_probe -B /absolute/isolated/rate-probe -DCMAKE_BUILD_TYPE=Release
cmake --build /absolute/isolated/rate-probe --parallel 2
```

The controller needs the detector interface package even when the engine is
omitted, plus the ROS composition interfaces and `ament_index_python`.
Reserve the host, set an isolated ROS domain and `ROS_LOG_DIR`, and run one camera
at a time. The output directory must not exist. A lit, stationary cone scene is
necessary to measure positive detection workload.

```bash
tools/with_local_frames.sh python3 tools/benchmark_rates.py \
  --output /absolute/isolated/oak-25 \
  --probe /absolute/isolated/rate-probe/rate_probe \
  --device-id YOUR_DEVICE_ID --backend oak --fps 25 \
  --engine /absolute/models/cones.engine --layout composed --seconds 120
```

Use `--backend realsense` with its serial for D455. The comparison defaults are
5 ms OAK pairing tolerance, 16.6 ms D455 through 30 Hz, or 8.3 ms above 30 Hz;
`--tolerance-ms` records an explicit alternative. `--layout standalone` puts the
camera and detector in separate processes. Omitting `--engine` measures camera
delivery alone. `--imu` enables and consumes both wrapped and standard accel/gyro
messages. `--images` enables and consumes standard RGB/depth images and both
CameraInfo topics. The external probe receives complete image buffers but logs
only metadata, without pixel processing. Add both flags to measure their combined
cost alongside the detector. Use `verify_hardware.py` for broader functional
checks such as owner conflicts and restart generations.

The report compares each reader's unique captures with `Status.frames` deltas
between matching acquisition boundaries after warm-up, rather than treating
D455 native sequence gaps as transport losses. It retains raw CSV, callback
arrival intervals, source-age percentiles, admission/rejection/overload counts,
native sequence gaps, positive/empty results and process resource samples.
Native gaps include intentional D455 output selection; they are not a delivery-loss count.
Atomic-frame and detector source age starts at RGB acquisition; depth skew is
retained separately in the frame CSV. Optional standard depth and IMU ages start
at their own acquisition timestamps. IMU delivery comparisons use each sensor's
independent status-counter and acquisition boundaries. Within the single test
session, standard headers are matched against received authoritative frame or
motion records. Standard-output missing/duplicate counts are relative to those
records; incomplete authoritative delivery prevents a complete standard-output
delivery claim. Image dimensions, encoding and payload sizes are checked too.
Do not describe an empty detection batch as a dropped frame or an accepted frame
as confirmed delivery. Warm-up, measurement and graceful shutdown are separate.
