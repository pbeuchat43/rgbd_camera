# Running the D455 at native 60 Hz

This recipe reproduces the settings used by the
[ten-minute all-output check](evidence/2026-10-26-all-output-rates/README.md):
640×480 rectified BGR and RGB-aligned optical-Z depth, native 60 Hz capture,
an explicit **8.3 ms pairing tolerance**, and optional images and IMU enabled.
That check delivered 35,855 frames and detector batches at 59.758 Hz, with a
38.56 ms p95 acquisition-to-external-detector-result age. These are observations
from a lit stationary scene, not a display-latency or moving-scene accuracy test.

## Prerequisites

- Ubuntu 24.04 arm64 and ROS 2 Jazzy, with `rgbd_camera` built using
  librealsense **2.56.5**, commit `38a41441971387197193ad3aeae3cefe6a11f2cb`,
  and D455 firmware **5.17.0.10**. The build uses the userspace RSUSB backend.
- A dual-backend build also uses DepthAI **3.9.0**, commit
  `ac928141ef7750bf3082b5e1c319a115618660e7`. A RealSense-only build does not
  require DepthAI. Follow the [build guide](verification.md#dependencies-and-build).
- The execution account already has USB access. Reserve the development host
  before testing; these commands do not install a deployment, change USB rules,
  or update firmware. One process owns the selected camera.
- The optional detector composition below requires `rgbd_object_detector`, its
  documented TensorRT dependencies, and an existing compatible engine with its
  sidecar manifest. Camera-only operation has no detector/model dependency.

Set the paths, device identity and an unused ROS domain for your workspace.
Run this setup in **every participating terminal**, including external readers:

```bash
RGBD_WORKSPACE=/absolute/path/to/isolated/ros-workspace
RGBD_CAMERA_REPO="$RGBD_WORKSPACE/src/rgbd_camera"
RGBD_REALSENSE_PREFIX=/absolute/path/to/librealsense-2.56.5
# Set this only for a dual-backend build:
RGBD_DEPTHAI_PREFIX=/absolute/path/to/depthai-3.9.0
RGBD_D455_SERIAL=YOUR_D455_SDK_SERIAL

source /opt/ros/jazzy/setup.bash
source "$RGBD_WORKSPACE/install/local_setup.bash"
export LD_LIBRARY_PATH="$RGBD_REALSENSE_PREFIX/lib:${LD_LIBRARY_PATH:-}"
if [[ -n ${RGBD_DEPTHAI_PREFIX:-} ]]; then
  export LD_LIBRARY_PATH="$RGBD_DEPTHAI_PREFIX/lib:$LD_LIBRARY_PATH"
fi
export ROS_DOMAIN_ID=228 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR="$RGBD_WORKSPACE/ros-log"
rgbd_local() { "$RGBD_CAMERA_REPO/tools/with_local_frames.sh" "$@"; }
```

`rgbd_local` applies the checked-in 32 MiB shared-memory / 4 MiB maximum-message
Fast DDS profile. ROS Jazzy's tested Fast DDS 2.14.6 reads
`FASTRTPS_DEFAULT_PROFILES_FILE`; the newer `FASTDDS_DEFAULT_PROFILES_FILE`
spelling was ignored in that version. The wrapper sets the supported variable
and `rmw_fastrtps_cpp` for each process. It is a same-host profile; network video
delivery requires a separate integration.

## Camera and standard outputs

Find the SDK serial if needed, then start the camera:

```bash
rgbd_local ros2 run rgbd_camera camera_discover --backend realsense
rgbd_local ros2 launch rgbd_camera camera.launch.py \
  backend:=realsense device_id:="$RGBD_D455_SERIAL" fps:=60.0 \
  pairing_tolerance_ms:=8.3 enable_imu:=true publish_images:=true
```

The node is `/camera`. Its authoritative output is `/camera/frames`.
Standard images and CameraInfo are under `/camera/rgb` and `/camera/depth`;
wrapped and standard accelerometer/gyroscope topics are under `/camera/imu`.
`/camera/status` must remain fresh and report both camera and IMU streaming.
Status frame counts are admissions, not proof that a reader received each frame.
Stop the launch with Ctrl-C to release the camera.

## Camera and detector in the measured composition

Use this alternative when reproducing the camera-plus-detector arrangement.
Start the multi-threaded component container in one prepared terminal:

```bash
rgbd_local ros2 run rclcpp_components component_container_mt \
  --ros-args -r __node:=perception_container
```

In a second prepared terminal, load the camera and detector into that container:

```bash
RGBD_CONE_ENGINE=/absolute/path/to/cones.engine

rgbd_local ros2 component load /perception_container \
  rgbd_camera rgbd_camera::CameraNode --no-daemon --spin-time 2 \
  --node-name camera --node-namespace /perception \
  -e use_intra_process_comms:=true \
  -p backend:=realsense -p "device_id:='$RGBD_D455_SERIAL'" \
  -p fps:=60.0 -p pairing_tolerance_ms:=8.3 \
  -p enable_imu:=true -p publish_images:=true

rgbd_local ros2 component load /perception_container \
  rgbd_object_detector rgbd_object_detector::DetectorNode --no-daemon --spin-time 2 \
  --node-name detector --node-namespace /perception \
  -e use_intra_process_comms:=true \
  -p "engine_path:='$RGBD_CONE_ENGINE'" -r frames:=camera/frames
```

The authoritative bundle is now `/perception/camera/frames`, and detector batches
are `/perception/detector/batches`. Both components must report fresh streaming
status. Each batch retains its input capture identity and acquisition time.
Each loading command allows two seconds for local discovery; the CLI's default
half-second discovery interval can miss a newly discovered container.
Stopping the container with Ctrl-C shuts down both components and releases the
camera. Installing or building these packages starts no workload.

## Interpreting the settings

At 60 Hz, 8.3 ms is below half a frame period and admits the measured asynchronous
RGB/depth exposures. It does not make the exposures simultaneous or compensate
for movement. Preserve the actual depth/RGB skew for spatial interpretation.
For native 30 Hz, select `fps:=30.0` and `pairing_tolerance_ms:=16.6`; the latest
campaign verified these two native rates. Other requested rates can have uneven
exposure intervals because the provider selects from native 30/60 Hz captures.

IMU sensors run independently of video. The measured D455 rates were about
101 Hz accelerometer and 200 Hz gyroscope. The declared 5 ms IMU future-time
allowance is an admission policy for SDK clock estimates, not a measured
synchronization accuracy bound; acquisition timestamps are preserved.

All image/frame queues remain best-effort depth 1. Additional consumers, image
processing, encoding, network transport and display can change delivery and age.
Use the [measurement tool](verification.md#rate-measurements-with-an-external-reader) to
measure your complete workload; ordinary topic-rate displays alone do not
establish capture completeness or source age.
