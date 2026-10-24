# Camera interface v1

The `rgbd_camera_interfaces` ROS package is independent of the SDK and provider.
`Frame`, `Motion`, `CaptureReference` and `Status` are version 1 contracts.
The reusable C++ `Provider` API is in `rgbd_camera/provider.hpp`; it has no ROS
or DepthAI types. One owner calls start, polls bounded products, then stops.
Providers must own their returned storage and stop on terminal faults.

## Ownership and configuration

Starting `camera_node` or explicitly loading `rgbd_camera::CameraNode` starts
capture. Discovery (`camera_discover`) lists SDK-visible IDs/state/protocol without
opening a device or starting capture. It cannot inspect sensor calibration or
full device capabilities; those require the selected session. An empty selection
requires exactly one available descriptor. Multiple candidates are rejected, even
if one might later prove incompatible. No fallback device is silently selected.

The OAK backend requires RVC2, CAM_A/B/C and a calibrated B/C stereo pair. The
resolved device, USB link and integrated IMU model are reported in status.
SDK exclusivity plus a per-user advisory lock prevent concurrent owners. Other
software must also respect SDK exclusivity; the lock is not a cross-user security
boundary. Shutdown releases the pipeline/device and lock. There is no automatic
reconnect or workload restart. A terminally failed node stays observable in
`error` with its hardware closed until explicitly restarted/unloaded.

All parameters are startup-only and have read-only ROS descriptors. A change
requires a coordinated owner restart, which creates a new generation. Consumers
may subscribe/unsubscribe; they cannot modify exposure, geometry or capture rate.

| Parameter | Default | Accepted values / meaning |
| --- | --- | --- |
| `device_id` | empty | Exact SDK identity, or exactly one available device |
| `fps` | 10.0 | Finite 1–30 Hz request; hardware rate is measured separately |
| `pairing_tolerance_ms` | 5.0 | 0.1–50 ms, strictly less than half a frame period |
| `max_frame_age_ms` | 500.0 | 10–2000 ms; maximum source age at admission |
| `startup_timeout_s` | 30.0 | 5–120 s first raw-pair deadline; synchronous SDK open cannot be preempted |
| `stall_timeout_s` | 2.0 | 1–30 s, greater than one frame period; no raw pair progress |
| `publish_images` | false | Enable the extra standard image/CameraInfo branches |
| `enable_imu` | false | Enable raw accel/gyro at a requested 100 Hz each |
| `optical_frame` | `oak_rgb_optical_frame` | Rectified virtual RGB optical coordinate frame |
| `imu_frame` | `oak_imu_native_frame` | Native sensor coordinate frame, distinct from optical frame |

`use_sim_time=true` is rejected for this physical provider. Exposure is the SDK
default auto exposure; this slice does not expose exposure/projector controls or
write device calibration, flash firmware, or change stored settings.

## Images and exact association

`~/frames` is the authoritative matched-pair interface. It contains two standard
`sensor_msgs/Image` records, their matching `CameraInfo`, and metadata in one
message. Consumers retain a const shared pointer or their own copy; they must not
annotate shared storage. A detection must retain the capture reference and
calibration identity from its actual inference input. It must not fetch the latest
depth after asynchronous inference completes.

Identity is `(source_id, generation, stream_id, sequence)`: stable device identity,
random session UUID, named stream (`rgbd` here), and RGB device sequence.
`depth_sequence` names the associated depth
capture; the two sequences need not be equal. Both acquisition timestamps and the
signed `depth - rgb` skew are explicit. A session restart, clock fault or geometry
change invalidates old associations. Consumers reset their bounded retained work
on a new generation. `header.frame_id` is a coordinate frame, never a capture ID.

The OAK pipeline requests a 640×640 undistorted RGB image using STRETCH, 640×400
stereo inputs, DENSITY/subpixel depth aligned to that virtual RGB image, and a
device Sync node. Sync attempts are finite (one): groups outside the configured
tolerance are exposed to host diagnostics and discarded, instead of hiding all
input progress. The host independently checks size, geometry, time, sequence,
skew and age. Excess skew/age is recoverable rejection; it increments counters.
Persistent rejection makes the accepted stream unavailable without closing the
camera. Device/pipeline loss, invalid geometry, or clock/sequence regression are
terminal. A process running is not proof of valid frame delivery.

`rgb` is packed `bgr8`. `depth` is little-endian `16UC1`, millimetres of optical Z
(not Euclidean range), with zero invalid. Both use the RGB optical frame: x right,
y down, z forward, positions in metres after multiplying depth by 0.001. Aligned
depth is resampled across camera viewpoints; occlusions/holes remain invalid.
Equal image dimensions do not establish valid correspondence by themselves.

Intrinsics are taken from the SDK's per-output transformation after rectification
and STRETCH. RGB and depth virtual intrinsics must agree. CameraInfo uses those K/P
values, identity R and zero distortion for the rectified output. `calibration_id`
is SHA-256 of the EEPROM calibration JSON plus output geometry/convention and K.
The raw EEPROM is not published. No physical object-centre or metric accuracy is
established by the projection equation `X=(u-cx)Z/fx`, `Y=(v-cy)Z/fy`.

Optional topics are `~/rgb/image_rect`, `~/rgb/camera_info`,
`~/depth/image_rect` and `~/depth/camera_info`. They preserve acquisition headers
but do not independently carry generation/capture identity. Use the atomic bundle
for exact research associations; ordinary image tools may use the standard branch.
External recording must record the bundle to preserve this complete contract.

## Clocks and freshness

DepthAI maps device acquisition timestamps into host `steady_clock`. On the tested
Linux platform this is monotonic time, labeled `linux_monotonic:<boot UUID>`.
Nanoseconds remain int64, including ROS metadata. `received_ns` records host receipt
before copying SDK image data; delivery time is not substituted for acquisition.
Device/host time synchronization error is not bounded by this slice.

A fixed `unix_offset_ns` sampled once per session maps monotonic acquisition time
into each standard ROS header. The original monotonic time and mapping remain in
metadata. This mapping does not follow subsequent wall-clock/NTP corrections;
cross-device global synchronization is not claimed. Consumers on another host
cannot subtract their own monotonic clock. Bridge receipt age is separate.

`~/status` is best-effort depth 1, emitted every 200 ms and expires after 1 s in its
declared host clock. It distinguishes starting, streaming, unavailable and error,
raw pair progress, accepted acquisition progress, rejected pairs, sequence gaps,
and independent IMU progress. Loss of status itself must expire readiness.
Image sequence gaps include SDK/output queue drops; they are not an exact count
of every dropped frame at every transport boundary. IMU gaps are similarly scoped.

## Optional integrated IMU

The same device owner creates the IMU branch only when `enable_imu=true`.
Unsupported requests fail clearly; disabled output creates no IMU node or topics.
The fixed initial request is raw accelerometer and raw gyroscope at 100 Hz each.
Actual supported rates may differ and are recorded in verification evidence.

`~/imu/accel_sample` and `~/imu/gyro_sample` contain `Motion` messages with separate
sensor time/counter, session identity, sensor model, native-axis declaration and
a standard `sensor_msgs/Imu`. The corresponding `~/imu/accel` and `~/imu/gyro`
standard topics support external tools. Each sensor preserves its own acquisition
time; repeated reports in SDK batches are deduplicated. The `accel` and `gyro`
stream identities have independent, monotonically increasing published sequences;
`sensor_sequence` preserves the original vendor counter, which may wrap.
Acceleration is m/s²,
angular velocity rad/s. Orientation and the other sensor field are unavailable
(`covariance[0] = -1`); the present field has zero covariance meaning unknown,
not perfect certainty. No interpolation, gravity removal or orientation fusion
is performed.

RAW uses the sensor's native axes; it is not relabeled as RGB optical or robot
body data. A physical sensor-to-camera transform must be established separately
before fusion. Mounting transforms belong to composition. RealSense can implement
this same per-sensor contract even when gyro and accel have different rates.
The [Luxonis IMU documentation](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/imu)
describes raw versus transformed reports; actual support remains SDK/device-specific.

## Bounded processing and reuse

Device Sync input queues and host image output are nonblocking depth 1. ROS frame
and standard image QoS are best-effort keep-last 1. The acquisition worker owns
the SDK and never runs inference or consumer callbacks. IMU SDK output retains
four batches of at most ten packets (at most twenty individual sensor reports
per poll); ROS IMU queues retain four messages. No work list grows with runtime.

Composition is enabled explicitly with intra-process communication. SDK-to-owned
image conversion copies bytes; the optional standard image branch copies again.
In-process const consumers can share the bundle; external subscribers introduce
serialization/copies. No end-to-end zero-copy or hard latency guarantee is made.
Caller-owned ROS queues/buffers and SDK internal processing also consume memory.

The first profile is deliberately narrow. A RealSense backend may select other
sizes/rates but must extend the validated profile contract and its acceptance
evidence. It cannot silently reinterpret optical-Z depth as range, distorted
pixels as rectified, or unaligned images as aligned.
