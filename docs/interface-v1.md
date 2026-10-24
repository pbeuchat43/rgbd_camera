# Camera interface v1

The `rgbd_camera_interfaces` ROS package is independent of the SDK and provider.
`Frame`, `Motion`, `CaptureReference` and `Status` are version 1 contracts.
The reusable C++ `Provider` API is in `rgbd_camera/provider.hpp`; it has no ROS
or camera SDK types. One owner calls start, polls bounded products, then stops.
Providers must own their returned storage and stop on terminal faults.

## Ownership and configuration

Starting `camera_node` or explicitly loading `rgbd_camera::CameraNode` starts
capture. Discovery (`camera_discover --backend oak|realsense`) lists SDK-visible
IDs/state/protocol without starting capture. OAK discovery reads descriptors;
RealSense discovery probes device controls and requires USB access. Full sensor
calibration and stream selection belong to the selected session. An empty selection
requires exactly one available descriptor. Multiple candidates are rejected, even
if one might later prove incompatible. No fallback device is silently selected.

The OAK backend requires RVC2, CAM_A/B/C and a calibrated B/C stereo pair. The
resolved device, USB link and integrated IMU model are reported in status.
The RealSense backend requires D455 (product ID `0B5C`), firmware `5.17.0.10`
with librealsense `2.56.5`, and matching supported 640×480
BGR8/Z16 profiles at 30 Hz (60 Hz for output requests above 30 Hz).
An unavailable backend or native-rate profile fails explicitly.
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
| `backend` | `oak` | Explicit family: `oak` or `realsense`; must be enabled in this build |
| `device_id` | empty | Exact SDK identity, or exactly one available device |
| `fps` | 10.0 | Finite 1–60 Hz request; hardware rate is measured separately |
| `pairing_tolerance_ms` | 5.0 | 0.1–50 ms, strictly less than half a frame period |
| `max_frame_age_ms` | 500.0 | 10–2000 ms; maximum source age at admission |
| `startup_timeout_s` | 30.0 | 5–120 s first raw-pair deadline; synchronous SDK open cannot be preempted |
| `stall_timeout_s` | 2.0 | 1–30 s, greater than one frame period; no raw pair progress |
| `publish_images` | false | Enable the extra standard image/CameraInfo branches |
| `enable_imu` | false | Enable independent accel/gyro; OAK requests 100 Hz each, D455 selects each sensor's lowest advertised rate up to 400 Hz |
| `optical_frame` | `<backend>_rgb_optical_frame` | Rectified virtual RGB optical coordinate frame |
| `imu_frame` | `oak_imu_native_frame` or `realsense_imu_frame` | Backend-declared IMU axes/origin, distinct from optical frame |

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

D455 uses independent 640×480 streams at 30 Hz for requests through 30 Hz, or
60 Hz for higher requests. It pairs the oldest retained frames within half a
native frame period without reusing frames, then admits
pairs at the requested output `fps`. Native capture and publication rates are
distinct. Actual exposure skew is retained and passes the same host gate; the
default 5 ms tolerance is unchanged. Unsynchronized streams may be rejected.
Selecting rates that do not divide the native rate, such as 20/25 Hz from 30 Hz
or 35 Hz from 60 Hz, produces nonuniform native-frame intervals; it does not
synthesize uniformly spaced exposures. The selected native rate is
included in backend metadata and the calibration fingerprint.
RGB is rectified into a virtual camera using the selected color intrinsics.
Each calibrated depth point is transformed into that RGB coordinate system,
projected to the nearest pixel, and stored as RGB optical Z in millimetres.
The nearest Z wins collisions. Unsampled pixels remain invalid; no hole filling
or SDK alignment that retains depth-camera Z is substituted.

`rgb` is packed `bgr8`. `depth` is little-endian `16UC1`, millimetres of optical Z
(not Euclidean range), with zero invalid. Both use the RGB optical frame: x right,
y down, z forward, positions in metres after multiplying depth by 0.001. Aligned
depth is resampled across camera viewpoints; occlusions/holes remain invalid.
Equal image dimensions do not establish valid correspondence by themselves.

For OAK, intrinsics are taken from the SDK's per-output transformation after rectification
and STRETCH. RGB and depth virtual intrinsics must agree. CameraInfo uses those K/P
values, identity R and zero distortion for the rectified output. `calibration_id`
is SHA-256 of the EEPROM calibration JSON plus output geometry/convention and K.
For D455, the hash covers native intrinsics and distortion, depth-to-color
extrinsics, depth scale, selected profile/device/firmware identity and the virtual
camera/projection convention. Unsupported distortion is rejected. Raw calibration
is not published. No physical object-centre or metric accuracy is
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

D455 enables SDK `GLOBAL_TIME` on each participating sensor and waits for two
continuous seconds of increasing timestamps within the clock checks across every
requested stream while its clock regression initializes. The SDK maps device
time into epoch milliseconds; the provider subtracts the session's fixed
epoch-minus-monotonic offset. Required video metadata moves readout time to the
reported exposure midpoint, handling the 32-bit microsecond wrap. Motion keeps
its own sensor timestamp. Missing video timing metadata, domain changes after
initialization, sequence/time regression, acquisition beyond the allowed clock checks or a host clock-offset
change exceeding 5 ms closes the session. The SDK's synchronization error and
RGB/depth synchronization accuracy have no measured bound here. Fast IMU delivery
can precede its SDK time estimate: samples estimated up to 5 ms after receipt are
admitted without adjusting their timestamps. Larger, still increasing IMU
estimates are individually rejected; IMU readiness expires if admissible samples
stop, while valid RGB/depth can continue. Regressions and domain changes remain
terminal. Each `Motion.future_tolerance_ns` declares this admission policy
(5000000 for D455, zero for OAK); it is not a measured synchronization error bound.
Motion consumers must allow acquisition ≤ receipt + future_tolerance_ns, preserve
the original timestamp, and treat a permitted small negative age as fresh.
A cumulative `realsense_imu_future_timestamps_admitted` note counts these samples
at provider admission, before subsequent bounded-queue/transport loss;
`rejected_over_tolerance` counts omitted IMU estimates. After
startup, individual image callbacks also tolerate up to 5 ms of clock-estimate
lead; the completed image bundle still requires both acquisition timestamps to
precede its combined receipt (the later individual receipt). Motion readers must rebuild for this
added message field; `Frame` and `CaptureReference` wire definitions are unchanged.

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
`Status.frames` counts admitted pairs before the publish call; it does not confirm
receipt by any consumer. Measure external delivery by comparing received capture
identities against admissions over the same acquisition interval. D455's deliberate
native-rate selection must not be counted as external transport loss.

## Optional integrated IMU

The same device owner creates the IMU branch only when `enable_imu=true`.
Unsupported requests fail clearly; disabled output creates no IMU node or topics.
The OAK request is raw accelerometer and raw gyroscope at 100 Hz each.
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
before fusion. Mounting transforms belong to composition. RealSense's SDK returns
motion in depth-oriented axes (x right, y down, z forward) at the IMU origin,
labeled `sdk_depth_oriented_at_imu_origin`. This is neither die-native axes nor
the RGB camera origin. SDK default motion correction is retained. D455 accel and
gyro have independent supported rates; no combined or interpolated IMU is emitted.
The [Luxonis IMU documentation](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/imu)
describes raw versus transformed reports; actual support remains SDK/device-specific.

## Bounded processing and reuse

OAK Device Sync input queues and host image output are nonblocking depth 1. ROS frame
and standard image QoS are best-effort keep-last 1. The acquisition worker owns
the SDK and never runs inference or consumer callbacks. IMU SDK output retains
four batches of at most ten packets (at most twenty individual sensor reports
per poll); ROS IMU queues retain four messages. No work list grows with runtime.
D455 retains at most two frames per image stream and twenty motion samples;
oldest entries are replaced when full. Its SDK queue request is four video frames
or sixteen motion frames per sensor. Intentional native-rate to output-rate admission
also produces sequence gaps, so D455 gaps do not alone indicate transport loss.

Composition is enabled explicitly with intra-process communication. SDK-to-owned
image conversion copies bytes; the optional standard image branch copies again.
In-process const consumers can share the bundle; external subscribers introduce
serialization/copies. No end-to-end zero-copy or hard latency guarantee is made.
Caller-owned ROS queues/buffers and SDK internal processing also consume memory.

These profiles are deliberately narrow. Other hardware, sizes or native rates
must extend the validated profile contract and its acceptance
evidence. It cannot silently reinterpret optical-Z depth as range, distorted
pixels as rectified, or unaligned images as aligned.
