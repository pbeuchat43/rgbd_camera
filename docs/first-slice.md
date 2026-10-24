# First slice: shared OAK RGB-D and optional IMU

The first bounded milestone is acquisition usable independently of detection.
It establishes the camera boundary needed by the later OAK → TensorRT cone → XYZ
integration. Only `rgbd_camera` is introduced. Existing runtime repositories and
installed deployments are preserved.

## Engineering choices

- C++ provider library with thin standalone/composable ROS adapter. Adapt the
  pinned DepthAI SDK instead of rewriting device processing.
- One fixed 640×640 rectified RGB/aligned-depth profile; 10 Hz default, validated
  startup selection within 1–30 Hz. No live reconfiguration or implicit restart.
- Atomic frame bundle containing standard Image/CameraInfo messages plus exact
  capture metadata. Optional standard image topics support external tools at an
  explicit copy cost. A separate metadata topic alone could lose its association
  with images; the bundle avoids a timestamp-only join across restarts.
- Device Sync tolerance defaults to 5 ms; latest-only image queues; reject frames
  older than 500 ms by default. Both are validated startup parameters. Tolerance
  is at most 50 ms and strictly below half a frame period, preventing adjacent
  frames becoming equally plausible matches. Actual skew and configured tolerance
  are carried in every accepted pair. Excess skew/age drops a pair; it does not
  terminate the node. Persistent rejection reports unavailable and may recover.
  These are admission policies, not an accuracy or throughput guarantee.
- Optional raw accelerometer and gyroscope streams preserve each sensor's own
  acquisition timestamp. Separate standard Imu messages avoid inventing a common
  timestamp or fused orientation. Native sensor axes are explicitly labeled.
- A configurable 30 s startup deadline and 2 s raw acquisition stall deadline
  report failure and close the owner; a new explicit process/component start is
  required. Raw progress and accepted-pair progress are tracked separately.
  Clock regression and invalid geometry are terminal; pairing jitter is not.

## Acceptance

1. Build the SDK and both ROS packages in an isolated prefix on the reserved host.
   The interface package must build without SDK headers or libraries.
2. Synthetic tests reject malformed geometry/data, clock regressions, duplicates,
   excess skew, stale frames and generation/calibration confusion. Check queue
   bounds and IMU unavailable-field semantics without claiming hardware behavior.
3. Observe at least 100 real matched pairs with valid sizes, calibration, advancing
   identities and bounded skew/age. Report actual rate, age distribution, sequence
   gaps, CPU and memory for the configured profile; do not optimize toward an
   undeclared numerical performance target.
4. Start standalone and in a component container. A second owner must fail.
   A slow consumer must skip frames without stopping the fast consumer; image
   delivery uses bounded queues. Verify no consumers can change startup settings.
5. Stop gracefully, observe handle release, and restart with a new generation.
   Invalid device selection must fail explicitly. Synthetic provider-loss tests
   establish failure policy; no unperformed physical unplug test is implied.
6. With IMU enabled on supported hardware, observe independent advancing accel
   and gyro timestamps and finite values; report rates and gaps. Disabled means
   no IMU pipeline or topics. Unsupported IMU requests fail explicitly.
7. Compare existing service identities and deployment selection before and after.
   Record exact source, SDK, configuration, host and camera identity locally;
   publish only evidence cleared for public disclosure.
8. Archive useful intermediate Git history, consolidate into a coherent initial
   commit, publish public GitHub `main`, and align `dev` to that verified commit.

Metric depth accuracy, inertial accuracy, camera-to-IMU mounting calibration,
fused orientation, RealSense, GPU inference, video, zero-copy, long-duration
reliability and deployment acceptance are outside this first slice.
