# rgbd_camera

Shared RGB-D acquisition for the research robot platform. Providers target
OAK-D/RVC2 with DepthAI 3.9.0 and RealSense D455 firmware 5.17.0.10 with librealsense 2.56.5
on Ubuntu 24.04 arm64 / ROS 2 Jazzy.
It provides a standalone executable and a composable C++ camera node. A detector,
model, GPU runtime or web server is not required.

One explicit node start owns one selected camera. Independent consumers share
its output. OAK provides undistorted 640×640 BGR plus aligned optical-Z depth.
D455 provides rectified 640×480 BGR and depth explicitly transformed into RGB
optical Z. Both default to 10 Hz output. D455 captures natively at 30 Hz for
requests through 30 Hz, and at 60 Hz for requests above 30 Hz (up to 60 Hz output).
Optional accelerometer and gyroscope outputs use the same device session and
are disabled by default. Select the camera family explicitly with `backend`.

Verified D455 sessions observed exposure-midpoint skew of about 1–16 ms,
sometimes exceeding the unchanged 5 ms default. Wider pairing tolerances are an
explicit association tradeoff, not synchronized capture. D455 IMU messages admit
SDK clock estimates up to 5 ms ahead of receipt, preserve those timestamps, and
declare the allowance. This policy is not a measured clock accuracy guarantee.

See the [interface contract](docs/interface-v1.md),
[first slice and acceptance](docs/first-slice.md), and
[build and verification guide](docs/verification.md).
For ordinary use, the [D455 native 60 Hz recipe](docs/running-d455-60.md) covers
camera-only operation and the camera-plus-detector composition, including IMU,
standard images, pairing tolerance and the local DDS setup.
The [OAK verification summary](docs/evidence/2026-10-24-oak-camera/README.md) and
[D455 verification summary](docs/evidence/2026-10-25-d455/README.md) cover real
capture, optional IMU, restarts, bounded consumers and unchanged cone-detector
compatibility. The D455 comparison used an explicit 16.6 ms pairing tolerance.
The subsequent [frame-delivery and rate investigation](docs/evidence/2026-10-25-frame-delivery-rates/README.md)
records the corrected transport setup, updated D455 firmware/SDK pair, and
measured throughput and source age. The configurable rate range is a request;
consult those results for observed limits and lighting conditions.
The [all-output campaign](docs/evidence/2026-10-26-all-output-rates/README.md)
verified ten minutes of complete D455 delivery at native 30 and 60 Hz with IMU
and standard images enabled. At 60 Hz it measured 59.758 Hz delivery and 38.56 ms
p95 acquisition-to-detector-result age. This is not camera-to-screen latency.
The [OAK timing and delivery follow-up](docs/evidence/2026-10-26-oak-hardening/README.md)
adds a declared 5 ms IMU clock-estimate allowance and verifies ten-minute all-output
runs at 35 and 40 Hz. It demonstrates external-reader queue loss during a callback
stall and identifies excessive RGB/depth skew at the 60 Hz request. The current
OAK profile did not sustain requested rates above 40 Hz; D455 remains the verified
native 60 Hz path. The follow-up retains the unreproduced historical-fault limits
and the D455 recipe check's additional invalid spatial results.
External best-effort readers can drop frames; no full-rate fan-out or physical
depth/IMU accuracy guarantee is implied.

For same-host external readers, prefix each participating command with
`tools/with_local_frames.sh`. It loads the large-frame shared-memory profile using
the `FASTRTPS_DEFAULT_PROFILES_FILE` variable supported by Fast DDS 2.14.6.
The earlier `FASTDDS_DEFAULT_PROFILES_FILE` spelling was ignored on this version;
see the correction in the verification guide. Apply the wrapper to readers as
well as the camera and detector; it changes no system settings.

The camera contract feeds the separate
[`rgbd_object_detector`](https://github.com/pbeuchat43/rgbd_object_detector)
for TensorRT inference and depth-associated XYZ. Object detection, ArUco, video
encoding and deployment migration remain separate responsibilities.

Canonical repository: https://github.com/pbeuchat43/rgbd_camera.
`main` is the verified baseline and `dev` the development branch.
Version: 0.2.0 (development milestone, not a stable platform release).

MIT License, copyright 2026 Paul N. Beuchat. See [NOTICE.md](NOTICE.md) for
upstream references and their separate licensing.
