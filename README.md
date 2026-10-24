# rgbd_camera

Shared RGB-D acquisition for the research robot platform. Providers target
OAK-D/RVC2 with DepthAI 3.9.0 and RealSense D455 firmware 5.16.0.1 with librealsense 2.56.3
on Ubuntu 24.04 arm64 / ROS 2 Jazzy.
It provides a standalone executable and a composable C++ camera node. A detector,
model, GPU runtime or web server is not required.

One explicit node start owns one selected camera. Independent consumers share
its output. OAK provides undistorted 640×640 BGR plus aligned optical-Z depth.
D455 provides rectified 640×480 BGR and depth explicitly transformed into RGB
optical Z. Both default to 10 Hz output; D455 captures natively at 30 Hz.
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
The [OAK verification summary](docs/evidence/2026-10-24-oak-camera/README.md) and
[D455 verification summary](docs/evidence/2026-10-25-d455/README.md) cover real
capture, optional IMU, restarts, bounded consumers and unchanged cone-detector
compatibility. The D455 comparison used an explicit 16.6 ms pairing tolerance.
External best-effort readers can drop frames; no full-rate fan-out or physical
depth/IMU accuracy guarantee is implied.

The camera contract feeds the separate
[`rgbd_object_detector`](https://github.com/pbeuchat43/rgbd_object_detector)
for TensorRT inference and depth-associated XYZ. Object detection, ArUco, video
encoding and deployment migration remain separate responsibilities.

Canonical repository: https://github.com/pbeuchat43/rgbd_camera.
`main` is the verified baseline and `dev` the development branch.
Version: 0.2.0 (development milestone, not a stable platform release).

MIT License, copyright 2026 Paul N. Beuchat. See [NOTICE.md](NOTICE.md) for
upstream references and their separate licensing.
