# rgbd_camera

Shared RGB-D acquisition for the research robot platform. The first development
slice targets OAK-D/RVC2 with DepthAI 3.9.0 on Ubuntu 24.04 arm64 / ROS 2 Jazzy.
It provides a standalone executable and a composable C++ camera node. A detector,
model, GPU runtime or web server is not required.

One explicit node start owns one selected camera. Independent consumers share
its output. The initial profile is undistorted 640×640 BGR plus aligned optical-Z
depth at 10 Hz. Optional accelerometer and gyroscope outputs use the same device
session. They are disabled by default. RealSense is a future provider, not a
claimed supported backend.

See the [interface contract](docs/interface-v1.md),
[first slice and acceptance](docs/first-slice.md), and
[build and verification guide](docs/verification.md).
The [verification summary](docs/evidence/2026-10-24-oak-camera/README.md) covers
real OAK capture, optional integrated IMU output, restarts and bounded consumers.
External best-effort readers can drop frames; no full-rate fan-out or physical
depth/IMU accuracy guarantee is implied.

The next broader milestone is this camera contract feeding
`rgbd_object_detector`: the existing cone model on TensorRT, followed by depth
association and valid/invalid XYZ results. This slice does not implement object
detection, ArUco, video encoding or deployment migration.

Canonical repository: https://github.com/pbeuchat43/rgbd_camera.
`main` is the verified baseline and `dev` the development branch.
Version: 0.1.0 (development milestone, not a stable platform release).

MIT License, copyright 2026 Paul N. Beuchat. See [NOTICE.md](NOTICE.md) for
upstream references and their separate licensing.
