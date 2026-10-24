# Repository guidance

`rgbd_camera` owns discovery, exclusive camera sessions, acquisition, calibration,
frame association and optional integrated IMU delivery. Detection and spatial
estimation belong to `rgbd_object_detector`. Read `README.md` and
`docs/interface-v1.md` before changing public behavior.

The research architecture is maintained in
https://github.com/pbeuchat43/robot_runtime/blob/main/docs/research-platform-architecture.md.
Preserve its ownership, freshness, geometry and bounded-processing contracts.

- Run execution checks on the designated reserved development host with isolated
  files and ROS domains. Never replace an existing deployment implicitly.
- `rgbd_camera_interfaces` is independently installable, without a camera SDK.
  Keep reusable provider logic behind thin C++ ROS component adapters.
- Preserve image acquisition times and source generations. Reject unsupported
  geometry; do not imply calibrated accuracy or unmeasured hardware support.
- All configuration is startup-only in this slice. Consumers cannot reconfigure
  the shared device. Ordinary shutdown releases the sole session.
- Follow the user's Solo/workflow preferences. No automatic approval ceremonies.
- New material is MIT, copyright Paul N. Beuchat. Preserve third-party notices.
- `main` is the verified baseline; `dev` is development. Archive intermediate
  history before consolidating a verified milestone; preserve existing baselines.

Verification commands and evidence limits are in `docs/verification.md`.
