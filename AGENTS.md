# Repository guidance

`rgbd_camera` owns discovery, exclusive camera sessions, acquisition, calibration,
frame association and optional integrated IMU delivery. Detection and spatial
estimation belong to `rgbd_object_detector`. Read `README.md` and
`docs/interface-v1.md` before changing public behavior.

For shared ownership and system boundaries, read the current
[system architecture](../robot_runtime/docs/research-platform-architecture.md)
in the corresponding runtime checkout. Sections labelled as original proposals
are historical context, not claims about current implementation.
Preserve its ownership, freshness, geometry and bounded-processing contracts.

- Run execution checks only on reserved Jah (`ssh jah`), using isolated files,
  processes, ROS domains and ports as applicable. Local editing, Git operations
  and static inspection are allowed. Never replace an existing deployment implicitly.
- Record exact software/configuration identities and evidence limits. Synthetic
  or software checks do not establish observed hardware or display performance.
- `rgbd_camera_interfaces` is independently installable, without a camera SDK.
  Keep reusable provider logic behind thin C++ ROS component adapters.
- Preserve image acquisition times and source generations. Reject unsupported
  geometry; do not imply calibrated accuracy or unmeasured hardware support.
- All configuration is startup-only in this slice. Consumers cannot reconfigure
  the shared device. Ordinary shutdown releases the sole session.
- Follow the user's global workflow preferences. No automatic approval ceremonies.
- New material is MIT, copyright Paul N. Beuchat. Preserve third-party notices.
- `main` is the verified baseline; `dev` is for development. Work on feature or
  maintenance branches from `dev`. Squash into `dev` after the user's finish
  confirmation. Promote `main` only with separate agreement on a coordinated
  milestone, then synchronize `dev`. Follow the user's global preferences for
  history retention and cleanup. Passing checks alone does not authorize promotion.
- Coordinate public interface/configuration changes with affected consumers.
  Update package/interface versions and `robot_deployment/sources.json` pins
  where required; record compatibility, rebuild requirements and tested revisions.
  Run focused checks on both sides of the changed contract within the authorized scope.

Verification commands and evidence limits are in `docs/verification.md`.
