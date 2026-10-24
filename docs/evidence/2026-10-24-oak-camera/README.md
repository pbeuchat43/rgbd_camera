# First camera milestone: verification summary

The bounded acquisition milestone passed native build, synthetic contract, and
short hardware checks. This summary omits private host, device, deployment and
service identities. Detailed raw reports, installed artifact hashes, environment
records and preservation audits are retained in the local development archive.

The [source manifest](source-sha256.json) identifies the implementation and final
verification tools. Those files were compared byte-for-byte with the tested
checkout. Consolidating development history does not change their bytes.
The declared SDK dependency is DepthAI 3.9.0, pinned to
`ac928141ef7750bf3082b5e1c319a115618660e7`; see the build guide for requirements.

## Checks performed

- Interface package built independently of the camera SDK.
- C++ provider, standalone executable and ROS component built successfully.
- Two native test cases passed: reusable core contracts and the ROS adapter with
  an injected synthetic provider. Checks include malformed geometry, stale/skewed
  pairs, recovery after prolonged skew, read-only parameters, exact metadata,
  unavailable IMU fields, and simulated device-loss cleanup.
- Hardware runs observed advancing RGB/depth pairs with matching calibration and
  acquisition identity in standalone and composed operation; standard image
  branches were exercised separately.
- Optional accelerometer/gyroscope delivery, independent sample identities and
  timestamp/availability semantics were observed. A non-default pairing tolerance
  was verified in output metadata.
- A second owner and an invalid device selection were rejected. Graceful shutdown
  released the camera; new starts produced distinct generations.
- Separate fast and deliberately slow external ROS consumers exercised latest-only
  bounded delivery. Existing deployment preservation checks passed.

## Interpretation and limits

External readers can drop frames. The slow-reader check maintained source progress
and recent fast-reader data, but did not establish full-rate fan-out to every
subscriber. An earlier shared Python-executor measurement also showed observer
interference; that result was not treated as a camera-rate failure. The raw
measurements and their exact conditions remain in the local archive.

Synthetic loss/recovery tests do not establish physical unplug behavior. No metric
depth or IMU accuracy ground truth, camera-to-IMU mounting calibration, orientation
fusion, endurance acceptance, deterministic latency or end-to-end zero-copy was
established. SDK calls remain synchronous and do not have a proven OS hard timeout.

RealSense, GPU cone inference, depth-derived object XYZ, ArUco, video encoding and
vehicle/driving integration were not part of this acquisition milestone. They
require their own component and integration evidence. This is a development
baseline, not a stable platform or deployment release.
