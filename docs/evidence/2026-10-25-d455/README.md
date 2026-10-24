# D455 milestone verification — 25 October 2026

The optional D455 provider passed bounded hardware acceptance on an isolated
Ubuntu 24.04 arm64 / ROS 2 Jazzy development host (Jetson Orin NX, 15 W).
The existing public OAK baseline is retained. No deployment or vehicle actuation
was part of this milestone.

## Verified configuration

- D455, USB 3.2, firmware **5.16.0.1**, librealsense **2.56.3** at
  `05d6eb7ec8efeed4d69b4025d9ff149e7132acd5`, userspace RSUSB backend.
- Firmware was explicitly upgraded from 5.13.0.55 after preserving its original
  flash. USB access was temporary; no persistent udev or kernel change was made.
- Native 640×480 BGR8/Z16 at 30 Hz; requested publication at 10 Hz;
  rectified RGB and depth transformed into RGB optical Z.
- Explicit **16.6 ms** pairing tolerance for admitted D455 runs; unchanged 5 ms
  default checked separately. Default SDK auto exposure and motion correction.
- Optional accelerometer at 100 Hz and gyroscope at 200 Hz; independent samples,
  SDK depth-oriented axes at the IMU origin, no fusion or interpolation.
- Fast DDS 2.14.6, localhost discovery, isolated ROS domain, and the checked-in
  [same-host shared-memory profile](../../../tools/fastdds_local_frames.xml).
  This profile changes transport settings per process, not system configuration.

The [source digest manifest](source-sha256.json) identifies the verified provider,
interfaces, tests and tools. Private records also identify installed binaries,
SDK builds, firmware image/backup, full logs and the unchanged detector engine.
Failed exploratory runs with SDK 2.56.5 / firmware 5.13.0.55 are retained privately
and are not final acceptance evidence.

## Builds and functional checks

The interface package built independently of either camera SDK. Fresh OAK-only,
RealSense-only and combined builds succeeded; OAK-only passed two native tests,
and RealSense-only/combined each passed three. Tests cover the shared gates and
ROS adapter, plus RealSense projection, occlusion, rectification, timestamp wrap
and epoch conversion. Synthetic tests also check the IMU future-time allowance
at 0.6 ms and 5 ms, rejection beyond the allowance, and independent loss/recovery
of IMU readiness while image delivery continues.

Real D455 checks passed standalone capture, IMU capture, composition with standard
image topics, exclusive ownership, invalid device selection, graceful stop and a
bounded slow consumer. Three measured cases each collected 120 image bundles:

| Camera case | External bundle rate (Hz) | Native sequence rate (Hz) | Bundle age p95 (ms) | CPU (% of one core) | RSS (MiB) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Standalone, slow reader also attached | 6.19 | 29.03 | 46.86 | 25.21 | 78.64 |
| Standalone with IMU | 9.91 | 29.98 | 50.51 | 38.13 | 79.94 |
| Composed with IMU and standard images | 9.83 | 29.99 | 52.64 | 42.83 | 152.29 |

These rates describe an external observer, not a lossless-delivery claim. The
standalone case's node status recorded 166 published frames while the observer
collected 120. Native 30 Hz to requested 10 Hz admission also creates sequence
gaps; those gaps do not alone measure transport loss. CPU/RSS cover the owned
process tree and exclude verification consumers; RSS is not PSS.

Observed accel/gyro rates were 100.23/196.68 Hz in the standalone IMU case and
100.25/188.40 Hz when composed. The observer received 458/897 and 620/1178 samples,
respectively, whose acquisition estimates were ahead of their recorded provider
receipt. The largest lead was **0.2896 ms**, within the declared 5 ms allowance.
Their acquisition timestamps were preserved. Provider counters reported no
over-budget IMU rejections in these final runs; counts cover different windows
from the external observer. Larger increasing estimates are individually omitted,
with independent IMU readiness expiry; clock/sequence regressions remain faults.

Observed D455 exposure skew varied from approximately 1 to 16 ms. A separate
default-tolerance run rejected 40 consecutive raw pairs at about 15.9 ms skew,
published none, and stayed observable as `unavailable: pairing_skew`. The firmware
and SDK update did not establish simultaneous exposures or eliminate SDK clock
initialization transients. The two-second clock-admission startup gate remains.

## Unchanged cone detector compatibility

The published `rgbd_object_detector` baseline
`b5a70986d1c210f4ef47cf3d3b2393b40de4dbd4` ran unchanged with its existing TensorRT
engine. Both visible cones produced valid XYZ in every measured batch:

| Layout | Batches | Valid XYZ | Observed batch rate (Hz) | Camera publication (Hz) | Source age p95 (ms) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Separate camera/detector processes | 120 | 240 | 8.94 | 10.00 | 54.98 |
| Composed with intra-process delivery | 120 | 240 | 10.00 | 9.99 | 53.04 |
| Composed exact-association run | 238 | 476 | 10.00 | 10.00 | 53.54 |

There were no empty batches, invalid XYZ, detector admission failures, stale
results or pending replacements in these measured runs. Camera removal from the
graph expired input readiness. Composed reload produced a new generation and
40 sustained batches with 80 valid XYZ at 9.99 Hz.

Ten batches were joined to their exact source bundles, independently checking
20 positions against their retained depth ROI and calibration. Valid pixel counts,
median Z, median absolute deviation and projected XYZ agreed; the numerical XYZ
comparison used 1e-9 m tolerance. This verifies software association and arithmetic,
not surveyed physical accuracy or model recall. `Frame` and `CaptureReference`
wire definitions are unchanged; `Motion` adds its explicit future-time allowance,
so motion consumers must rebuild.

## OAK regression and preservation

The combined build also passed a real OAK composed run at its unchanged 5 ms
pairing tolerance: 120 bundles at 9.75 Hz externally, 10.00 Hz native sequence
progress, 47.19 ms p95 age, and both IMU streams at 98.02 Hz. No pairs were rejected.
Standard image branches, shutdown and invalid selection checks passed.

The installed dev17 file inventory and all 1,453 file hashes, selected deployment,
runtime profile, service process/invocation identities and classroom checkout
were checked against the pre-work baseline. Detailed device/host identities,
scene images, raw measurements, firmware records and preservation audits remain
in the private development archive.

No endurance test, physical unplug test, surveyed geometry accuracy, detector
recall, cross-device synchronization, Isaac ROS deployment, browser/runtime
migration or driving result is established by this milestone.
