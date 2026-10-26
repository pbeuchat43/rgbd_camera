# OAK timing and delivery follow-up — 2026-10-26/27

This follows the [all-output rate campaign](../2026-10-26-all-output-rates/README.md).
It investigates the interrupted OAK IMU run, the single external frame/batch loss
at 40 Hz, and the 60 Hz startup rejections. The cameras remained connected to the
freshly restarted development host, viewing two stationary cones in a lit room.
Only one camera streamed at a time. The existing dev17 deployment was preserved.

## Clock policy and diagnostic limits

The previous 35 Hz session stopped on `invalid IMU clock mapping` without logging
the offending sample. That sample cannot be reconstructed from the accepted-sample
CSV. The nearest accepted sample in that run was only 18.312 microseconds behind
host receipt; ordinary completed sessions had millisecond margins.

New diagnostics record sensor, sequence, acquisition/receipt/current timestamps,
the allowance and previous acquisition time when the clock guard fails. Ten-minute
control runs with the original zero allowance passed at both 35 and 40 Hz. Thus
this session did not reproduce or establish the exact offset of the historical
offending sample.

The pinned SDK's `IMUReport::getTimestamp()` returns the stored report timestamp;
its repeated calls do not perform a new conversion. `dai::Clock` is
`std::chrono::steady_clock`. Luxonis describes continuous device-to-host clock
estimation, with a USB synchronization figure expressed statistically rather than
as a strict maximum error. An exact acquisition-before-receipt requirement is
therefore stronger than that SDK contract.

OAK now declares the existing motion interface's **5 ms future-estimate
allowance**. Acquisition timestamps and standard headers are preserved. A larger
lead, nonpositive timestamp, impossible receipt ordering or regression remains a
terminal OAK clock fault. Image admission and the 5 ms RGB/depth pairing limit are
unchanged. This allowance is an explicit admission policy, not a measured clock
accuracy guarantee. The historical failure is consistent with the removed strict
assumption; its individual sample remains unavailable.

The adapter regression replays a 0.5 ms lead and checks that consumers receive
the declared allowance, original acquisition time and corresponding standard IMU
header. Existing core checks retain the exact 5 ms boundary, rejection beyond it,
regression and strict image-clock cases. Synthetic checks establish the policy;
hardware observations are reported separately.

References: [SDK IMU timestamp](https://github.com/luxonis/depthai-core/blob/v3.9.0/include/depthai/pipeline/datatype/IMUData.hpp),
[SDK host-clock synchronization](https://docs.luxonis.com/software-v3/depthai/depthai-components/device#host-clock-syncing).

## OAK 60 Hz rejection diagnosis

With the unchanged DENSITY/subpixel/aligned 640×640 profile and all optional
outputs, the 60 Hz detector composition again failed readiness. The final sampled
status recorded one admitted frame, 1,503 pairs, 1,502 rejections and 1,799 native
video gaps. The retained status details identify `pairing_skew`, with depth often
about 67–83 ms older than RGB. Representative RGB/depth ages were about 200/270 ms,
below the unchanged 500 ms age limit.

Removing the detector while retaining the camera profile and optional outputs
also failed to provide a usable steady video stream. Its 30-second timed window
had **zero admitted frames and 813 rejected pairs**. Its empty frame-age statistics
are not zero latency, and zero expected image messages do not establish a useful
stream. IMU continued independently.

This isolates the rejection ahead of detection. The current profile accumulates
latency and loses synchronization under the 60 Hz request; the host correctly
rejects those bundles. These observations do not identify a specific internal SDK
queue defect. The pinned RVC2 DENSITY preset includes left/right checking, hole
filling and adaptive median filtering, with five-bit subpixel depth. Generic
standalone stereo FPS figures do not characterize this full aligned RGB pipeline.

Together with the earlier 45–55 Hz throughput degradation, this supports using
the tested 10–40 Hz steps for the current OAK profile. Sustained 60 Hz requires a
separately evaluated profile change; widening skew or freshness limits would
change the association contract. The D455 remains the verified native 60 Hz path.

References: [pinned RVC2 preset](https://github.com/luxonis/depthai-core/blob/v3.9.0/src/pipeline/node/StereoDepth.cpp#L210),
[subpixel defaults](https://github.com/luxonis/depthai-core/blob/v3.9.0/include/depthai/pipeline/datatype/StereoDepthConfig.hpp#L62).

## External delivery at 40 Hz

Re-examining the previous missing capture showed normal surrounding provider
receipt cadence. Across the enclosing status intervals, camera admissions and
both detector received/processed counters each advanced by eight, without any
detector rejection, replacement, stale or superseded count. The external frame
reader instead had a 67.189 ms arrival interval. This is consistent with an
external-reader scheduling or transport delay while internal detection continued;
the old records cannot identify the exact pause or queue.

The observer uses one executor thread for all subscribed streams and synchronous
CSV writes. Its image and batch subscriptions retain one message by default.
Diagnostics now record file-flush duration and detector acquisition progress,
and `--reader-depth` permits a bounded reader-only comparison. Camera and detector
queues are unchanged.

The ten-minute 40 Hz control delivered all 23,992 frames and batches. Its maximum
recorded flush duration was 0.732 ms, so it did not reproduce the previous long
pause. The corresponding 35 Hz control delivered all 20,992 frames and batches,
with maximum flush duration 2.215 ms. Every received batch in both controls had
two valid spatial results and no invalid XYZ results. Both controls delivered all
standard image/CameraInfo messages and admitted wrapped/standard IMU samples.

## Ten-minute observations

| Policy | Requested Hz | Admitted = frames = batches | Delivered Hz | Frame age p95 ms | Result age p95 ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| Original zero allowance, added diagnostics | 35 | 20,992 | 35.000 | 53.77 | 61.90 |
| Original zero allowance, added diagnostics | 40 | 23,992 | 40.001 | 54.14 | 62.15 |
| Declared 5 ms allowance | 35 | 20,994 | 35.001 | 53.41 | 61.60 |
| Declared 5 ms allowance | 40 | 23,993 | 40.001 | 54.06 | 62.15 |

The updated 35 Hz run delivered all four standard image/CameraInfo streams and
all 59,307 admitted samples from each IMU sensor, including their standard Imu
counterparts. It had no timed native video/IMU gaps, pair rejections, detector
overload/failure counters, unavailable IMU statuses, duplicates, malformed images
or declared future-timestamp violations.

The updated 40 Hz run likewise delivered all four standard streams and all
59,303 samples from each IMU sensor and their standard counterparts, with the
same zero-error counters. Both updated runs had exactly two valid spatial results
per batch and no invalid XYZ. Neither contained an OAK acquisition timestamp
ahead of receipt, so the positive-lead acceptance path is established by the
synthetic regression, not by a reproduced OAK clock fault in these runs.

These are bounded observations with one external reader. They do not establish
indefinite lossless delivery or a hard latency deadline.

## Controlled reader pauses

A diagnostic-only observer variant paused its single callback executor for
75 ms, once during each 60-second 40 Hz measurement. Its middleware receive
threads continued. Only the observer's queue depth changed between the two
attempts; production camera and detector queues stayed at depth 1.

| Observer image/batch depth | Admitted | Frames / batches received | Missing frames / batches | Maximum frame / result age ms |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 2,392 | 2,390 / 2,389 | 2 / 3 | 65.16 / 69.73 |
| 4 | 2,391 | 2,391 / 2,391 | 0 / 0 | 129.55 / 127.69 |

Internal camera/detector rejection and overload counters remained zero. This
demonstrates external queue replacement during an executor stall while the
internal pipeline continues. It does not prove the cause of the historical pause.
Depth four retained older frames and increased maximum source age; it is a
bounded diagnostic comparison, not an unconditional delivery solution.

IMU subscriptions remained at depth four in both attempts and lost three / four
samples per sensor during the respective callback pauses. Their standard-output
references are marked incomplete. Standard-image completeness is also marked
incomplete for the depth-one attempt because atomic references were missing.
Zero relative missing counts must not be read as complete camera-to-reader
delivery when those references are incomplete.

Two earlier control attempts paused the entire observer process for 75 ms. They
retained every frame and batch at both queue depths, with increased maximum ages.
They also paused middleware threads, so they do not isolate executor queue
replacement. All four deliberate-pause attempts remain in the tables, labeled
`intentional_reader_pause=True`; none is an uninterrupted acceptance run.

Consumers should keep callbacks short and choose an explicit bounded queue and
source-age budget when they need to tolerate scheduling pauses. The provider's
freshness-oriented production defaults are unchanged.

## Ordinary D455 60 Hz launch

The [launch recipe](../../running-d455-60.md) was exercised through ordinary
`ros2 run` and `ros2 component load` commands, with the tested 8.3 ms pairing
tolerance, intra-process composition, optional outputs and local DDS profile.
The first attempt loaded the camera but the next CLI process failed to discover
the container within its default 0.5-second graph-discovery interval. That attempt
is retained as a launch failure, with no timed result. The recipe now explicitly
uses `--no-daemon --spin-time 2` for each loading command.

The corrected commands completed a 30-second window with all **1,781** frames and
batches delivered at **59.747 Hz**, 27.98 ms p95 frame age and **37.36 ms p95 result
age**. All four standard image/CameraInfo streams, all 3,009 accelerometer samples,
all 5,963 gyroscope samples, and their standard counterparts arrived. There were
no timed camera rejections, native sequence gaps, unavailable IMU statuses,
detector overload/failure counters, duplicates, malformed images or declared
clock-tolerance violations.

Every batch contained exactly two valid spatial results. There were also **172
additional invalid spatial results**. The zero-invalid-XYZ screen therefore
failed for this recipe check, and remains explicitly false in the private check
record; the launch and delivery checks passed. Invalid spatial results retain
their 2D detections and carry an invalid flag/reason, as required by the
[detector contract](https://github.com/pbeuchat43/rgbd_object_detector/blob/main/docs/interface-v1.md).
The metadata observer did not retain those individual reasons. These are not
dropped frames, and this check does not establish detector precision, recall or
metric depth accuracy. The earlier ten-minute D455 results remain separate
observations with their original scene and screening outcomes.

## Verification and preservation

- All five native camera/detector checks passed after the code change, including
  the future-estimate adapter regression. The standard and diagnostic observers
  compiled successfully. Detector implementation and the archived engine were
  unchanged.
- The [source manifest](source-sha256.json) records 54 source/configuration files
  for each of the diagnostic and final variants, plus pinned dependency commits.
  Final implementation hashes matched the tested files.
- All 1,453 installed dev17 files, four profile files, deployment selection,
  service process/invocation identities, pre-existing classroom Git state and
  15 W power setting were unchanged after testing.
- Task ROS processes ended, the reservation was released, and the temporary USB
  helper restored the D455's original permissions. Service identities and tested
  binary hashes were verified again during cleanup.

[rates.csv](rates.csv) contains ten completed windows, including the camera-only
zero-output window and deliberately paused readers. [outputs.csv](outputs.csv)
contains 80 optional-stream rows with reference completeness. [failures.csv](failures.csv)
retains the OAK 60 Hz readiness failure and the first D455 CLI discovery failure.
Full raw records, both observer variants, diagnostic and final runtime binaries,
SDK installation prefixes, engine, logs and preservation records are retained in
the private archive. The exact historical OAK clock fault and original external
pause were not reproduced; the policy and controlled queue observations above
state the limits of the conclusions.
