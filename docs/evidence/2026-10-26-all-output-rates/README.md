# Cone-pipeline rates with IMU and standard images — 2026-10-26

This campaign measures camera acquisition, GPU cone detection and spatial
estimation with all optional camera outputs enabled and consumed. It follows the
[earlier frame-delivery and rate investigation](../2026-10-25-frame-delivery-rates/README.md),
whose throughput sweeps disabled those optional outputs.

The D455 completed ten-minute checks at native 30 and 60 Hz with complete delivery
on every measured output. OAK passed the two-minute criteria through 40 Hz and the
ten-minute 10 Hz check. Its 40 Hz confirmation lost one external frame and batch;
the subsequent 35 Hz confirmation stopped on an IMU clock-mapping error. This
campaign therefore does not establish a fully successful ten-minute OAK run above
10 Hz with all outputs enabled.

## Configuration and method

The camera and detector run in one intra-process container. A separate C++
observer receives the complete atomic RGB-D frames, detector batches, standard
RGB/depth images, both CameraInfo topics, wrapped accelerometer/gyroscope samples,
and both standard Imu topics. Image buffers are received in full; the observer
logs metadata without pixel processing, display, image encoding or recording.
Resource sampling runs in a separate controller.

The same local Fast DDS profile is applied to every participant, with a 32 MiB
shared-memory segment, 4 MiB maximum message and best-effort queues. Image queues
retain one message; IMU queues retain four. These observations do not establish
unconditional delivery to slower or additional consumers.

| Backend | Video profile | Pairing tolerance | Requested output rates |
| --- | --- | --- | --- |
| OAK | DepthAI 3.9.0; 640×640 rectified BGR and aligned depth | 5 ms | 10–60 Hz in 5 Hz steps |
| D455 | Firmware 5.17.0.10; librealsense 2.56.5; 640×480 BGR and aligned depth | 16.6 ms at 30 Hz; 8.3 ms at 60 Hz | Native 30 and 60 Hz modes only |

Each completed sweep point measures 120 seconds after startup and warm-up. Failed
startup attempts are retained separately and have no timed throughput values. Ten-minute
checks use OAK 10 Hz and its highest qualifying sweep rate, plus both native
D455 rates. IMU rates are selected independently of video: OAK requests 100 Hz
per sensor; D455 selects each sensor's lowest advertised rate up to 400 Hz.
Observed rates are reported separately from these requests.

Frame and detector counts use capture identities between matching camera-status
acquisition boundaries. Each IMU sensor uses its own admission counters and
acquisition boundaries. Within each session, standard headers are matched to
received authoritative frame or motion records. Complete standard-output
delivery therefore also requires complete authoritative delivery.

Frame and detector result age starts at RGB acquisition and ends at the external
callback. Standard depth and IMU ages start at their own acquisition times.
RGB/depth skew remains explicit; the wider D455 tolerances admit asynchronous
exposures and do not establish motion compensation. The D455 IMU's declared
5 ms tolerance for future timestamp estimates is preserved without retimestamping.

The qualifying-sweep check requires at least 98% of the requested video rate,
complete delivery on every subscribed output, at least two valid spatial results
per detector batch, and no timed rejection, overload, unavailable IMU status,
malformed image payload, duplicate message or unintended native sequence gap.
All measured points are retained even when they do not qualify. A valid spatial
result means usable depth for a detection; detector precision, recall and metric
depth accuracy are not evaluated here.

The host and laptop were restarted before this campaign, clearing the earlier
temporary build directories. Both SDKs and the camera/detector were rebuilt in
an isolated directory that survives restarts. The provider, interfaces and
detector source match the earlier campaign, and the exact archived TensorRT
engine was restored. Only the observer and controller changed to measure the
optional outputs. The scene remains lit with two visible cones, but its framing
differs from the earlier session, so cross-session differences are not a strictly
controlled measurement of the cost of enabling outputs.

## D455 two-minute sweep

Both points delivered every admitted frame and detector batch, every standard
image and CameraInfo message, and every admitted wrapped and standard IMU
sample. They had zero pair rejections, native video/IMU sequence gaps, detector
overload/failure counters, unavailable IMU statuses, duplicate messages and image
geometry/payload mismatches. Every batch contained two valid spatial results.

| Requested Hz | Delivered Hz | Admitted = frames = batches | Frame age p95 ms | Result age p50 / p95 / p99 / max ms | Frame arrival interval p95 / max ms | Valid / invalid XYZ |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 30 | 29.481 | 3,532 | 31.81 | 39.10 / 40.07 / 40.59 / 42.89 | 35.24 / 40.08 | 7,064 / 0 |
| 60 | 59.753 | 7,159 | 30.30 | 37.14 / 38.84 / 39.63 / 41.16 | 18.23 / 20.99 | 14,318 / 0 |

The nominal 30 Hz point's lower average rate was accompanied by a 34.00 ms median
arrival interval and a 40.08 ms maximum interval. It did not conceal a long pause
or missing consumer deliveries. The report retains the measured rate.

## D455 ten-minute confirmations

Both native rates delivered all admitted frames and detector batches, all four
standard image/CameraInfo streams, and all admitted wrapped and standard IMU
samples. Neither timed run had pair rejections, native video/IMU sequence gaps,
unavailable IMU statuses, detector overload/failure counters, duplicate messages,
future-timestamp violations or image geometry/payload mismatches.

| Requested Hz | Delivered Hz | Admitted = frames = batches | Frame age p95 ms | Result age p50 / p95 / p99 / max ms | Frame arrival interval p95 / max ms | Valid / invalid XYZ |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 30 | 29.962 | 17,971 | 31.69 | 38.99 / 39.93 / 40.43 / 43.24 | 34.93 / 43.90 | 35,942 / 0 |
| 60 | 59.758 | 35,855 | 29.95 | 37.02 / 38.56 / 39.09 / 41.96 | 18.21 / 21.78 | 71,710 / 0 |

Two detector replacements occurred during startup of the 60 Hz run, before the
measurement window; none occurred during its ten-minute window. Detector status
counter deltas use their own periodic sampling boundaries, so they need not equal
the identity-matched frame/batch counts above.

| Video Hz | Accelerometer samples / measured Hz | Gyroscope samples / measured Hz | Wrapped accel / gyro age p95 ms | Standard RGB / depth age p95 ms |
| ---: | ---: | ---: | ---: | ---: |
| 30 | 60,471 / 100.817 | 120,041 / 200.135 | 13.76 / 13.72 | 28.90 / 42.27 |
| 60 | 60,441 / 100.735 | 120,081 / 200.136 | 12.31 / 12.45 | 27.60 / 35.67 |

Each standard Imu topic delivered the same count as its wrapped sensor topic.
Depth ages start at depth acquisition, which precedes RGB; they are not directly
interchangeable with the RGB-based frame/result ages. Standard message callbacks
can run before the external atomic-frame callback, so their RGB ages can be lower.

At 30 / 60 Hz, average container CPU was 140.9% / 247.4% of one core and maximum
observed RSS was 720.9 / 721.2 MiB. The external observer used 15.7% / 22.3% of one
core and 73.2 / 73.6 MiB maximum observed RSS. CPU is calculated from process tick
deltas with the verified 100 Hz host clock; RSS comes from periodic samples.

## OAK two-minute sweep

Every completed point delivered all admitted atomic frames and detector batches,
all four standard image/CameraInfo streams, and all admitted wrapped and standard
IMU samples. Every batch contained exactly two valid spatial results, with no
invalid XYZ results or empty batches. There were no timed detector overload/failure
counters, IMU sequence gaps, unavailable IMU statuses, duplicate messages,
future-timestamp violations or image geometry/payload mismatches.

| Requested Hz | Delivered Hz | Admitted = frames = batches | Native video gaps | Rejected pairs | Frame age p95 ms | Result age p50 / p95 / p99 / max ms |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 10 | 10.001 | 1,200 | 0 | 0 | 43.33 | 48.94 / 50.21 / 50.73 / 51.33 |
| 15 | 15.000 | 1,797 | 0 | 0 | 43.16 | 48.85 / 49.94 / 50.56 / 51.17 |
| 20 | 20.001 | 2,400 | 0 | 0 | 43.13 | 48.76 / 49.83 / 50.54 / 52.93 |
| 25 | 25.001 | 2,995 | 0 | 0 | 43.11 | 48.76 / 49.89 / 50.37 / 51.74 |
| 30 | 30.001 | 3,595 | 0 | 0 | 51.83 | 57.23 / 58.29 / 58.93 / 61.72 |
| 35 | 35.001 | 4,193 | 0 | 0 | 52.06 | 57.54 / 58.60 / 59.19 / 61.54 |
| 40 | 40.002 | 4,793 | 0 | 0 | 52.73 | 58.19 / 59.22 / 59.69 / 64.29 |
| 45 | 42.243 | 5,060 | 331 | 0 | 202.91 | 195.73 / 209.33 / 211.94 / 226.31 |
| 50 | 36.898 | 4,420 | 1,569 | 0 | 239.18 | 228.22 / 245.69 / 247.98 / 252.00 |
| 55 | 27.802 | 3,330 | 3,233 | 25 | 212.46 | 216.61 / 219.25 / 224.74 / 229.05 |
| 60 | Not measured: readiness failed twice | — | — | — | — | — |

The qualifying range was 10–40 Hz. Increasing the request to 45 Hz or above
reduced throughput relative to the request and substantially increased source
age. These native sequence gaps occurred before publication; no published frame
was missing at the external frame or detector reader. At 45 Hz, inference p95
remained 5.72 ms while frame age p95 was already 202.91 ms. The observed latency
increase was therefore largely present before detector processing.

The lower result-age percentile at 55 Hz than at 50 Hz does not indicate better
service: throughput fell to 27.80 Hz and 25 pairs were rejected. Ages describe
admitted and delivered frames, not skipped or rejected captures. Both IMU sensors
remained near 98.9 Hz; wrapped IMU age p95 grew from about 12 ms at 10 Hz video to
46 ms at the 55 Hz request.

### OAK 60 Hz startup failures

The original 60 Hz attempt and one fresh attempt with identical settings each
failed the 60-second readiness deadline. Neither entered the two-minute
measurement window. Their final camera status snapshots recorded:

| Attempt | Admitted frames | Received pairs | Rejected pairs | Native video gaps | Accel / gyro admissions |
| --- | ---: | ---: | ---: | ---: | ---: |
| Original | 1 | 1,515 | 1,514 | 1,805 | 5,490 / 5,490 |
| Fresh retry | 2 | 1,512 | 1,510 | 1,806 | 5,489 / 5,489 |

The IMU was streaming while video was unavailable. These are startup diagnostic
counts, not timed throughput measurements. The observer did not record a reason
for each rejected pair, so this evidence does not distinguish skew rejection from
age rejection. Both attempts and their failures remain in the private records;
the confirmation-rate selection excludes this point.

## OAK ten-minute confirmations

| Requested Hz | Delivered frame Hz | Admitted | Frames / batches received | Missing frames / batches | Frame age p95 ms | Result age p50 / p95 / p99 / max ms |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 10 | 10.000 | 5,998 | 5,998 / 5,998 | 0 / 0 | 43.35 | 49.02 / 50.15 / 50.75 / 52.28 |
| 40 | 39.999 | 24,000 | 23,999 / 23,999 | 1 / 1 | 52.75 | 58.16 / 59.33 / 62.31 / 71.36 |

At 10 Hz, all standard images and CameraInfo messages arrived, as did all 59,306
admitted samples from each IMU sensor and their standard Imu counterparts. Wrapped
accelerometer/gyroscope age p95 was 12.11 / 12.16 ms; their measured rate was
98.875 Hz. There were no timed native video/IMU gaps, rejected pairs, detector
overload/failure counters, duplicates or malformed images.

The 40 Hz run missed the same capture identity in both external frame and batch
logs. Raw records place that missing capture inside the measurement window;
both boundary captures are present, ruling out a boundary-counting artifact.
This was one missing capture out of 24,000 admissions (99.9958% delivered), so
40 Hz did not pass the zero-loss long-run criterion. There were no timed native
video gaps, pair rejections or detector overload/failure counters. These records
do not establish the exact queue or scheduling cause of the external loss.

The standard-image report matched all 23,999 received atomic references, but
marks those references incomplete. It therefore does not claim complete delivery
against all 24,000 camera admissions. A separate raw RGB timestamp-window check
counted 24,000 RGB messages; the atomic/batch loss must not be described as a
proven loss on every image topic. All 59,318 admitted samples per IMU sensor and
their standard Imu counterparts arrived, with no native IMU gaps. Wrapped
accelerometer/gyroscope age p95 was 21.79 / 21.92 ms at 98.866 Hz.

One rejected pair and two native video gaps occurred during 40 Hz startup, before
the measurement window; those counters did not increase during measurement.
The observed external loss motivated one additional ten-minute check at 35 Hz,
the next lower qualifying sweep rate.

At 10 / 40 Hz, container CPU averaged 22.3% / 71.3% of one core and maximum
observed RSS was 851.0 / 849.6 MiB. Observer CPU averaged 7.4% / 19.1% of one core,
with 74.5 / 77.8 MiB maximum observed RSS.

### Interrupted OAK 35 Hz confirmation

The 35 Hz follow-up entered measurement, then stopped after approximately
499.85 seconds (8 minutes 20 seconds) with `invalid IMU clock mapping`. The camera
entered its error state and the controller stopped the session. It did not
complete the requested 600 seconds and is recorded in `failures.csv`, without
presenting a completed ten-minute rate or latency result.

The final diagnostic status recorded 17,607 admitted frames and 49,747 admissions
per IMU sensor. These totals include startup and are not a successful measurement
window. Raw records and the error log are retained. The
[motion clock guard](../../../rgbd_camera/src/core.cpp) requires positive
acquisition/receipt times, receipt no later than the current monotonic time, and
acquisition no later than receipt plus the backend's declared future tolerance.
OAK declares zero future tolerance. The rejected sample's timestamps were not
logged, so the exact violated condition and offset are not established by these
records. This failure does not establish that the clock issue is specific to
35 Hz. Provider code and timestamp settings were unchanged throughout the tests.

## Verification and preservation

- The rebuilt dual-backend camera and detector passed all five native software
  checks. The extended external observer compiled, and both cameras passed short
  hardware smoke runs with all optional outputs before the longer measurements.
  The failed hardware outcomes above remain separate from those software results.
- All 53 camera, detector and observer source hashes in the
  [source manifest](source-sha256.json) matched the tested files. The exact archived
  cone engine was reused. No provider or detector implementation changed.
- Every received detector batch in the 16 completed measurements had exactly two
  valid spatial results and no invalid XYZ results. This establishes the workload
  observed in the lit scene, not detection accuracy or metric depth accuracy.
- The selected dev17 deployment, both service process/invocation identities,
  all 1,453 installed files, all four profile files, the existing classroom Git
  state and the 15 W power mode were unchanged after testing.
- The task reservation and USB helper ended, original USB permissions were
  restored, and no task ROS processes remained. Service identities and binary
  hashes were checked again during cleanup.

These are bounded observations with one external metadata-logging consumer,
best-effort queues, the stated resolutions and a stationary scene. The OAK faults
and external loss require further diagnosis before claiming robust high-rate
operation with all outputs. No indefinite delivery, hard-real-time deadline,
motion compensation or deployment compatibility is established.

[rates.csv](rates.csv) contains the 16 completed frame/detector measurements,
including the degraded rates and the 40 Hz delivery loss.
[outputs.csv](outputs.csv) contains 128 optional-stream rows, including reference
completeness, counts, rates and ages. [failures.csv](failures.csv) records both
60 Hz startup failures and the interrupted 35 Hz confirmation. Full raw records,
scene images, build identities, installed SDKs, model and preservation audits
are retained in the private runtime/evidence archive.
