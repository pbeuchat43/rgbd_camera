# External frame delivery and cone-pipeline rates — 2026-10-25

This investigation separates camera admission, external frame delivery and
detector-result delivery. Checks use the reserved development Jetson, isolated
files and ROS domains, and the existing GPU cone model. The installed runtime
is not replaced. Exact device identities, raw CSV, logs, binary hashes and host
preservation checks belong in the private development archive.

## External-reader loss

The earlier profile export used `FASTDDS_DEFAULT_PROFILES_FILE`. The installed
Fast DDS 2.14.6 recognizes `FASTRTPS_DEFAULT_PROFILES_FILE`, so the intended
large-frame transport profile was not loaded. This was verified against the
loaded library, process environment and actual shared-memory segment sizes.
See the [2.14.6 environment-variable documentation](https://fast-dds.docs.eprosima.com/en/v2.14.6/fastdds/env_vars/env_vars.html).

With the ignored variable, an isolated OAK 10 Hz run delivered only 172 of 298
admitted frames to a lightweight C++ reader. Applying the correct variable to
every participant delivered all 298 frames to each of three independent readers
(C++, minimal Python, and the existing Python validator). The D455 repeated this
result while running the cone detector in a separate process: all readers and
the detector received 298 of 298, with 596 valid XYZ results and no empty batches.

`tools/with_local_frames.sh` makes the correct per-process setup repeatable. The
XML remains at a 32 MiB shared-memory segment and 4 MiB maximum message, with
best-effort reliability and a depth-one frame queue.
Slow consumers can still lose frames by design; these results are bounded
measurements, not an unconditional DDS delivery guarantee.

## Method

Each sweep point measures 120 seconds after startup and warm-up. The camera and
GPU detector share an intra-process container. A separate C++ process subscribes
to the complete atomic RGB-D frame, detector batches and status, logging metadata
without image work. A separate controller records resource samples. The native
probe and controller are available under `tools/rate_probe` and
`tools/benchmark_rates.py`.
The [source digest manifest](source-sha256.json) identifies the final provider,
interfaces and tools. Earlier build variants are identified below; private
archives retain each exact source snapshot and corresponding binary hashes.

Counts compare consumer capture identities with `Status.frames` admission
deltas over identical acquisition boundaries. Native sequence gaps from D455
output selection are not DDS losses. Small differences between detector status
counter deltas and received-batch counts can occur at the status-window edges;
the delivery comparison uses actual batch capture IDs.

Source age is external callback time minus **RGB acquisition time**. It includes
camera processing, admission, inference where applicable, and transport. It is
not the age of the older of the two image exposures. RGB/depth timestamps and
skew remain in the raw records. The lit, stationary cone scene exercises positive
detection and spatial estimation. A first OAK sweep affected by lights turning
off was retained as diagnostic evidence and excluded from positive-workload
results; those rates were repeated after lighting was restored.

## OAK sweep

DepthAI 3.9.0; 640×640 rectified BGR and aligned depth; 5 ms pairing tolerance.
IMU and standard-image branches are disabled during these rate measurements.
Every row had zero missing frames and batches, zero rejected pairs, zero
detector overload/failure counters, zero empty batches and zero invalid XYZ.

| Requested Hz | Delivered Hz | Admitted = frames = batches | Frame age p95 ms | Result age p50 / p95 / p99 / max ms | Valid XYZ |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 10 | 10.000 | 1,198 | 41.72 | 46.81 / 48.30 / 48.96 / 51.12 | 2,584 |
| 15 | 15.000 | 1,797 | 41.34 | 46.69 / 47.89 / 48.67 / 50.87 | 3,969 |
| 20 | 19.999 | 2,396 | 41.15 | 46.63 / 47.73 / 48.30 / 50.07 | 5,244 |
| 25 | 24.999 | 2,995 | 40.86 | 46.53 / 47.42 / 47.99 / 49.64 | 6,321 |
| 30 | 30.001 | 3,595 | 49.03 | 48.64 / 55.45 / 55.95 / 57.14 | 7,458 |
| 35 | 34.999 | 4,193 | 49.12 | 54.82 / 55.58 / 56.01 / 59.03 | 8,694 |

Container CPU rose from about 15.6% to 50.1% of one core across 10–35 Hz.
Maximum observed container RSS was 681.8–699.7 MiB across the sweep. These
figures exclude the external probe/controller and are RSS, not PSS or total
system/GPU memory. The OAK 10 Hz point uses the preceding provider baseline;
the other points use the candidate with the output limit extended to 35 Hz.
The OAK acquisition path is unchanged.

### OAK ten-minute confirmations

The updated SDK/firmware candidate, before the later rate-cap extension to 60 Hz,
repeated 10 and 35 Hz for 600 seconds each. Every admitted
frame and detection batch reached the external reader. Neither run had pair
rejections, detector overload/failure counters, empty batches or invalid XYZ.

| Requested Hz | Delivered Hz | Admitted = frames = batches | Frame age p95 ms | Result age p50 / p95 / p99 / max ms | Arrival interval p95 / max ms | Valid XYZ |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 10 | 10.000 | 5,998 | 41.74 | 46.89 / 48.37 / 49.25 / 52.02 | 101.44 / 108.22 | 13,045 |
| 35 | 35.000 | 20,993 | 49.04 | 54.71 / 55.46 / 55.87 / 59.47 | 29.31 / 31.51 | 44,545 |

The 35 Hz session recorded one native sequence gap at frame 6 during startup,
before the timed window. Its native sequence-gap counter did not increase during
the measurement. Container CPU averaged 15.5% / 49.9% of one core, with maximum
observed RSS 714.8 / 712.4 MiB at 10 / 35 Hz. These are bounded stationary-scene
measurements, not evidence of indefinite or hard-real-time delivery.

### OAK 40–60 Hz extension in a dark scene

The scene became dark again during the subsequent D455 35 Hz measurement.
Saved images from both cameras confirmed this, and every OAK result below is an
empty detection batch. The network still runs, but these measurements do not
establish high-rate positive detection or XYZ workload. Geometry, the DENSITY
stereo preset, subpixel depth, model and transport settings were unchanged.

| Requested Hz | Delivered Hz | Admitted = frames = batches | Frame age p95 ms | Result age p50 / p95 / p99 / max ms |
| ---: | ---: | ---: | ---: | ---: |
| 40 | 40.000 | 4,792 | 48.93 | 54.48 / 55.27 / 55.74 / 57.93 |
| 45 | 45.006 | 5,391 | 80.12 | 77.26 / 86.35 / 91.47 / 99.89 |
| 50 | 44.311 | 5,308 | 195.63 | 188.19 / 202.00 / 204.75 / 218.46 |
| 55 | 35.143 | 4,211 | 173.10 | 172.62 / 179.42 / 188.68 / 200.24 |
| 60 | 30.002 | 3,595 | 200.72 | 198.43 / 207.11 / 208.07 / 213.17 |

All admitted frames and batches arrived, with zero pair rejections or detector
overload/failure counters. At 40 and 45 Hz, native sequence numbers had no gaps.
At 50 Hz, the measured frame sequence contained 681 missing source numbers.
Those losses preceded admission and are distinct from external delivery.

The added delay was before the provider's receive timestamp: acquisition to
provider receipt p95 rose from 45.60 ms at 40 Hz to 76.81 ms at 45 Hz and
192.35 ms at 50 Hz. Provider receipt to external frame callback remained about
3.5 ms p95, inference about 5.7 ms p95, and completed batch to external callback
about 0.4 ms p95. First/last thirds showed stable latency within each run, rather
than a continuously growing backlog. These boundaries locate the degradation
upstream of ROS delivery and detector work; they do not distinguish device
processing, synchronization, SDK queues or device-to-host transport.

For this dark-scene configuration, 40 Hz retained the low source age seen at
35 Hz. The 45 Hz point delivered the requested rate at greater age and jitter;
requests of 50 Hz and above were counterproductive. Positive cone/XYZ workload
was established only through 35 Hz at this stage; the later lit repeats below
extend that evidence.

A subsequent 600-second dark-scene confirmation at 45 Hz delivered all 26,991
admitted frames and batches at 45.000 Hz, with no timed-window source gaps,
pair rejections or detector overload/failure counters. Frame age p95 was
79.64 ms; result age p50/p95/p99/max was 77.24/85.74/91.29/99.95 ms. External
frame arrival intervals were 25.18 ms p95 and 41.69 ms maximum. CPU averaged
64.1% of one core and maximum observed RSS was 718.0 MiB. All batches were empty.
The session's three native sequence gaps and one rejected pair occurred at the
third admitted frame during startup, before warm-up and measurement.

### OAK 40–60 Hz repeat after lighting was restored

The same final candidate repeated the extension with visible cones. Every
admitted frame and batch arrived, with zero timed-window pair rejections,
detector overload/failure counters or invalid XYZ. There were no empty batches
at 40, 45, 50 or 60 Hz. The 55 Hz run had 291 empty batches, with positive
detections present again at its end; the cause of that variation was not isolated.

| Requested Hz | Delivered Hz | Admitted = frames = batches | Frame age p95 ms | Result age p50 / p95 / p99 / max ms | Valid XYZ |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 40 | 40.000 | 4,792 | 49.03 | 54.68 / 55.56 / 55.92 / 57.36 | 9,453 |
| 45 | 45.004 | 5,391 | 80.54 | 77.55 / 86.93 / 92.15 / 100.27 | 10,782 |
| 50 | 44.425 | 5,322 | 195.54 | 188.11 / 201.94 / 205.07 / 218.21 | 10,644 |
| 55 | 35.139 | 4,210 | 178.28 | 172.76 / 184.75 / 188.61 / 201.94 | 5,970 |
| 60 | 30.002 | 3,593 | 200.58 | 198.61 / 207.15 / 208.25 / 215.00 | 7,186 |

The timed native sequence-gap counters increased by 0 / 0 / 668 / 2,379 / 3,593
at 40 / 45 / 50 / 55 / 60 Hz. The 40 and 45 Hz runs therefore sustained their
requested rates with positive spatial results; higher requests did not.

Lighting did not materially alter the latency boundary observed earlier.
Acquisition to provider receipt p95 was 45.68 ms at 40 Hz and 77.27 ms at 45 Hz;
provider receipt to external frame callback remained about 3.5 ms, and inference
about 5.7 ms. With positive detections, downstream timings and zero overload
counters remained comparable. The device/SDK/transport cause of the upstream
limit remains unresolved.

A subsequent 600-second lit confirmation at 45 Hz delivered every admitted
frame and batch, with two valid cone positions in every timed batch. There were
no timed-window source gaps, pair rejections, detector overload/failure counters,
empty batches or invalid XYZ.

| Requested Hz | Delivered Hz | Admitted = frames = batches | Frame age p95 ms | Result age p50 / p95 / p99 / max ms | Arrival interval p95 / max ms | Valid XYZ |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 45 | 45.000 | 26,991 | 85.14 | 78.10 / 91.57 / 92.41 / 116.79 | 28.99 / 59.05 | 53,982 |

The session's three native sequence gaps and one rejected pair occurred before
the timed window. This confirmation establishes positive workload at 45 Hz,
while also showing greater source age and jitter than the 40 Hz sweep point.

## D455 startup fault isolated below ROS

The first D455 sweep used firmware 5.16.0.1 and SDK 2.56.3. All admitted frames
reached the external reader and detector, but some starts produced poor depth
and lower camera admission rates:

| Requested Hz | Delivered Hz | Admitted = frames = batches | Observation |
| ---: | ---: | ---: | --- |
| 10 | 9.998 | 1,197 | 2,394 valid XYZ; no invalid/empty results |
| 15 | 12.581 | 1,508 | 1,124 valid and 1,892 invalid XYZ |
| 20 | 20.000 | 2,396 | 4,792 valid XYZ; no invalid/empty results |
| 25 | 20.126 | 2,411 | Exposure/depth fault; all published frames delivered |
| 30 | 23.266 | 2,784 | Exposure/depth fault; all published frames delivered |

Three direct-SDK 30-second restarts reproduced the fault without ROS, geometry
alignment or inference. The first and third starts ran both streams at about
29.987 Hz with stable depth exposure around 24–25 ms. The middle start ran at
about 29.065 Hz, with exposure repeatedly cycling from 1 µs to 31,979 µs and
gain rising to 246 before resetting. Raw depth became sparse or saturated.
Neither stream lost native sequence numbers. Settings read before streaming
were identical, including enabled depth AE, enabled projector, disabled HDR and
RGB auto-exposure priority zero.

Thus the lower result rate was not external transport loss or GPU overload.
Zero `rejected_pairs` does not count all provider discards: unmatched native
frames and deliberately rate-selected frames are removed before the pair gate.
Changing exposure also changes exposure-midpoint pairing offsets.

The [2.56.4 release](https://github.com/realsenseai/librealsense/releases/tag/v2.56.4)
documents a D455 depth-AE fix when RGB streams. The subsequent
[2.56.5 release](https://github.com/realsenseai/librealsense/releases/tag/v2.56.5)
adds a D455 depth/IR corruption fix at 60 Hz and pairs with firmware 5.17.0.10.
The user separately approved that firmware update and isolated SDK pair. This
supersedes the earlier Isaac ROS-specific pair; it does not establish Isaac ROS
deployment compatibility.

The approved update completed and the device reported firmware 5.17.0.10.
Three subsequent direct-SDK 30-second restarts at native 30 Hz had no sequence
gaps and stable depth exposure of 24.6–25.3 ms. Both streams ran at about
29.964 Hz. A native 60 Hz check ran both streams at about 59.757 Hz, with no
sequence gaps and stable 15.366 ms depth exposure. The sparse/saturated-depth
cycle was not observed in these checks. This is repeated-start evidence for
the updated pair, not proof that every possible camera fault is eliminated.

## D455 sweep after the update

Firmware 5.17.0.10 / librealsense 2.56.5; rectified 640×480 BGR and RGB-aligned
optical-Z depth. Requests through 30 Hz use native 30 Hz capture and an explicit
16.6 ms pairing tolerance. Higher requests use native 60 Hz and 8.3 ms tolerance.
The default 5 ms tolerance is unchanged. These wider comparison settings admit
asynchronous exposures; they do not establish motion compensation.
Measured depth preceded RGB by 11.82–12.42 ms in the lit 10–30 Hz sweep and
7.17–8.18 ms in native-60-Hz measurements, including the later lit repeats.
These observed offsets exceed the unchanged default 5 ms tolerance. The reported
source ages begin at RGB acquisition; the corresponding depth exposure is older
by the measured offset.

The lit 10–30 Hz runs delivered every admitted frame and batch, with zero pair
rejections, detector overload/failure counters, invalid XYZ or empty batches.

| Requested Hz | Delivered Hz | Admitted = frames = batches | Frame age p95 ms | Result age p50 / p95 / p99 / max ms | Valid XYZ |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 10 | 9.999 | 1,198 | 29.74 | 37.18 / 38.23 / 38.70 / 40.88 | 2,396 |
| 15 | 14.999 | 1,796 | 29.66 | 36.95 / 38.07 / 38.69 / 39.80 | 3,592 |
| 20 | 20.001 | 2,396 | 29.58 | 36.83 / 37.88 / 38.37 / 40.23 | 4,792 |
| 25 | 25.003 | 2,995 | 29.65 | 36.94 / 37.97 / 38.54 / 41.04 | 5,990 |
| 30 | 29.964 | 3,589 | 29.58 | 36.82 / 37.82 / 38.39 / 40.54 | 7,178 |

Container CPU averaged 49.0–127.4% of one core across these rates, with maximum
observed RSS 554.3–583.0 MiB. D455 rectification and depth transformation run
on the host; these measurements include that work and the GPU detector.

The 35 Hz run delivered all 4,193 frames and batches at 35.000 Hz, with frame
age p95 29.42 ms and result age p50/p95/p99/max 36.76/37.73/38.32/40.31 ms.
However, the scene became dark during the run: 479 batches contained two valid XYZ
results and 3,714 were empty. It is retained as a mixed-lighting diagnostic,
not a completed positive-workload measurement at 35 Hz.

The native clocks measured about 29.96 and 59.76 Hz. Nondivisor output rates
select native exposures with nonuniform intervals: for example, the 20/25 Hz
runs had arrival-interval p95 around 67 ms. Average output frequency alone does
not imply a uniformly spaced exposure stream.

### D455 40–60 Hz extension in a dark scene

All admitted frames and batches arrived, with zero pair rejections or detector
overload/failure counters. There were no valid XYZ results. At 40 Hz, one batch
contained an invalid spatial result and 4,790 were empty; all batches at the
other rates were empty. This measures capture, alignment, inference and delivery
throughput in the dark scene, not high-rate positive cone/XYZ performance.

| Requested Hz | Delivered Hz | Admitted = frames = batches | Frame age p95 ms | Result age p50 / p95 / p99 / max ms |
| ---: | ---: | ---: | ---: | ---: |
| 40 | 39.998 | 4,791 | 30.19 | 36.65 / 38.27 / 39.14 / 40.56 |
| 45 | 44.995 | 5,391 | 29.83 | 36.68 / 37.97 / 39.11 / 40.76 |
| 50 | 49.996 | 5,990 | 30.15 | 36.70 / 38.15 / 39.16 / 41.28 |
| 55 | 55.005 | 6,590 | 29.62 | 36.66 / 37.86 / 38.75 / 41.09 |
| 60 | 59.755 | 7,170 | 29.42 | 36.34 / 37.68 / 38.46 / 43.14 |

The 60 Hz output matched the measured native clock rather than an exact 60.000
Hz. At that point, external frame arrival intervals were 18.01 ms p95 and
21.32 ms maximum. Container CPU averaged 173.5–245.7% of one core across 40–60 Hz;
maximum observed RSS was 557.0–586.5 MiB. No throughput or source-age collapse
was observed within this requested range. The selected 640×480 RGB hardware
profile is capped at native 60 Hz; higher rates were not tested.

### D455 ten-minute confirmations

These runs use the final candidate and the dark scene. IMU and standard-image
branches remain disabled during rate measurements.

| Requested Hz | Delivered Hz | Admitted = frames = batches | Frame age p95 ms | Result age p50 / p95 / p99 / max ms | Arrival interval p95 / max ms |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 10 | 10.000 | 5,997 | 29.42 | 36.63 / 37.67 / 38.27 / 40.83 | 101.41 / 106.82 |
| 60 | 59.756 | 35,841 | 29.12 | 36.31 / 37.44 / 38.22 / 40.95 | 18.05 / 22.99 |

Both runs delivered every admitted frame and batch with zero timed-window pair
rejections or detector overload/failure counters. Every batch was empty, with
no invalid XYZ. The 60 Hz run had no native sequence gaps; the 10 Hz gaps are
intentional native-rate output selection. One detector replacement in the
60 Hz session occurred around its first dozen frames, before warm-up, and did
not recur during measurement. CPU averaged 48.5% / 244.4% of one core, with
maximum observed RSS 581.8 / 585.2 MiB at 10 / 60 Hz.

### D455 35–60 Hz repeat after lighting was restored

The same final candidate repeated all six rates with visible cones. Every
admitted frame and detection batch reached the external reader. Every timed
batch contained at least two valid spatial results, with no empty batches,
pair rejections or detector overload/failure counters.

| Requested Hz | Delivered Hz | Admitted = frames = batches | Frame age p95 ms | Result age p50 / p95 / p99 / max ms | Valid / invalid XYZ |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 35 | 35.000 | 4,193 | 29.59 | 36.85 / 37.97 / 38.66 / 40.97 | 8,413 / 949 |
| 40 | 40.001 | 4,792 | 29.95 | 37.02 / 38.33 / 38.97 / 41.01 | 9,627 / 938 |
| 45 | 45.003 | 5,391 | 29.50 | 36.96 / 37.84 / 38.37 / 40.00 | 10,832 / 1,330 |
| 50 | 50.006 | 5,991 | 29.46 | 36.90 / 37.82 / 38.31 / 40.06 | 12,064 / 2,538 |
| 55 | 54.998 | 6,589 | 29.50 | 36.85 / 37.82 / 38.39 / 40.74 | 13,218 / 2,968 |
| 60 | 59.756 | 7,159 | 29.35 | 36.75 / 37.74 / 38.30 / 42.15 | 14,342 / 2,071 |

The 60 Hz request matched the measured native clock with no native sequence
gaps. One detector replacement occurred around the first eleven received frames,
three seconds before measurement began; it did not recur in the timed window.
Container CPU averaged 155.9–246.4% of one core across 35–60 Hz, with maximum
observed RSS of 556.7–587.1 MiB.

The additional detections need a separate interpretation from delivery. A
30-batch diagnostic, outside the accepted rate measurements, captured a matched
image and complete detection/spatial metadata. The two physical cones had valid
positions. An extra `blue_cone` detection at approximately pixel (364, 364),
confidence 0.55, covered blue foreground hardware. Its central depth region had
zero usable pixels and was correctly reported as `insufficient_depth` with NaN
XYZ. The diagnostic's invalid results were nearby extra blue detections with the
same reason.

Some sweep batches also contained a third valid spatial result. A valid XYZ flag
establishes usable depth for a detection, not that the detection is a real cone.
The counts above therefore retain both extra valid and invalid results. No model
thresholds or spatial filters were changed. These runs establish throughput with
positive spatial workload; they do not establish detector precision or recall.

### D455 lit ten-minute confirmations

Both lit confirmations delivered every admitted frame and batch, with at least
two valid spatial results per batch. There were no empty batches, pair rejections
or detector overload/failure counters during measurement. Additional detections
remain included in the valid/invalid XYZ counts, as discussed above.

| Requested Hz | Delivered Hz | Admitted = frames = batches | Frame age p95 ms | Result age p50 / p95 / p99 / max ms | Arrival interval p95 / max ms | Valid / invalid XYZ |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 10 | 10.000 | 5,998 | 29.55 | 36.94 / 37.96 / 38.49 / 40.66 | 101.29 / 105.85 | 12,019 / 6,608 |
| 60 | 59.756 | 35,841 | 29.29 | 36.69 / 37.66 / 38.13 / 40.82 | 18.06 / 22.51 | 71,777 / 17,314 |

The 60 Hz run had no native sequence gaps. One detector replacement occurred
at startup, 3.25 seconds before measurement, and did not recur. Container CPU
averaged 48.9% / 245.5% of one core, with maximum observed RSS 583.3 / 586.6 MiB
at 10 / 60 Hz. Depth preceded RGB by 6.91–10.83 ms at 10 Hz and 7.17–7.63 ms at
60 Hz; the explicit 16.6 / 8.3 ms comparison tolerances remained unchanged.

## Verification and preservation

- The final dual-backend build passed the core, ROS-adapter and RealSense-geometry
  tests. The external probe compiled, and the benchmark/wrapper passed syntax
  checks and real hardware execution. An invalid-device benchmark failed
  explicitly and cleaned up its owned processes without forced termination.
- Both cameras passed the standalone, IMU and composed functional checks at
  10 Hz, including slow-reader isolation, second-owner rejection, restart
  generation changes, standard-image consumers and invalid-device selection.
  These checks do not establish high-rate performance with those extra outputs.
- All 33 final source/interface/tool hashes matched the tested source and tool
  copies. Binary hashes, loaded-library records and the approved firmware backup
  remain in the private archive.
- The existing deployment selection, both service process/invocation identities,
  all 1,453 installed files and all four profile files were unchanged. Existing
  classroom Git state and the 15 W power mode were also unchanged. No runtime
  deployment, system SDK replacement, permanent USB rule or power-mode change
  was made.
- After the initial campaign, the task reservation and temporary USB helper ended,
  original USB access permissions were restored, and no task camera processes
  remained. Service identities were checked again after cleanup.
- The lit OAK and D455 follow-ups reused the same tested binaries and passed a
  preservation audit of the deployment, service identities, installed files,
  profiles, classroom Git state and power mode after all measurements. Their
  reservation and temporary USB helper ended, original USB permissions were
  restored, and no task camera processes remained. Service identities and binary
  hashes were checked again during cleanup. No implementation changed.

## What these results establish

The corrected shared-memory profile supports the observed consistent 10 Hz
external delivery on both cameras. The OAK sustained positive cone/XYZ workload
through 45 Hz, including ten-minute confirmations at 10, 35 and 45 Hz. Its lit
40 Hz sweep retained low source age; 45 Hz added throughput with greater age and
jitter, while requests of 50 Hz or more degraded output. The updated D455
sustained positive spatial workload through its native 60 Hz profile, with at
least two valid spatial results in every timed batch of the lit 35–60 Hz repeat
and the ten-minute confirmations at 10 and 60 Hz.
Additional detections on foreground hardware remain a separate model-accuracy
limitation; the report retains their valid and invalid spatial outcomes.

These are bounded, stationary-scene, same-host observations with the specified
profiles and model. They do not establish indefinite delivery, hard-real-time
deadlines, other resolutions/presets, remote transport, metric depth accuracy,
motion-compensated association or deployment/Isaac ROS compatibility.

The [machine-readable measurements](rates.csv) retain frame/result ages,
intervals, counts, native gaps, rejection/overload counters, XYZ outcomes,
exposure skew and CPU/RSS with explicit lit, mixed and dark scene labels.
