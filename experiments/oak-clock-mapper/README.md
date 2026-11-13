# Offline shared clock mapper prototype

This experiment is **not linked into the camera**. Production `MotionGate`,
camera shutdown behavior and configuration remain unchanged. It compares:

| Clock choice | Fault policy | Replay behavior |
| --- | --- | --- |
| `local` | `stop` | One shared raw-device mapping; unchanged strict IMU gate |
| `sdk` | `drop` | Keep SDK timestamps; count and discard gate-invalid samples, continue processing |
| `sdk` | `stop` | Existing strict behavior, used as the control |

Clock conversion and fault policy are separate choices in the harness. Invalid
SDK timestamps are never passed to the IMU combiner. The graceful mode does not
repair large forward SDK steps that the existing gate admits. A raw device-clock
or sequence reset always ends the prototype generation, regardless of policy.

## Scope and algorithm

One `Mapper` belongs to one camera generation and its polling owner. It observes
existing IMU dequeue receipts; no new callback handoff, service, thread, lock,
sample queue or waiting operation is required by the mapping algorithm. All image
and IMU event timestamps use the same function. A production integration would
also map exposure midpoints through that function and preserve frame associations.

Eight 250 ms bins retain minima of `receipt - device time`. After one second of
startup observations, the first mapping uses that lower envelope minus a 5 ms
margin. Startup samples are counted and discarded, never buffered. Every bin,
the mapper joins a new linear segment continuously to the previous one, slewing
toward the target offset with a one-second correction horizon and a maximum rate
change of ±5,000 ppm. It never clamps individual timestamps or revises previously
published segments. A fixed ring of 64 segments lets delayed image events use
the same past mapping; older events are unavailable. Arithmetic uses integer
nanoseconds relative to each segment's origin.

These constants are **prototype parameters**, not calibrated hardware limits.
Receipt includes unknown transport/queue delay. A lower envelope cannot separate
that delay from clock offset. A margin does not guarantee that a future latency
change cannot defeat admission, and a bounded slew can accumulate timing lag.
Any production configuration must report mapping readiness/quality, sample drops
and epoch faults; it must not imply clock accuracy from successful admission.

## Reproduction on reserved Jah

From this repository, with an absolute path to the private experiment directory:

```bash
g++ -std=c++17 -O3 -DNDEBUG -Wall -Wextra -I rgbd_camera/include \
  experiments/oak-clock-mapper/replay.cpp rgbd_camera/src/core.cpp \
  rgbd_camera/src/imu_combiner.cpp -lcrypto -o /PRIVATE/replay
python3 experiments/oak-clock-mapper/run_replay.py \
  --executable /PRIVATE/replay \
  --trace sdk39-future=/PRIVATE/TRACE_A \
  --trace sdk39-backwards=/PRIVATE/TRACE_B \
  --trace sdk312-future=/PRIVATE/TRACE_C \
  --output /PRIVATE/mapper-replay.json
```

Each private trace has complete `arrival.bin`, `dequeue.bin`, `images.bin`, their
`*-window.json` records and `complete.json` in the retained `oak-clock-raw-v1`
format. The wrapper verifies layouts, counts and zero overflow, and records input
hashes. The executable causally merges dequeue/image stages by receipt time,
preserving order within each stage. It never uses future observations. The actual
production `MotionGate` and `ImuCombiner` are linked unchanged. Dequeue time stands
in for admission time, and combiner drains follow retained provider poll groups.
Image results concern recorded acquisition events only: the traces do not contain
full exposures, pixels or pose outputs, so this is not an end-to-end VIO replay.

The self-check covers continuous increasing conversion, immutable past mappings,
bounded/expired history, exposure endpoint ordering, raw-reset refusal and SDK
drop/recovery through the unchanged gate. Benchmark timing includes observation,
two conversions and the measurement clock overhead. It is a short isolated cost
measurement, not a hard real-time bound. Replay storage itself is offline and
unbounded by the mapper's fixed-state contract.

## Observed result, 14 November 2026

The final polling-receipt prototype completed all three retained traces with
zero strict IMU clock faults, zero combined-IMU rejections/overflows, and no
backward or future image events after startup. It suppressed 198–199 initial
reports per IMU channel and 25–29 events per image stream. Local maximum combined
IMU intervals remained below 5.50 ms; maximum RGB/depth and stereo event skew were
0.101 ms and 0.028 ms. This is admission and consistency evidence, not accuracy.

SDK strict mode reproduced each terminal fault. SDK graceful mode discarded one
accelerometer and one gyroscope report per trace and finished the retained input.
Those captures end at the original fatal packet; continued hardware operation
after the fault is not observed. Recovery on a subsequent valid sample is covered
only by the synthetic check. Forward SDK steps still caused 4 and 9 combined-IMU
rejections in the two future-fault traces.

On Jah, mapper state was 1,720 bytes. Across 100,000 iterations, observation plus
two conversions measured 96 ns median, 128 ns p95, 160 ns p99 and 19.010 µs maximum.
The whole measurement loop took 16.222 ms. There is no steady-state buffering
delay, but startup suppresses one second of samples. More significantly, maximum
mapped age reached **221.929 ms for IMU and 245.712 ms for RGB** during the larger
receipt/device timing disturbances. These values are not proven physical latency;
the conservative clock estimate itself contributes to them. Slow convergence,
unknown absolute timing error and resulting VIO alignment remain reasons **not
to enable this prototype as the default**.

See the coordinated [SDK and mapper evidence](../../../robot_deployment/docs/evidence/2026-11-14-oak-sdk312/README.md)
for exact input hashes, software identities and limits. The next meaningful
qualification would need independent timing evidence and a live integration that
observes recovery beyond the original fault, including actual exposure midpoints
and VIO. More replay tuning alone would not establish those properties.
