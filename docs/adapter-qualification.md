# Camera-adapter qualification checklist

Use this checklist when adding or substantially changing a camera provider.
It records the scenarios to qualify, not results or support claims. The planned
ZED 2i and Orbbec integrations are unqualified; the exact Orbbec model, both SDK
selections and their usable stream combinations remain to be established.

The [camera interface](interface-v1.md) defines ownership, geometry, timestamps
and identity. The [verification guide](verification.md) supplies existing build
and probe procedures. `rgbd_camera` owns provider qualification;
`robot_deployment` owns installation, named profiles, mounting transforms and
composed-workload acceptance. Use the corresponding deployment checkout's
[verification procedures](https://github.com/pbeuchat43/robot_deployment/blob/dev/tests/README.md)
for the installed checks below. These procedures currently name OAK/D455 modes;
extend their admitted selections for a new provider rather than claiming they
already exercise it. This document changes no runtime contract or numerical gate.

## Define the candidate and evidence first

- [ ] Record exact model/revision, private device identity, firmware, SDK/source
  commits, build flags and artifact hashes; host/module, OS, ROS/RMW, GPU/runtime,
  power mode, USB controller/link/cable, and source/deployment revisions. Record
  SDK installation and redistribution requirements. Do not implicitly flash
  firmware or replace a persistent deployment to run a check.
- [ ] Fill a capability matrix for **each admitted profile**: native and output
  RGB/depth/stereo dimensions, rates, formats and raw/rectified convention;
  independent/combined IMU; exposure controls; required host processing; and
  simultaneous streams. Mark Off, VO and VIO separately as supported, unsupported
  or unqualified. SDK advertising and a model name are not qualification.
- [ ] Record each stream's timestamp source, units, clock domain, acquisition
  event versus exposure midpoint, wrap period, sequence semantics and calibration
  provenance. Mark unknown fields explicitly. A shared USB connection or host
  timestamp does not prove cross-sensor or cross-device synchronization.
- [ ] Set the intended workload, warmup, measurement window, repetitions and
  numerical acceptance criteria before measuring. Include rates, latency tails,
  loss/overload, pose coverage, resource growth, recovery outage and any physical
  accuracy requirement. Preserve a failed result if a later revision changes a
  criterion. Unsupported or untested combinations stay outside the qualified set.
- [ ] Follow the reservation/isolation rules in the verification guide. Keep
  private imagery, serials, calibration backups and credentials outside Git.
  Plan cleanup and retained evidence before injecting failures.

## 1. Build, discovery and device ownership

- [ ] Build the interfaces without vendor SDKs. Build the new backend with its
  declared dependencies and verify that disabling it removes those requirements
  from other backend/interface builds. Exercise the standalone and composed ROS
  adapter where supported. Keep SDK types behind the provider boundary.
- [ ] Discover with zero, one and multiple candidates, explicit valid/invalid
  identities, missing permissions, occupied hardware and incompatible firmware
  or profiles. Selection must be deterministic, ambiguity explicit and unsupported
  requests rejected; never silently choose another camera or downgrade a profile.
- [ ] Confirm one device owner with multiple consumers. A second owner must fail
  cleanly. Subscribe/unsubscribe and slow readers must not reopen or reconfigure
  the camera. Check bounded provider poll work and owned output-buffer lifetime.
- [ ] Exercise successful start/stop, failed open, partial initialization,
  readiness timeout and repeated explicit restart. Device/SDK resources and locks
  must release. Settings remain startup-only; a restart creates a fresh generation.
  Terminal faults remain observable with hardware closed, without automatic
  reconnect or silent workload restart.

## 2. RGB-D and geometry with odometry Off

- [ ] Verify authoritative atomic frames, optional standard images/CameraInfo,
  encoding, stride, payload size and independent RGB/depth sequence/timing fields.
  Confirm exact capture/calibration identity survives detections and overlays;
  delayed inference must never use the latest unrelated depth or mutate shared RGB.
- [ ] Verify conversion to the existing aligned-depth contract: `bgr8` RGB and
  little-endian `16UC1` depth in millimetres of **RGB optical Z**, zero invalid.
  Distinguish range from optical Z and native depth resolution from the aligned
  output grid. Check holes, out-of-range values, collisions and occlusions without
  inventing depth. Equal image dimensions do not prove correspondence.
- [ ] Check axes, handedness, length units and transform direction; validate
  intrinsics/distortion and crop/resize/rectification for every admitted profile.
  Use synthetic nonidentity geometry for arithmetic and measured hardware targets
  for alignment/depth error across distances and image locations. Report those
  two kinds of evidence separately, including target uncertainty.
- [ ] Exercise missing, stale, skewed, duplicate and malformed inputs, empty
  scenes and invalid spatial estimates. Check omission counters and readiness
  expiry rather than treating process liveness as valid capture. Compare output
  selection against native capture sequences: intentional rate selection is not
  transport loss. Off must not enable unrequested stereo/interpolation work.

## 3. Stereo VO and optional VIO

- [ ] Verify the actual left/right convention, calibrated baseline, distortion,
  rectification state and matching CameraInfo. Verify real exposure midpoints,
  later-midpoint stereo sidecar and exact metadata/image association, including
  reordered/delayed arrivals. The current VO input contract admits at most **1 ms
  midpoint skew**; do not relax it or invent midpoints to admit a new device.
- [ ] For supported IMU output, verify sensor axes/origins, units (`m/s²`, `rad/s`),
  independent acquisition times, actual rates, gaps and duplicates. Missing IMU
  must not silently select VO when VIO was requested; disabled IMU remains absent.
  Preserve unknown orientation/covariance semantics rather than manufacturing data.
- [ ] Verify combined IMU at the gyro timestamp using only bracketing acceleration,
  with bounded queues, declared bracket-age limits and counted rejection/overflow.
  Test sensor starvation and independent stereo/IMU readiness expiry. Do not
  extrapolate. Measure camera-to-IMU alignment; vendor clock claims alone are not
  bounds. OAK's 200 Hz request and OAK/D455's different interpolation limits are
  backend choices, not defaults to copy into every future provider.
- [ ] Verify exact-stamp estimates and upstream success join against the actual
  stereo metadata, bounded retention, stale-output rejection and fresh startup
  readiness. Check positive/negative axis motion and new estimator identity after
  restart. Keep mapping/localization, loop closure and unsolicited odometry TF
  disabled under the current VO/VIO integration contract.

## 4. Clocks, loss and recovery

- [ ] Capture startup, warmup and sustained numeric traces for all enabled streams:
  raw device time/sequence, SDK-mapped time, exposure timing, host receipt and
  publication/reader arrival. Add bounded queue-arrival/dequeue traces when needed
  to distinguish SDK clock changes, device loss, host stalls and observer drops.
  Verify fixed session ROS-clock mapping and truthful unknown exposure fields;
  never replace acquisition with receipt or infer depth timing from RGB.
- [ ] Exercise small invalid mapped estimates, positive/negative mapping steps,
  startup counter settling, genuine raw resets and supported counter wraps.
  Use deterministic synthetic cases first, then scoped instrumented hardware
  cases where needed. Keep raw-reset/sequence faults terminal under the current
  contract; a documented wrap is distinct from a reset, and a raw sampling gap
  is distinct from a mapping jump. Do not conceal errors by sorting/clamping time.
- [ ] Under `recover`, confirm invalid samples are omitted/countable without
  advancing accepted watermarks. A mapped discontinuity holds publication, clears
  temporal state, rotates generation and resumes only after fresh stable inputs,
  using the same open camera session. Verify `strict` terminates as declared.
  Record the new provider's applicable policies and justification; the existing
  5 ms allowances/step thresholds and two-second settling windows are not measured
  accuracy guarantees or automatic evidence for a new SDK.
- [ ] Observe a real raw-counter wrap where relevant, with sufficient capture to
  cross it and a post-wrap window. Host uptime and simulated overflow alone do not
  prove that the device counter wrapped. If it cannot be observed, record it as
  untested. Also measure timing under load and over a duration suited to clock
  drift and memory growth; a short successful run is not endurance evidence.
- [ ] Exercise unplug/replug, SDK/process failure and explicit restart, including
  replacement by another same-model unit and switching between camera backends.
  Respect explicit selection and the existing no-auto-reconnect contract. New
  source/calibration/generation identity must invalidate old joins while preserving
  prior recording files and starting separate generation files or an explicit new
  session. Invalid replacement preparation must leave the current runtime run intact.
- [ ] Carry at least one controlled mapped-clock step through the full workload.
  Verify fresh camera and estimator generations, supervised estimator/adapter
  restart, no stale accepted poses, measured pose outage and sustained resumed
  tracking. Confirm viewer decoding resumes and each recording track uses disjoint
  files for old/new generations within its retained session. Measure the recording
  gaps independently of pose gaps. Retain failed attempts and cleanup records.

## 5. Delivery, installed workload and shutdown

- [ ] Establish camera-only and Off baselines for each intended profile, then
  matched VO/VIO comparisons for supported modes. Compare native/Docker only when
  both are supported. Hold scene, lighting, exposure, transport and reader settings
  constant; record changes. Account for camera SDK GPU work alongside detector,
  estimator and encoder resource use. Repeat existing-camera checks where shared
  code or interfaces change.
- [ ] Measure external readers as well as intra-process delivery, with slow or
  paused consumers and CPU/I/O pressure. Separate admitted captures from received,
  processed, encoded and finalized outputs. Report rate, median/p95/p99/max age,
  per-window tails, sequence gaps and queue counters. Verify the observer itself
  is not introducing the apparent failure. Record CPU/GPU, memory trend, USB/link
  limits, temperature and throttling where available.
- [ ] Exercise the normal pinned installer and verified rerun, dependency identity,
  profile validation and supported launch combinations. Freeze exact installed
  sources/SDKs/models/engines/backend settings. Check unsupported selections before
  stopping an active run; do not substitute a private build for release evidence.
- [ ] Run both detectors, VIO (or the explicitly supported mode), a finite decoding
  WebRTC viewer and RGB/depth/combined-overlay recording together. A 15-minute
  measured run after warmup is a useful initial reference from the existing
  campaign; choose and record longer coverage when the claimed risk needs it.
  Confirm exact-capture overlays, pose-bracket coverage and fresh health throughout.
- [ ] Verify recording session identity, completed manifest/state, frame indexes,
  segment ledgers/file sizes, per-track timestamp ordering and one generation per
  file. Compare encoded versus finalized counts and retain all drop/rejection
  categories. Decode representative files from every track and generation, stating
  the sampling extent. A completed manifest alone does not establish playback.
- [ ] Exercise viewer disconnect, recorder/estimator failure, cancellation and
  manager recovery as relevant. Include graceful shutdown and stop during startup
  or recovery. Allow cleanup to finish and then inspect owned native processes,
  services/cgroups, camera handles and containers. No remaining resource ownership
  may be hidden by a successful child exit or a vanished launcher. Reconcile failed
  fixture cleanup explicitly; do not overwrite the failed attempt with the retry.

## 6. Physical acceptance before relying on poses

- [ ] Measure camera-to-vehicle mount and verify internal camera/IMU transforms,
  noise assumptions and timing independently. Keep internal sensor calibration
  separate from deployment-owned mount calibration.
- [ ] Measure known translations and rotations, stationary bias, scale/axis errors,
  trajectory drift, tracking loss/relocalization behavior and pose continuity
  across explicit resets. Use suitable reference measurements and declared error
  and outage limits. A fresh upstream success flag is not calibrated confidence.
- [ ] Evaluate texture/lighting changes, exposure and motion blur, vibration,
  low-feature scenes and occlusion; check detection/depth quality at the selected
  resolution and intended working distances. Use supervised motion experiments;
  this checklist does not authorize actuator control or qualify vehicle control.

## Record the outcome

For each scenario retain its ID/profile, procedure or command, exact candidate,
measurement boundaries, criterion, result (`pass`, `fail`, `not run`, or
`not applicable` with reason), numeric evidence reference and cleanup result.
Identify unmodified release runs, instrumented hardware, synthetic fixtures and
operator observations separately. Publish a redacted summary with evidence hashes;
retain failed attempts. State exactly which modes/profiles are qualified and what
remains open. Rebuild affected consumers for an interface change and update
deployment selections only with accurate tested identities.

Existing reference campaigns illustrate the scenarios, not acceptance of new
hardware:

- [Matched Off/native/Docker comparison](https://github.com/pbeuchat43/robot_deployment/blob/dev/docs/evidence/2026-11-13-visual-odometry/README.md):
  rates, latency overhead and pose brackets under controlled conditions. Its
  29 Hz/98% and comparative latency gates belong to that stated 30 Hz campaign;
  do not silently apply or weaken them for a different profile.
- [Clock recovery and RealSense wrap evaluation](https://github.com/pbeuchat43/robot_deployment/blob/dev/docs/evidence/2026-11-14-clock-recovery/README.md):
  distinguish injected steps, observed wraps, firmware restoration and failed cleanup.
- [Standard release and full-workload qualification](https://github.com/pbeuchat43/robot_deployment/blob/dev/docs/evidence/2026-11-15-vio-release-qualification/README.md):
  two 900-second baselines, controlled +182 ms recovery, resumed decoding,
  recording generation boundaries and final ownership audit. Its observed
  4.75-second pose and 2.25-second recording gaps are measurements, not universal
  recovery limits or proof of suitability for driving.
