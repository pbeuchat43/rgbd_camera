# RealSense provider milestone

Add an explicit `backend=realsense` option alongside the existing default `oak`.
The initial RealSense hardware profile targets D455, with 640x480 RGB and depth
capture at 30 Hz, and timestamp-based admission at the requested `fps` (default
10 Hz). Native capture and delivered rates are distinct. Unsupported hardware,
geometry, timing or missing requested IMU streams fail explicitly.

The output remains the v1 atomic frame contract: owned rectified BGR8 and aligned
little-endian millimetre optical-Z depth in one virtual RGB camera. Rectify RGB
using calibrated projection; transform depth points into RGB coordinates and
store their transformed Z. SDK alignment alone retains depth-camera Z and is
insufficient for this contract. Nearest-point projection with a nearest-Z buffer
leaves holes invalid; this milestone does not invent filled depth.

Image acquisition is the device-reported exposure midpoint, mapped through the
SDK global timestamp estimate into the local monotonic domain. Preserve actual
skew and reject excessive skew/age. Host receipt is not acquisition. Timing
initialization, clock/domain discontinuities and mapping limits need explicit
verification and documentation. No cross-camera synchronization claim is implied.

Optional accelerometer and gyroscope streams retain their own native supported
rates and timestamps. The SDK expresses motion in depth-oriented axes at the IMU
origin, which must be labeled separately from raw sensor-die or RGB coordinates.

Both SDK backends must be independently selectable at build time. Keep OAK's
existing default build usable; enabling RealSense must not require TensorRT or a
detector. Test geometry/timestamp primitives with synthetic nonidentity extrinsics,
then verify real standalone/composed capture, IMU, ownership, rejection recovery,
restarts, bounded consumers and compatibility with the unchanged detector.

Reserve the development host; keep sources, SDK and runtime builds isolated.
Preserve dev17, classroom state, existing camera/detector baselines and system
configuration. Preserve the existing public OAK commit, archive new development
history locally, and publish one coherent follow-on milestone on `main` and `dev`.
Keep raw host identities, images and detailed evidence in a private archive.
