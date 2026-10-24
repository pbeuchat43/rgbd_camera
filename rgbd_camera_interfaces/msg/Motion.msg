# One sensor sample, preserving its own acquisition time. No synthetic fusion.
rgbd_camera_interfaces/CaptureReference capture
# Original vendor counter, which may wrap independently of capture.sequence.
uint64 sensor_sequence
# Maximum admitted acquisition-minus-receipt estimate: OAK 0, D455 5000000 ns.
# Admission policy, not a measured synchronization accuracy guarantee.
# Acquisition/header timestamps are never clamped to receipt.
int64 future_tolerance_ns
string sensor
# accelerometer or gyroscope; other Imu fields have covariance[0] = -1.
string kind
# Backend declaration: sensor_native (OAK) or sdk_depth_oriented_at_imu_origin
# (RealSense, x-right/y-down/z-forward). Neither implies RGB/robot origin.
string axes
sensor_msgs/Imu sample
