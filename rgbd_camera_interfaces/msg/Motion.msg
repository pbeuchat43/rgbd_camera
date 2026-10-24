# One sensor sample, preserving its own acquisition time. No synthetic fusion.
rgbd_camera_interfaces/CaptureReference capture
# Original vendor counter, which may wrap independently of capture.sequence.
uint64 sensor_sequence
string sensor
# accelerometer or gyroscope; other Imu fields have covariance[0] = -1.
string kind
# sensor_native; no camera/robot frame transform is implied.
string axes
sensor_msgs/Imu sample
