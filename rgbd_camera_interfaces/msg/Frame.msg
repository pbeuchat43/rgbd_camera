# One immutable matched pair. RGB stamp is the batch acquisition time.
rgbd_camera_interfaces/CaptureReference capture
uint64 depth_sequence
int64 depth_acquisition_ns
int64 skew_ns
int64 pairing_tolerance_ns
string calibration_id
string backend
# Only aligned_to_rgb + optical_z is supported in v1.
string alignment
string depth_convention
float64 depth_unit_m
uint16 invalid_depth_value
sensor_msgs/Image rgb
sensor_msgs/CameraInfo rgb_info
sensor_msgs/Image depth
sensor_msgs/CameraInfo depth_info
