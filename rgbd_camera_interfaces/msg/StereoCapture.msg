# Atomic sidecar for the two standard mono8 images. Header key is the later
# midpoint, matching pinned Isaac ROS 4.6 (override_publishing_stamp=false).
uint16 schema_version 1
std_msgs/Header header
rgbd_camera_interfaces/CaptureReference left
rgbd_camera_interfaces/CaptureReference right
string calibration_id
bool rectified
int64 pairing_tolerance_ns
