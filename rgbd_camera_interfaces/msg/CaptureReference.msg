# Version 2. Times are signed nanoseconds in clock_domain.
# Identity is (source_id, generation, stream_id, sequence), never frame_id alone.
uint16 schema_version 2
string source_id
string generation
string stream_id
uint64 sequence
int64 acquisition_ns
# Explicit exposure timing; zero means unknown/not applicable (e.g. motion).
# acquisition_ns retains its original backend event convention.
int64 exposure_midpoint_ns
int64 exposure_duration_ns
int64 received_ns
string clock_domain
# Fixed per-session mapping used for standard ROS header stamps.
int64 unix_offset_ns
