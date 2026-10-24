# Version 1. Acquisition times are signed nanoseconds in clock_domain.
# Identity is (source_id, generation, stream_id, sequence), never frame_id alone.
uint16 schema_version 1
string source_id
string generation
string stream_id
uint64 sequence
int64 acquisition_ns
int64 received_ns
string clock_domain
# Fixed per-session mapping used for standard ROS header stamps.
int64 unix_offset_ns
