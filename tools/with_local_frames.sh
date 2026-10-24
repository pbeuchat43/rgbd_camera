#!/usr/bin/env bash
# Run one same-host ROS process with the supported Fast DDS large-frame profile.
# Apply to the camera, external readers and detector; changes no system settings.
set -euo pipefail
[[ $# -gt 0 ]] || { echo 'usage: with_local_frames.sh COMMAND [ARGUMENTS...]' >&2; exit 2; }
directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Fast DDS 2.14.x / ROS Jazzy uses the FASTRTPS name. FASTDDS_DEFAULT_PROFILES_FILE
# belongs to newer releases and is silently ignored by the verified 2.14.6 SDK.
export FASTRTPS_DEFAULT_PROFILES_FILE="$directory/fastdds_local_frames.xml"
exec "$@"
