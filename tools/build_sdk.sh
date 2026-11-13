#!/usr/bin/env bash
# Build a standalone pinned SDK; no system package/deployment changes.
set -euo pipefail
prefix=${1:?usage: build_sdk.sh ABSOLUTE_EMPTY_PREFIX}
[[ "$prefix" = /* ]] || { echo 'prefix must be absolute' >&2; exit 2; }
[[ ! -e "$prefix" ]] || { echo 'prefix must not already exist' >&2; exit 2; }
scratch="${prefix}-build"
[[ ! -e "$scratch" ]] || { echo 'build directory already exists' >&2; exit 2; }
mkdir -p "$scratch"
git clone --recursive --branch v3.12.1 --depth 1 https://github.com/luxonis/depthai-core.git "$scratch/source"
test "$(git -C "$scratch/source" rev-parse HEAD)" = da14320483864f7cdabada9e70d4d6a2115eff9a
# In 3.12.1, stable Pipeline code links the beta multi-device calibration helper.
# Keep upstream's beta-source default enabled; our camera instantiates no beta nodes.
VCPKG_MAX_CONCURRENCY=2 cmake -S "$scratch/source" -B "$scratch/build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX="$prefix" \
  -DDEPTHAI_ENABLE_CURL=OFF -DDEPTHAI_ENABLE_PROTOBUF=OFF \
  -DDEPTHAI_ENABLE_REMOTE_CONNECTION=OFF -DDEPTHAI_ENABLE_EVENTS_MANAGER=OFF \
  -DDEPTHAI_ENABLE_MP4V2=OFF -DDEPTHAI_ENABLE_APRIL_TAG=OFF \
  -DDEPTHAI_ENABLE_BACKWARD=OFF -DDEPTHAI_DYNAMIC_CALIBRATION_SUPPORT=OFF \
  -DDEPTHAI_ENABLE_DEVICE_RVC4_FW=OFF -DDEPTHAI_BUILD_BETA=ON \
  -DDEPTHAI_OPENCV_SUPPORT=OFF -DDEPTHAI_XTENSOR_SUPPORT=OFF \
  -DDEPTHAI_EMBED_FRONTEND=OFF
cmake --build "$scratch/build" --parallel 2
cmake --install "$scratch/build"
