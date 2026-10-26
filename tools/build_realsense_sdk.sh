#!/usr/bin/env bash
# Pinned userspace SDK in a separate prefix; no apt, udev, firmware or kernel edits.
set -euo pipefail
prefix=${1:?usage: build_realsense_sdk.sh ABSOLUTE_EMPTY_PREFIX}
[[ "$prefix" = /* && ! -e "$prefix" ]] || { echo 'prefix must be absolute and absent' >&2; exit 2; }
scratch="${prefix}-build"
[[ ! -e "$scratch" ]] || { echo 'build directory already exists' >&2; exit 2; }
mkdir -p "$scratch"
git clone --depth 1 --branch v2.56.5 https://github.com/realsenseai/librealsense.git "$scratch/source"
test "$(git -C "$scratch/source" rev-parse HEAD)" = 38a41441971387197193ad3aeae3cefe6a11f2cb
cmake -S "$scratch/source" -B "$scratch/build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix" -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_DDS=OFF -DBUILD_EXAMPLES=OFF \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_TOOLS=ON -DBUILD_GLSL_EXTENSIONS=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_UNIT_TESTS=OFF -DIMPORT_DEPTH_CAM_FW=OFF
cmake --build "$scratch/build" --parallel 2
cmake --install "$scratch/build"
