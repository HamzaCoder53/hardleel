#!/bin/bash
# macOS: build ros2_ws with the link flags RoboStack binaries need (libpython for the *_generator_py message libs).
# Run from the repo root:  pixi run bash scripts/sim/mac/build_ws.sh [colcon args]
set -e
cd ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  "-DCMAKE_EXE_LINKER_FLAGS=-L$CONDA_PREFIX/lib -Wl,-rpath,$CONDA_PREFIX/lib -lpython3.12" \
  "-DCMAKE_SHARED_LINKER_FLAGS=-L$CONDA_PREFIX/lib -Wl,-rpath,$CONDA_PREFIX/lib -lpython3.12" "$@"
