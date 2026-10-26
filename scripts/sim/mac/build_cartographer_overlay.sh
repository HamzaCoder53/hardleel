#!/bin/bash
# macOS only: RoboStack's cartographer_node dies at start ("flag 'collect_metrics' defined more than once") and its
# packaged sources need glog/absl fixes. Build cartographer_ros 2.0.9003 from source with our patch into mac_ws/.
# Run from the repo root:  pixi run bash scripts/sim/mac/build_cartographer_overlay.sh
set -e
mkdir -p mac_ws/src && cd mac_ws/src
[ -d cartographer_ros ] || git clone -q https://github.com/ros2/cartographer_ros.git
cd cartographer_ros && git checkout -q 1bbf9af && git apply --check ../../../scripts/sim/mac/cartographer_ros_macos.patch 2>/dev/null \
  && git apply ../../../scripts/sim/mac/cartographer_ros_macos.patch; cd ../..
colcon build --packages-select cartographer_ros --cmake-args -DCMAKE_BUILD_TYPE=Release \
  "-DCMAKE_EXE_LINKER_FLAGS=-L$CONDA_PREFIX/lib -Wl,-rpath,$CONDA_PREFIX/lib -lpython3.12" "-DCMAKE_SHARED_LINKER_FLAGS=-L$CONDA_PREFIX/lib -Wl,-rpath,$CONDA_PREFIX/lib -lpython3.12"
