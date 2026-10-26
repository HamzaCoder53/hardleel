#!/bin/bash
# Install the one missing package and rebuild robot_slam + motor_teleop. Logs power flags at each stage.
flags() { echo "[power] $(vcgencmd get_throttled) events=$(journalctl -k -b --no-pager | grep -c "Undervoltage detected") $(vcgencmd measure_temp)"; }
echo "== start $(date +%T)"; flags
echo "== apt update"; echo hardleel | sudo -S -p "" apt-get update -qq 2>&1 | tail -3; flags
echo "== install twist-mux"; echo hardleel | sudo -S -p "" DEBIAN_FRONTEND=noninteractive apt-get install -y ros-jazzy-twist-mux 2>&1 | tail -4; flags
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
echo "== colcon build"; colcon build --symlink-install --packages-select motor_teleop robot_slam --parallel-workers 2 2>&1 | tail -15; flags
source ~/ros2_ws/install/setup.bash
echo "== executables"; ros2 pkg executables robot_slam; ros2 pkg executables motor_teleop
echo "== done $(date +%T)"; flags
