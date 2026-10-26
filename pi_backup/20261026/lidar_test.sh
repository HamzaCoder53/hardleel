#!/bin/bash
tmux send-keys -t mapping C-c 2>/dev/null; sleep 10; tmux kill-session -t mapping 2>/dev/null
pkill -f "[c]artographer_node|[h]ardleel_guardian|[i]mu_serial|[e]kf_node|[f]oxglove|[t]eleop_twist|[s]llidar" ; sleep 3
ps -eo pid,cmd | grep -E "[s]llidar|[c]artographer" | cut -c1-80
source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash
echo "--- test 1: driver alone, no reset"
timeout 14 ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0 serial_baudrate:=115200 frame_id:=laser angle_compensate:=true 2>&1 | grep -E "ERROR|health|scan mode|S/N|died" | cut -c1-170
