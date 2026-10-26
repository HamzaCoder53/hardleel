#!/bin/bash
source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash
ps -eo pid,cmd | grep -E "[s]llidar|[c]artographer|[f]inish_traj" | cut -c1-80
echo "--- kernel usb"; (journalctl -k --no-pager 2>/dev/null || dmesg 2>/dev/null) | grep -i -E "usb|ftdi|ttyUSB|under-voltage|voltage" | tail -12
echo "--- vcgencmd throttled"; vcgencmd get_throttled
echo "--- reset then driver (full log)"
python3 ~/lidar_reset.py | cut -c1-20
timeout 12 ros2 run sllidar_ros2 sllidar_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=115200 -p frame_id:=laser -p angle_compensate:=true 2>&1 | cut -c1-200 | head -12
