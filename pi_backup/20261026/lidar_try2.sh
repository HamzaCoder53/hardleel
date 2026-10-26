#!/bin/bash
export ROS_DOMAIN_ID=7
source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash
pkill -f "[s]llidar_node"; sleep 1
timeout 25 ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0 serial_baudrate:=115200 frame_id:=laser angle_compensate:=true scan_mode:=Standard > ~/lidar_try2.log 2>&1 &
sleep 10
ros2 topic hz /scan --window 15 &
sleep 8
kill %2 2>/dev/null
grep -E "ERROR|health|scan mode|S/N|Firmware|died|running|current" ~/lidar_try2.log | cut -c1-160
