#!/bin/bash
export ROS_DOMAIN_ID=7
source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash
pkill -f sllidar_node; sleep 1
echo "reset:"; python3 ~/lidar_reset.py; sleep 3
echo "driver:"
timeout 20 ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0 serial_baudrate:=115200 frame_id:=laser angle_compensate:=true 2>&1 | grep -E "ERROR|health|scan mode|S/N|Firmware|died|running" | cut -c1-160
echo "done"; vcgencmd get_throttled
