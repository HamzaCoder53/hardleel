#!/bin/bash
export ROS_DOMAIN_ID=7
source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash
tmux kill-session -t hwtest 2>/dev/null; pkill -f "[s]llidar_node"; sleep 1
for BAUD in 115200 256000; do
 for MODE in Boost Sensitivity Express Standard; do
  python3 ~/lidar_stream.py >/dev/null 2>&1; sleep 1
  timeout 16 ros2 run sllidar_ros2 sllidar_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=$BAUD -p frame_id:=laser -p angle_compensate:=true -p scan_mode:=$MODE > ~/mode_$MODE.log 2>&1 &
  sleep 9
  HZ=$(timeout 5 ros2 topic hz /scan --window 8 2>&1 | grep "average rate" | tail -1)
  echo "baud=$BAUD mode=$MODE -> ${HZ:-NO SCAN} | $(grep -o 'current scan mode.*\|Can not start scan.*\|OPERATION_TIMEOUT' ~/mode_$MODE.log | head -1 | cut -c1-110)"
  pkill -f "[s]llidar_node"; sleep 3
 done
done
grep -h "Supported\|scan mode" ~/mode_Boost.log | head -8 | cut -c1-160
vcgencmd get_throttled
