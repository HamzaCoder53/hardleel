#!/bin/bash
# Real robot (domain 7): mapping + bag recording in the background; log ~/office_map.log
cd ~/hardleel && PI_IP=192.168.137.231 source scripts/laptop/ros_env.sh
setsid nohup ros2 launch robot_slam laptop_mapping.launch.py rviz:=true > ~/office_map.log 2>&1 < /dev/null &
sleep 25
pgrep -af "cartographer_node|ros2 bag record|collision_monitor|rviz2" | cut -c1-90
d=$(ls -dt ~/hardleel_bags/mapping_* 2>/dev/null | head -1); echo "bag: $d"; [ -n "$d" ] && du -sh "$d"
grep -E "ERROR|Traceback" ~/office_map.log | head -5
python3 scripts/laptop/topic_probe.py 4 /map /scan_filtered 2>&1 | grep Hz
