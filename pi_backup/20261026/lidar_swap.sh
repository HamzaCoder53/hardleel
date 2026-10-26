#!/bin/bash
export ROS_DOMAIN_ID=7
source /opt/ros/jazzy/setup.bash
tmux kill-window -t hwtest:lidar 2>/dev/null
pkill -f "[l]ib/sllidar_ros2/sllidar_node"; sleep 2
tmux new-window -t hwtest -n lidar "python3 ~/a1_lidar_node.py > ~/hwtest_logs/lidar_py.log 2>&1"
for i in $(seq 10); do sleep 3; timeout 4 ros2 topic echo /scan --once --field angle_min >/dev/null 2>&1 && { echo "SCAN_FLOWING after $((i*3))s"; break; }; done
tail -4 ~/hwtest_logs/lidar_py.log | cut -c1-150
timeout 8 ros2 topic hz /scan --window 10 2>&1 | tail -2
vcgencmd get_throttled
