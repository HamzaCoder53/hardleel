#!/bin/bash
export ROS_DOMAIN_ID=7
source /opt/ros/jazzy/setup.bash
for k in 1 2 3 4; do
  pkill -f "[l]ib/sllidar_ros2/sllidar_node"; sleep 6
  if timeout 5 ros2 topic echo /scan --once --field angle_min >/dev/null 2>&1; then echo "SCAN_FLOWING after kick $k"; break; fi
  echo "kick $k: no scan yet"; tail -1 ~/hwtest_logs/lidar.log | cut -c1-140
done
timeout 8 ros2 topic hz /scan --window 10 2>&1 | tail -2
