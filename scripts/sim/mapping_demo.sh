#!/bin/bash
# Virtual SLAM step 1: scripted mapping run -> saved map + bag.   Usage: mapping_demo.sh [world] [name]
source "$(dirname "$0")/lib.sh"
WORLD="${1:-course}"; NAME="${2:-mapping_$WORLD}"
ROUTE="${ROUTE:-0.45,-0.45, 0.8,-0.7, 0.8,-2.3, -2.0,-2.3, -4.2,-2.3, -4.2,-1.0, -4.2,0.15, -3.9,-0.55, -2.3,-0.55, -1.2,-0.3, -2.3,-0.55, -4.2,-0.55, -4.2,-2.3, 0.8,-2.3, 0.8,-0.7}"
sim_start "$WORLD" || exit 1
LOGNAME_=carto bg ros2 launch rdj2026_sim mapping.launch.py
sleep 5
LOGNAME_=bag bg ros2 bag record -s mcap -o "$OUT/${NAME}_bag" /scan /imu/data_raw /tf /tf_static /clock /ground_truth /map
LOGNAME_=route ros2 run rdj2026_sim route_driver.py --ros-args -p use_sim_time:=true -p speed:=0.3 \
   -p route:="[${ROUTE//[[:space:]]/}]" 2>&1 | tee -a "$OUT/route.log" | tail -3
sleep 3
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}" >> "$OUT/carto.log" 2>&1
sleep 2
ros2 run nav2_map_server map_saver_cli -f "$OUT/$NAME" --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0 2>&1 | tail -3
ls -la "$OUT/$NAME".*
