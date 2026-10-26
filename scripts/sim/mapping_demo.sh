#!/bin/bash
# Virtual SLAM step 1: scripted mapping run with the REAL stack (laptop_mapping.launch.py sim:=true: sensor prep,
# Cartographer, twist_mux, velocity smoother, collision monitor) -> map (.pgm/.yaml) + .pbstream + bag.
#   mapping_demo.sh [world] [name]       env: ROUTE="x,y, x,y, ..." (world metres), OUT=<dir>, MAPARGS="loc_v2:=true"
source "$(dirname "$0")/lib.sh"
WORLD="${1:-course}"; NAME="${2:-mapping_$WORLD}"
ROUTE="${ROUTE:-0.5,0.0, 0.8,-0.6, 0.8,-2.3, -2.0,-2.3, -4.2,-2.3, -4.2,-1.0, -4.2,0.15, -4.2,-0.55, -3.5,-0.55, -2.3,-0.55, -1.2,-0.3, -2.3,-0.55, -4.2,-0.55, -4.2,-2.3, 0.8,-2.3, 0.8,-0.7}"
sim_start "$WORLD" odom_tf:=false || exit 1
LOGNAME_=mapping bg ros2 launch robot_slam laptop_mapping.launch.py sim:=true rviz:=false record_bag:=false $MAPARGS
sleep 15
record_bag "$NAME" /scan /scan_filtered /imu/data /tf /tf_static /clock /ground_truth /map /cmd_vel
ros2 run rdj2026_sim route_driver.py --ros-args -p use_sim_time:=true -p speed:=0.3 -r cmd_vel:=/cmd_vel_teleop \
   -p route:="[${ROUTE//[[:space:]]/}]" 2>&1 | tee -a "$OUT/route.log" | tail -3
sleep 3
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$OUT/$NAME.pbstream', include_unfinished_submaps: true}" | tail -1
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}" | tail -1
sleep 3
ros2 run nav2_map_server map_saver_cli -f "$OUT/$NAME" --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0 2>&1 | tail -2
ls -la "$OUT/$NAME".*
