#!/bin/bash
# Nav2 on the laptop for the hardleel robot. usage: nav.sh [map.yaml]  (default: newest ~/maps/*.yaml)
# Retries the whole launch (max 3) if Nav2's navigation half fails to come up: AMCL's first map->odom
# sometimes arrives after the planner's global costmap has given up waiting (14 Nov).
source /opt/ros/jazzy/setup.bash
source ~/hardleel_rb_ws/install/setup.bash
MAP=${1:-$(ls -t ~/maps/*.yaml | head -1)}
LOG=~/hardleel_viz/nav.log
stop_nav() {
  kill -INT "$PID" 2>/dev/null
  for i in $(seq 1 20); do kill -0 "$PID" 2>/dev/null || break; sleep 1; done
  pkill -9 -f "component_container_isolated|rf2o_laser_odometry_node" 2>/dev/null
}
trap 'stop_nav; exit 0' INT TERM
for try in 1 2 3; do
  echo "Nav2 on map: $MAP   (try $try, ROS_DOMAIN_ID=$ROS_DOMAIN_ID, $RMW_IMPLEMENTATION)"
  ros2 launch robot_slam laptop_nav.launch.py map:="$MAP" > >(tee "$LOG") 2>&1 &
  PID=$!
  while kill -0 "$PID" 2>/dev/null; do
    if grep -q "lifecycle_manager_navigation.*Failed to bring up" "$LOG"; then
      echo; echo ">>> navigation failed to start (startup race) - restarting Nav2"; stop_nav; sleep 2; continue 2
    fi
    grep -q "lifecycle_manager_navigation.*Managed nodes are active" "$LOG" && [ -z "$ready" ] && {
      ready=1; echo; echo ">>> Nav2 READY: set '2D Pose Estimate' in RViz, then send a 'Nav2 Goal'"; }
    sleep 2
  done
  exit 0
done
echo ">>> Nav2 failed to start 3 times - see $LOG"
