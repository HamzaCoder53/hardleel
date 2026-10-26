#!/bin/bash
# Manual mapping session ON THE LAPTOP (the Pi only runs the drivers: up.ps1 / robot_hw.sh).
#   bash scripts/laptop/map_session.sh <name>
# Starts laptop_mapping with the scan gate ON (no scans into the map while tilted > 3 deg or turning fast, e.g.
# after a bump), recording on, RViz, and map snapshots every 30 s. Drive with drive.sh / teleop on
# /cmd_vel_teleop (through the collision monitor). Ctrl-C stops everything; then save_map.sh <name>.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/ros_env.sh"
NAME=${1:-map_$(date +%H%M)}
ros2 launch robot_slam laptop_mapping.launch.py rviz:=true record_bag:=true scan_gate:=true \
  > ~/hardleel_maps_session_$NAME.log 2>&1 &
LAUNCH=$!
trap 'kill -INT $LAUNCH $SNAP 2>/dev/null; wait' INT TERM
sleep 15
bash "$HERE/map_snapshots.sh" "$NAME" 30 &
SNAP=$!
echo "mapping '$NAME' running (log ~/hardleel_maps_session_$NAME.log). Drive slowly; Ctrl-C to stop."
wait $LAUNCH
