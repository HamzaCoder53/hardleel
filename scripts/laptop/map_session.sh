#!/bin/bash
# Manual mapping session ON THE LAPTOP (the Pi only runs the drivers: up.ps1 / robot_hw.sh).
#   bash scripts/laptop/map_session.sh <name>
# Starts laptop_mapping (every scan goes to Cartographer; the scan gate was removed 26 Oct, see
# docs/mapping_diagnosis_20261026.md), recording on, RViz, and map snapshots every 30 s. Drive with teleop.sh
# (straight to /cmd_vel). Ctrl-C stops everything; then save_map.sh <name>.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/ros_env.sh"
NAME=${1:-map_$(date +%H%M)}
ros2 launch robot_slam laptop_mapping.launch.py rviz:=true record_bag:=true \
  > ~/hardleel_maps_session_$NAME.log 2>&1 &
LAUNCH=$!
trap 'kill -INT $LAUNCH $SNAP 2>/dev/null; wait' INT TERM
sleep 15
bash "$HERE/map_snapshots.sh" "$NAME" 30 &
SNAP=$!
echo "mapping '$NAME' running (log ~/hardleel_maps_session_$NAME.log). Drive slowly; Ctrl-C to stop."
wait $LAUNCH
