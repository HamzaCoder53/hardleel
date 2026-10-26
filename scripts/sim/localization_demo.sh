#!/bin/bash
# Virtual SLAM steps 3+4: localization on the saved map (real laptop_localization.launch.py sim:=true) + Nav2 goals
# (real laptop_mission.launch.py nav2 stack) scored against the simulator's truth. Optional recovery (teleport) test.
#   localization_demo.sh [world] [pbstream] [goals...]        env: TELEPORT_AFTER=N  EXTRA_LOC="loc_v2:=true ..."
source "$(dirname "$0")/lib.sh"
WORLD="${1:-course}"; PB="${2:-$OUT/mapping_course.pbstream}"; shift 2
GOALS="${*:-loading_metal terrain_1 terrain_2 terrain_3 junction path1_1 path1_2 finish}"
sim_start "$WORLD" odom_tf:=false || exit 1
LOGNAME_=localization bg ros2 launch robot_slam laptop_localization.launch.py sim:=true rviz:=false record_bag:=false pbstream:=$PB $EXTRA_LOC
LOGNAME_=mission bg ros2 launch robot_slam laptop_mission.launch.py sim:=true classifier:=stub waypoints:=$REPO/ros2_ws/src/robot_slam/config/waypoints_course.yaml
sleep 30
NAME="${NAV_NAME:-nav_$WORLD}"
record_bag "$NAME" /scan_filtered /tf /tf_static /clock /ground_truth /map /cmd_vel /plan /local_plan
TP=(); [ -n "$TELEPORT_AFTER" ] && TP=(--teleport-after "$TELEPORT_AFTER" ${TELEPORT_TO:+--teleport-to="$TELEPORT_TO"})
PYTHONPATH="$REPO/scripts/sim:$PYTHONPATH" python "$REPO/scripts/sim/nav_goals.py" $GOALS --out "$OUT/$NAME.json" --waypoints "${WAYPOINTS:-$REPO/ros2_ws/src/robot_slam/config/waypoints_course.yaml}" "${TP[@]}" | tee -a "$OUT/$NAME.log"
