#!/bin/bash
# Virtual SLAM step 5: full mission rehearsal with the REAL mission node (stub classifier), phase 2 on the saved map.
#   mission_demo.sh NAME [gate=open|closed] [worker=none|x,y[,remove_after_s]] [pbstream]
#   gate=timed:S  closed until S seconds after the mission reaches GATE_DECISION, then it opens (spec: ~2 s)
#   gate=closed: a 0.6 m panel across the gated lane (x -3.02) the whole run (lidar sees a wall -> mission takes path 1)
#   worker=x,y : a 0.25 m cylinder at that MAP position from GO (removed after N s if given; map = world - 0.06 in x)
source "$(dirname "$0")/lib.sh"
NAME="${1:-mission}"; GATE=open; WORKER=none; PB="$OUT/mapping_course.pbstream"
for a in "${@:2}"; do case "$a" in gate=*) GATE=${a#gate=};; worker=*) WORKER=${a#worker=};; *.pbstream) PB=$a;; esac; done
WP="$REPO/ros2_ws/src/robot_slam/config/waypoints_course.yaml"
sim_start course odom_tf:=false || exit 1
case "$GATE" in closed|timed:*) spawn_box gate -3.02 0.07 0.2 0.05 0.60 0.4;; esac
LOGNAME_=localization bg ros2 launch robot_slam laptop_localization.launch.py sim:=true rviz:=false record_bag:=false pbstream:=$PB $EXTRA_LOC
LOGNAME_=mission bg ros2 launch robot_slam laptop_mission.launch.py sim:=true classifier:=stub stub_class:=metal waypoints:=$WP $EXTRA_MISSION
sleep 35
record_bag "$NAME" /scan_filtered /tf /tf_static /clock /ground_truth /map /cmd_vel /mission/state
python "$REPO/scripts/laptop/mission_go.py" 90
case "$GATE" in timed:*) ( python "$REPO/scripts/sim/wait_state.py" GATE_DECISION "${GATE#timed:}" > "$OUT/gate.log" 2>&1 && remove_model gate && echo "gate opened (removed)" >> "$OUT/gate.log" ) & ;; esac
if [ "$WORKER" != none ]; then
  IFS=, read -r wx wy wr <<<"$WORKER"
  spawn_cyl worker "$(python3 -c "print($wx+0.06)")" "$wy" 0.125 0.4
  [ -n "$wr" ] && ( sleep "$wr"; remove_model worker ) &
fi
t0=$(date +%s)
until grep -q "DONE: mission finished\|ABORTED" "$OUT/mission.log" || [ $(( $(date +%s) - t0 )) -ge ${MISSION_TIMEOUT:-600} ]; do sleep 5; done
echo "--- mission ended after $(( $(date +%s) - t0 )) s (gate=$GATE worker=$WORKER)"
grep -E "\] \[ *[0-9.]+s\]|blocked|Controller patience|Failed to make progress|DONE|ABORTED|gate" "$OUT/mission.log" | sed 's/.*\]: //' | cut -c1-200 | head -60 | tee "$OUT/$NAME.log"
