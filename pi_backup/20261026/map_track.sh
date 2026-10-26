#!/bin/bash
# Drive the robot with the keyboard while Cartographer maps the track.
#   ~/map_track.sh [name] [autosave_seconds]
# The map is autosaved every few seconds to ~/maps/<name>_autosave.* and
# saved for good to ~/maps/<name>.* when you press Ctrl+C.
# Live view: connect Lichtblick/Foxglove to ws://hardleel.local:8765
# The motor bridge is left running afterwards.
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
NAME=${1:-track_$(date +%Y%m%d_%H%M)}
EVERY=${2:-5}
MAPS=~/maps; LOG=$MAPS/logs_$NAME; TMP=$MAPS/.autosave_tmp; mkdir -p $LOG $TMP
PICO=$(ls /dev/serial/by-id/*Raspberry_Pi_Pico* 2>/dev/null | head -1)
[ -n "$PICO" ] || { echo "Pico (motors) not found."; exit 1; }
python3 ~/lidar_reset.py | grep -q '^OK' || { echo "LiDAR did not reply. Unplug it, wait 10 s, replug, then try again."; exit 1; }

PIDS=()
ros2 launch robot_slam imu.launch.py > $LOG/imu.log 2>&1 & PIDS+=($!)
if pgrep -f lib/motor_teleop/motor_bridge >/dev/null; then echo "Motor bridge already running - using it."
else setsid nohup ros2 launch motor_teleop motor_bridge.launch.py port:=$PICO > ~/motor_bridge.log 2>&1 < /dev/null & disown; fi
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 > $LOG/foxglove.log 2>&1 & PIDS+=($!)
sleep 3
ros2 launch robot_slam carto_mapping.launch.py autonomous_explore:=false guardian_cmd:=false > $LOG/carto.log 2>&1 & PIDS+=($!)
sleep 5

# Save /map (+ Cartographer state) to <dir>/<base>.{pgm,yaml,pbstream}
# Every call is time-limited so one stuck call can't block the next save.
save_map() {
  timeout 15 ros2 run nav2_map_server map_saver_cli -f "$1/$2" --ros-args -p save_map_timeout:=3.0 >>${3:-/dev/null} 2>&1 &&
  timeout 15 ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
    "{filename: '$1/$2.pbstream', include_unfinished_submaps: true}" >>${3:-/dev/null} 2>&1
}

# Wait until Cartographer is up (its services exist) before saving anything
for i in $(seq 30); do ros2 service list 2>/dev/null | grep -q '^/write_state$' && break; sleep 1; done

# Autosave: write to a temp folder, then move into place, so a crash mid-save
# never leaves a broken file - the previous good copy survives.
(
  N=0; SNAP=$MAPS/${NAME}_snapshots; mkdir -p $SNAP   # numbered copy of every autosave, never overwritten
  while sleep $EVERY; do
    rm -f $TMP/*
    if save_map $TMP ${NAME}_autosave && [ -s $TMP/${NAME}_autosave.pgm ]; then
      if [ "$(df --output=avail -m $MAPS | tail -1)" -gt 2000 ]; then      # keep 2 GB free on the SD card
        N=$((N+1)); ID=$(printf "%04d_%s" $N $(date +%H%M%S))
        ln $TMP/${NAME}_autosave.pgm $SNAP/map_$ID.pgm
        sed "s|^image: .*|image: map_$ID.pgm|" $TMP/${NAME}_autosave.yaml > $SNAP/map_$ID.yaml
        [ -s $TMP/${NAME}_autosave.pbstream ] && ln $TMP/${NAME}_autosave.pbstream $SNAP/map_$ID.pbstream
      fi
      mv -f $TMP/${NAME}_autosave.* $MAPS/
      echo "$(date +%T) autosaved" >> $LOG/autosave.log
    fi
  done
) & AUTOSAVE=$!

save_and_stop() {
  pkill -P $AUTOSAVE 2>/dev/null; kill $AUTOSAVE 2>/dev/null; wait $AUTOSAVE 2>/dev/null
  echo; echo "Saving final map as $MAPS/$NAME ..."
  timeout 15 ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}" >>$LOG/final_save.log 2>&1
  sleep 2
  for try in 1 2 3; do
    save_map $MAPS $NAME $LOG/final_save.log && [ -s $MAPS/$NAME.pgm ] && break
    echo "Save attempt $try failed, retrying..."; sleep 2
  done
  kill -TERM ${PIDS[@]} 2>/dev/null; wait ${PIDS[@]} 2>/dev/null
  ls -1 $MAPS/$NAME.* $MAPS/${NAME}_autosave.* 2>/dev/null; echo "Logs: $LOG"
}
trap save_and_stop EXIT

echo "Mapping started (autosave every ${EVERY}s). Live view: ws://$(hostname -I | awk '{print $1}'):8765"
echo "Drive slowly, finish where you started. Ctrl+C to stop and save."
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.3 -p turn:=1.2
