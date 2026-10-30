#!/bin/bash
# The laptop mapping stack (Cartographer + sensor prep, no teleop, no RViz) in a tmux session, with a one-click reset.
#   scripts/laptop/map_stack.sh start     start it (does nothing if it is already running)
#   scripts/laptop/map_stack.sh reset     save the current map as a backup, then restart => EMPTY map, robot stays where it is
#   scripts/laptop/map_stack.sh stop      stop it
#   scripts/laptop/map_stack.sh status
# The Pi must already be up (scripts\laptop\up.ps1). RViz: scripts/laptop/lidar_view.sh. Backups: ~/hardleel_maps/reset_backup_*.
HERE="$(cd "$(dirname "$0")" && pwd)"
SESSION=hl_map
LOG=~/hardleel_map_stack.log

running() { tmux has-session -t "$SESSION" 2>/dev/null; }

start() {
  if running; then echo "map stack already running (tmux session $SESSION)"; return 0; fi
  tmux new-session -d -s "$SESSION" "source '$HERE/ros_env.sh' >/dev/null; ros2 launch robot_slam laptop_mapping.launch.py rviz:=false record_bag:=true 2>&1 | tee $LOG"
  echo "map stack started (log $LOG)"
}

stop() {
  running || { echo "map stack not running"; return 0; }
  tmux send-keys -t "$SESSION" C-c            # clean shutdown: lets the bag recorder close its file
  for _ in $(seq 15); do running || break; sleep 1; done
  running && tmux kill-session -t "$SESSION"
  echo "map stack stopped"
}

case "${1:-status}" in
  start)  start ;;
  stop)   stop ;;
  reset)
    if running; then
      name="reset_backup_$(date +%Y%m%d_%H%M%S)"
      echo "saving the current map as $name ..."
      timeout 60 "$HERE/save_map.sh" "$name" >/dev/null 2>&1 && echo "backup saved in ~/hardleel_maps/$name.*" || echo "backup skipped (nothing to save yet)"
    fi
    stop; sleep 2; start
    echo "RESET DONE - the map starts empty"
    ;;
  status) running && echo "running (tmux attach -t $SESSION)" || echo "not running" ;;
  *) echo "usage: $0 start|reset|stop|status"; exit 1 ;;
esac
