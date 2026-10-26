#!/bin/bash
# Save the live map every N seconds while mapping, so a bump that ruins the map later can be undone:
#   ~/hardleel_maps/snapshots/<name>/<name>_<NNN>_<HHMMSS>.{pbstream,pgm,yaml}   (never overwritten)
#   bash scripts/laptop/map_snapshots.sh <name> [period_s=30]        (Ctrl-C to stop)
# Pick the last good one afterwards and copy it to ~/hardleel_maps/<name>.* (or use it directly).
source "$(dirname "$0")/ros_env.sh" >/dev/null
NAME=${1:-map_$(date +%H%M)}; P=${2:-30}
DIR=~/hardleel_maps/snapshots/$NAME; mkdir -p "$DIR"
k=0
echo "map snapshots every ${P}s -> $DIR (Ctrl-C to stop)"
while true; do
  sleep "$P"
  k=$((k + 1)); f="$DIR/${NAME}_$(printf %03d $k)_$(date +%H%M%S)"
  timeout 20 ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
    "{filename: '$f.pbstream', include_unfinished_submaps: true}" >/dev/null 2>&1
  timeout 20 ros2 run nav2_map_server map_saver_cli -f "$f" --ros-args -p map_subscribe_transient_local:=false \
    >/dev/null 2>&1
  [ -f "$f.pgm" ] && echo "$(date +%T) snapshot $k: $(basename $f)" || echo "$(date +%T) snapshot $k FAILED (no /map?)"
done
