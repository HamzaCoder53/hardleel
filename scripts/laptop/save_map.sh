#!/bin/bash
# End of phase 1: save the live Cartographer map WITHOUT stopping Cartographer (it keeps running into phase 2).
#   scripts/laptop/save_map.sh <name>     -> ~/hardleel_maps/<name>.pbstream (+ .pgm/.yaml for viewing / click tool)
# The .pbstream is what laptop_localization.launch.py needs if Cartographer ever has to be restarted.
source "$(dirname "$0")/ros_env.sh" >/dev/null
name="${1:-map_$(date +%H%M%S)}"; dir=~/hardleel_maps; mkdir -p "$dir"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$dir/$name.pbstream', include_unfinished_submaps: true}" | tail -2
timeout 30 ros2 run nav2_map_server map_saver_cli -f "$dir/$name" --ros-args -p map_subscribe_transient_local:=false \
  2>&1 | grep -E "Saving|saved|rror" | tail -3
ls -la "$dir/$name".* 2>/dev/null || echo "WARNING: nothing saved in $dir"
