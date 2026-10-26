#!/bin/bash
# End of phase 1: save the live Cartographer map.
#   scripts/laptop/save_map.sh <name>            keeps Cartographer mapping (it runs on into phase 2)
#   scripts/laptop/save_map.sh <name> --finish   finish_trajectory FIRST (final optimisation, nothing half-built in
#                                                the .pbstream) - final stack; phase 2 then needs a new trajectory
#                                                (laptop_localization, or loc_monitor / supervisor start_trajectory)
# -> ~/hardleel_maps/<name>.pbstream (+ .pgm/.yaml for viewing / click tool)
source "$(dirname "$0")/ros_env.sh" >/dev/null
name="${1:-map_$(date +%H%M%S)}"; dir=~/hardleel_maps; mkdir -p "$dir"
if [ "$2" = --finish ]; then
  ids=$(ros2 service call /get_trajectory_states cartographer_ros_msgs/srv/GetTrajectoryStates 2>/dev/null \
        | python3 -c "import sys,re; t=sys.stdin.read(); i=re.search(r'trajectory_id=\[([^\]]*)\]',t); s=re.search(r'trajectory_state=\[([^\]]*)\]',t)
ids=[int(v) for v in (i.group(1).split(',') if i and i.group(1).strip() else [])]; st=[int(v) for v in (s.group(1).split(',') if s and s.group(1).strip() else [])]
print(' '.join(str(a) for a,b in zip(ids,st) if b==0))")
  for id in $ids; do
    echo "finish_trajectory $id"; timeout 60 ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: $id}" | tail -1
  done
fi
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$dir/$name.pbstream', include_unfinished_submaps: true}" | tail -2
timeout 30 ros2 run nav2_map_server map_saver_cli -f "$dir/$name" --ros-args -p map_subscribe_transient_local:=false \
  2>&1 | grep -E "Saving|saved|rror" | tail -3
ls -la "$dir/$name".* 2>/dev/null || echo "WARNING: nothing saved in $dir"
