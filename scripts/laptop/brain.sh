#!/bin/bash
# Laptop brain for the real robot. Start the Pi first: ssh hardleel@hardleel.local ./robot_hw.sh
#   scripts/laptop/brain.sh mapping                                  # map by hand (teleop on /cmd_vel_teleop)
#   scripts/laptop/brain.sh mission ~/maps/<name>.pbstream           # localize + Nav2 (no driving)
#   scripts/laptop/brain.sh mission ~/maps/<name>.pbstream run_mission:=true
# Extra args are passed to brain.launch.py (e.g. rviz:=false record_bag:=false). PI_IP=... to override.
source "$(dirname "$0")/ros_env.sh"
mode="${1:-mapping}"; shift
args=(mode:=$mode)
[ "$mode" = mission ] && { args+=(pbstream:="$1"); shift; }
exec ros2 launch robot_slam brain.launch.py "${args[@]}" "$@"
