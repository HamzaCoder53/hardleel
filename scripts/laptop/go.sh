#!/bin/bash
# THE single start command for phase 2 (laptop_mission.launch.py must already be running and idle).
source "$(dirname "$0")/ros_env.sh" >/dev/null
# 3 messages: --once reaches only the first matched subscriber; mission + map_freezer both need it (repeats are ignored)
ros2 topic pub --times 3 -r 5 -w 1 /mission/start std_msgs/msg/Empty '{}' && echo "MISSION STARTED $(date +%T)"
