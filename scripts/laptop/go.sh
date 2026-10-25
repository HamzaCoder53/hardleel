#!/bin/bash
# THE single start command for phase 2 (laptop_mission.launch.py must already be running and idle).
source "$(dirname "$0")/ros_env.sh" >/dev/null
ros2 topic pub --once -w 1 /mission/start std_msgs/msg/Empty '{}' && echo "MISSION STARTED $(date +%T)"
