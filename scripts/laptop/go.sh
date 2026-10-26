#!/bin/bash
# THE single start command for phase 2 (laptop_mission.launch.py must already be running and idle).
# Closed loop: repeats /mission/start until /mission/state leaves WAIT_START (a single message could miss the
# mission - map_freezer listens too). Prints MISSION STARTED or MISSION DID NOT START (exit 1).
source "$(dirname "$0")/ros_env.sh" >/dev/null
exec python3 "$(dirname "$0")/mission_go.py" 20
