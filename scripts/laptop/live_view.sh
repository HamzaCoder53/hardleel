#!/bin/bash
# Live lidar + Cartographer map in RViz (fixed frame: map). usage: scripts/laptop/live_view.sh [pi_ip]
source "$(dirname "$0")/ros_env.sh" "$1"
exec rviz2 -d "$(dirname "$0")/hardleel_live.rviz"
