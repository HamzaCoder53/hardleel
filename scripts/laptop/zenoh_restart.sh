#!/bin/bash
# Restart the laptop zenoh router (needed after the Pi rebooted: the old link keeps topic names but no data)
pkill -x rmw_zenohd; sleep 1
cd ~/hardleel; source scripts/laptop/ros_env.sh
sleep 2; python3 scripts/laptop/topic_probe.py 3 /scan /imu/data 2>&1 | grep -E "Hz|NO PUB"
