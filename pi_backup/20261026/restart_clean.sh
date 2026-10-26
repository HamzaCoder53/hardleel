#!/bin/bash
source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash
tmux send-keys -t mapping C-c 2>/dev/null; sleep 14; tmux kill-session -t mapping 2>/dev/null
echo "teleop processes left after stop: $(pgrep -f "[t]eleop_twist_keyboard" | wc -l)"
for p in $(pgrep -f "[t]eleop_twist_keyboard"); do kill $p; done; sleep 2
echo "teleop processes after cleanup:   $(pgrep -f "[t]eleop_twist_keyboard" | wc -l)"
python3 ~/patch_snap.py
bash -n ~/map_track.sh && echo "map_track.sh syntax OK"
~/start_mapping.sh 2>&1 | grep -E "session started|OK |health status : OK|did not reply"
sleep 25
echo "--- teleop nodes publishing /cmd_vel:"; ros2 topic info /cmd_vel -v 2>&1 | grep -E "Node name" | sort | uniq -c
ls ~/maps | grep snapshots | tail -1
echo "[power] $(vcgencmd get_throttled) events=$(journalctl -k -b --no-pager | grep -c "Undervoltage detected") $(vcgencmd measure_temp)"
