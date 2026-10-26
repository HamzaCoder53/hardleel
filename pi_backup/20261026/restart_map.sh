#!/bin/bash
echo "== $(date +%T) saving and stopping the old run"
OLD=$(ls -td ~/maps/logs_track_* | head -1 | sed "s|.*/logs_||")
tmux send-keys -t mapping C-c 2>/dev/null
for i in $(seq 1 20); do tmux capture-pane -p -t mapping 2>/dev/null | grep -q EXITED && break; sleep 2; done
tmux kill-session -t mapping 2>/dev/null; pkill -f "[m]ap_saver_cli"; sleep 3
echo "old run: $OLD  final map: $(ls ~/maps/$OLD.pgm 2>/dev/null) snapshots: $(ls ~/maps/${OLD}_snapshots 2>/dev/null | wc -l) files"
echo "leftover: carto=$(pgrep -f "[c]artographer_node" | wc -l) teleop=$(pgrep -f "[t]eleop_twist_keyboard" | wc -l) bridge=$(pgrep -f "[m]otor_bridge" | wc -l)"
sleep 8
echo "== $(date +%T) starting new map"
~/start_mapping.sh 2>&1 | grep -E "session started|OK |health status : OK|did not reply"
sleep 40
L=$(ls -td ~/maps/logs_track_* | head -1)
echo "== checks $(date +%T)  ($(basename $L))"
echo "lidar errors: $(grep -c "Can not start scan\|operation time out" $L/carto.log)   cartographer waiting (last 60 lines): $(tail -60 $L/carto.log | grep -c "Queue waiting")"
echo "autosave: $(tail -1 $L/autosave.log 2>/dev/null)   snapshots: $(ls $(ls -td ~/maps/*_snapshots | head -1) | wc -l) files"
echo "teleop nodes: $(pgrep -f "[l]ib/teleop_twist_keyboard" | wc -l)"
echo "load: $(uptime | sed "s/.*load average/load average/")  $(vcgencmd get_throttled) events=$(journalctl -k -b --no-pager | grep -c "Undervoltage detected") $(vcgencmd measure_temp) $(vcgencmd measure_clock arm | cut -d= -f2)"
echo FINISHED
