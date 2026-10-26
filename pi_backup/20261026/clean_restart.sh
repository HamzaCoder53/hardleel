#!/bin/bash
echo "== $(date +%T) stopping session"
tmux send-keys -t mapping C-c 2>/dev/null; sleep 10; tmux kill-session -t mapping 2>/dev/null
pkill -f "[m]ap_saver_cli"; pkill -f "[m]otor_bridge"; sleep 4
echo "leftover: map_saver=$(pgrep -f "[m]ap_saver_cli" | wc -l) bridge=$(pgrep -f "[m]otor_bridge" | wc -l) teleop=$(pgrep -f "[t]eleop_twist_keyboard" | wc -l) carto=$(pgrep -f "[c]artographer_node" | wc -l)"
echo "load: $(uptime | sed "s/.*load average/load average/")"; sleep 10
echo "== $(date +%T) starting"
~/start_mapping.sh 2>&1 | grep -E "session started|OK |health status : OK|did not reply|Motor bridge"
sleep 45
L=$(ls -td ~/maps/logs_track_* | head -1)
echo "== checks $(date +%T)"
echo "lidar errors: $(grep -c "Can not start scan\|operation time out" $L/carto.log)"; grep -E "sllidar_node\]" $L/carto.log | grep -E "ERROR|scan mode" | cut -c1-150 | tail -2
echo "cartographer waiting warnings in last 60 lines: $(tail -60 $L/carto.log | grep -c "Queue waiting")"
echo "autosave log: $(cat $L/autosave.log 2>/dev/null | tail -2 | tr "\n" " ")"
echo "snapshot files: $(ls $(ls -td ~/maps/*_snapshots | head -1) | wc -l)"
echo "bridge: $(grep "Connected" ~/motor_bridge.log | tail -1 | grep -o "send_rate=[0-9.]*Hz")"
echo "load: $(uptime | sed "s/.*load average/load average/")  $(vcgencmd get_throttled) events=$(journalctl -k -b --no-pager | grep -c "Undervoltage detected") $(vcgencmd measure_temp) $(vcgencmd measure_clock arm | cut -d= -f2)"
ps -eo pcpu,cmd --sort=-pcpu | sed -n 2,5p | cut -c1-80
echo FINISHED
