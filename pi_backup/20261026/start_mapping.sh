#!/bin/bash
python3 ~/lidar_reset.py 2>&1 | tail -3
NAME=track_$(date +%Y%m%d_%H%M)
tmux new-session -d -s mapping "~/map_track.sh $NAME; echo EXITED; sleep 600"
echo "session started: $NAME"
sleep 30
tmux capture-pane -p -t mapping | grep -v "^$" | tail -4
grep -E "sllidar_node\]" ~/maps/logs_$NAME/carto.log | cut -c1-170 | tail -4
ps -eo pid,cmd | grep -E "sllidar_node|guardian_node|cartographer_node" | grep -v grep | cut -c1-80
vcgencmd measure_temp
