#!/bin/bash
# Soak test monitor (Pi): CPU, temperature, throttling, disk, bag growth every 5 s for N minutes -> CSV + summary.
#   bash ~/scripts/soak_monitor.sh [minutes=10] [out=~/soak_$(date).csv]
# Drive (teleop) during the soak with full recording on. Verdict lines: max temp, any throttle flag, CPU load.
# If the camera's 1080p recording makes it struggle (temp > 80 C or throttled), drop the video to 720p.
MIN=${1:-10}; OUT=${2:-~/soak_$(date +%Y%m%d_%H%M%S).csv}
echo "t,cpu_pct,load1,temp_c,throttled,free_gb,runs_mb" > $OUT
end=$(( $(date +%s) + MIN * 60 ))
prev=($(head -1 /proc/stat))
while [ $(date +%s) -lt $end ]; do
  sleep 5
  cur=($(head -1 /proc/stat))
  idle=$(( ${cur[4]} - ${prev[4]} )); tot=0; for i in 1 2 3 4 5 6 7; do tot=$(( tot + ${cur[$i]} - ${prev[$i]} )); done
  prev=("${cur[@]}")
  cpu=$(( tot > 0 ? 100 * (tot - idle) / tot : 0 ))
  temp=$(awk '{printf "%.1f", $1/1000}' /sys/class/thermal/thermal_zone0/temp)
  thr=$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)
  free=$(df -BG --output=avail ~ | tail -1 | tr -dc 0-9)
  runs=$(du -sm ~/runs 2>/dev/null | cut -f1)
  echo "$(date +%T),$cpu,$(cut -d' ' -f1 /proc/loadavg),$temp,$thr,$free,${runs:-0}" >> $OUT
done
awk -F, 'NR>1 {n++; c+=$2; if ($2>cm) cm=$2; if ($4>tm) tm=$4; if ($5!="0x0") th=th" "$1"="$5} END {
  printf "SOAK %d samples: CPU mean %.0f%% max %d%%, temp max %.1f C, throttle flags:%s\n", n, c/n, cm, tm, (th==""?" none":th)}' $OUT
echo "csv: $OUT"
