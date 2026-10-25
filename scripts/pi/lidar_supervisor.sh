#!/bin/bash
# Keeps the RPLIDAR A1 publishing /scan. sllidar_node hangs (does not exit) after
# "Can not start scan" / "operation time out" when the lidar's 5 V sags at motor start,
# so watch its log and do stop -> lidar_reset.py -> restart. Tries the densest mode first
# (A1 fw 1.29: Boost ~1450 pts/rev, Express ~730, Standard ~360) and falls back after 3 failures;
# a mode the driver reports as unsupported is skipped at once.
# usage: lidar_supervisor.sh   (env from robot_hw.sh)
LOG=~/hwtest_logs/lidar.log; mkdir -p ~/hwtest_logs
# by-id path: the FTDI adapter is ttyUSB0 or ttyUSB1 depending on plug order
PORT=${LIDAR_PORT:-$(ls /dev/serial/by-id/usb-FTDI_* 2>/dev/null | head -1)}; PORT=${PORT:-/dev/ttyUSB0}
MODES=(${LIDAR_MODES:-Boost Express Standard})
fails=0
while true; do
  mode=${MODES[$(( (fails / 3) % ${#MODES[@]} ))]}
  # no reply = power/cable, not the scan mode: retry without counting it against the mode
  python3 ~/lidar_reset.py "$PORT" >/dev/null 2>&1 || { echo "$(date +%T) lidar_reset: no reply"; sleep 2; continue; }
  echo "$(date +%T) starting sllidar_node scan_mode=$mode (fails=$fails)"
  ros2 run sllidar_ros2 sllidar_node --ros-args -p serial_port:=$PORT -p serial_baudrate:=115200 \
      -p frame_id:=laser -p angle_compensate:=true -p scan_mode:=$mode > "$LOG" 2>&1 &
  pid=$!
  # healthy = no error line and the process alive; check every second
  while kill -0 $pid 2>/dev/null; do
    if grep -q -i "not supported" "$LOG"; then
      echo "$(date +%T) scan_mode=$mode not supported, next mode"; fails=$(( (fails / 3 + 1) * 3 - 1 )); break
    fi
    if grep -q -E "Error|Can not|cannot" "$LOG"; then
      echo "$(date +%T) lidar error: $(grep -m1 -E 'Error|Can not|cannot' "$LOG" | cut -c1-120)"
      break
    fi
    grep -q -m1 "current scan mode" "$LOG" && [ -z "$announced" ] && { grep -m1 "current scan mode" "$LOG"; announced=1; fails=0; }
    sleep 1
  done
  announced=
  kill -INT $pid 2>/dev/null; sleep 2; kill -KILL $pid 2>/dev/null; pkill -KILL -x sllidar_node 2>/dev/null
  fails=$((fails+1)); sleep 1
done
