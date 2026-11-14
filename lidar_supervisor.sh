#!/bin/bash
# Keeps the RPLIDAR A1 publishing /scan. sllidar_node hangs (does not exit) after
# "Can not start scan" / "operation time out" when the lidar's 5 V sags at motor start,
# so watch its log and do stop -> lidar_reset.py -> restart. Standard first, Express after 3 failures
# (A1 fw 1.29: Standard ~360 pts/rev, Express ~730; Boost ~1450 is NOT used: it timed out on the robot);
# a mode the driver reports as unsupported is skipped at once.
# usage: lidar_supervisor.sh   (started by start_slam.sh; needs ROS sourced)
LOG=~/hwtest_logs/lidar.log; mkdir -p ~/hwtest_logs
# Lidar = CP2102 on hub port 1.2 (ESP32 motors = CP2102 on port 1.1). Both CP2102s share serial 0001, so
# by-id cannot tell them apart and ttyUSB0/1 swap with plug order: use the physical by-path port.
PORT=${LIDAR_PORT:-$(ls /dev/serial/by-path/*usb-0:1.2:1.0-port0 2>/dev/null | head -1)}; PORT=${PORT:-/dev/ttyUSB0}
# values saved in the tuning GUI (sllidar_node section) win over the -p defaults above
[ -s ~/.hardleel/saved_params.yaml ] && SAVED_PFILE="--params-file $HOME/.hardleel/saved_params.yaml"
MODES=(${LIDAR_MODES:-Standard Express})   # NO Boost (team, 26 Oct: Boost timed out); LIDAR_MODES overrides
fails=0
while true; do
  mode=${MODES[$(( (fails / 3) % ${#MODES[@]} ))]}
  # no reply = power/cable, not the scan mode: retry without counting it against the mode
  python3 ~/lidar_reset.py "$PORT" >/dev/null 2>&1 || { echo "$(date +%T) lidar_reset: no reply"; sleep 2; continue; }
  echo "$(date +%T) starting sllidar_node scan_mode=$mode (fails=$fails)"
  ros2 run sllidar_ros2 sllidar_node --ros-args -p serial_port:=$PORT -p serial_baudrate:=115200 \
      -p frame_id:=laser -p angle_compensate:=true -p scan_mode:=$mode $SAVED_PFILE > "$LOG" 2>&1 &
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
