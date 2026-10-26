#!/bin/bash
# Shared helpers for the sim demo scripts. Works in the pixi env (macOS) or a sourced ROS 2 Jazzy (WSL/apt).
# Usage: source lib.sh ; sim_start [world] ; ... ; sim_stop
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
OUT="${OUT:-$REPO/docs/sim_demo}"; mkdir -p "$OUT"
PIDS=()
if [ -f "$REPO/ros2_ws/install/setup.bash" ]; then source "$REPO/ros2_ws/install/setup.bash"; fi
if [ -f "$REPO/mac_ws/install/setup.bash" ]; then source "$REPO/mac_ws/install/setup.bash"; fi   # macOS cartographer fix
bg() { "$@" >>"$OUT/$LOGNAME_.log" 2>&1 & PIDS+=($!); }
sim_start() {   # world (default course), extra launch args
  local world="${1:-course}"; shift
  LOGNAME_=sim bg ros2 launch rdj2026_sim sim.launch.py gui:=false world:=$world "$@"
  local t=0; until ros2 topic echo --once /scan --field header >/dev/null 2>&1 || [ $t -ge 60 ]; do sleep 2; t=$((t+2)); done
  if [ $t -ge 60 ]; then echo "sim did not come up"; return 1; fi
  return 0
}
sim_stop() {
  for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null; done; sleep 3
  for p in "${PIDS[@]}"; do kill -9 "$p" 2>/dev/null; done
  pkill -f "gz sim" 2>/dev/null; pkill -f parameter_bridge 2>/dev/null; PIDS=()
}
trap sim_stop EXIT
