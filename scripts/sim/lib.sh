#!/bin/bash
# Shared helpers for the sim demo scripts. Works in the pixi env (macOS) or a sourced ROS 2 Jazzy (WSL/apt).
# Usage: source lib.sh ; sim_start [world] ; ... ; sim_stop
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
[ "$(uname)" = Darwin ] && export CYCLONEDDS_URI="file://$REPO/scripts/sim/mac/cyclonedds.xml"
OUT="${OUT:-$REPO/docs/sim_demo}"; mkdir -p "$OUT"
PIDS=()
if [ -f "$REPO/ros2_ws/install/setup.bash" ]; then source "$REPO/ros2_ws/install/setup.bash"; fi
if [ -f "$REPO/mac_ws/install/setup.bash" ]; then source "$REPO/mac_ws/install/setup.bash"; fi   # macOS cartographer fix
bg() { : >"$OUT/$LOGNAME_.log"; "$@" >>"$OUT/$LOGNAME_.log" 2>&1 & PIDS+=($!); }
sim_start() {   # world (default course), extra launch args
  local world="${1:-course}"; shift
  bash "$REPO/scripts/sim/kill_sim.sh"; sleep 2    # clean slate: leftover nodes from an earlier run break Nav2
  LOGNAME_=sim bg ros2 launch rdj2026_sim sim.launch.py gui:=false world:=$world "$@"
  local t=0; until ros2 topic echo --once /scan --field header >/dev/null 2>&1 || [ $t -ge 60 ]; do sleep 2; t=$((t+2)); done
  if [ $t -ge 60 ]; then echo "sim did not come up"; return 1; fi
  return 0
}
sim_stop() {
  for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null; done
  for i in $(seq 1 20); do alive=0; for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done; [ $alive = 0 ] && break; sleep 1; done   # let `ros2 bag record` write its metadata
  bash "$REPO/scripts/sim/kill_sim.sh"      # also the nodes the launch files started (killing only the wrappers leaves them running)
  PIDS=()
}
trap sim_stop EXIT

# --- world objects (Gazebo Harmonic, world rdj2026) -------------------------------------------------------------
spawn_box() {   # name x y z sx sy sz
  ros2 run ros_gz_sim create -world rdj2026 -name "$1" -x "$2" -y "$3" -z "$4" -string \
   "<sdf version=\"1.6\"><model name=\"$1\"><static>true</static><link name=\"l\"><collision name=\"c\"><geometry><box><size>$5 $6 $7</size></box></geometry></collision><visual name=\"v\"><geometry><box><size>$5 $6 $7</size></box></geometry></visual></link></model></sdf>" >/dev/null 2>&1
}
spawn_cyl() {   # name x y radius height
  ros2 run ros_gz_sim create -world rdj2026 -name "$1" -x "$2" -y "$3" -z "$(python3 -c "print($5/2)")" -string \
   "<sdf version=\"1.6\"><model name=\"$1\"><static>true</static><link name=\"l\"><collision name=\"c\"><geometry><cylinder><radius>$4</radius><length>$5</length></cylinder></geometry></collision><visual name=\"v\"><geometry><cylinder><radius>$4</radius><length>$5</length></cylinder></geometry></visual></link></model></sdf>" >/dev/null 2>&1
}
remove_model() { gz service -s /world/rdj2026/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 --req "name: \"$1\" type: MODEL" >/dev/null 2>&1; }

record_bag() {   # name topics...   (removes a stale directory first: rosbag2 refuses to overwrite)
  local name="$1"; shift
  rm -rf "$OUT/${name}_bag"
  LOGNAME_=bag bg ros2 bag record -s mcap -o "$OUT/${name}_bag" "$@"
}
