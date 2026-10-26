#!/bin/bash
# Foxglove bridge on localhost for the sim (run inside the pixi env, with the sim already up):
#   pixi run bash scripts/sim/foxglove.sh      -> ws://localhost:8765  (open Foxglove app -> Open connection -> Foxglove WebSocket)
source "$(dirname "$0")/lib.sh"
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=127.0.0.1 port:=8765 use_sim_time:=true
