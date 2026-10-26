#!/bin/bash
# Keyboard driving for mapping. Default (team 26 Oct): straight to /cmd_vel -> Pi motor_bridge, no safety chain.
# With mapping safety:=true use:  TELEOP_TOPIC=/cmd_vel_teleop bash teleop.sh  (twist_mux -> smoother -> monitor)
source "$(dirname "$0")/ros_env.sh"
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=${TELEOP_TOPIC:-/cmd_vel} -p speed:=0.15 -p turn:=0.6
