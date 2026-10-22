#!/bin/bash
# Keyboard driving for mapping: goes through twist_mux -> velocity_smoother -> collision_monitor.
source "$(dirname "$0")/ros_env.sh"
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop -p speed:=0.15 -p turn:=0.6
