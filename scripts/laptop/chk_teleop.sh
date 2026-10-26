#!/bin/bash
# Real robot (domain 7): is the teleop -> /cmd_vel chain alive? Publishes ZERO commands only.
cd ~/hardleel && PI_IP=192.168.137.237 source scripts/laptop/ros_env.sh >/dev/null
echo "--- laptop processes"
ps -eo pid,args | grep -E "twist_mux|velocity_smoother|collision_monitor|cartographer_node|teleop_twist|rmw_zenohd|bag record" | grep -v grep | cut -c1-90
echo "--- nodes"
timeout 10 ros2 node list 2>/dev/null | grep -E "twist_mux|velocity_smoother|collision_monitor|motor_bridge|teleop"
echo "--- collision monitor state"
timeout 5 ros2 topic echo --once /collision_monitor_state 2>&1 | head -4
echo "--- zero commands on /cmd_vel_teleop, watching /cmd_vel"
timeout 8 ros2 topic pub -r 10 /cmd_vel_teleop geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 &
sleep 2
timeout 5 ros2 topic hz /cmd_vel 2>&1 | grep -m2 -E "average|WARNING|does not"
wait
