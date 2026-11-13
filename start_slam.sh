#!/bin/bash
# Lidar is run by lidar_supervisor.sh (DTR-low reset + automatic restart on timeouts,
# Standard mode then Express); slam.launch.py runs SLAM Toolbox without its own lidar driver.
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
# Middleware: domain 0 + CycloneDDS (set here: tmux shells do not read .bashrc)
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
bash ~/lidar_supervisor.sh > ~/lidar_supervisor.log 2>&1 &
SUP=$!
trap 'kill $SUP 2>/dev/null; pkill -KILL -x sllidar_node 2>/dev/null' EXIT
ros2 launch robot_slam slam.launch.py use_lidar:=false
