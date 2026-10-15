#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 ~/lidar_reset.py || { echo "LiDAR did not reply. Unplug it, wait 10 s, replug, then try again."; exit 1; }
ros2 launch robot_slam slam.launch.py
