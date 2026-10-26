#!/bin/bash
export ROS_DOMAIN_ID=7
source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash
for t in /imu/data_raw /imu/data /imu/data_lpf /odometry/filtered; do echo "--- $t"; timeout 5 ros2 topic hz $t --window 20 2>&1 | tail -1; done
echo "--- imu log"; tail -6 ~/hwtest_logs/imu.log | cut -c1-170
echo "--- carto imu topic"; grep -n "imu" ~/ros2_ws/src/robot_slam/launch/carto_mapping.launch.py | head -5; grep -n "imu_topic\|TOPIC\|remap" ~/ros2_ws/src/robot_slam/launch/carto_mapping.launch.py | head -8
