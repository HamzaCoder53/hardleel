#!/bin/bash
# Pi side of the laptop-brain split: hardware drivers only, in tmux session "hw".
#   lidar: lidar_supervisor.sh (/scan)   hw: pi_bringup.launch.py (IMU, motors, camera, drop servo)
# The laptop runs everything else (scripts/laptop/brain.sh). Same domain on both: 7.
# usage: ~/robot_hw.sh [camera:=false] [motors:=false] [camera_fps:=5.0] [cmd_timeout:=0.3]
export ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash
tmux kill-session -t hw 2>/dev/null; tmux kill-session -t hwtest 2>/dev/null; tmux kill-session -t mapping 2>/dev/null
pkill -INT -x sllidar_node 2>/dev/null; sleep 1
mkdir -p ~/hwtest_logs
tmux new-session -d -s hw -n lidar "bash ~/lidar_supervisor.sh 2>&1 | tee ~/hwtest_logs/lidar_supervisor.log"
tmux new-window -t hw:1 -n hw "ros2 launch robot_slam pi_bringup.launch.py $* 2>&1 | tee ~/hwtest_logs/robot_hw.log"
sleep 15
vcgencmd get_throttled; vcgencmd measure_temp
tail -3 ~/hwtest_logs/lidar_supervisor.log
echo "HW STARTED (tmux attach -t hw)"
