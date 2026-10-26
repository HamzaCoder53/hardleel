#!/bin/bash
# start sensors + camera + motor bridge in tmux session hwtest (no mapping, no guardian)
export ROS_DOMAIN_ID=7
source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash
tmux kill-session -t hwtest 2>/dev/null
PICO=$(ls /dev/serial/by-id/*Raspberry_Pi_Pico* 2>/dev/null | head -1)
[ -n "$PICO" ] || { echo "NO PICO"; exit 1; }
LIDAR=0; python3 ~/lidar_reset.py | grep -q '^OK' && LIDAR=1; echo "LIDAR_OK=$LIDAR"
mkdir -p ~/hwtest_logs
tmux new-session -d -s hwtest -n imu "ros2 launch robot_slam imu.launch.py > ~/hwtest_logs/imu.log 2>&1"
[ $LIDAR = 1 ] && tmux new-window -t hwtest -n lidar "ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser > ~/hwtest_logs/lidar.log 2>&1"
tmux new-window -t hwtest -n motor "ros2 launch motor_teleop motor_bridge.launch.py port:=$PICO > ~/hwtest_logs/motor.log 2>&1"
tmux new-window -t hwtest -n cam "python3 ~/pi_cam/pi_cam_node.py > ~/hwtest_logs/cam.log 2>&1"
sleep 12
vcgencmd get_throttled; vcgencmd measure_temp
tmux ls
echo STARTED
