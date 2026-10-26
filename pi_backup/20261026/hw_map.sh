#!/bin/bash
# lidar (retry loop) + IMU + camera + Cartographer + Foxglove on ROS_DOMAIN_ID=7. No autonomy; motor bridge idle.
export ROS_DOMAIN_ID=7
source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash
tmux kill-session -t hwtest 2>/dev/null; sleep 1
cp ~/ros2_ws/src/robot_slam/launch/carto_mapping.launch.py ~/hw_map.launch.py
sed -i '24,32d' ~/hw_map.launch.py          # lidar include removed: the driver runs in its own retry loop below
grep -c sllidar_a1_launch ~/hw_map.launch.py
PICO=$(ls /dev/serial/by-id/*Raspberry_Pi_Pico* 2>/dev/null | head -1)
mkdir -p ~/hwtest_logs
tmux new-session -d -s hwtest -n lidar "python3 ~/a1_lidar_node.py > ~/hwtest_logs/lidar_py.log 2>&1"
tmux new-window -t hwtest -n imu "ros2 launch robot_slam imu.launch.py > ~/hwtest_logs/imu.log 2>&1"
tmux new-window -t hwtest -n motor "ros2 launch motor_teleop motor_bridge.launch.py port:=$PICO > ~/hwtest_logs/motor.log 2>&1"
tmux new-window -t hwtest -n cam "python3 ~/pi_cam/pi_cam_node.py > ~/hwtest_logs/cam.log 2>&1"
tmux new-window -t hwtest -n fox "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 > ~/hwtest_logs/fox.log 2>&1"
tmux new-window -t hwtest -n carto "ros2 launch ~/hw_map.launch.py autonomous_explore:=false guardian_cmd:=false > ~/hwtest_logs/carto.log 2>&1"
for i in $(seq 12); do sleep 3; N=$(ros2 topic info /scan 2>/dev/null | grep -c "Publisher count: 1"); [ "$N" = 1 ] && break; done
grep -c RESTART ~/hwtest_logs/lidar.log
timeout 8 ros2 topic hz /scan --window 10 2>&1 | tail -2
ros2 topic hz /map --window 3 2>&1 | head -1 &
sleep 6; kill %1 2>/dev/null
vcgencmd get_throttled; vcgencmd measure_temp
