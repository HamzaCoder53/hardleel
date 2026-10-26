#!/bin/bash
# Stop every robot driver on the Pi (started with or without tmux): motors first, then the rest.
PAT="sllidar_ros2 sllidar_node|sllidar_ros2/lib|lidar_supervisor|imu_serial_node|imu_filter_madgwick|static_transform_publisher|motor_bridge|pi_bringup.launch|rmw_zenohd|pi_cam_node|camera_node.py|drop_servo_node|ramp_dash|run_recorder"
for p in $(pgrep -f "motor_bridge"); do kill -INT $p 2>/dev/null; done; sleep 1
for p in $(pgrep -f "$PAT"); do kill -INT $p 2>/dev/null; done; sleep 3
for p in $(pgrep -f "$PAT"); do kill -9 $p 2>/dev/null; done
tmux kill-session -t hw 2>/dev/null
echo "robot drivers left: $(pgrep -fc "$PAT")"
