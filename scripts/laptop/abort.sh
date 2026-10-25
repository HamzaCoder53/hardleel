#!/bin/bash
# Emergency: stop the mission at once (the robot also stops by itself 0.3 s after commands stop).
source "$(dirname "$0")/ros_env.sh" >/dev/null
ros2 topic pub --once /mission/abort std_msgs/msg/Empty '{}'
ros2 topic pub --times 5 -r 10 /cmd_vel geometry_msgs/msg/Twist '{}'
