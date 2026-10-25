#!/bin/bash
# Copy the Pi-side code from this repo to the robot and build it there. Run from any shell with ssh
# (WSL or Git Bash). Asks for the Pi password unless SSH_ASKPASS is set up.
#   scripts/laptop/deploy_pi.sh [pi_host]        default: hardleel@192.168.137.8 (laptop hotspot)
# Copies: ros2_ws/src/{robot_slam,motor_teleop} -> ~/ros2_ws/src, pi_cam/ -> ~/pi_cam,
#         scripts/pi/{robot_hw.sh,lidar_supervisor.sh,lidar_reset.py} -> ~ ; then colcon builds the two packages.
# The previous Pi sources are kept in ~/deploy_backup_<time>.tgz.
set -e
HOST="${1:-hardleel@192.168.137.8}"
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO"
tar czf - --exclude=__pycache__ --exclude=.pytest_cache \
    ros2_ws/src/robot_slam ros2_ws/src/motor_teleop pi_cam \
    scripts/pi/robot_hw.sh scripts/pi/lidar_supervisor.sh scripts/pi/lidar_reset.py \
  | ssh "$HOST" 'set -e
    ts=$(date +%Y%m%d_%H%M%S)
    tar czf ~/deploy_backup_$ts.tgz -C ~ ros2_ws/src/robot_slam ros2_ws/src/motor_teleop pi_cam \
        robot_hw.sh lidar_supervisor.sh lidar_reset.py 2>/dev/null || true
    d=$(mktemp -d); tar xzf - -C $d
    rm -rf ~/ros2_ws/src/robot_slam ~/ros2_ws/src/motor_teleop
    cp -r $d/ros2_ws/src/robot_slam $d/ros2_ws/src/motor_teleop ~/ros2_ws/src/
    mkdir -p ~/pi_cam && cp $d/pi_cam/* ~/pi_cam/
    cp $d/scripts/pi/robot_hw.sh $d/scripts/pi/lidar_supervisor.sh $d/scripts/pi/lidar_reset.py ~/
    chmod +x ~/robot_hw.sh ~/lidar_supervisor.sh; rm -rf $d
    source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws
    colcon build --packages-select robot_slam motor_teleop 2>&1 | tail -3
    echo "DEPLOYED (backup ~/deploy_backup_$ts.tgz)"'
