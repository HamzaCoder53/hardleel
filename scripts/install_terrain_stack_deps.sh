#!/bin/bash
# Step 1 of the terrain-navigation work order: install packages and update the source packages on the Pi.
# NOT run automatically. Run it on the robot (it needs sudo and internet):   bash ~/install_terrain_stack_deps.sh
# It never starts a node and never touches the motors.
set -e
source /opt/ros/jazzy/setup.bash

sudo apt update
# Note: the velocity smoother is the Nav2 one (ros-jazzy-nav2-velocity-smoother), not ros-jazzy-velocity-smoother.
sudo apt install -y \
  ros-jazzy-twist-mux \
  ros-jazzy-nav2-velocity-smoother ros-jazzy-nav2-collision-monitor ros-jazzy-nav2-bringup \
  ros-jazzy-imu-filter-madgwick ros-jazzy-rosbag2-py ros-jazzy-rosbag2 \
  ros-jazzy-cartographer-ros ros-jazzy-robot-localization

cd ~/ros2_ws/src
if [ -d sllidar_ros2/.git ]; then
  (cd sllidar_ros2 && git pull --ff-only)
elif [ ! -d sllidar_ros2 ]; then
  git clone https://github.com/Slamtec/sllidar_ros2.git
else
  echo "sllidar_ros2 exists but is not a git checkout (copied in): leaving it as it is."
fi

# Frontier exploration (only needed for an exploring mode; the deterministic coverage mission does not use it).
if [ ! -d m-explore-ros2 ]; then
  git clone https://github.com/robo-friends/m-explore-ros2.git
  (cd m-explore-ros2 && git branch -a | grep -i jazzy || echo "no jazzy branch: building the default branch, patch if it fails")
else
  (cd m-explore-ros2 && git pull --ff-only)
fi

cd ~/ros2_ws
colcon build --symlink-install --event-handlers console_cohesion+
echo "Build finished. Verify with: ros2 pkg executables robot_slam"
