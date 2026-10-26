# robot_slam

LiDAR-only SLAM (RPLIDAR A1 + SLAM Toolbox) for a robot with no wheel encoders.

## Install (on the Pi)
    sudo apt install -y ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server ros-jazzy-teleop-twist-keyboard
    unzip robot_slam.zip -d ~/ros2_ws/src/
    cd ~/ros2_ws
    colcon build --symlink-install --packages-select robot_slam
    source install/setup.bash

## Run (3 terminals)
1. ros2 launch robot_slam slam.launch.py            # add lidar_port:=/dev/rplidar if you use a udev symlink
2. your existing Pico bridge (must subscribe to /cmd_vel)
3. ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.6

## Check TF
    ros2 run tf2_tools view_frames      # expect map -> odom -> base_link -> laser

## Save the map
    mkdir -p ~/maps
    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

## Edit before first run
Laser mounting offset/yaw in launch/slam.launch.py (static_tf base_link -> laser).
