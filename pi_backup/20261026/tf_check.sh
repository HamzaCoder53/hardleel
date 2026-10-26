#!/bin/bash
export ROS_DOMAIN_ID=7
source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash
echo "--- tf map->laser"; timeout 6 ros2 run tf2_ros tf2_echo map laser 2>&1 | head -6
echo "--- tf odom->base_link"; timeout 5 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -4
echo "--- carto log"; grep -v "^\s*$" ~/hwtest_logs/carto.log | tail -8 | cut -c1-170
echo "--- map msgs"; timeout 6 ros2 topic hz /map --window 2 2>&1 | tail -1
