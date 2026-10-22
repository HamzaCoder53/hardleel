#!/bin/bash
# Bench test of the laptop brain with fake sensors (no robot needed). Private domain 42, localhost only.
#   scripts/laptop/bench_test.sh
# 1) brain mapping on a synthetic room -> /scan_filtered, /map, map->base_link TF; save a .pbstream
# 2) brain mission on that map -> Nav2 active; a NavigateToPose goal must make /cmd_vel move the fake robot
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# loopback only, unicast, enough participant slots for ~30 processes (ROS_LOCALHOST_ONLY breaks Cyclone in WSL)
mkdir -p ~/.hardleel; cat > ~/.hardleel/cyclonedds_bench.xml <<X
<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any"><General>
  <Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General>
  <Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
  <Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>
X
export CYCLONEDDS_URI=file://$HOME/.hardleel/cyclonedds_bench.xml

source /opt/ros/humble/setup.bash; source ~/hardleel_ws/install/setup.bash
HERE="$(cd "$(dirname "$0")" && pwd)"; FAKE="$HERE/../../ros2_ws/src/robot_slam/test/fake_sensors.py"
OUT=/tmp/hardleel_bench; rm -rf $OUT; mkdir -p $OUT
pass=0; fail=0
check() { if eval "$2"; then echo "PASS  $1"; pass=$((pass+1)); else echo "FAIL  $1"; fail=$((fail+1)); fi; }
cleanup() { kill $LAUNCH $FAKEP 2>/dev/null; sleep 3; pkill -INT -f "brain.launch.py" 2>/dev/null; sleep 2; }
trap cleanup EXIT

python3 "$FAKE" > $OUT/fake.log 2>&1 & FAKEP=$!
sleep 3
check "fake /scan publishes" "timeout 8 ros2 topic echo --once /scan --qos-reliability best_effort --field header.frame_id >/dev/null 2>&1"

echo "== mapping"
ros2 launch robot_slam brain.launch.py mode:=mapping rviz:=false record_bag:=false > $OUT/mapping.log 2>&1 & LAUNCH=$!
sleep 25
check "/scan_filtered publishes" "timeout 8 ros2 topic echo --once /scan_filtered --qos-reliability best_effort --field header.frame_id >/dev/null 2>&1"
check "/map published by Cartographer" "timeout 10 ros2 topic echo --once /map --field info.resolution >/dev/null 2>&1"
check "TF map -> base_link" "timeout 8 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -q Translation"
check "safety chain active" "ros2 lifecycle get /collision_monitor_mapping 2>/dev/null | grep -q active"
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}" >/dev/null 2>&1
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$OUT/bench.pbstream', include_unfinished_submaps: true}" >/dev/null 2>&1
check "pbstream saved" "[ -s $OUT/bench.pbstream ]"
kill -INT $LAUNCH; sleep 6; kill $LAUNCH 2>/dev/null

echo "== mission"
ros2 launch robot_slam brain.launch.py mode:=mission pbstream:=$OUT/bench.pbstream rviz:=false record_bag:=false > $OUT/mission.log 2>&1 & LAUNCH=$!
sleep 40
check "TF map -> base_link (localization)" "timeout 10 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -q Translation"
for n in controller_server planner_server bt_navigator behavior_server velocity_smoother collision_monitor_mission; do
  check "$n active" "ros2 lifecycle get /$n 2>/dev/null | grep -q active"
done
timeout 45 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: -0.9}, orientation: {w: 1.0}}}}" > $OUT/goal.log 2>&1 &
GOAL=$!
sleep 8
check "path planned (/plan)" "timeout 8 ros2 topic echo --once /plan --field header.frame_id >/dev/null 2>&1"
check "/cmd_vel non-zero through the safety chain" "timeout 8 ros2 topic echo /cmd_vel --field linear.x 2>/dev/null | grep -m1 -qE '^[0-9.-]*[1-9]'"
wait $GOAL
check "goal reached" "grep -q -i 'SUCCEEDED' $OUT/goal.log"
echo "== $pass passed, $fail failed (logs in $OUT)"
grep -h -E "\[ERROR\]|Traceback|process has died" $OUT/mapping.log $OUT/mission.log | sort | uniq -c | sort -rn | head -15
