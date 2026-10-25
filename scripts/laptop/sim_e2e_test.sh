#!/bin/bash
# usage (Humble distro, ~/hardleel_m1_ws built): bash scripts/laptop/sim_e2e_test.sh   -> ~/simtest_m1/*.log
# End-to-end M1 sim test (Humble distro): sim + laptop_mapping + laptop_mission (stub classifier) + go.
# FastDDS: CycloneDDS over WSL loopback drops the big LaserScan messages from the sim (tested 25 Oct)
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp LIBGL_ALWAYS_SOFTWARE=1
cat > /tmp/cyclone_lo.xml <<X
<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface address="127.0.0.1"/></Interfaces>
<AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex>
<MaxAutoParticipantIndex>150</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>
X
unset CYCLONEDDS_URI
source /opt/ros/humble/setup.bash; source ~/hardleel_m1_ws/install/setup.bash
L=~/simtest_m1; rm -rf $L ~/logs/runs_sim; mkdir -p $L
pkill -f "ign gazebo" ; pkill -f ros2; sleep 2
ros2 launch rdj2026_sim sim.launch.py gui:=false odom_tf:=false > $L/sim.log 2>&1 &
sleep 20
ros2 launch robot_slam laptop_mapping.launch.py sim:=true rviz:=false record_bag:=false > $L/mapping.log 2>&1 &
sleep 15
ros2 launch robot_slam laptop_mission.launch.py sim:=true classifier:=stub stub_class:=metal \
  waypoints:=$HOME/hardleel_m1_ws/src/robot_slam/config/waypoints_sim.yaml > $L/mission.log 2>&1 &
sleep 40
echo "--- topics"; timeout -k 2 -s INT 10 ros2 topic list | tr '\n' ' '; echo
echo "--- lifecycle"; for n in collision_monitor velocity_smoother controller_server planner_server bt_navigator; do echo "$n: $(timeout -k 2 -s INT 8 ros2 lifecycle get /$n 2>&1)"; done
python3 "$(dirname "$0")/topic_probe.py" 4 /scan /scan_filtered /map /odom 2>&1 | grep Hz
echo "--- GO $(date +%T)"; ros2 topic pub --once -w 1 /mission/start std_msgs/msg/Empty '{}' >/dev/null
for i in $(seq 1 60); do sleep 5; grep -q "DONE: mission finished\|ABORTED" $L/mission.log && break; done
echo "--- end $(date +%T) after $((i*5)) s"
grep -E "competition_mission.*\] \[|EVENT|wp:|ERROR|error" $L/mission.log | grep -v "^\s*$" | sed 's/.*\]: //' | head -80
echo "--- event_logger summary"; ls ~/logs/runs 2>/dev/null | tail -2; f=$(ls -t ~/logs/runs/*.json 2>/dev/null | head -1); [ -n "$f" ] && python3 -c "import json,sys; d=json.load(open('$f')); [print(r) for r in d.get('required_summary', [])]"
pkill -INT -f "ros2 launch"; sleep 3; pkill -f "ign gazebo"; pkill -f ros2
