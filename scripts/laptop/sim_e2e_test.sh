#!/bin/bash
# End-to-end mission test in the sim: rdj2026_sim + laptop_mapping + laptop_mission (stub classifier) + go.
#   bash scripts/laptop/sim_e2e_test.sh [worker] [localization]      -> logs in ~/simtest_m1/
# Works in the Jazzy distro (Gazebo Harmonic, ~/hardleel_ws) and the Humble one (Fortress, WS=~/hardleel_m1_ws).
#   worker        spawn a 0.25 m "worker" cylinder in the right corridor at GO, remove it 75 s later
#   localization  phase 2 on laptop_localization (saved .pbstream) instead of the live mapping session
#   ramp=<mm>     a 1.14 m ramp of that height in the right corridor (map x ~1.5) before mapping starts
# Private domain 42 and FastDDS: CycloneDDS over WSL loopback drops the sim's LaserScan messages (tested 25 Oct).
HERE="$(cd "$(dirname "$0")" && pwd)"
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp LIBGL_ALWAYS_SOFTWARE=1
unset CYCLONEDDS_URI ZENOH_CONFIG_OVERRIDE
for d in jazzy humble; do [ -f /opt/ros/$d/setup.bash ] && { source /opt/ros/$d/setup.bash; break; }; done
WS="${WS:-$HOME/hardleel_ws}"; source "$WS/install/setup.bash"
WORKER=0; LOC=0; RAMP=0
for a in "$@"; do [ "$a" = worker ] && WORKER=1; [ "$a" = localization ] && LOC=1; [ "${a%%=*}" = ramp ] && RAMP=${a#ramp=}; done
WP="$WS/src/robot_slam/config/waypoints_sim.yaml"
L=~/simtest_m1; rm -rf $L; mkdir -p $L
GZ="gz"; MSG="gz.msgs"; [ "$ROS_DISTRO" = humble ] && { GZ="ign"; MSG="ignition.msgs"; }
cleanup() { pkill -TERM -f "ros2 launch"; sleep 4; pkill -9 -f "gz sim|ign gazebo"; pkill -9 -f "ros2 bag|ros2 run"; }
cleanup 2>/dev/null; sleep 1
ros2 launch rdj2026_sim sim.launch.py gui:=false odom_tf:=false > $L/sim.log 2>&1 &
sleep 20
if [ "$RAMP" != 0 ]; then   # ramp across the right corridor, arena (6352, 1800) = map x 1.5, along the corridor
  ros2 run ros_gz_sim create -world rdj2026 -name ramp -x 6.352 -y 1.8 -z 0 -Y 1.5708 -string "$(python3 "$HERE/ramp_sdf.py" $RAMP)" > $L/ramp.log 2>&1
fi
ros2 launch robot_slam laptop_mapping.launch.py sim:=true rviz:=false record_bag:=false > $L/mapping.log 2>&1 &
sleep 15
if [ $LOC = 1 ]; then      # save the map, restart as pure localization from the start pose (robot is still there)
  ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$L/sim.pbstream', include_unfinished_submaps: true}" >/dev/null
  pkill -TERM -f "laptop_mapping.launch.py"   # TERM: background jobs of a script ignore SIGINT
  NODES="cartographer_node|occupancy_grid|sensor_prep|collision_monitor|velocity_smoother|twist_mux|imu_filter_madgwick"
  for i in $(seq 1 20); do pgrep -f "$NODES" >/dev/null || break; sleep 1; done
  pkill -9 -f "$NODES"; sleep 1
  ros2 launch robot_slam laptop_localization.launch.py sim:=true rviz:=false record_bag:=false pbstream:=$L/sim.pbstream > $L/localization.log 2>&1 &
  sleep 15
fi
ros2 launch robot_slam laptop_mission.launch.py sim:=true classifier:=stub stub_class:=metal waypoints:=$WP > $L/mission.log 2>&1 &
sleep 40
echo "--- topics"; python3 "$HERE/topic_probe.py" 4 /scan /scan_filtered /map /odom 2>&1 | grep Hz
echo "--- GO $(date +%T)"; ros2 topic pub --once -w 1 /mission/start std_msgs/msg/Empty "{}" >/dev/null
if [ $WORKER = 1 ]; then   # arena (6352, 2350) = map (2.05, -0.14): between terrain_1 and terrain_2; spawned at GO
  ros2 run ros_gz_sim create -world rdj2026 -name worker -x 6.352 -y 2.35 -z 0.2 -string \
    '<sdf version="1.6"><model name="worker"><static>true</static><link name="l"><collision name="c"><geometry><cylinder><radius>0.125</radius><length>0.4</length></cylinder></geometry></collision><visual name="v"><geometry><cylinder><radius>0.125</radius><length>0.4</length></cylinder></geometry></visual></link></model></sdf>' > $L/worker.log 2>&1
  ( sleep 75; $GZ service -s /world/rdj2026/remove --reqtype $MSG.Entity --reptype $MSG.Boolean \
      --timeout 2000 --req 'name: "worker" type: MODEL' >> $L/worker.log 2>&1; echo "worker removed $(date +%T)" >> $L/worker.log ) &
fi
for i in $(seq 1 60); do sleep 5; grep -q "DONE: mission finished\|ABORTED" $L/mission.log && break; done
echo "--- end $(date +%T) after $((i*5)) s"
grep -E "\] \[ *[0-9.]+s\]|blocked|Controller patience|Failed to make progress" $L/mission.log | sed 's/.*\]: //' | head -80
tail -2 $L/worker.log 2>/dev/null
echo "--- event_logger"; f=$(ls -t ~/logs/runs/*.json 2>/dev/null | head -1)
[ -n "$f" ] && python3 -c "import json; d=json.load(open('$f')); [print(r['field'], r['t_rel'], r['value'], r['extra']) for r in d['required_summary']]"
cleanup 2>/dev/null
