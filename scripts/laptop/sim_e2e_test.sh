#!/bin/bash
# End-to-end mission test in the sim: rdj2026_sim + laptop_mapping + laptop_mission (stub classifier) + go.
#   bash scripts/laptop/sim_e2e_test.sh [worker] [localization]      -> logs in ~/simtest_m1/
# Works in the Jazzy distro (Gazebo Harmonic, ~/hardleel_ws) and the Humble one (Fortress, WS=~/hardleel_m1_ws).
#   worker        spawn a 0.25 m "worker" cylinder in the right corridor at GO, remove it 75 s later
#   localization  phase 2 on laptop_localization (saved .pbstream) instead of the live mapping session
#   ramp=<mm>     a 1.14 m ramp of that height in the walled right corridor (map x 2.1-3.3) before mapping
# Private domain 42 and FastDDS: CycloneDDS over WSL loopback drops the sim's LaserScan messages (tested 25 Oct).
HERE="$(cd "$(dirname "$0")" && pwd)"
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp LIBGL_ALWAYS_SOFTWARE=1
renice -n 19 $$ >/dev/null   # lowest CPU priority: the real robot stack may be running on this laptop
unset CYCLONEDDS_URI ZENOH_CONFIG_OVERRIDE
for d in jazzy humble; do [ -f /opt/ros/$d/setup.bash ] && { source /opt/ros/$d/setup.bash; break; }; done
WS="${WS:-$HOME/hardleel_ws}"; source "$WS/install/setup.bash"
WORKER=0; PREMAP=; LOC=0; RAMP=0; COURSE=0; GATE=0; EXPLORE=0; WREMOVE=75
for a in "$@"; do [ "$a" = worker ] && WORKER=center; [ "${a%%=*}" = worker ] && [ "$a" != worker ] && WORKER=${a#worker=}; [ "${a%%=*}" = worker_remove ] && WREMOVE=${a#worker_remove=}; [ "$a" = localization ] && LOC=1; [ "${a%%=*}" = ramp ] && RAMP=${a#ramp=}
  [ "${a%%=*}" = worker_premap ] && PREMAP=${a#worker_premap=}   # x,y (world): worker stands there while mapping
  [ "$a" = course ] && COURSE=1; [ "$a" = gate_closed ] && GATE=1; [ "${a%%=*}" = explore ] && EXPLORE=${a#explore=}; done
WORLD=rdj2026; [ $COURSE = 1 ] && WORLD=course
WP="$WS/src/robot_slam/config/waypoints_sim.yaml"
[ $COURSE = 1 ] && WP="$WS/src/robot_slam/config/waypoints_course.yaml"
WXY="6.352 2.35"; [ $COURSE = 1 ] && WXY="-1.0 -2.27"     # worker centred: Figure-1 0.7 m right corridor / course bottom corridor
[ "$WORKER" = side ] && WXY="6.56 2.35"                   # against the right wall: 0.42 m gap left (7-8 cm a side)
L=~/simtest_m1; rm -rf $L; mkdir -p $L
GZ="gz"; MSG="gz.msgs"; [ "$ROS_DISTRO" = humble ] && { GZ="ign"; MSG="ignition.msgs"; }
cleanup() { bash "$HERE/kill_domain.sh" 42 TERM; sleep 4; bash "$HERE/kill_domain.sh" 42 KILL; }   # sim (domain 42) only
cleanup 2>/dev/null; sleep 1
ros2 launch rdj2026_sim sim.launch.py gui:=false odom_tf:=false world:=$WORLD > $L/sim.log 2>&1 &
sleep 20
if [ $GATE = 1 ]; then      # course only: the gate closed for the whole run (top lane at x -3.02) -> expect path1
  ros2 run ros_gz_sim create -world rdj2026 -name gate -x -3.02 -y 0.07 -z 0.2 -string \
    '<sdf version="1.6"><model name="gate"><static>true</static><link name="l"><collision name="c"><geometry><box><size>0.05 0.60 0.4</size></box></geometry></collision><visual name="v"><geometry><box><size>0.05 0.60 0.4</size></box></geometry></visual></link></model></sdf>' > $L/gate.log 2>&1
fi
if [ "$RAMP" != 0 ]; then   # ramp in the walled part of the right corridor, arena (6352, 3000) = map x 2.13-3.27, end-on
  ros2 run ros_gz_sim create -world rdj2026 -name ramp -x 6.352 -y 3.0 -z 0 -Y 1.5708 -string "$(python3 "$HERE/ramp_sdf.py" $RAMP)" > $L/ramp.log 2>&1
fi
if [ -n "$PREMAP" ]; then
  [ "$WORKER" = 0 ] && WORKER=center
  ros2 run ros_gz_sim create -world rdj2026 -name worker -x ${PREMAP%,*} -y ${PREMAP#*,} -z 0.2 -string \
    '<sdf version="1.6"><model name="worker"><static>true</static><link name="l"><collision name="c"><geometry><cylinder><radius>0.125</radius><length>0.4</length></cylinder></geometry></collision><visual name="v"><geometry><cylinder><radius>0.125</radius><length>0.4</length></cylinder></geometry></visual></link></model></sdf>' > $L/worker.log 2>&1
fi
NAV2=true
if [ "$EXPLORE" != 0 ]; then   # phase 1 rehearsal: auto mapping (Nav2 + frontier explorer, returns to start)
  ros2 launch robot_slam laptop_mapping.launch.py sim:=true rviz:=false record_bag:=true bag_dir:=~/hardleel_bags_sim mapping_mode:=auto > $L/mapping.log 2>&1 &
  NAV2=false
  for i in $(seq 1 $((EXPLORE / 5))); do sleep 5; grep -q "exploration finished" $L/mapping.log && break; done
  sleep 15                      # return-to-start goal
  echo "--- phase 1: $(grep -m1 -oE 'exploration finished.*' $L/mapping.log) after $((i * 5)) s"
else
  ros2 launch robot_slam laptop_mapping.launch.py sim:=true rviz:=false record_bag:=true bag_dir:=~/hardleel_bags_sim > $L/mapping.log 2>&1 &
  sleep 15
fi
if [ $LOC = 1 ]; then      # save the map, restart as pure localization from the start pose (robot is still there)
  ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$L/sim.pbstream', include_unfinished_submaps: true}" >/dev/null
  pkill -TERM -f "laptop_mapping.launch.py sim:=true"   # sim one only; TERM: script background jobs ignore SIGINT
  NODES="cartographer_node|occupancy_grid|sensor_prep|collision_monitor|velocity_smoother|twist_mux|imu_filter_madgwick"
  for i in $(seq 1 20); do pgrep -f "$NODES" >/dev/null || break; sleep 1; done
  for p in $(pgrep -f "$NODES"); do tr "\0" "\n" < /proc/$p/environ 2>/dev/null | grep -qx ROS_DOMAIN_ID=42 && kill -9 $p; done; sleep 1
  ros2 launch robot_slam laptop_localization.launch.py sim:=true rviz:=false record_bag:=true bag_dir:=~/hardleel_bags_sim pbstream:=$L/sim.pbstream > $L/localization.log 2>&1 &
  sleep 15
fi
ros2 launch robot_slam laptop_mission.launch.py sim:=true classifier:=stub stub_class:=metal waypoints:=$WP nav2:=$NAV2 > $L/mission.log 2>&1 &
sleep 40
echo "--- topics"; python3 "$HERE/topic_probe.py" 4 /scan /scan_filtered /map /odom 2>&1 | grep Hz
echo "--- GO $(date +%T)"; ros2 topic pub --times 3 -r 5 -w 1 /mission/start std_msgs/msg/Empty "{}" >/dev/null
if [ -n "$PREMAP" ]; then     # worker was in the scan: move it to the run position (map_freezer must drop the old blob)
  set -- $WXY
  $GZ service -s /world/rdj2026/set_pose --reqtype $MSG.Pose --reptype $MSG.Boolean --timeout 2000 \
    --req "name: \"worker\" position: {x: $1 y: $2 z: 0.2}" >> $L/worker.log 2>&1; echo "worker moved $(date +%T)" >> $L/worker.log
elif [ "$WORKER" != 0 ]; then   # arena (6352, 2350) = map (2.05, -0.14): between terrain_1 and terrain_2; spawned at GO
  set -- $WXY
  ros2 run ros_gz_sim create -world rdj2026 -name worker -x $1 -y $2 -z 0.2 -string \
    '<sdf version="1.6"><model name="worker"><static>true</static><link name="l"><collision name="c"><geometry><cylinder><radius>0.125</radius><length>0.4</length></cylinder></geometry></collision><visual name="v"><geometry><cylinder><radius>0.125</radius><length>0.4</length></cylinder></geometry></visual></link></model></sdf>' > $L/worker.log 2>&1
  [ "$WREMOVE" != never ] && ( sleep $WREMOVE; $GZ service -s /world/rdj2026/remove --reqtype $MSG.Entity --reptype $MSG.Boolean \
      --timeout 2000 --req 'name: "worker" type: MODEL' >> $L/worker.log 2>&1; echo "worker removed $(date +%T)" >> $L/worker.log ) &
fi
for i in $(seq 1 60); do sleep 5; grep -q "DONE: mission finished\|ABORTED" $L/mission.log && break; done
echo "--- end $(date +%T) after $((i*5)) s"
grep -E "\] \[ *[0-9.]+s\]|blocked|Controller patience|Failed to make progress" $L/mission.log | sed 's/.*\]: //' | head -80
tail -2 $L/worker.log 2>/dev/null
echo "--- event_logger"; f=$(ls -t ~/logs/runs/*.json 2>/dev/null | head -1)
[ -n "$f" ] && python3 -c "import json; d=json.load(open('$f')); [print(r['field'], r['t_rel'], r['value'], r['extra']) for r in d['required_summary']]"
cleanup 2>/dev/null
