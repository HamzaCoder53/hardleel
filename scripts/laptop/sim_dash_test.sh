#!/bin/bash
# Ramp dash in the sim (final stack item 5): course world, REAL motor_bridge (dash_input) via fake_pico, the Pi's
# ramp_dash node, a dirt-ramp model in the left strip, optional side slip noise and a bog-down.
#   bash scripts/laptop/sim_dash_test.sh [noise=0.05] [bias=0.1] [bog=1] [height=100] [lat=0.10] [yaw=5]
#   -> ~/simdash/: result, dash CSV summary, ground truth before/after. Domain 42 only, never the real robot.
HERE="$(cd "$(dirname "$0")" && pwd)"
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp LIBGL_ALWAYS_SOFTWARE=1
renice -n 19 $$ >/dev/null
unset CYCLONEDDS_URI ZENOH_CONFIG_OVERRIDE
source /opt/ros/jazzy/setup.bash; source ~/hardleel_ws/install/setup.bash
NOISE=0.0; BIAS=0.0; BOG=0; H=100; LAT=0.0; YAW=0
for a in "$@"; do v=${a#*=}; case ${a%%=*} in noise) NOISE=$v;; bias) BIAS=$v;; bog) BOG=$v;; height) H=$v;; lat) LAT=$v;; yaw) YAW=$v;; esac; done
L=~/simdash; rm -rf $L; mkdir -p $L
cleanup() { bash "$HERE/kill_domain.sh" 42 TERM; sleep 3; bash "$HERE/kill_domain.sh" 42 KILL; }
cleanup 2>/dev/null; sleep 1
RX=-4.18; RY=-1.30                 # ramp centre (world): dirt strip x -4.54..-3.82 (map -4.60..-3.88 + 0.06 spawn)
SY=-2.25                           # robot starts at the foot, facing +y (up the strip)
BOGZ="$(python3 -c "print(f'{$RX-0.4},{$RY-0.57},{$RX+0.4},{$RY-0.19}')")"   # the up-slope third
ros2 launch rdj2026_sim sim.launch.py gui:=false world:=course hardware:=true pico_side_noise:=$NOISE \
  pico_side_bias:=$BIAS pico_bog_zone:=$BOGZ pico_bog_times:=$BOG > $L/sim.log 2>&1 &
sleep 22
ros2 run ros_gz_sim create -world rdj2026 -name ramp -x $RX -y $RY -z 0 -Y 1.5708 -string "$(python3 "$HERE/ramp_sdf.py" $H 1140 700)" > $L/ramp.log 2>&1
gz service -s /world/rdj2026/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req "name: \"hardleel\" position: {x: $(python3 -c "print(-4.18+$LAT)") y: $SY z: 0.05} orientation: {z: $(python3 -c "import math; print(math.sin(math.radians(90+$YAW)/2))") w: $(python3 -c "import math; print(math.cos(math.radians(90+$YAW)/2))")}" >> $L/ramp.log 2>&1
sleep 3
ros2 param set /motor_bridge dash_input true > $L/param.log 2>&1
ros2 run robot_slam ramp_dash --ros-args -p use_sim_time:=true --params-file ~/hardleel_ws/install/robot_slam/share/robot_slam/config/ramp_dash.yaml -p log_dir:=$L > $L/dash.log 2>&1 &
sleep 6
python3 - $L <<'P'
import json, math, sys, time
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
L = sys.argv[1]
rclpy.init(); n = rclpy.create_node('dash_test', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
st = {'res': None, 'gt': None, 'gt0': None, 'maxz': 0.0, 'minx': 9, 'maxx': -9}
n.create_subscription(String, '/ramp_dash/result', lambda m: st.__setitem__('res', json.loads(m.data)), 10)
def gt(m):
    p = m.pose.pose.position
    st['gt'] = p; st['gt0'] = st['gt0'] or (p.x, p.y)
    st['maxz'] = max(st['maxz'], p.z); st['minx'] = min(st['minx'], p.x); st['maxx'] = max(st['maxx'], p.x)
n.create_subscription(Odometry, '/ground_truth', gt, 10)
pub = n.create_publisher(String, '/ramp_dash/start', 10)
t0 = time.time()
while st['gt'] is None and time.time() - t0 < 10: rclpy.spin_once(n, timeout_sec=0.1)
for _ in range(3):
    pub.publish(String(data='')); rclpy.spin_once(n, timeout_sec=0.1)
end = time.time() + 120
while st['res'] is None and time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)
p = st['gt']
print('RESULT', json.dumps(st['res']))
print(f'ground truth: start ({st["gt0"][0]:.2f}, {st["gt0"][1]:.2f}) end ({p.x:.2f}, {p.y:.2f}) max height {st["maxz"]:.3f} m, '
      f'x range {st["minx"]:.2f}..{st["maxx"]:.2f} (strip -4.54..-3.82)')
P
f=$(ls $L/ramp_dash_*.csv 2>/dev/null | head -1)
[ -n "$f" ] && python3 - $f <<'P'
import csv, sys, math
rows = list(csv.DictReader(open(sys.argv[1])))
lat = [abs(float(r['lat'])) for r in rows if r['lat'] != 'nan' and r['phase'] in ('4.0000',)]
err = [abs(float(r['err_deg'])) for r in rows if r['phase'] in ('4.0000',)]
pw = [max(abs(float(r['pwm_left'])), abs(float(r['pwm_right']))) for r in rows]
print(f'dash log: {len(rows)} rows ({len(rows)/max(float(rows[-1]["t"])-float(rows[0]["t"]),1e-3):.0f} Hz), '
      f'DASH |lat| max {max(lat) if lat else float("nan"):.3f} m, |heading err| max {max(err) if err else float("nan"):.1f} deg, max PWM {max(pw):.0f}')
P
grep -aE "\[dash|DASH" $L/dash.log | sed 's/.*\]: //' | cut -c1-150 | head -25
cleanup 2>/dev/null
