#!/bin/bash
# Hardware-in-the-loop sim suite (Jazzy distro): the REAL motor_bridge drives Gazebo through fake_pico
# (deadband 60/70, v_max 0.9, slow zone x0.6) with Cartographer running, then checks the motor features:
#   deadband compensation, yaw feedback, turn calibration, speed calibration (auto), speed governor,
#   speed fit from the mapping bag (manual).          bash scripts/laptop/hw_sim_suite.sh  -> ~/hwsim2/
HERE="$(cd "$(dirname "$0")" && pwd)"
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp LIBGL_ALWAYS_SOFTWARE=1
renice -n 19 $$ >/dev/null   # lowest CPU priority: the real robot stack may be running on this laptop
unset CYCLONEDDS_URI ZENOH_CONFIG_OVERRIDE
source /opt/ros/jazzy/setup.bash; source ~/hardleel_ws/install/setup.bash
L=~/hwsim2; rm -rf $L ~/hardleel_bags_sim/mapping_*; mkdir -p $L   # SIM bags only - never touch ~/hardleel_bags (real runs)
CAL=/tmp/sim_motor_calib.yaml
printf 'min_pwm_left: 60\nmin_pwm_right: 70\n' > $CAL          # as if deadband_calibrate had run
T="python3 $HERE/hw_sim_test.py"
ros2 launch rdj2026_sim sim.launch.py gui:=false hardware:=true > $L/sim.log 2>&1 &
sleep 22
ros2 launch robot_slam laptop_mapping.launch.py sim:=true rviz:=false record_bag:=true bag_dir:=~/hardleel_bags_sim > $L/mapping.log 2>&1 &
sleep 12
python3 $HERE/topic_probe.py 3 /tf /scan_filtered 2>&1 | grep -E "Hz|pubs"
ros2 run motor_teleop motor_calibrator --ros-args -p use_sim_time:=true -p calib_file:=$CAL > $L/cal.log 2>&1 &
sleep 5
for t in deadband yaw; do echo "=== $t"; $T $t | grep -E "cmd|PASS|FAIL"; done
echo "=== calibrate_turn"; $T cal_turn
echo "=== calibrate_speed"; $T cal_speed
echo "--- calib file"; grep -E "^[a-z_]+:" $CAL
ros2 run robot_slam speed_governor --ros-args -p use_sim_time:=true > $L/gov.log 2>&1 &
sleep 3
echo "=== speed governor"; $T speed | grep -E "slow zone|PASS|FAIL"
for p in $(pgrep -f speed_governor); do tr "\0" "\n" < /proc/$p/environ 2>/dev/null | grep -qx ROS_DOMAIN_ID=42 && kill $p; done
for v in 0.15 0.25 0.35; do                                    # straight legs for the manual (bag) fit
  python3 - <<P
import rclpy, time
from geometry_msgs.msg import Twist
rclpy.init(); n = rclpy.create_node('leg'); p = n.create_publisher(Twist, '/cmd_vel', 10)
for s in (1, -1):
    t0 = time.monotonic()
    while time.monotonic() - t0 < 4:
        m = Twist(); m.linear.x = s * $v; p.publish(m); time.sleep(0.05)
    p.publish(Twist()); time.sleep(1)
P
done
for p in $(pgrep -f "ros2 bag record"); do tr "\0" "\n" < /proc/$p/environ 2>/dev/null | grep -qx ROS_DOMAIN_ID=42 && kill -INT $p; done; sleep 3
bash "$HERE/kill_domain.sh" 42 TERM; sleep 5; bash "$HERE/kill_domain.sh" 42 KILL   # sim only
B=$(ls -d ~/hardleel_bags_sim/mapping_* | tail -1)
echo "=== fit_speed_from_bag (true max_speed 0.9)"; timeout -k 2 120 ros2 run motor_teleop fit_speed_from_bag "$B" --calib-file $CAL 2>&1 | tail -7
