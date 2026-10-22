# Terrain-navigation stack: what was added and how to test it

Written 22 Oct 2026. Nothing here was run on the robot (the Pi was unreachable / under-voltage) and no code in this
change spins the motors. Logic is covered by `ros2_ws/src/robot_slam/test/` (run in WSL, 15 + 1 end-to-end tests pass).

## Files
| File | What it is |
|---|---|
| `robot_slam/robot_slam/sensor_prep.py` | `/scan` -> `/scan_filtered` (per-beam ground/overhead gate, ramp mode arm 2.0 deg / release 1.0 deg); `/imu/data_raw` + `/imu/data` -> `/imu/data_lpf` (gyro bias captured over 200 still samples, low-passed) |
| `robot_slam/robot_slam/coverage_mission.py` | boustrophedon / explicit waypoints through Nav2, retry + costmap clear, speed regimes 0.18 / 0.30, CSV pose log in `~/logs` |
| `robot_slam/robot_slam/stack_common.py` | shared launch building blocks (lidar, IMU, Cartographer, safety chain, Nav2) |
| `robot_slam/launch/mapping_stack.launch.py`, `mission_stack.launch.py`, `record_bag.launch.py` | the two stacks and the bag recorder |
| `robot_slam/config/collision_monitor.yaml` | `collision_monitor_mapping` (stop 0.30 / slow 0.42) and `collision_monitor_mission` (0.40 / 0.52) |
| `robot_slam/config/twist_mux.yaml`, `coverage_mission.yaml`, `nav2_params_mission.yaml`, `cartographer_2d_localization.lua` | new configs |
| `robot_slam/config/nav2_params.yaml`, `cartographer_2d.lua`, `launch/carto_mapping.launch.py` | edited per the work order (mapping profile) |
| `motor_teleop/motor_teleop/motor_bridge.py` | surface profiles, gravity feedforward, surface state machine, 50 Hz |
| `scripts/install_terrain_stack_deps.sh` | Step 1 (apt + git + colcon) for the Pi; **not run** |
| `bags/` | empty folder for recordings |

Command chain: `twist_mux -> velocity_smoother -> collision_monitor -> /cmd_vel -> motor_bridge`
(teleop on `/cmd_vel_teleop`, Nav2 on `/cmd_vel_nav`). The guardian node is not started by these stacks.

## Manual test checklist (robot on blocks / wheels OFF the ground until stated)
1. Install and build on the Pi: `bash ~/install_terrain_stack_deps.sh` (copy it over first). Expect `colcon build` with no errors, then
   `ros2 pkg executables robot_slam` lists `sensor_prep` and `coverage_mission`.
2. Power first: `vcgencmd get_throttled` -> `throttled=0x0` and no "Undervoltage" in `journalctl -k -b`.
3. Bring up sensors only: `ros2 launch robot_slam mapping_stack.launch.py` (motor_bridge not started).
4. LiDAR density: `ros2 topic echo /scan --once --field ranges | tr ',' '\n' | wc -l` -> **>= 1300** (Sensitivity mode, about 1400).
   `ros2 topic hz /scan` -> about 5.5 Hz.
5. IMU bias: keep the robot still for the first 5 s. Log line `Gyro bias captured: [.. .. -12.4] deg/s` (or about 0 if
   `imu_serial_node` already removes it). `ros2 topic echo /imu/data_lpf --field angular_velocity.z` -> **about 0.000** (|z| < 0.002 rad/s).
6. Tilt gate: tilt the robot nose-down by hand about 8 deg. Log `Ramp mode ON`; `/scan_filtered` has NaN for forward beams
   far away (`ros2 topic echo /scan_filtered --once`). Back to flat (< 1 deg): `Ramp mode OFF`, scan identical to `/scan`.
7. Safety chain (wheels off the ground): `ros2 launch robot_slam mapping_stack.launch.py motor_bridge:=true`, then
   `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop -p speed:=0.15`.
   `ros2 topic echo /cmd_vel` shows the smoothed commands; put a hand 0.25 m in front of the LiDAR: `/cmd_vel` -> 0 (stop zone 0.30 m);
   hand at 0.38 m: speed drops to about 40 % (slowdown zone 0.42 m).
8. Surface state: `ros2 topic pub --once /surface_override std_msgs/String "{data: SAND}"` -> `/surface_state` = SAND and the
   bridge log shows `Surface FLAT -> SAND`; `AUTO` returns control to the pitch ramp-arm.
9. Feedforward sign (wheels OFF the ground, watch `ros2 topic echo` of the bridge log `left= right=`): tilt the nose UP about 8 deg
   and command forward: PWM should be **higher** than flat (up to +90); nose DOWN: **lower**. If reversed, set `pitch_up_sign:=1.0`.
10. Rosbag: `ros2 launch robot_slam record_bag.launch.py`, stop with Ctrl+C, `ros2 bag info bags/<name>` lists /scan, /scan_filtered, /imu/*, /cmd_vel.
11. Mission dry run (no driving): `ros2 launch robot_slam mission_stack.launch.py pbstream:=<map>.pbstream` (run_mission defaults to false);
    check Nav2 lifecycle nodes become active. Only then try `run_mission:=true` on the floor, with the e-stop in hand.
