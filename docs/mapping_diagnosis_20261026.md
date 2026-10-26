# Real-robot mapping diagnosis (26 Oct, offline, read-only)

Data: bags `~/hardleel_bags/mapping_20261026_1551..1648` (live Cartographer `/tracked_pose`, `/scan_gate/state`,
`/imu/data_raw`, `/scan_filtered`). Offline replays: isolated (domain 55, CycloneDDS localhost), the repo
`cartographer_2d.lua` with one-line overrides, scans `/scan_filtered`, IMU `/imu/data_raw`. Scripts and maps are in
`tmp/dx/`.

## Root cause 1 (high confidence): scan_gate starves Cartographer of scans

| bag | gate blocked | live final pose | live max distance from start |
|---|---|---|---|
| 164206 | 31 % | (-0.1, +0.3) m | 1.1 m (good run) |
| 155146 | 10 % | (-2.9, +20.5) | 21 m |
| 155833 | 27 % | (-4.3, -9.5) | 26 m |
| 164437 | **73 %** | **(-118, +40)** | **127 m** |
| 162049 | **84 %** | **(-70, +68)** | **97 m** |

- Almost all blocking is `tilt` (3 deg threshold). In 162049 it sat at `tilt -8.3/+1.0` for **220 s straight**, so
  the boot reference was taken while the robot was tilted or moving. Since 8ddb451 the tilt comes from the
  low-passed **accelerometer**. With raw PWM 70-75 steps and no ramps, every start and stop reads as a 3-12 deg
  "tilt": 1 m/s^2 of forward acceleration gives 5.8 deg.
- While blocked, Cartographer gets no scans and its pose extrapolator keeps integrating. 162049 moved **102 m
  while blocked** against 6 m while open. When scans come back, the 0.2 m / 25 deg search window cannot recover,
  so the map is wrecked.
- **Replay proof (bag 162049):** ungated, the replay stays within 10.8 x 6 m (final (-9.6,-3.2), 26 submaps). With
  scans dropped exactly where the live gate blocked (399 of 2743 kept), the replay runs away to (-14.9, +34.8)
  with only 2 submaps. Bag 164437 ungated ends at (-18, +9) along a real ~25 m corridor, against (-118, +40) live.

## Ruled out (with numbers)
- **IMU sign, scale and frame are correct.** Gyro-integrated yaw agrees with scan-correlation rotation: +168 vs
  +173 deg in 164437. Big turns agree in sign. Static TF base_link->imu_link is yaw 0. At rest the gyro mean is
  +0.04 deg per scan.
- **Lidar is not mirrored.** Scan rotation has the same sign as the gyro.
- **Timestamps are fine.** IMU stamps track receive time (median -2 ms). Scan stamps are ~150 ms behind receive,
  which equals the scan duration (stamp = first ray, as Cartographer expects). No out-of-order stamps; max scan
  gap 355 ms.
- **Not the IMU, not loop closure.** Replay variants of 164437 (base / no global SLAM / use_imu_data=false / both)
  all give 140-163 m path. Per turn, Cartographer tracks 0.87-1.16 of the gyro rotation.
- **max_range 12 / stronger translation prior** did not help (146 / 171 m).
- **EKF path:** no TF conflict. odom->base_link comes only from the EKF and map->odom only from Cartographer
  (164437 /tf).

## Contributing cause 2 (medium confidence): very fast spins with a 6 Hz lidar
- Most turn events peak at **120-290 deg/s** (mean about 35 deg per key tap). The deadband of 70 against a max of
  75 means every turn is near full power. That is 20-30 deg per scan at 6.5 Hz, so even the ungated maps fan out
  around turn spots.
- Scans are sparse: median 144 of 360 rays in 0.18-5.5 m, 57 % inf. The test area is a long corridor of parallel
  walls, which leaves little to fix position along the corridor.

## Fixes (proposed, not applied)
1. **Map without the gate:** remove `scan_gate:=true` from the mapping start command (the launch default is
   already `false`).
   - `scripts/laptop/map_session.sh` line 10: `scan_gate:=true` -> `scan_gate:=false`
   - the restart command given to the team: `ros2 launch robot_slam laptop_mapping.launch.py rviz:=true`
     (no scan_gate)
2. If the gate must stay, make it unable to starve Cartographer:
   - `ros2_ws/src/robot_slam/robot_slam/scan_gate.py` line ~62: `tilt_deg` default `3.0` -> `10.0`
   - `accel_tau_s` `0.3` -> `1.0`
   - add a cap so it never blocks longer than about 1 s in a row (pass the scan through if blocked > 1.0 s).
3. Slow the spins: lower the in-place turn PWM, for example `min_turn_pwm` / `turn_gain`, or make teleop turns
   short taps. Keep the turn rate below about 60 deg/s (under 10 deg per scan).
4. Map in the real arena (closed walls), not the corridor. Corridor tests will always slide along the walls
   without wheel odometry.
