# hardleel — to-do list

Mini Competition: **26 Oct 2026** · Final: **25 Nov 2026**. Last updated 22 Oct 2026.

## When the new wheels & motors are fitted (hardware)
- [ ] **Gyro scale calibration** — robot flat on a table against an edge, turn it by hand exactly 4 full turns,
      compare with the IMU's integrated yaw, set the scale factor in `imu_serial_node.py`.
      Then lower the 0.3 °/s yaw deadband and re-zero the gyro bias whenever the robot is stopped.
      (Measured 16 Oct: raw yaw bias −12.4 °/s, 1.3 °/min drift after calibration when still; the map drifted
      ~35° over a 700° drive → suspect gyro scale error + deadband.)
- [ ] **Motor characterisation** (robot lifted, wheels off the ground first): PWM → speed curve per side,
      dead zone (`min_pwm`), left/right mismatch, acceleration — measured with the LiDAR as reference
      (no encoders needed). Update `motor_bridge` parameters (`invert_linear`, `min_pwm`, `turn_gain`).
- [ ] **Command-based odometry**: publish predicted speed from the motor commands (from the characterisation)
      as a low-confidence twist into the EKF.
- [ ] **Cooling**: fan or heatsink on the Pi 4 (it throttled at 83.8 °C on 15 Oct).
- [ ] **Re-aim the camera** forward/level with the display screen (it points at the ceiling).
- [ ] Optional: optical-flow floor sensor (PAA5100JE) as an extra odometry source.

## Mapping & navigation without encoders (software)
- [ ] Lidar odometry: run **rf2o + IMU in the EKF** as the `odom` frame for Nav2 (`nav.launch.py` already has rf2o).
- [ ] Use **Cartographer pure localization** instead of AMCL (sim test 16 Oct with drifting odometry:
      Cartographer ~5 cm / ~1°, AMCL lost at 0.7–1.3 m).
- [ ] Re-run the sim localization comparison with a fixed route (old route clipped the loading table)
      and a proper kidnap test.
- [ ] Tune Cartographer for no odometry (scan-matcher search windows); try a denser LiDAR scan mode;
      map slowly with gentle turns.
- [ ] `map_track.sh`: also record a ROS bag (`/scan`, `/imu/data_raw`) every run so maps can be rebuilt offline.
- [ ] `lidar_reset.py`: exit with an error code when the LiDAR doesn't reply.

## Virtual arena (rdj2026_sim)
- [ ] Fix the real-scan scoring test (fails its positive control) before trusting any layout score.
- [ ] Review Gemini 4's two layouts (photos-only and all-sources) and score them once the test works.
- [ ] Update `arena_layout.yaml` from tape measurements or photos of the finished field
      (corridor widths, ramp / grass / doorway positions, gate, start, screen, bay colours).

## Competition tasks (software)
- [ ] **Mission node** (state machine): screen → detect → loading → ramps → obstacles → gate decision
      (wait vs reroute) → deposit → finish, using Nav2; test end-to-end in the sim.
- [ ] **Timing logger** for the rulebook §8 timestamps; send to the judges once the protocol is published.
- [ ] **Waste classifier retraining** (on the Mac): add an "empty" class, organic and fabric examples.
      Collect with `ros2 launch waste_classifier classify.launch.py save_dir:=/home/hardleel/captures`.
- [ ] **Bay colour detection** (blue / white / black) with the camera.
- [ ] Practise **Virtual SLAM** in the sim (map → save → navigate).

## Added 16 Oct
- [ ] **Behaviour tree for the competition run** (instead of a plain state machine): BehaviorTree.CPP/Nav2 BT
      or py_trees — sequence screen → detect → load → terrain → gate decision (wait / reroute with timeout)
      → deposit → finish, with retry/recovery branches (classifier "unknown" → re-position and retry,
      Nav2 failure → back up and replan). Build and test in the sim.
- [ ] **FlySky FS-i6 remote control** for the mapping phase (manual driving is allowed only then):
      FS-iA6B receiver → iBUS serial into the Pi (or Pico) → `ibus_teleop` node publishing `/cmd_vel`;
      one switch = manual / autonomous mode, one switch = **emergency stop** (rulebook "stopping mechanism", 2 pts);
      failsafe: stop if the signal is lost. RC must be off / ignored during the autonomous phase.
- [ ] **Data logger → CSV**: one row per tick (e.g. 10 Hz) with a timestamp and the latest value of every sensor:
      LiDAR (min range per sector), IMU (gyro, accel), cmd_vel, motor PWM, odometry / pose (map frame),
      classifier result, gate state, CPU temperature & load, battery if available, mission/BT state.
      Also covers the rulebook §8 event timestamps.
- [ ] **Speed / PWM per floor type** (wood, soil, gravel, sand, grass, ramp up/down): a terrain profile table
      (max speed, min PWM, acceleration ramp) + switch profile by map zone (Nav2 speed filter mask from
      `arena_layout.yaml`) or by detected tilt (IMU pitch on ramps). Values tuned after the new motors are fitted.

## Added 22 Oct — better mapping runs
- [ ] **Wall-safe mapping**: LiDAR safety layer between teleop/autonomy and the motors (slow down < 30 cm,
      stop < 15 cm in the direction of travel) so the robot can't hit the walls and ruin the map.
- [ ] **Self-driving mapping** (if the organisers allow it — the rulebook says teams drive manually in the
      mapping phase): frontier exploration / wall-following on Cartographer's live map, slow and smooth.
- [ ] **Ramp handling (dirt + gravel)**: detect tilt/vibration with the IMU → slow to ~0.1 m/s, pause map
      insertion while pitched or shaking, resume on flat floor; soft-mount the LiDAR; gentle accel ramps.
- [ ] Rehearse the mapping route in the sim (same route, same speeds) before the real run.

## Ask the organisers (roboticsdojo@jkuat.ac.ke)
- [ ] May the robot map **autonomously** in the mapping phase, or must it be driven by hand?
- [ ] Which class does **fabric** belong to?
- [ ] Is **off-board (laptop) processing** allowed during the mapping phase?
- [ ] The gate's "standard waiting time", and the judges' data/communication protocol.

## Done
- [x] Pi backup on GitHub; LiDAR mounting yaw fixed; Cartographer config installed; `map_track.sh` with autosave.
- [x] Virtual arena (Gazebo Fortress + ROS 2 Humble in WSL), updated from field photos; 2D plan.
- [x] Waste classifier: rejects unsure answers ("unknown"), reports bay colour, launch file fixed.
- [x] Sim tools: drifting-odometry mode, ground truth, route driver, localization error logger.

## Added 22 Oct (evening) — power, lidar, terrain stack
- [ ] **Fix the LiDAR / Pi power** (blocks all mapping): wall adapter gave undervoltage every 30–60 s; power bank is steady
      at idle. With the lidar motor on (DTR cleared) the lidar stops replying, with no USB disconnect in the kernel log
      → its 5 V sags at motor start. Try: short thick USB cable, different Pi port, powered USB hub or a separate 5 V
      supply for the lidar, a USB power meter inline (A1 needs ≥ 5 V / 0.5 A). Then re-run `~/start_mapping.sh`.
- [ ] Put a **power check** at the start of `map_track.sh`: warn if `vcgencmd get_throttled` != 0x0 or undervoltage in `journalctl -k -b`.
- [ ] `lidar_reset.py` now retries (6×) and exits 1 on failure (done on the Pi, 22 Oct) — copy it into the repo.
- [ ] **Terrain stack** (`TERRAIN_STACK.md`): code is in the repo and copied to the Pi; still to do on the robot: build check,
      `/scan` ≥ 1300 pts/rev, gyro-bias check, tilt gate, collision-monitor stop/slow zones, feedforward sign (`pitch_up_sign`),
      bag recording, mission dry run. Confirm the real arena size and the lidar mounting yaw (`LASER_YAW`).
- [ ] The wall-follow explorer (`wall_follow_explorer_node.py`) and the guardian's built-in `autonomous_explore` both
      circled a ~1 m pocket in the sim and never left the start area → do not run autonomous mapping on the robot.
- [ ] Sim on WSL: needs `LIBGL_ALWAYS_SOFTWARE=1` and one `wsl` session kept open for the whole run.

## Added 22 Oct (night) — laptop brain (organisers allow off-board processing)
Pi = drivers only, laptop (WSL, Humble) = everything else, both on `ROS_DOMAIN_ID=7`.
- Pi: `~/robot_hw.sh` (tmux `hw`: `lidar_supervisor.sh` + `robot_hw.launch.py`: IMU, motors, camera 10 fps).
- Laptop: `scripts/laptop/brain.sh mapping` · `scripts/laptop/teleop.sh` · `scripts/laptop/brain.sh mission <map>.pbstream [run_mission:=true]`
  · `scripts/laptop/lidar_view.sh` (lidar only). Workspace `~/hardleel_ws` (symlinks to the repo), bags in `~/hardleel_bags`.
- [ ] **Run once on the Pi:** `sudo bash ~/setup_chrony.sh` (Pi serves time to the laptop; venue may have no internet).
- [ ] **Plug the Arduino IMU back in** — not on USB at 21:00 (`lsusb`); `/imu/*` silent.
- [ ] **Lidar power** — still stops replying after a restart; `lidar_supervisor.sh` retries (Boost → Express → Standard).
- [ ] Bench test `scripts/laptop/bench_test.sh` (fake sensors, domain 42): mapping run saved a .pbstream but Cartographer
      received no scans — find out why (QoS / fake scan) and finish the mission half (Nav2 active, goal reached).
      The PC has 8 GB: close Chrome etc. before running the brain (background jobs were killed for low memory).
- [ ] Competition waypoints (`config/competition_course.yaml`) are ASSUMED (screen, doorway, gate, bays, finish):
      Figure 1 has no opening from the start room. Measure the real field and fix the yaml.
- [ ] Ask organisers: may the mission be started from the laptop (`/mission/start`) or must it auto-start?
- [ ] Sim mode (`brain.launch.py sim:=true`) is untested: the sim publishes its own odom->base_link, which clashes
      with Cartographer's.
