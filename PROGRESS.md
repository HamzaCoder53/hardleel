# PROGRESS — Milestone 1 (mini-competition 26 Oct 2026)

Living handoff file. A fresh Claude Code session should read this first, then `TODO.md`, then continue at
"Next". Branch `feat/mission-m1` (worktree `.claude/worktrees/mission-m1`, from `feat/laptop-brain`).

## Decisions
- Middleware: CycloneDDS (`cyclonedds.xml`), `ROS_DOMAIN_ID=7`, chrony for time. Never run ROS on domain 7 in two
  WSL distros at once.
- Laptop moves to ROS 2 Jazzy in a second WSL distro (Ubuntu 24.04); the Humble distro stays as the fallback.
  Hard limit 2 h for the switch; if it fails, finish Milestone 1 on Humble.
- Network: Pi on the laptop hotspot (Pi 192.168.137.8, laptop 192.168.137.1).
- Keep and adapt the existing `competition_mission.py` (tested core) rather than rewriting with BasicNavigator.
- Drop bin is NOT mounted yet: `has_drop_bin: false` in config. Servo pins unknown until the user confirms.
- Models: Claude (Opus) leads (mission node, launch files, Nav2/Cartographer configs, Pi code, integration).
  Sonnet subagents (`.claude/agents/ros2-impl.md`) for self-contained nodes/tools. Gemini via the Antigravity CLI
  (`agy -p ... --mode plan --model gemini-3.8-flash-high`) for review, RUNBOOK.md draft, monitor test script.

## Known state (checked 25 Oct 12:22)
- Pi: /scan 6.1 Hz (frame laser, lidar /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0),
  /imu/data_raw + /imu/data 93 Hz (imu_link), camera 30 Hz 640x480 (69 % of a core), motor_bridge stops after 0.4 s.
  tmux `hwtest` (`~/hw_start.sh`) also runs an EKF + base_link->imu_link static TF -> to be dropped from pi_bringup.
- Laptop: WSL Ubuntu-22.04 Humble (Nav2 1.1.20), sim in `~/rdj_ws`, workspace `~/hardleel_ws` (symlink).

## Steps
- [x] Step 0: worktree + branch, Sonnet agent file, agy check (Gemini 3.8 Flash available, already signed in)
- [x] Step 1: backups verified; user ran the Pi apt upgrade (26 kernel/firmware pkgs held) 14:13, reboot 14:17.
      Re-check vs 12:22 baseline: NO differences (/scan 6.26 Hz, IMU 93.6, camera 30 Hz upright, lidar by-id
      present, gyro bias -12.385, motor timeout 0.4 s with the old hwtest). No more Pi updates after the office test.
- [~] Step 2: Jazzy WSL distro (started ~13:05). DONE: Ubuntu-24.04 + ROS Jazzy + Nav2/Carto/etc, workspace
      ~/hardleel_ws (src symlinked to this worktree) builds incl. rf2o; CLIP deps + weights; .wslconfig
      5 GB + 4 GB swap. Pi deployed with pi_bringup (Boost lidar 6 Hz, camera 5 Hz = 7.7 % CPU, cmd_timeout
      0.3 s, IMU calib OK). From Jazzy over zenoh: all Pi topics at rate, laptop_mapping -> Cartographer /map,
      TF map->laser, collision_monitor active, /cmd_vel Twist on both ends, bag recording.
      LEFT (needs hands on the robot): teleop drives it, motors stop when the laptop stops, map while driving.
- NETWORK DECISION (user, 14:30): keep the hotspot, use rmw_zenoh. WSL (mirrored or NAT) cannot reach the
  hotspot subnet; laptop rmw_zenohd -> Windows portproxy 0.0.0.0:7447 -> Pi rmw_zenohd:7447
  (scripts/laptop/windows_zenoh_proxy.ps1, admin, done on this laptop). .wslconfig now networkingMode=nat.
  ros_env.sh starts the laptop router; robot_hw.sh starts the Pi router. HARDLEEL_RMW/RMW=cyclone = old setup.
- [x] M1.1 IMU calibration motion check (imu_serial_node: CalibStats, retry, /imu/recalibrate, /imu/calib_status)
- [x] M1.2 laptop_mapping / laptop_mission / laptop_localization launch files (untested on ROS yet)
- [x] M1.3 waypoints.py, save_pose, click_waypoints (+selftest), waypoints_course.yaml draft, waypoints_office.yaml
- [~] M1.4 CLIP classifier (Sonnet agent running) + monitor test script (Gemini)
- [x] M1.5 gate_check.py (sector median + GatePoller) wired into the mission
- [x] M1.6 mission node: waypoints files, events, loading adapter (stub), /dump if has_drop_bin, replan
- [x] M1.7 event_logger (+ judges_sender stub)
- [x] M1.8 collision_monitor boxes (one node, both phases), obstacle_watch (lidar vs /map), wait-then-replan
- [~] M1.9 pi_bringup + robot_hw.sh done (not deployed to the Pi yet); RUNBOOK.md (Gemini draft) pending
- [x] Bag recording: built into laptop_mapping/localization (record_bag:=true default, mcap, ~/hardleel_bags)
- [x] FIRST END-TO-END SIM RUN (13:48, Humble distro, scripts/laptop/sim_e2e_test.sh, log docs/sim_runs/):
      all stages + every required timestamp; gate -> path2 in 0.41 s; bin_metal skipped (sim waypoint in a
      narrow lane, 2 x 45 s timeouts) -> goal_timeout_s now 30 s. Total 174 s.
- [x] RUNBOOK.md (Gemini draft, corrected) incl. office test sequence

## Decisions added during the build
- Sim stays on Gazebo Fortress in the Humble distro tonight (porting to Harmonic = risk, no gain for tomorrow).
  The laptop launch files work on both distros (stack_common.nav2_params_for_distro rewrites Jazzy-only bits).
- Sim tests use FastDDS: CycloneDDS over WSL loopback silently drops the sim's LaserScan messages (IMU gets
  through). The real robot keeps CycloneDDS (unicast to the Pi over the hotspot interface: worked on 22 Oct).
- `ros2 topic hz` ignores SIGTERM in WSL; use scripts/laptop/topic_probe.py or `timeout -k 2 -s INT`.
- Humble distro: a pip --user setuptools (pulled in by torch) broke colcon; removed. Same guard in the Jazzy setup.
- One collision_monitor for both phases; StopBox only 12-22 cm ahead -> loading/bin waypoints ~0.30 m off faces.
- Obstacle detection = lidar points ahead that are not near an occupied /map cell (obstacle_watch.py).
- [ ] First end-to-end sim run (stub classifier)
- [ ] Office test

## Findings (25 Oct ~12:50)
- Main checkout's uncommitted work: 22 of 26 files are byte-identical in feat/mission-m1; the other 4
  (TODO.md, robot_slam/package.xml, setup.py, stack_common.py) are older versions that mission-m1 supersedes.
  The Pi runs mission-m1's versions of everything except those 3 robot_slam files (it has main's older copies).
  Nothing needed to be carried over. User made a full copy: `Documents\hardleel-pi-repo-backup-25oct`.
- Course scans: NO rosbags of the course exist (repo `bags/` empty, none on the Pi). Only Cartographer pbstreams
  on the Pi `~/maps/`: track_20261015_1232 (course-shaped, ~35 deg drift, doubled walls), track_20261022_1644
  (~5 x 5 m, clean-ish), track_20261022_1648_final (fragments spread over 18 x 39 m, unusable). WSL
  `~/map_rebuild/scans_bag` (16 Oct) holds thinned scans extracted from the 15 Oct pbstream (no IMU).
  Each map's origin is wherever that run started; whether that was the official start spot is user knowledge.

- 25 Oct 13:20, user: the 22 Oct maps ARE the course (15 Oct is not). Reference = track_20261022_1644.
  Chamfer fit of Figure 1 walls to it (normal and mirrored, all rotations) is NOT credible (67 % inliers, start
  lands at the bins, the map and its own autosave disagree). The map shows: wall 0.3 m behind the start, a
  ~0.7 m x 5 m corridor straight ahead (fits Figure 1's right corridor), and a ~2.6 x 2.3 m room on the robot's
  RIGHT, whereas Figure 1 has only 0.49 m of floor right of the start pad. Either the real field differs from
  Figure 1 or the map is mirrored/partial. Covers ~30 % of the field: junction, gate, bins, path 1/2 missing.
  1626 / 1626_autosave do not fill the gap (2 x 2 m / broken 53 x 34 m). Renders: docs/maps_review/.
  Decision: waypoints_course.yaml = Figure 1 draft relative to the start pose (all LOW confidence); tomorrow the
  click tool on the fresh phase-1 map is the primary way to set the points.
- Camera (13:00 frame): upright and level, no rotation needed (param rotate_deg: 0 kept). Mounted low: the
  frame is mostly floor/desk legs. Cable not clearly in view (small white blur at the bottom-left edge).
- Backups: WSL Humble export `%USERPROFILE%\wsl-backups\Ubuntu-22.04-humble-25oct.tar` (6.68 GB, verified
  listing); Pi `Documents\hardleel-backups\pi_home_20261025.tgz` (45 MB, 415 files = Pi count, md5 spot-checked).
- Robot ID: "HARDLEEL" (config, replace if the organisers give an official one).

## Interfaces (all Milestone 1 code builds against these)
- Waypoints file (`robot_slam/config/waypoints_course.yaml`, `waypoints_office.yaml`), map frame, origin = start pose
  (x forward, y left), yaw in degrees:
  ```yaml
  frame: map
  poses:                      # ordered: the click tool asks for them in this order
    screen: {x: 0.8, y: -0.14, yaw_deg: -90, conf: low, note: '...'}
  sequences:                  # ordered pose-name lists
    terrain: [terrain_1, terrain_2]
    path1: [path1_1, path1_2]
    path2: [path2_1, path2_2]
  ```
  Required pose names: screen, loading_plastic, loading_metal, loading_organic, terrain_* , junction, gate_check,
  path1_*, path2_*, bin_plastic, bin_organic, bin_metal, finish.
- Events: topic `/mission/event` (std_msgs/String, JSON `{"event": name, "detail": {...}}`). Names: autonomous_start,
  waste_detected {class, confidence}, loading_complete, terrain_complete, obstacle_detected, obstacle_response
  {action, response_s}, gate_decision {decision: path1|path2, wait_s}, deposit_complete {bin, dropped}, finish.
  The event_logger stamps them, computes total_run_time and gate_wait_time, writes JSON+CSV, prints a summary.
- Classifier: service `/classify` (std_srvs/Trigger) -> message `class:conf:colour` (existing parse_classify format;
  class `unknown` when below threshold); also `/classifier/result` (String JSON with per-class scores).
- Drop: service `/dump` (std_srvs/Trigger) on the Pi; the mission calls it only if `has_drop_bin: true`.
- Gate: `robot_slam/gate_check.py` pure helpers (sector median, poller) used by the mission.

## Open questions for the user
- Robot ID; camera rotation/cable status; saved course scans (path, format, start spot?); servo pins.

## Next
Step 1.
