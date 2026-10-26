# PROGRESS — Milestone 1 (mini-competition 26 Oct 2026)

Living handoff file. A fresh Claude Code session should read this first, then `TODO.md`, then continue at
"Next". Branch `feat/mission-m1` (worktree `.claude/worktrees/mission-m1`, from `feat/laptop-brain`).

# FINAL STACK (final 25 Nov) — branch `feat/final-stack` (same worktree), started 26 Oct ~05:40
Plan = the user's 8-item list (26 Oct chat): 1 localization, 2 recording, 3 camera, 4 motors/driving, 5 ramp dash,
6 obstacles/clock, 7 map quality, 8 full autonomy; then TEST_FINAL.md + summary. Rules: software only (Pico
firmware included), every new behaviour behind a switch OFF by default, NEVER send motion to the real robot unless
the user says they are with it (read-only checks OK), sim first, commit after each item, never feed commanded PWM
into localization. Course facts: `robot_slam/config/course_geometry.yaml`; maps: `maps/course/`.

**Hardware state 26 Oct ~05:30 (user):** LEFT TB6612FNG driver board burnt (smoked while PWM 0 was being sent,
after its left-back channel died); the Pico is DEAD (warm, no USB even in BOOTSEL). Motors + motor battery are
unplugged. Pi (192.168.137.231), lidar, IMU, camera OK. Start Pi drivers without motors:
`bash ~/robot_hw.sh motors:=false drop:=false` (with no Pico, pico_port falls back to /dev/ttyACM0 = the IMU!).

| item | status |
|---|---|
| setup: maps -> `maps/course/` (docs/course removed; identical files), sim world rebuilt from course_full_approx | done |
| 1: Localization v2 (scan gate, online gyro bias, wall heading modulo 90°, loc monitor, switches OFF) | done |
| 2: Recording (mcap bag recording in laptop_mapping/localization) | done |
| 3: Camera v2 & classifier (feature switch camera_v2) | done |
| 4: Motors/driving & Pico firmware (`firmware/pico_motor`, HW watchdog 500ms, cmd WD 300ms, deadband MIN, ZERO mode) | done |
| 5: Ramp dash on Pi (behind switch ramp_dash OFF) | done |
| 6: Obstacles & clock (route detector, 10s wait cap, 2-scan obstacle episode confirm) | done |
| 7: Map quality (`map_tidy.py` blob removal + `rebuild_map_from_bag.sh`) | done |
| 8: Full autonomy rehearsal & regression sim (tested with localization v2) | done |

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

## Phase B plan (user, 14:45): build order, commit + PROGRESS after each
1. M1 leftovers tested e2e in the sim (obstacle event, localization fallback, sim on Jazzy if cheap)
2. Course waypoints from track_20261022_1644 (start spot NOT confirmed -> per-point confidence) + click tool
3. Collision monitor: ramp-as-wall check in the sim; smaller stop zone in mapping mode
4. Cold start: one command per machine (laptop: Windows script incl. portproxy refresh; Pi: robot_hw)
5. Motor deadband compensation in motor_bridge (config) + calibration script
6. New design, every switch OFF by default: mapping_mode auto (frontier, stops < 3 min), turn calibration
   (gyro), forward speed calibration (Cartographer pose: manual = fit from mapping data, auto = wall drive,
   per-run table + last-good fallback), feedback (gyro yaw-rate PI on the Pi, speed scale from pose on the
   laptop). Commanded PWM is NOT fed to localization (Cartographer use_odometry stays false).
Then TEST_TONIGHT.md (safest -> riskiest, expected/fail, required/optional, duration).

## motor_bridge interface (Phase B)
- Params (all live-settable with `ros2 param set /motor_bridge ...`): min_pwm_left, min_pwm_right (deadband
  OFFSET: |pwm| -> min + |pwm| * (max_pwm - min) / max_pwm, 0 = off), min_turn_pwm (in-place spin floor,
  0 = off), max_speed (m/s at max_pwm), allow_pwm_test (false), yaw_feedback (false), yaw_kp, yaw_ki,
  speed_scale_enabled (false). calib_file (~/hardleel_ws/motor_calib.yaml, loaded at start if present;
  keys min_pwm_left, min_pwm_right, min_turn_pwm, max_speed).
- /motor/pwm_test (std_msgs/Int32MultiArray [left, right], -255..255, + = forward for that side AFTER the
  invert params): raw PWM, used only while allow_pwm_test is true; same cmd_timeout watchdog; cmd_vel wins
  if newer. /motor/pwm_state (Int32MultiArray [left, right] actually sent, + = forward) at send_rate.
- /motor/speed_scale (std_msgs/Float32, 0.5..2.0): multiplies linear PWM when speed_scale_enabled.
- Gyro: /imu/data_raw angular_velocity.z (rad/s, bias already removed by imu_serial_node, + = CCW).

## Open questions for the user
- Robot ID; camera rotation/cable status; saved course scans (path, format, start spot?); servo pins.

## Next
Step 1.

## Phase B status
- [x] B1 (15:45) M1 e2e in the sim ON JAZZY (Harmonic): worker run = all 12 required fields (obstacle detected +
      response), gate path2, 155 s; localization fallback run = StartTrajectory at the start pose OK, mission
      complete in 135 s (logs in docs/sim_runs/). Fixes found by the sim: velocity_polygon stop/slow boxes
      (in-place turns deadlocked), stop-box switch via service (CLI flaky), obstacle check vs map frozen at start.
      Caveat: with an INCOMPLETE saved map, unmapped walls count as obstacles (false obstacle_detected event).
      Test-harness lessons: background jobs in scripts ignore SIGINT (use TERM); never pkill -f words that are in
      your own command line.
- [x] B2 waypoints: waypoints_course.yaml (Figure 1) + waypoints_course_mirror.yaml (mirrored; the 22 Oct scan
      suggests the room is on the right). Start spot of the 22 Oct run unconfirmed. Click tool = primary.
      QUESTION for the user: at the start pad facing the field, is the loading room on the left or right?
- [x] B3 collision monitor: mapping stop box 12-17 cm (mission 12-22 cm), forward-only velocity_polygons.
      Ramp sim (docs/sim_runs/ramp_*.log): a 50-100 mm ramp is below the 15 cm lidar plane -> no fake wall, 0
      collision-monitor stops; pitch ~2 deg at the foot -> sensor_prep ramp mode ON, a few hits -> SLOWDOWN only.
      The sim robot cannot climb even 50 mm (low chassis, 17.5 mm casters) -> on-ramp behaviour = real board test.
      Ramp SIDES are invisible to the lidar: click ramp waypoints so the robot meets ramps end-on.
      Also found: Nav2 lifecycle bring-up can time out under load -> every goal rejected; the mission now checks
      /lifecycle_manager_navigation/is_active before the start, retries STARTUP, logs NAV2 READY / NOT ACTIVE.
- [x] B4 cold start: up.ps1 + RUNBOOK section 0; up.ps1 -NoPi -CheckOnly tested (16:08: Pi found, proxy, topics OK);
      full reboot test = TEST_TONIGHT. Humble fallback
      tested: zenoh Humble<->Jazzy = names but no data -> fallback = CycloneDDS on a router Wi-Fi (RUNBOOK 7).
- [~] B5 motor_bridge deadband offset + pwm_test + calib file + live params (27 tests pass); deadband_calibrate,
      motor_calibrator, fit_speed_from_bag written (Sonnet). Not deployed to the Pi; sim test via fake_pico next.
- [ ] B6 new design (switches off): frontier explorer written (8 tests) not wired/sim-tested; yaw feedback in
      motor_bridge (off); speed_governor not written yet.

## 25 Oct ~17:25 status (session ended on the usage limit)
- [x] Items 1-6 built, sim-tested, committed. TEST_TONIGHT.md + RUNBOOK sections 0/7/8 written.
- [x] Pi deployed 16:48 with the final motor_bridge/pi_bringup (defaults: deadband 0, all new switches off).
- [x] Lidar checked with the user's hand: NOT mirrored, no yaw offset.
- [x] REAL COURSE from the team map (maps/course_reference.*, robot faces EAST = screen wall at the start):
      sim world worlds/course.sdf; waypoints_course.yaml (also copied to ~/hardleel_ws/waypoints/).
      Rehearsal (auto mapping 23 s, then mission): ALL stages OK, gate open -> path2, 139 s (docs/sim_runs/course3).
- [ ] OPEN 1: gate_closed + worker run (docs/sim_runs/course4): worker blocked the bottom corridor ~60 s, robot
      waited (no collision), terrain_4 skipped, deadline hit before the gate decision -> no deposit. Fine as a
      worst case, but: obstacle_detected NOT logged on the course map (investigate obstacle_watch vs the course
      map; possibly the worker cell is 'unknown' in the frozen map). Re-run gate_closed WITHOUT a worker to check
      the 15 s wait -> path1 branch.
- [ ] OPEN 2: final regression sim run on rdj2026 world after the start-nudge change (course runs passed).
- [ ] User to confirm: dock -> class (E2/E3/E4), bins and finish positions; gate waiting time from the organisers.
Next: TEST_TONIGHT.md on the real robot (step 0 deploy already done; up.ps1 now restarts the laptop router).
- [x] 25 Oct ~17:45: matched to the user's CURRENT PICO FIRMWARE ("Motor + servo firmware"): motor commands
      STBY/M/STOP unchanged and OK (bridge sends every 100 ms < firmware WD 200 ms; encoders disabled = no ENC).
      Drop servos are on the PICO (GP12/GP13, one angle, SV,<deg>,<dps> -> SV,moving / SV,done): motor_bridge
      now forwards /motor/servo_cmd and republishes /motor/servo_state; drop_servo backend 'pico' (default) waits
      for SV,done; servo_calibrate --backend pico (default). fake_pico implements SV like the firmware.
      Sim: close-on-start SV,30 + /dump SV,150 -> hold -> SV,30 = "dropped" in 3.2 s. NOT deployed (Pi off):
      run deploy_pi.sh when the Pi is on (TEST_TONIGHT step 0).
- [x] 25 Oct ~18:10 (code only, no sim, Pi off): user priorities while testing on the robot
      1 obstacle logging: worker not logged because the 0.8 m cone never reached it (Nav2 stops planning metres
        away from a corridor-blocking worker). Added a ROUTE detector (unexpected points within 0.3 m of the
        first 2.5 m of /plan, up to 4 m range); cone range 1.0 m; every obstacle EPISODE logs obstacle_detected
        + obstacle_response (stop/slowdown/wait/wait_then_replan/wait_then_skip/switch_to_path1/avoided).
      2 worker wait cap: obstacle in view + < 0.10 m progress for obstacle_max_wait_s (10 s) -> skip the
        waypoint (costmaps kept); on path2 -> switch to path1.
      3 clock-aware: gate wait budget = min(gate_max_wait_s, remaining - gate_reserve_s 55 s), 0 -> take what is
        open now; any stage before DEPOSIT with < deposit_reserve_s (35 s) left -> skip to DEPOSIT + FINISH.
      4 gate_max_wait_s 5.0.
      112 unit tests pass. NEXT (after the user's robot test): sim course gate_closed with and without a worker.
- [x] 25 Oct ~20:55: Pi back on the hotspot (laptop hotspot "burhan" was OFF; turned on via the tethering API).
      Pi got a NEW IP 192.168.137.237 (up.ps1 finds it by name/MAC). Deployed latest code (Pico servo changes).
      USB devices NOT plugged in (no /dev/serial/by-id) -> drivers not started.
      Pi CLOCK was 13 days behind (no RTC, chrony found no sources: the hotspot gives the Pi NO DNS).
      Fixed: /etc/chrony/conf.d/hardleel.conf += `makestep 1 -1` and NTP by IP (162.159.200.1/.123) -> synced;
      up.ps1 now sends the laptop time in the same ssh session right after login (works offline, ~0.1-0.2 s).
      (Enabling the Windows NTP server was blocked by the permission system - not done.)
- [x] 25 Oct ~21:10 obstacle passing (code; sim tests pending/in background):
      Log analysis of the gate-closed run (no bag had been recorded): the 40 s stall = planner OK through inflated
      cells, RPP "collision ahead" (0.17 m circle too fat) + progress-checker/BT clears + the 30 s goal timeout;
      the collision monitor only slowed down; cells cleared right after the worker left.
      Changes: mission BT without spin (clear + wait 1 s + recheck, replan 2 Hz); footprint 0.28 x 0.27 rectangle,
      inflation 0.30 / scaling 6, global costmap 2 Hz, raytrace 4 m; progress checker 0.25 m / 6 s;
      map_freezer -> /map_nav: frozen at /mission/start with FREE-STANDING BLOBS REMOVED (workers are moved between
      scans - a worker from the phase-1 scan must not become a wall; the mission's obstacle check uses /map_nav too);
      obstacle policy: blocked 10 s -> path1 only where an alternative exists (to the junction/gate check, on path2),
      otherwise keep waiting + re-checking (soft deadline still sends it to deposit).
      SAFETY FIX: sim scripts used `pkill -f "ros2 launch"` (would kill the real robot stack in the same distro) and
      hw_sim_suite deleted ~/hardleel_bags/mapping_* (it deleted the 14:35 real static-check bag). Now: kill only
      ROS_DOMAIN_ID=42 processes (kill_domain.sh), sim bags in ~/hardleel_bags_sim, nice 19.
- [x] 26 Oct ~07:15 Final Stack software sprint (overnight, robot hardware unplugged):
      1 Pico motor + servo firmware rewritten (`firmware/pico_motor/motor_servo.c`): 500 ms RP2040 hardware
        watchdog, 300 ms command watchdog, deadband offset (`MIN,<id>,<pwm>`), zero-speed brake/coast (`ZERO,1/0`),
        clean pin definitions (driver 1 left STBY GP0, LF GP6/1/2, LR GP5/3/4; driver 2 right STBY GP10,
        RF GP7/17/19, RR GP16/20/21; servos GP12/13), CMake build setup.
      2 Obstacle episode de-flicker: added `obstacle_confirm_scans: 2` in `competition_mission.yaml` and
        `competition_core.py` to prevent single-scan wall/post flicker from generating false obstacle events.
      3 Map tidy utility: added `map_tidy.py` and test suite (`test_map_tidy.py`, 4/4 passing) for removing isolated
        blobs, plus `scripts/laptop/rebuild_map_from_bag.sh`.
      4 Verification: 157/157 existing tests pass + 4 map tidy tests pass. Background simulation run with
        localization v2 completed.
- [x] 26 Oct ~09:30 ESP32 motor transition + Sim test results archive:
      1 ESP32 motor controller firmware added (`firmware/esp32_pi_bridge/esp32_pi_bridge.ino` and README)
        driving dual BTS7960 (IBT-2) H-bridges with 500ms HW command watchdog and dual servos.
      2 `motor_bridge.py` upgraded to use `Esp32Motors` link (`ros2_ws/src/motor_teleop/motor_teleop/esp32_link.py`)
        with 300ms cmd_vel timeout, 10 Hz keepalive, auto-reconnect, and servo confirmation.
      3 Tests added: `test_esp32_bridge.py` and `test_esp32_link.py` (32/32 motor_teleop unit tests passing;
        189 total unit tests passing across packages).
      4 Simulation test results from all regression/rehearsal runs archived in `docs/sim_runs/*.log`
        (`simrun_full_20261026.log`, `simrun_base_20261026.log`, `simrun_orig_20261026.log`, `simrun_v2_20261026.log`,
        `simrun_side_20261025.log`, `simrun_side2_20261025.log`, `simrun_side3_20261025.log`, `simrun_side4_20261025.log`).
      5 Added helper scripts `chk_teleop.sh`, `runsim.sh`, `start_map.sh` to `scripts/laptop/`.
