# RUNBOOK: Robotics Dojo 2026 (Robot: hardleel / ID: HARDLEEL)

> Drafted by Gemini 3.8 Flash from the team decisions, reviewed and corrected by Claude. Office test sequence at the end.

**Mini-Competition:** 26 Oct 2026 | **Final:** 25 Nov 2026  
**Setup:** WSL Ubuntu-24.04 (ROS 2 Jazzy). Repo root: `~/hardleel`. Domain ID: `7`. Middleware: **rmw_zenoh**
(laptop router -> Windows port proxy :7447 -> Pi router), because WSL cannot reach the hotspot subnet with DDS.  
**Pi 4:** `192.168.137.8` (`ssh hardleel@192.168.137.8`).  
**Rules:** Phase 1 (3 min mapping, manual teleop). Phase 2 (3 min autonomous, NO human input, 1 start command). Exactly 1 restart per round allowed. Motors auto-stop 0.3 s after laptop commands stop.

---

## 0. Cold start (laptop rebooted and/or Pi power-cycled) - one command

1. Windows: turn the **Mobile hotspot ON** (Settings > Network > Mobile hotspot). Power the robot; wait ~60 s
   for the Pi to boot and join the hotspot. Put the robot on the start pad.
2. Open **PowerShell as administrator**, then:
   ```powershell
   cd "$env:USERPROFILE\Documents\hardleel-pi-repo\.claude\worktrees\mission-m1"   # branch feat/mission-m1 (= WSL ~/hardleel)
   powershell -ExecutionPolicy Bypass -File scripts\laptop\up.ps1
   ```
   It finds the Pi, re-points the port proxy (handles a new Pi IP or WSL IP), starts the Pi drivers over
   ssh (type the Pi password when asked; **don't touch the robot for 5 s after that**), checks
   the Pi topics from WSL and opens two windows: **A = laptop_mapping** (keep it open into phase 2) and
   **B = teleop**. Good: step 5 prints `/scan ~6 Hz`, `/imu/data ~90 Hz`, camera `~5 Hz`.
   - Pi already running? add `-NoPi`.
   - Only the Pi was power-cycled? Same command (the Pi part restarts everything on the Pi).
3. Every other WSL terminal: `wsl -d Ubuntu-24.04`, then `cd ~/hardleel && source scripts/laptop/ros_env.sh`.

Optional, removes the password prompt (run it yourself once, in Git Bash or PowerShell):
`ssh-keygen -t ed25519 -f $HOME/.ssh/hardleel_ed25519` then
`type $HOME\.ssh\hardleel_ed25519.pub | ssh hardleel@192.168.137.8 "cat >> ~/.ssh/authorized_keys"` and add
`-i $HOME\.ssh\hardleel_ed25519` to the ssh line in `up.ps1`.

---

## 1. Night Before

- [ ] Charge all robot battery packs and the laptop to 100%.
- [ ] Commit all laptop code: `git commit`.
- [ ] If code changed, deploy to Pi (do NOT deploy on competition morning unless needed):
  ```bash
  scripts/laptop/deploy_pi.sh
  ```
- [ ] Clean up disk space: copy old maps (`~/hardleel_maps`) and bags (`~/hardleel_bags`) off the laptop.
- [ ] Pack checklist:
  - [ ] Robot ("hardleel")
  - [ ] Charged robot batteries
  - [ ] Laptop and charger
  - [ ] USB cables
  - [ ] Spare lidar cable

---

## 2. Pre-Run Checklist

Run all commands in WSL from `~/hardleel`. Source the environment in every terminal first:
```bash
source scripts/laptop/ros_env.sh
```
*(Confirms Pi IP `192.168.137.8`, laptop IP, ROS_DOMAIN_ID 7, and clock sync).*

- [ ] Turn on Windows mobile hotspot on the laptop. Connect Pi to it.
- [ ] Verify distro: Ubuntu-24.04 Jazzy running (Fallback: Ubuntu-22.04 Humble; NEVER run both).
- [ ] Check config files for organiser announcements:
  - [ ] Robot ID: `robot_id` in `ros2_ws/src/robot_slam/config/event_logger.yaml` ("HARDLEEL" or TBD (organisers)).
  - [ ] Judges' data format: sender stub in `ros2_ws/src/robot_slam/config/event_logger.yaml` (TBD (organisers)).
  - [ ] Gate wait: `gate_max_wait_s` (15 s placeholder / TBD (organisers)) & `gate_open_threshold_m` in `ros2_ws/src/robot_slam/config/competition_mission.yaml`.
  - [ ] Loading protocol: `load_wait_s` / `loading_adapter` stub in `ros2_ws/src/robot_slam/config/competition_mission.yaml`.
  - [ ] Drop bin: `has_drop_bin` in `ros2_ws/src/robot_slam/config/competition_mission.yaml` (`false` until mounted).
  - [ ] Classifier prompts: `label_map` in `ros2_ws/src/waste_classifier/config/clip_classifier.yaml` (fabric -> organic placeholder).
- [ ] Optional classifier bench test with monitor (robot camera aimed at screen):
  ```bash
  ros2 run waste_classifier clip_monitor_test <folder of images named plastic_*.jpg, metal_*, organic_*>
  ```
- [ ] Place robot on the start spot, start the Pi drivers, and do NOT touch the robot for the next 5 s (gyro calibration runs during the first 3 s):
  ```bash
  ssh hardleel@192.168.137.8 ./robot_hw.sh
  ```
- [ ] Verify IMU calibration starts with "OK":
  ```bash
  ros2 topic echo --once /imu/calib_status
  ```
  *(If "WARN moved": keep robot still and run `ros2 service call /imu/recalibrate std_srvs/srv/Trigger`)*.
- [ ] Check sensor rates on laptop:
  - [ ] Lidar (~6 Hz): `ros2 topic hz /scan`
  - [ ] IMU (~90 Hz): `ros2 topic hz /imu/data`
  - [ ] Camera (~5 Hz): `ros2 topic hz /camera/image_raw/compressed`
- [ ] Check Pi temperature and throttling:
  ```bash
  ssh hardleel@192.168.137.8 'vcgencmd measure_temp; vcgencmd get_throttled'
  ```
  *(Must be < 75 C and `throttled=0x0`)*.

---

## 3. Round Procedure Step by Step

### Phase 1: Mapping (3 min limit)
1. **Terminal A (Mapping):** Start Cartographer, safety chain, RViz, and bag recording:
   ```bash
   ros2 launch robot_slam laptop_mapping.launch.py
   ```
   *Good:* RViz appears, map builds, recording to `~/hardleel_bags`.  
   **DO NOT CLOSE OR RESTART TERMINAL A.** Leave it running into Phase 2 (start pose is map origin).
2. **Terminal B (Teleop):** Start keyboard driving:
   ```bash
   scripts/laptop/teleop.sh
   ```
3. Drive slowly (0.15 m/s) with gentle turns. Map every corridor, screen, loading stations, gate, bins, finish.  
   *(Optional: record current pose anytime with `ros2 run robot_slam save_pose <name>`)*.

### Between Phases: Save Map & Waypoints (< 2 min target)
1. **Terminal B:** Save the map:
   ```bash
   scripts/laptop/save_map.sh course_r1
   ```
   *Good:* Saves `course_r1.pbstream`, `.pgm`, `.yaml` to `~/hardleel_maps`. Cartographer stays running.
2. **Terminal B:** Press `Ctrl-C` to exit teleop.  
   - [ ] **CONFIRM TELEOP TERMINAL IS CLOSED.** Teleop running in Phase 2 = disqualification.
3. **Terminal B:** Click waypoints on the fresh map:
   ```bash
   ros2 run robot_slam click_waypoints ~/hardleel_maps/course_r1.yaml --waypoints ~/hardleel_ws/waypoints/waypoints_course.yaml
   ```
   *(Or pass `--topic /map` for live map)*.
   - **Clicks:** Left-click position, then left-click heading direction.
   - **Keys:** `Enter`/`Space` keep drafted pose; `s` skip; `u` undo; `h` toggle heading clicks; `q` save and quit; `Esc` quit without saving.
   - **Names required:** `screen`, `loading_plastic`, `loading_metal`, `loading_organic`, `terrain_1`, `terrain_2`, `junction`, `gate_check`, `path1_1`.., `path2_1`.., `bin_plastic`, `bin_organic`, `bin_metal`, `finish`.
   - **Placement rules:**
     - Loading stations & bins: robot centre ~0.30 m from the face (collision monitor stops at 10 cm from bumper).
     - `gate_check`: robot facing the gate directly.

### Phase 2: Autonomous (3 min limit)
1. **Terminal C (Mission):** Launch mission before the start signal:
   ```bash
   ros2 launch robot_slam laptop_mission.launch.py waypoints:=~/hardleel_ws/waypoints/waypoints_course.yaml
   ```
2. Wait for initialization (10–30 s):
   - [ ] Check the CLIP model loaded: `"model ready in ... s"` appears.
   - [ ] Check no `"waypoints:"` errors exist in log.
   - [ ] Check event logger path is printed: `~/logs/runs/run_<time>_HARDLEEL.json`.
   - [ ] Verify log displays: `"Waiting for /mission/start"`.
3. **Terminal B:** At official start signal, send the ONE allowed command:
   ```bash
   scripts/laptop/go.sh
   ```
4. **HANDS OFF THE KEYBOARD.** No human contact allowed. Watch live table printed by event logger.
5. *(Emergency only — ends run)*:
   ```bash
   scripts/laptop/abort.sh
   ```

### Restart Procedure (1 allowed per round)
1. Stop robot: `scripts/laptop/abort.sh`.
2. Carry robot back to start pose and orientation.
3. In **Terminal C only**: Press `Ctrl-C` to stop `laptop_mission`.
4. In **Terminal C**: Relaunch `laptop_mission.launch.py` with waypoints argument.
5. Wait for `"Waiting for /mission/start"`, then run: `scripts/laptop/go.sh`.  
   *(If Cartographer crashed or map was lost, use Fallback Localization below before relaunching mission)*.

---

## 4. If Something Goes Wrong

| Symptom | Cause | Action |
| :--- | :--- | :--- |
| Lidar `/scan` missing | Supervisor restarting or USB loose | Replug USB cable. Check supervisor on Pi: `ssh hardleel@192.168.137.8` then `tmux attach -t hw` (supervisor auto-restarts lidar). |
| IMU calib WARN | Robot bumped during 3 s gyro boot | Keep robot completely still. Run: `ros2 service call /imu/recalibrate std_srvs/srv/Trigger`. Status must return "OK". |
| Robot not moving | Collision monitor or stopped commands | Check `/cmd_vel`. Check if obstacle is within 10 cm of bumper. Motors auto-stop 0.3 s after laptop stops sending commands. |
| Cartographer lost / map rotated / crashed | SLAM tracking failure | Carry robot to start pose and heading. Run fallback localization instead of mapping: `ros2 launch robot_slam laptop_localization.launch.py pbstream:=~/hardleel_maps/course_r1.pbstream`, then relaunch mission in Terminal C. |
| Classifier unavailable | Network/model error | No action needed. Mission automatically falls back to `default_class plastic` after retries. |
| Wi-Fi drop | Hotspot disconnected | Robot stops safely within 0.3 s. Reconnect laptop Wi-Fi hotspot. Count as the 1 allowed round restart if in Phase 2. |

---

## 5. After the Round

1. Stop nodes: `Ctrl-C` in Terminal C (`laptop_mission`), then `Ctrl-C` in Terminal A (`laptop_mapping`).
2. Verify output files exist:
   - Judges' timestamps: `~/logs/runs/run_<time>_HARDLEEL.json` and `.csv`  
     *(Contains: `autonomous_start`, `waste_detected`, `loading_complete`, `terrain_complete`, `obstacle_detected`, `obstacle_response`, `gate_decision`, `gate_wait_time`, `deposit_complete`, `finish`, `total_run_time`)*.
   - Mission trace: `~/logs/mission_<time>.csv`.
   - Bag files: `~/hardleel_bags` (keep all bags for final tuning on 25 Nov).
3. Back up logs and bags to an external flash drive or secondary directory.
4. Record any competition field discrepancies, organiser timing changes, or hardware faults.
5. Swap robot battery with a fresh, fully charged pack for the next round.

---

## 6. Office test sequence (tonight, before the mini-competition)

Use `waypoints:=~/hardleel_ws/waypoints/waypoints_office.yaml` and re-record its poses in the office first.
Tick each line only when you have seen it work.

1. [ ] **Pi up**: `scripts/laptop/deploy_pi.sh` (once), then `ssh hardleel@192.168.137.8 ./robot_hw.sh`.
   `/imu/calib_status` = OK; `/scan` ~6 Hz, `/imu/data` ~90 Hz, camera ~5 Hz; Pi < 75 C.
2. [ ] **Motor stop without the laptop**: drive with `scripts/laptop/teleop.sh`, hold a key, then turn the
   laptop hotspot OFF. The robot must stop within ~0.4 s. Turn the hotspot back on and reconnect.
3. [ ] **Mapping**: `ros2 launch robot_slam laptop_mapping.launch.py`, drive a loop of the office,
   `scripts/laptop/save_map.sh office_1`. Walls straight and single in RViz.
4. [ ] **Waypoints**: `ros2 run robot_slam click_waypoints ~/hardleel_maps/office_1.yaml --waypoints ~/hardleel_ws/waypoints/waypoints_office.yaml`
   (all points in under 2 min), or drive to each spot and `ros2 run robot_slam save_pose <name> --file ~/hardleel_ws/waypoints/waypoints_office.yaml`.
5. [ ] **Cartographer stays up**: close teleop, do NOT stop terminal A, start `laptop_mission.launch.py
   waypoints:=~/hardleel_ws/waypoints/waypoints_office.yaml classifier:=stub stub_class:=plastic`. Then `go.sh`.
   The robot drives to every office waypoint in order; the event logger table fills up.
6. [ ] **Obstacle**: during a run, step in front of the robot. It must stop (StopBox), wait `obstacle_wait_s`,
   then re-plan; the log shows `obstacle_detected` and `obstacle_response`.
7. [ ] **Classifier on a monitor**: `ros2 launch robot_slam laptop_mission.launch.py` with the real classifier
   (default `classifier:=clip`), then in another terminal
   `ros2 run waste_classifier clip_monitor_test <folder of plastic_*.jpg, metal_*.jpg, organic_*.jpg>` with the
   robot camera ~0.5-1 m from the laptop screen. Note the accuracy and the latency.
8. [ ] **Drop**: with the bin NOT mounted, `ros2 service call /dump std_srvs/srv/Trigger` answers
   "no drop bin: skipped". Once mounted: calibrate (`python3 -m robot_slam.servo_calibrate --gpio 12 --gpio-b 13`
   on the Pi), set `has_drop_bin: true` in `drop_servo.yaml` AND `competition_mission.yaml`, check the robot
   still fits 30 x 30 x 30 cm, the bin blocks neither the lidar's 360 deg view nor the camera, update the Nav2
   footprint (`robot_radius` in `nav2_params_mission.yaml`, `FootprintApproach` in `collision_monitor.yaml`),
   then one full office run with the bin on.
9. [ ] **Timestamp log**: `~/logs/runs/run_<time>_HARDLEEL.json` has every field (obstacle fields only if
   step 6 was done) and `total_run_time`.
10. [ ] **Fallback**: Ctrl-C terminal A, put the robot on the start pose,
    `ros2 launch robot_slam laptop_localization.launch.py pbstream:=~/hardleel_maps/office_1.pbstream`,
    then a short mission run.

---

## 7. Fallback: back to the Humble laptop distro (only if the Jazzy distro is broken)

Tested 25 Oct: **zenoh does NOT work Humble <-> Jazzy** (the Humble laptop sees the Pi's topic names but
receives no data: rmw_zenoh 0.1 vs 0.2 key formats). CycloneDDS does work across the two distros, but DDS
cannot cross the Windows hotspot. So the Humble fallback needs a normal Wi-Fi router that both the laptop
and the Pi join (no internet needed):
1. Pi: add the router to `/etc/netplan/50-cloud-init.yaml` (next to the hotspot entry), `sudo netplan apply`;
   note the Pi's new IP.
2. Pi drivers on DDS: `ssh hardleel@<pi_ip> 'RMW=cyclone ./robot_hw.sh'`.
3. `.wslconfig`: `networkingMode=mirrored`, then `wsl --shutdown` (WSL then sees the router network).
4. Humble distro: `wsl -d Ubuntu-22.04`, then go to the worktree:
   `cd "/mnt/c/Users/Aljamea Nairobi/Documents/hardleel-pi-repo/.claude/worktrees/mission-m1"`,
   `source /opt/ros/humble/setup.bash && source ~/hardleel_m1_ws/install/setup.bash`,
   `HARDLEEL_RMW=cyclone PI_IP=<pi_ip> source scripts/laptop/ros_env.sh`.
5. The same launch files work on Humble (stack_common rewrites the Jazzy-only Nav2 / collision monitor bits;
   the stop boxes become plain boxes, so in-place turns next to walls can be blocked - drive wider).
6. Never run ROS on domain 7 in both distros at once: stop everything in Jazzy first (`wsl -t Ubuntu-24.04`).
Switch back: `.wslconfig` -> `networkingMode=nat`, `wsl --shutdown`, Pi `./robot_hw.sh` (zenoh), Jazzy distro.

---

## 8. New features (all OFF by default — switch on only what passed TEST_TONIGHT step 6)

| Feature | How to switch it on |
|---|---|
| Gyro yaw-rate feedback (Pi) | `~/robot_hw.sh yaw_feedback:=true` (or live: `ros2 param set /motor_bridge yaw_feedback true`) |
| Speed feedback from the Cartographer pose | `laptop_mission.launch.py ... speed_feedback:=true` |
| Turn calibration at the start of the run | `laptop_mission.launch.py ... turn_calibration:=true` |
| Autonomous mapping (frontier, stops < 165 s) | `laptop_mapping.launch.py mapping_mode:=auto` [+ `auto_calibrate:=true`], then `laptop_mission.launch.py ... nav2:=false` |
| Motor deadband compensation | `ros2 run motor_teleop deadband_calibrate` (wheels in the air), then copy `~/hardleel_ws/motor_calib.yaml` to the Pi (TEST_TONIGHT 1h). Values 0 = off. |
| Speed fit from a mapping bag | `ros2 run motor_teleop fit_speed_from_bag ~/hardleel_bags/<mapping_...> --apply` |
Commanded PWM is never used as odometry: Cartographer runs on lidar + IMU only.
Ramps: their sides are invisible to the 15 cm lidar plane, so place waypoints so the robot meets a ramp end-on.
