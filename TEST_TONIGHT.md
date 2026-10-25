# TEST_TONIGHT — office test before the mini-competition (26 Oct)

Order: safest to riskiest. Each step: what to run, what you should see, what counts as a FAIL.
**REQUIRED** = must pass for tomorrow's run. **OPTIONAL** = a new feature; it stays OFF tomorrow unless it passes.
Time: required steps ~1 h 45 min, all steps ~2 h 45 min.

Terminals: **W** = WSL: `wsl -d Ubuntu-24.04`, then `cd ~/hardleel && source scripts/laptop/ros_env.sh`
(each W terminal). **P** = PowerShell (admin). The robot is **HOT** only when the Pi drivers run.
Stop everything at any time: Ctrl-C the teleop/mission terminal, or `scripts/laptop/abort.sh` in a W terminal;
the motors also stop by themselves 0.3 s after commands stop.

---

## 0. Bring-up and deploy (10 min) — REQUIRED

| Do | See | FAIL if |
|---|---|---|
| Robot on, hotspot on. In **Git Bash** (Windows reaches the Pi, WSL does not): `cd ~/Documents/hardleel-pi-repo/.claude/worktrees/mission-m1 && bash scripts/laptop/deploy_pi.sh` (asks the Pi password). Then P: `cd "$env:USERPROFILE\Documents\hardleel-pi-repo\.claude\worktrees\mission-m1"` for the next line | `DEPLOYED (backup ...)` | build error |
| P: `powershell -ExecutionPolicy Bypass -File scripts\laptop\up.ps1 -CheckOnly` (robot still for 5 s after "starting the Pi drivers") | step 5 prints `/scan` ~6 Hz, `/imu/data` ~90 Hz, camera ~5 Hz | any topic missing |
| W: `ros2 topic echo --once /imu/calib_status` | `data: OK gyro std ...` | `WARN moved` twice in a row with the robot still |

## 1. Wheels in the air — robot on a box, wheels free (20 min)

| # | Do | See | FAIL if | |
|---|---|---|---|---|
| 1a | W1: `ros2 launch robot_slam laptop_mapping.launch.py rviz:=false` · W2: `scripts/laptop/teleop.sh`, keys `i` `,` `j` `l` | `i`: all wheels forward; `,`: back; `j`: left side back + right forward (CCW); `l`: opposite | any wheel the wrong way (note which: fix `invert_*` in `motor_bridge.launch`/pi_bringup) | **REQUIRED** |
| 1b | Hold `i`, put your hand 10 cm in front of the lidar | wheels stop within ~0.3 s; `ros2 topic echo /collision_monitor_state` shows action 1 (stop) | wheels keep turning | **REQUIRED** |
| 1c | Hand still there, press `j` (turn) | wheels turn (turning in place is allowed; only driving forward is stopped) | no movement | **REQUIRED** |
| 1d | Hand at 40 cm while holding `i` | wheels slow down (slowdown zone) | full speed | OPTIONAL |
| 1e | Hold `i`, switch the Windows hotspot OFF | wheels stop within ~0.5 s; Pi log `Pico watchdog`/timeout | wheels keep turning > 1 s | **REQUIRED** |
| 1f | Hotspot back ON; P: `up.ps1 -NoPi -CheckOnly` | topics back | topics never come back (then: rerun `up.ps1`) | **REQUIRED** |
| 1g | Deadband: W2 Ctrl-C teleop, then `ros2 run motor_teleop deadband_calibrate` — press ENTER the moment a wheel starts turning; confirm with `y` | a value per side, typically 30-120; saved to `~/hardleel_ws/motor_calib.yaml` and set live | a wheel never turns up to 200 / values differ wildly between runs (> 30) | **REQUIRED to measure** |
| 1h | Copy it to the Pi so it survives a restart. P: `scp \\wsl$\Ubuntu-24.04\home\hamza\hardleel_ws\motor_calib.yaml hardleel@192.168.137.8:hardleel_ws/` (first time: `ssh hardleel@192.168.137.8 mkdir -p hardleel_ws`) | file copied | — | **REQUIRED if 1g kept** |
| 1i | W2: `scripts/laptop/teleop.sh`, press `z` until speed ~0.05, hold `i` | wheels turn slowly and evenly (before 1g they did not move at this speed) | they don't turn, or one side much faster: set both back to 0 (`ros2 param set /motor_bridge min_pwm_left 0`, same for right, delete the file on the Pi) | **REQUIRED** |

## 2. On the floor (25 min)

| # | Do | See | FAIL if | |
|---|---|---|---|---|
| 2a | Robot on the floor, W1 mapping still running (or restart it). Drive a slow loop of the office with teleop. | RViz (`rviz:=true`) or the map: walls straight and single | doubled / rotated walls | **REQUIRED** |
| 2b | W2 Ctrl-C teleop. W3: `scripts/laptop/save_map.sh office_1` | `~/hardleel_maps/office_1.pbstream/.pgm/.yaml` listed | nothing saved | **REQUIRED** |
| 2c | Turn calibration: W3 `ros2 run motor_teleop motor_calibrator`; W4 `ros2 service call /motor/calibrate_turn std_srvs/srv/Trigger {}` (1 m free around the robot) | short spins left and right (max 180 deg each); `min_turn_pwm N` saved | spins more than half a turn, or no threshold found | OPTIONAL |
| 2d | Speed calibration: robot facing a free straight lane >= 1.5 m; W4 `ros2 service call /motor/calibrate_speed std_srvs/srv/Trigger {}` | 3 short straight drives (<= 0.8 m each); `max_speed X` saved (expect 0.4-1.5 m/s) | drives into something / wild value (store keeps the last good one) | OPTIONAL |
| 2e | Manual fit from the mapping bag: `ros2 run motor_teleop fit_speed_from_bag ~/hardleel_bags/<latest mapping_...>` | table + `max_speed`, rms < 0.05 | rms > 0.1 (then ignore it) | OPTIONAL |
| 2f | If 2c/2d kept: copy the calib file to the Pi again (as 1h) | | | — |

## 3. Full office mission (30 min) — REQUIRED

| # | Do | See | FAIL if |
|---|---|---|---|
| 3a | Set the office waypoints on `office_1`: `ros2 run robot_slam click_waypoints ~/hardleel_maps/office_1.yaml --waypoints ~/hardleel_ws/waypoints/waypoints_office.yaml` (time yourself: < 2 min). Loading/bin points: robot centre ~0.3 m from the face. | file saved, table printed | > 2 min or the GUI does not open |
| 3b | Mapping (W1) still running. W3: `ros2 launch robot_slam laptop_mission.launch.py waypoints:=~/hardleel_ws/waypoints/waypoints_office.yaml classifier:=stub stub_class:=plastic` | `collision monitor: mission stop box`, `NAV2 READY`, `Waiting for /mission/start` | `NAV2 NOT ACTIVE` after the retries |
| 3c | W4: `scripts/laptop/go.sh`. Hands off. | robot visits screen, loading_plastic (waits 6 s), terrain, junction, gate_check (waits up to 15 s), path, bin_plastic (2 s), finish. Event logger table fills. | collision; stuck > 30 s without skipping; robot leaves the office area |
| 3d | During the run, step in front of the robot for ~5 s | it stops, waits, continues or re-plans; log shows `obstacle_detected` and `obstacle_response` | it touches you |
| 3e | `cat ~/logs/runs/$(ls -t ~/logs/runs | head -1)` | robot_id HARDLEEL and all 12 fields with times (obstacle ones only if 3d done) | MISSING fields other than obstacle |
| 3f | Classifier: restart W3 without `classifier:=stub` (wait for `model ready`), W4: `ros2 run waste_classifier clip_monitor_test <folder of plastic_*.jpg metal_*.jpg organic_*.jpg>` with the robot camera 0.5-1 m from the laptop screen | accuracy line + latency (~1-1.5 s) | accuracy < 70 % (tune prompts/`min_conf` in `clip_classifier.yaml`) |
| 3g | Drop bin (not mounted): `ros2 service call /dump std_srvs/srv/Trigger {}` | `no drop bin: skipped` | error |

## 4. Cold start from reboot using only RUNBOOK.md (15 min) — REQUIRED

Restart Windows and power-cycle the robot. Then follow **RUNBOOK section 0** only (no help from this file).
See: `up.ps1` reaches step 6 and opens the mapping + teleop windows; teleop drives the robot.
FAIL: any step needs a command that is not in the RUNBOOK (write it down and tell Claude).

## 5. Ramp test with a propped board (15 min) — REQUIRED

Board ~1 m long, raised to roughly the course ramp height (if unknown: 8-10 cm), approached end-on.
| Do | See | FAIL if |
|---|---|---|
| Mapping running; teleop the robot slowly up and down the board | robot climbs; `ros2 topic echo /surface_state` shows the ramp surface while tilted; map walls stay clean | robot cannot climb (chassis/caster touches, wheels slip) -> tell Claude the board height |
| Click two waypoints (before / after the board), run them as the office mission's terrain_1/terrain_2 | Nav2 drives over without a collision-monitor stop | stops at the board foot: check `/collision_monitor_state`; note the pitch |

## 6. New features, one at a time (30-40 min) — OPTIONAL; keep each ON only if it passes

Do these only after 0-5 pass. After each, switch it OFF again unless it clearly helped.
| Feature | Switch on | Pass |
|---|---|---|
| Gyro yaw feedback | `ros2 param set /motor_bridge yaw_feedback true` | in-place turns reach the commanded rate (Nav2 turns look smoother, no oscillation). Off: same command with `false`. Tomorrow ON: `~/robot_hw.sh yaw_feedback:=true` |
| Speed feedback | `laptop_mission ... speed_feedback:=true` | log `speed scale ~1.0` on the floor, >1 on carpet/board; no surging |
| Turn calibration at the start | `laptop_mission ... turn_calibration:=true` | mission starts with a short left/right spin and logs `TURN_CALIBRATION ok` (costs ~10 s of the 3 min) |
| Auto mapping | `laptop_mapping.launch.py mapping_mode:=auto` then mission with `nav2:=false` | explores the office, `exploration finished` before 165 s, no collision. (Rules: mapping is by hand unless the organisers allow autonomous mapping — ask first.) |
| Auto speed calibration | `mapping_mode:=auto auto_calibrate:=true` | a short straight drive before exploring, `max_speed` saved |

## Decide for tomorrow (write it in PROGRESS.md)
- Which waypoint draft: at the start pad facing the field, is the loading room on your LEFT
  (`waypoints_course.yaml`) or RIGHT (`waypoints_course_mirror.yaml`)? Re-click on tomorrow's map anyway.
- Deadband values kept? Which optional features passed (default: none ON)?
- Organisers: gate waiting time -> `gate_max_wait_s`; loading protocol; fabric class; robot ID; data format.
