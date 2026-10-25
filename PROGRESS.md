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
- [ ] Step 1: commit; user runs `wsl --export` backup; Pi apt update with kernel/firmware held; reboot; re-check topics
- [ ] Step 2: Jazzy WSL distro (time-boxed 2 h)
- [ ] M1.1 IMU calibration motion check
- [ ] M1.2 laptop_mapping / laptop_mission / localization fallback (Cartographer continuity)
- [ ] M1.3 waypoint files + save_pose tool
- [ ] M1.4 CLIP classifier + monitor test
- [ ] M1.5 gate check (median range in sector)
- [ ] M1.6 mission node adaptations
- [ ] M1.7 event logger
- [ ] M1.8 obstacle wait-then-replan + logging
- [ ] M1.9 pi_bringup, RUNBOOK.md
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

## Open questions for the user
- Robot ID; camera rotation/cable status; saved course scans (path, format, start spot?); servo pins.

## Next
Step 1.
