# CLAUDE.md — Hardleel robot (Robotics Dojo 2026)

Repo `HamzaCoder53/hardleel`. Work branch: `feat/final-stack`. Several computers share this repo, so the
repo is the single source of truth for state.

## Session start
1. `git pull` on the work branch.
2. Read `HANDOFF.md` (wins if anything differs), then `PROGRESS.md`, then `TODO.md`. Read `RUNBOOK.md` and
   `TEST_*.md` when touching real-robot procedures.

## While working
- After each working step: update `PROGRESS.md`, commit, push to the work branch.
- Before stopping: everything committed and pushed.
- NEVER commit or push to `main`. NEVER commit raw bags, video or other large data (code, configs, maps, logs and
  the run index only; data goes to Drive/external).
- Every new behaviour sits behind a config switch that is OFF by default until it passes on the real robot.
- Never send motion commands to the real robot unless the user says they are with it. Read-only checks are fine.
- Test in the sim first. Unknowns go in config, not code. Ask before big installs and about anything that depends
  on hardware or rules you cannot see.
- No commanded PWM in localization.

## Hardware note
The Pico + TB6612 burned out. Motors are now driven by an ESP32 with two BTS7960 (IBT-2) drivers on a 2200 mAh
LiPo; "Pico" in older code/docs now means the ESP32. Ask the user for the board, pin wiring and LiPo cell count
before any firmware work. Safe power-up (motors off until a valid command) and a command timeout are mandatory.

## Environment
- ROS 2 Jazzy, Ubuntu 24.04 (WSL2 on Windows), `ROS_DOMAIN_ID=7`, `rmw_zenoh_cpp`
  (`HARDLEEL_RMW=cyclone` = CycloneDDS fallback). `source scripts/laptop/ros_env.sh` for the robot side.
- Workspace: `~/hardleel_ws` (`src` -> `~/hardleel/ros2_ws/src`); Python venv `~/hardleel-venv`.
- Never `pkill -f "ros2 launch"`; sim runs use `ROS_DOMAIN_ID=42` (`scripts/laptop/kill_domain.sh`).
