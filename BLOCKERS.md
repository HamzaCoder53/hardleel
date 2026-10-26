# BLOCKERS

Items that need you, the real robot, or a decision. Resolved items are kept at the bottom.

## Open
- **ESP32 firmware:** `firmware/esp32_pi_bridge/` already exists from an earlier session (untested on hardware). I added nothing new. Needs: board, pin wiring, LiPo cell count (CLAUDE.md 'Hardware note').
- **Foxglove desktop app not tested.** The bridge runs and advertises the topics (checked in its log); the layout JSON
  `foxglove/hardleel_sim_layout.json` is valid JSON written from the layout schema, but I could not open the app here. If a
  panel is blank, re-add that panel by hand and re-export the layout over this file.
- **Windows laptop (WSL) not re-tested.** The launch files are shared and the Harmonic/Humble selection is by `ROS_DISTRO`,
  but I had no access to that machine. Mac-only code is behind `sys.platform == 'darwin'` or lives in `scripts/sim/mac/`.
- **Section 7 behaviours.** They were built in earlier sessions (PROGRESS.md items 1-8) and are behind switches that default OFF.
  In this session only `loc_v2` + `loc_apply_corrections` (localization recovery) and the new `gate_align_heading` were run in the
  simulator; the unit tests for all of them pass (158 passed, 1 skipped). The others (ramp dash, camera v2, map tidy-up, supervisor)
  were not re-run in the simulator in this session.
- **Wall height 0.45 m** is in `course_features.yaml` but the generated world still uses 0.40 m (no effect on the 2D lidar).
- **Standing-still relocalization is slow and variable:** 24 s and 75 s in two runs of the same test (see SIM_REPORT.md).
- **Per-side motor speed variation** is only available through the `hardware:=true` fake-Pico path, not in the plain drive.

## Resolved this session
- B1 disk floor: you lifted the 20 GB rule; environment installed (`.pixi`, 5.7 GB).
- B2 Humble/Fortress -> Jazzy/Harmonic: `sim.launch.py` rewrites plugin names at launch; headless lidar works, RTF 0.99.
- B4 `sllidar_ros2` does not build on macOS (Linux-only pthread calls): excluded locally with an untracked `COLCON_IGNORE`.
- `colcon test` reports "no tests" for robot_slam/motor_teleop: the ROS pytest plugins clash with this pytest. Use
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pixi run python -m pytest ros2_ws/src/robot_slam/test`.
