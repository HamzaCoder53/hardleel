# SIM_REPORT - Virtual SLAM simulation on the Mac mini (26 Oct)

Status: working end to end. Numbers marked (2-wheel) were measured before the model became a 4-wheel skid-steer; the course mapping/nav and the ramp mission were re-run on the 4-wheel model (marked 4-wheel). Run `pixi run python scripts/sim/summarize.py` for tables from the result files in `docs/sim_demo/`.

## What works
- Gazebo Harmonic server-only on macOS, lidar (gpu_lidar, 360 pts, 6.4 Hz), IMU, 640x480 camera (10 Hz), 4-wheel skid steer. Real-time factor 0.99 (also about 1.0 with the full stack running).
- Real launch files unchanged: `laptop_mapping` / `laptop_localization` / `laptop_mission` with `sim:=true`.
- Course world + maze world, ramps (dirt/gravel), docks, screen with real waste photos (camera sees them), bins, timed gate, workers.

## Numbers
| test | result |
|---|---|
| Course map vs ground truth (4-wheel) | wall error mean 0.6 cm, p95 5 cm, 99.6 % of true walls covered, free space 5.65 x 2.90 m (truth map itself 5.65 x 2.90; rulebook 5.75 x 3.05) |
| Maze map (2-wheel) | mean 5.1 cm raw / 2.7 cm after rigid fit, p95 15 / 10 cm, 90 % coverage |
| Nav2 goals, course (4-wheel) | 8/8, localization error at goal 2.5-4 cm |
| Nav2 goals, maze (2-wheel) | 6/6, localization error 3-14 cm |
| Teleport 5 m, default stack (2-wheel, clean run) | does not recover standing still in 90 s; recovers in about 25 s once driving |
| Teleport 5 m, loc_v2 + loc_apply_corrections | recovers standing still in 24 s and 75 s (two runs), all goals reached |
| Mission rehearsal (2-wheel, flat floor) | gate open -> path2 111 s; closed -> path1 126 s; timed (opens 2 s) -> path2 113 s; open+worker -> path2 131 s; closed+worker -> path1 150 s |
| Mission with dirt+gravel ramps, docks, bins (4-wheel) | finished in 121 s, climbs the ramp (terrain_4 ok), gate open -> path2 |

## Findings fixed on the way (also affect the real stack)
1. `ekf_nav_v2.yaml` was keyed `ekf_filter_node` but the node is `ekf_loc`: the v2 EKF ignored its config (fixed with `/**`).
2. Nav2 accepts final yaw within 0.4 rad but the gate sector is +-10 deg: an unlucky arrival made an open gate read closed. New switch `gate_align_heading` (default OFF) turns to the gate heading first (with lag lead).
3. macOS: RoboStack cartographer_node is broken (patched build in `mac_ws/`), Lua `include` inlined at launch, CycloneDDS participant limit, libpython link flags. See DECISIONS.md.

## Demo commands
See RUNBOOK.md ("Virtual SLAM simulation"). One command per step plus `scripts/sim/rehearsal_all.sh`, `scripts/sim/regression_all.sh`, `scripts/sim/final_matrix.sh`. Overlays: `docs/sim_demo/*_overlay.png`; camera proof: `docs/sim_demo/camera_view_screen_plastic.png`.

## Waiting on real measurements / open items
See SIM_MEASUREMENTS_NEEDED.md and BLOCKERS.md.
