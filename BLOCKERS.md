# BLOCKERS

## B1 - RESOLVED 2026-10-26: user lifted the 20 GB floor; env installed (5.7 GB .pixi). Original note:
- Task safety rule: keep >= 20 GB free. At session start the Mac had **19 GiB free (92% used)**, already below it.
- `pixi lock` succeeds (osx-arm64, 1928 packages, ~1.2 GB download, est. 4-6 GB unpacked incl. Gazebo Harmonic,
  Nav2, Cartographer, PyTorch). `pixi install` would leave ~13-15 GiB free -> breaches the floor, so it was not run.
- Reclaimable inside the allowed scope (Homebrew cache) is only ~0.3 GB; freeing real space means touching personal
  files / other projects (~/waqaf_colmap, ~/rc_car_vision, ...), which is out of bounds.
- **To unblock:** free >= ~11 GB elsewhere (or confirm the floor may drop to ~14 GB), then
  `cd ~/hardleel && pixi install`. Everything in sections 1-2, 5 and 7 that needs ROS/Gazebo waits on this.
- Best next idea once unblocked: `pixi run colcon build`, then try Gazebo server-only with `--render-engine ogre`
  for the lidar (gpu_lidar on macOS/Metal is the likely trouble spot; fall back to CPU `lidar` sensor if needed).

## B2 - Existing sim is Humble + Gazebo Fortress (`rdj2026_sim`), target is Jazzy + Harmonic
- Porting needs `ros_gz` Harmonic world/bridge changes (`gz sim` / `gz-sim8` names, bridge yaml). Not testable without B1.

## B3 - ESP32 firmware
- Not started; will be written with marked placeholder pins and left untested (no hardware), per the task.

## B4 - sllidar_ros2 does not build on macOS (Linux-only pthread calls)
- Real-hardware lidar driver; not needed in sim. Excluded locally with an untracked COLCON_IGNORE (in .git/info/exclude).
- `colcon test`: robot_slam / motor_teleop have no tests (exit 5 = "no tests collected"), not real failures.
