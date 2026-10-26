# DECISIONS

- **2026-10-26 Work branch `feat/sim`** (from `feat/final-stack`): nothing pushed to main.
- **pixi.toml, osx-arm64 only**: channels robostack-jazzy + conda-forge; Windows/WSL apt setup untouched.
  Alternatives: conda/mamba directly (pixi is what RoboStack recommends, gives a committed lockfile).
- **Env vars** in pixi activation: CycloneDDS, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `ROS_DOMAIN_ID=42`
  (matches the repo's sim-domain rule).
- **Did not run `pixi install`**: would breach the 20 GB free-disk floor (see BLOCKERS.md B1). Chose to respect the
  safety rule over progress; alternative was deleting other projects' data, which is forbidden.
- Installed via Homebrew: pixi, tmux (allowed).
- **2026-10-26 floor lifted by user** ("dont mind the storage floor"): ran `pixi install` (.pixi 5.7 GB).
- Added `ros-jazzy-opennav-docking(-bt)` (Jazzy name of nav2_docking) and pinned `setuptools<80` (colcon --symlink-install breaks on newer).
- sim.launch.py: Harmonic plugin-name rewrite (ignition-gazebo-* -> gz-sim-*), world rewritten at launch; no
  `--headless-rendering` on macOS. Result: server-only `gz sim -r -s`, gpu_lidar works (360 finite ranges, 0.15-12 m).
  Ogre2 logs one plugin-path error but rendering sensors still run. **Real-time factor measured 0.99.**
- Sensor rates set to spec: lidar 6.4 Hz, IMU 93 Hz.
- **macOS Cartographer fixes** (RoboStack package is broken on osx-arm64): (1) `cartographer_node` dies "flag collect_metrics
  defined more than once" -> built cartographer_ros 2.0.9003 from source into the gitignored overlay `mac_ws/` with
  `scripts/sim/mac/cartographer_ros_macos.patch` (offline_node moved to its own exe, GUARDED_BY->ABSL_*, glog call),
  + rpath/libpython link flags (`scripts/sim/mac/build_cartographer_overlay.sh`). (2) the packaged Lua 5.4 does not export
  globals from `include "x.lua"` (stock backpack_2d.lua fails too) -> `launch/carto_lua.py` inlines includes at launch
  on macOS only; repo configs unchanged. Alternative (rebuild libcartographer with Lua 5.3) was far longer.
- Demo scripts live in `scripts/sim/` (lib.sh, mapping_demo.sh, accuracy_report.py), domain 42, localhost discovery.
- Mapping demo used `rdj2026_sim/config/cartographer_sim.lua` (same as the robot's cartographer_2d.lua; sim has no EKF).
