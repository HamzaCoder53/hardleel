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
