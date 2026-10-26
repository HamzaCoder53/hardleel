# DECISIONS

- **2026-10-26 Work branch `feat/sim`** (from `feat/final-stack`): nothing pushed to main.
- **pixi.toml, osx-arm64 only**: channels robostack-jazzy + conda-forge; Windows/WSL apt setup untouched.
  Alternatives: conda/mamba directly (pixi is what RoboStack recommends, gives a committed lockfile).
- **Env vars** in pixi activation: CycloneDDS, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `ROS_DOMAIN_ID=42`
  (matches the repo's sim-domain rule).
- **Did not run `pixi install`**: would breach the 20 GB free-disk floor (see BLOCKERS.md B1). Chose to respect the
  safety rule over progress; alternative was deleting other projects' data, which is forbidden.
- Installed via Homebrew: pixi, tmux (allowed).
