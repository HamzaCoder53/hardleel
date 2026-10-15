# hardleel — Raspberry Pi robot backup

Backup of the home folder (`/home/hardleel`) from the Raspberry Pi **hardleel**, taken 2026-10-15.

- **Hardware:** Raspberry Pi 4 Model B (4 GB)
- **OS:** Ubuntu 24.04.5 LTS (kernel 6.8.0-raspi)
- **ROS:** ROS 2 Jazzy (`/opt/ros/jazzy`, not included — reinstall via apt)

## Contents

| Path | What it is |
|---|---|
| `ros2_ws/src/` | ROS 2 workspace sources (incl. `waste_classifier`) |
| `pi_cam/` | Camera node (`pi_cam_node.py`) |
| `waste_files/` | Waste classifier service + packaging script |
| `start_slam.sh`, `lidar_*.py`, `dir_check.py` | Helper scripts |
| `maps/` | Saved SLAM map (`my_map.*`) |
| `test_photos/`, `frame-*.ppm` | Camera captures |
| `frames_*.pdf/.gv` | TF frame tree exports |
| `motor_teleop.zip`, `robot_slam.zip` | Packaged projects |
| `cyclonedds.xml` | DDS config |
| `installed-packages.txt` | apt / snap / pip package lists for rebuilding the system |

### Third-party packages (vendored, unmodified at backup time)

- `ros2_ws/src/sllidar_ros2` — https://github.com/Slamtec/sllidar_ros2 @ `3430009`
- `ros2_ws/src/rf2o_laser_odometry` — https://github.com/MAPIRlab/rf2o_laser_odometry @ `b38c68e`

## Not included

SSH keys, shell history, `/etc` (contains Wi-Fi and password data), installed Python
packages in `~/.local/lib`, and ROS build output (`build/`, `install/`, `log/`).

## Restore

```bash
git clone <this repo> ~/restore && rsync -a ~/restore/ ~/
cd ~/ros2_ws && rosdep install --from-paths src -y --ignore-src && colcon build
```
