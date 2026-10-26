# Pi backup, 26 Oct 2026 (20:30)

Snapshot of the robot's Raspberry Pi 4 (`hardleel`, Ubuntu 24.04, ROS 2 Jazzy), copied over SSH.

**Included**
- `ros2_ws/src/*` - the packages as they ran on the Pi (motor_teleop, robot_slam, rf2o_laser_odometry, sllidar_ros2, waste_classifier), files under 3 MB.
- Home-directory scripts: `robot_hw.sh`, `stop_hw.sh`, `map_track.sh`, lidar helpers (`lidar_reset.py`, `lidar_probe.py`, `lidar_stream.py`, `a1_lidar_node.py`), `hw_*` test scripts, `cyclonedds.xml`, setup/update scripts.
- `launch_backup_2026-10-15/`, `backup_pre_terrain_2026-10-22/`, `hw_logs/`, `hwtest_logs/`, `motor_bridge*.log`.
- `maps/` files under 2 MB (`.pgm`, `.yaml`); `pi_cam/`, `camera/` scripts.
- `/boot/firmware/config.txt` and `cmdline.txt`.

**Left out on purpose**
- ROS build output (`build/`, `install/`, `log/`), nested `.git` folders of third-party packages, `__pycache__`.
- Camera dumps (`*.ppm`, `test_photos`, `waste_files` data beyond small files), old `deploy_backup_*.tgz` archives.
- Map files over 2 MB (large `.pbstream`) - still on the Pi in `~/maps`; copy them with `backup_run.ps1` if needed.
- Credentials: the Pi's ssh keys, WiFi settings and passwords are not copied. A grep for password/ssid/key/token found nothing in this folder.

The repo's own `ros2_ws/src` stays the source of truth; this folder is a reference copy of what was actually on the robot.
