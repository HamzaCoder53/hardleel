# Handoff — paste this into a new Claude chat

I'm building **"hardleel"**, our robot for **Robotics Dojo 2026** (JKUAT, waste-management theme; team
Aljamea-Tus-Saifiyah). Mini Competition **26 Oct 2026**, final **25 Nov 2026**. Repo:
**github.com/HamzaCoder53/hardleel** (local copy `Documents\hardleel-pi-repo`). Task list: **`TODO.md`**.

## Robot
- Raspberry Pi 4 (4 GB), Ubuntu 24.04, ROS 2 Jazzy. SSH `hardleel@hardleel.local` (IP changes by network).
- RPLIDAR A1 (`/dev/ttyUSB0`, mounted yaw 0), IMU = Arduino Nano 33 BLE (`robot_slam/imu_serial_node`),
  motors = Pico running TB6612 firmware driven by `motor_teleop/motor_bridge` (`/cmd_vel`),
  Pi Camera Module 3 (works, 30 fps, but points at the ceiling).
- **No wheel encoders** (not possible right now). New wheels/motors still to be fitted.
- Pi overheats under Nav2 + camera (83.8 °C) → needs a fan.

## Software that works
- Mapping: `~/map_track.sh <name>` on the Pi (Cartographer + IMU, autosave every ~13 s, keyboard teleop in tmux).
- Waste classifier: `ros2 launch waste_classifier classify.launch.py`, service `/classify` →
  `class:conf:colour`; answers "unknown" when unsure.
- Simulation (PC, WSL Ubuntu-22.04, ROS 2 Humble + Gazebo Fortress, `~/rdj_ws`):
  `ros2 launch rdj2026_sim sim.launch.py`, `mapping.launch.py`, `nav.launch.py`.
  Course = exact copy of rulebook **Figure 1** top view (verified by overlay). `noisy_odom:=true` mimics no encoders.

## Findings
- Last real mapping run: heading drifted ~35° (doubled walls) and the robot hit the walls. Gyro: raw bias −12.4 °/s,
  1.3 °/min after calibration when still → suspect gyro scale error + 0.3 °/s deadband.
- Sim test with drifting odometry: **Cartographer localization ~5 cm / ~1°, AMCL lost** → use Cartographer localization.
- Rules: no external communication/processing in the autonomous phase. Mapping phase: rulebook says teams
  *manually* drive, but objective 1 says "autonomously scan" → ask organisers.

## Next up
Gyro scale calibration · self-driving or assisted mapping with wall avoidance · ramp handling (pause map
updates when tilted, slow down, damp the LiDAR) · mission behaviour tree + timing logger · FS-i6 RC with
e-stop · CSV data logger · per-floor speed profiles · servos (Pi hardware PWM GPIO12/13 recommended).
