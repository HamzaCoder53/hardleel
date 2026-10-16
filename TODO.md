# hardleel — to-do list

Mini Competition: **26 Oct 2026** · Final: **25 Nov 2026**. Last updated 16 Oct 2026.

## When the new wheels & motors are fitted (hardware)
- [ ] **Gyro scale calibration** — robot flat on a table against an edge, turn it by hand exactly 4 full turns,
      compare with the IMU's integrated yaw, set the scale factor in `imu_serial_node.py`.
      Then lower the 0.3 °/s yaw deadband and re-zero the gyro bias whenever the robot is stopped.
      (Measured 16 Oct: raw yaw bias −12.4 °/s, 1.3 °/min drift after calibration when still; the map drifted
      ~35° over a 700° drive → suspect gyro scale error + deadband.)
- [ ] **Motor characterisation** (robot lifted, wheels off the ground first): PWM → speed curve per side,
      dead zone (`min_pwm`), left/right mismatch, acceleration — measured with the LiDAR as reference
      (no encoders needed). Update `motor_bridge` parameters (`invert_linear`, `min_pwm`, `turn_gain`).
- [ ] **Command-based odometry**: publish predicted speed from the motor commands (from the characterisation)
      as a low-confidence twist into the EKF.
- [ ] **Cooling**: fan or heatsink on the Pi 4 (it throttled at 83.8 °C on 15 Oct).
- [ ] **Re-aim the camera** forward/level with the display screen (it points at the ceiling).
- [ ] Optional: optical-flow floor sensor (PAA5100JE) as an extra odometry source.

## Mapping & navigation without encoders (software)
- [ ] Lidar odometry: run **rf2o + IMU in the EKF** as the `odom` frame for Nav2 (`nav.launch.py` already has rf2o).
- [ ] Use **Cartographer pure localization** instead of AMCL (sim test 16 Oct with drifting odometry:
      Cartographer ~5 cm / ~1°, AMCL lost at 0.7–1.3 m).
- [ ] Re-run the sim localization comparison with a fixed route (old route clipped the loading table)
      and a proper kidnap test.
- [ ] Tune Cartographer for no odometry (scan-matcher search windows); try a denser LiDAR scan mode;
      map slowly with gentle turns.
- [ ] `map_track.sh`: also record a ROS bag (`/scan`, `/imu/data_raw`) every run so maps can be rebuilt offline.
- [ ] `lidar_reset.py`: exit with an error code when the LiDAR doesn't reply.

## Virtual arena (rdj2026_sim)
- [ ] Fix the real-scan scoring test (fails its positive control) before trusting any layout score.
- [ ] Review Gemini 4's two layouts (photos-only and all-sources) and score them once the test works.
- [ ] Update `arena_layout.yaml` from tape measurements or photos of the finished field
      (corridor widths, ramp / grass / doorway positions, gate, start, screen, bay colours).

## Competition tasks (software)
- [ ] **Mission node** (state machine): screen → detect → loading → ramps → obstacles → gate decision
      (wait vs reroute) → deposit → finish, using Nav2; test end-to-end in the sim.
- [ ] **Timing logger** for the rulebook §8 timestamps; send to the judges once the protocol is published.
- [ ] **Waste classifier retraining** (on the Mac): add an "empty" class, organic and fabric examples.
      Collect with `ros2 launch waste_classifier classify.launch.py save_dir:=/home/hardleel/captures`.
- [ ] **Bay colour detection** (blue / white / black) with the camera.
- [ ] Practise **Virtual SLAM** in the sim (map → save → navigate).

## Ask the organisers (roboticsdojo@jkuat.ac.ke)
- [ ] Which class does **fabric** belong to?
- [ ] Is **off-board (laptop) processing** allowed during the mapping phase?
- [ ] The gate's "standard waiting time", and the judges' data/communication protocol.

## Done
- [x] Pi backup on GitHub; LiDAR mounting yaw fixed; Cartographer config installed; `map_track.sh` with autosave.
- [x] Virtual arena (Gazebo Fortress + ROS 2 Humble in WSL), updated from field photos; 2D plan.
- [x] Waste classifier: rejects unsure answers ("unknown"), reports bay colour, launch file fixed.
- [x] Sim tools: drifting-odometry mode, ground truth, route driver, localization error logger.
