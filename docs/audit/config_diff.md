| Parameter / Setting | Current `cartographer_2d.lua` on Pi | Target in Engineering Plan | Status / Deviation |
|---|---|---|---|
| `use_online_correlative_scan_matching` | `true` | `true` | MATCH |
| `real_time_correlative_scan_matcher.linear_search_window` | `0.15` | `0.20` | DEVIATION (`0.15` m current vs `0.20` m target) |
| `real_time_correlative_scan_matcher.angular_search_window` | `math.rad(25.)` | Not specified | Current has 25° search window |
| `real_time_correlative_scan_matcher.translation_delta_cost_weight` | `1e-1` | Not specified | Current is `0.1` |
| `real_time_correlative_scan_matcher.rotation_delta_cost_weight` | `1e-1` | Not specified | Current is `0.1` |
| `ceres_scan_matcher.occupied_space_weight` | `20.0` | `5.0` | DEVIATION (`20.0` current vs `5.0` target) |
| `ceres_scan_matcher.translation_weight` | `0.5` | `3.0` | DEVIATION (`0.5` current vs `3.0` target) |
| `ceres_scan_matcher.rotation_weight` | `15.0` | `40.0` | DEVIATION (`15.0` current vs `40.0` target) |
| `submaps.num_range_data` | `35` | `30` | DEVIATION (`35` current vs `30` target) |
| `submaps.range_data_inserter...hit_probability` | `0.58` | `0.65` | DEVIATION (`0.58` current vs `0.65` target) |
| `submaps.range_data_inserter...miss_probability` | `0.47` | Unspecified | Current is `0.47` |
| `POSE_GRAPH.optimize_every_n_nodes` | `20` | `35` | DEVIATION (`20` current vs `35` target) |
| `POSE_GRAPH.constraint_builder.min_score` | `0.62` | `0.62` | MATCH |
| `POSE_GRAPH.constraint_builder.global_localization_min_score` | `0.66` | Unspecified | Current is `0.66` |
| `POSE_GRAPH.constraint_builder.max_constraint_distance` | Not set (uses default) | `4.0` | DEVIATION (Missing explicit limit `4.0` m) |
| `POSE_GRAPH.optimization_problem.huber_scale` | `5e1` (50.0) | `1e2` (100.0) | DEVIATION (`50.0` current vs `100.0` target) |
| `provide_odom_frame` | `true` | `true` | MATCH |
| `publish_frame_projected_to_2d` | `true` | `true` | MATCH |
| `tracking_frame` | `"imu_link"` | `"imu_link"` | MATCH |
| `publish_tracked_pose` | Not set (defaults to `false` in standard options) | `true` | DEVIATION (Missing explicit `publish_tracked_pose = true`) |
| `min_range` | `0.18` | Unspecified | Current is `0.18` m |
| `max_range` | `5.50` | Unspecified | Current is `5.50` m (matches 5.44 m arena diagonal) |
| `imu_gravity_time_constant` | `20.0` | Unspecified | Current is `20.0` s (damps gravel acceleration) |
| `MAP_BUILDER.num_background_threads` | `3` | Unspecified | Current is `3` threads |

---

### Launch & Topic Remappings Summary (Q23 - Q26)

* **Cartographer Launch File:** `~/ros2_ws/src/robot_slam/launch/carto_mapping.launch.py`
  * **Input Remappings:**
    * `('scan', '/scan_filtered')` - fed by `hardleel_guardian_node`
    * `('imu', '/imu/data')` - fed by `imu_filter_madgwick_node`
  * **Submap/Grid Node:** `cartographer_occupancy_grid_node` launched with `-resolution 0.03` and `-publish_period_sec 0.5`.
* **Static Transforms (Q24):**
  * `base_link -> laser`: `x=0.0, y=0.0, z=0.15, yaw=0.0, pitch=0.0, roll=0.0` broadcast by `static_transform_publisher` in `carto_mapping.launch.py` and `slam.launch.py`.
  * `base_link -> imu_link`: `x=0.0, y=0.0, z=0.05, yaw=0.0, pitch=0.0, roll=0.0` broadcast by `static_transform_publisher` in `imu.launch.py`.
* **Nav2 / Costmap / Teleop Config (Q25):**
  * `~/ros2_ws/src/robot_slam/config/nav2_params.yaml` configures DWB controller (`max_vel_x: 0.26`, `robot_radius: 0.22`), costmap with `ObstacleLayer` subscribed to `/scan`, `InflationLayer`, and NavfnPlanner.
  * `motor_teleop`: `motor_bridge.py` enforces ramping (`ramp_accel: 600 PWM/s`, `ramp_decel: 1500 PWM/s`, `min_pwm: 0`, `min_turn: 0.0`).
* **Topic Collisions & Remappings (Q26):**
  * Raw sensor drivers publish on `/scan` (`sllidar_node`) and `/imu/data_raw` (`imu_serial_node`).
  * `hardleel_guardian_node` consumes `/scan` and `/imu/data`, publishing `/scan_filtered` and safety-clamped `/cmd_vel`.
  * Cartographer consumes `/scan_filtered` and `/imu/data`.
  * **Conflict Risk Identified:** Nav2 costmap in `nav2_params.yaml` points to `/scan` instead of `/scan_filtered`. If Nav2 is run simultaneously without remapping, floor strikes on ramps will inject obstacles directly into the Nav2 costmap.
