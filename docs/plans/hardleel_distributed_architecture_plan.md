# Distributed Architecture Plan: Raspberry Pi (Jazzy) Hardware / WSL Laptop (Humble) Brain

## Goal Description

Robot `hardleel` is competing in the Robotics Dojo 2026 (RDJ 2026) autonomous waste collection competition. The Raspberry Pi 4 onboard computer was observed throttling due to CPU overload and undervoltage when running the entire navigation, SLAM, and vision stack locally. 

This plan defines the distributed hardware/compute architecture:
1. **Raspberry Pi 4 (Ubuntu 24.04, ROS 2 Jazzy)**: Runs **ONLY** hardware drivers:
   - `sllidar_node` (RPLIDAR A1 serial driver -> `/scan`)
   - `imu_serial_node` + `imu_filter_madgwick` (Arduino Nano 33 BLE Sense -> `/imu/data_raw`, `/imu/data`)
   - `pi_cam_node` (Raspberry Pi Camera -> `/camera/image_raw/compressed`)
   - `motor_bridge` (Pico USB-serial driver -> `/cmd_vel` -> Pico dual TB6612 H-bridge)
2. **Windows Laptop (WSL Ubuntu 22.04, ROS 2 Humble)**: Runs **everything else** over Wi-Fi (`ROS_DOMAIN_ID=7`, CycloneDDS):
   - `sensor_prep` (tilt gating and bias capture)
   - Cartographer (2D mapping and pure localization)
   - Odometry (`rf2o_laser_odometry` + `robot_localization` EKF)
   - Nav2 (planners, controllers, costmaps, behaviors, lifecycle managers)
   - Safety chain (`twist_mux` -> `velocity_smoother` -> `collision_monitor`)
   - Autonomy & Perception (`competition_mission` / `coverage_mission`, `waste_classifier`)
   - Visualization (`rviz2`)

```mermaid
flowchart TB
    subgraph RPi["Raspberry Pi 4 (Ubuntu 24.04, ROS 2 Jazzy) - HW Drivers Only"]
        direction TB
        LIDAR_HW["RPLIDAR A1\n(/dev/ttyUSB0)"] --> SLLIDAR["sllidar_node"]
        IMU_HW["Arduino Nano 33 BLE\n(/dev/ttyACM1)"] --> IMU_NODE["imu_serial_node"]
        CAM_HW["Pi Camera Module\n(CSI/GStreamer)"] --> PICAM["pi_cam_node"]
        PICO_HW["RP2040 Pico Motors\n(/dev/pico)"] <--> M_BRIDGE["motor_bridge"]

        IMU_NODE -- "/imu/data_raw" --> MADGWICK["imu_filter_madgwick_node"]
        MADGWICK -- "/imu/data (local)" --> M_BRIDGE
    end

    subgraph WiFi["Wi-Fi Network (ROS_DOMAIN_ID=7, CycloneDDS)"]
        SLLIDAR -. "/scan (LaserScan) ."-> PREP
        IMU_NODE -. "/imu/data_raw (Imu) ."-> EKF
        MADGWICK -. "/imu/data (Imu) ."-> PREP
        PICAM -. "/camera/image_raw/compressed ."-> CLASSIFIER
        M_BRIDGE -. "/encoders, /surface_state ."-> LAPTOP_NODES
        COLLISION_MON -. "/cmd_vel (Twist) ."-> M_BRIDGE
        MISSION -. "/surface_override (String) ."-> M_BRIDGE
    end

    subgraph Laptop["Laptop WSL2 (Ubuntu 22.04, ROS 2 Humble) - Compute Brain"]
        direction TB
        subgraph StaticTF["Static TF Publisher (Owned by Laptop)"]
            TF_PUB["static_transform_publisher\nbase_link -> laser\nbase_link -> imu_link\nbase_link -> camera"]
        end

        PREP["sensor_prep"]
        PREP -- "/scan_filtered" --> CARTO["Cartographer 2D\n(Mapping / Pure Loc)"]
        PREP -- "/scan_filtered" --> RF2O["rf2o_laser_odometry"]
        PREP -- "/scan_filtered" --> COSTMAPS["Nav2 Costmaps"]
        PREP -- "/scan_filtered" --> COLLISION_MON["collision_monitor"]
        PREP -- "/imu/data_lpf" --> EKF["robot_localization EKF"]

        RF2O -- "/odom_rf2o" --> EKF
        EKF -- "/odom (no tf)" --> NAV2["Nav2 Stack"]
        CARTO -- "map -> odom -> base_link" --> TF_TREE["TF Tree"]

        TELEOP["teleop_twist_keyboard"] -- "/cmd_vel_teleop" --> TMUX["twist_mux"]
        NAV2 -- "/cmd_vel_nav" --> TMUX
        TMUX -- "/cmd_vel_mux" --> VSMOOTH["velocity_smoother"]
        VSMOOTH -- "/cmd_vel_smoothed" --> COLLISION_MON

        MISSION["competition_mission / coverage_mission"] <--> NAV2
        CLASSIFIER["waste_classifier_service"] <--> MISSION
        RVIZ["rviz2"]
    end

    classDef rpi fill:#3d0c11,stroke:#d90429,color:#fff;
    classDef laptop fill:#0b2545,stroke:#134074,color:#fff;
    classDef wifi fill:#1b4332,stroke:#40916c,color:#fff;
    class RPi rpi;
    class Laptop laptop;
    class WiFi wifi;
```

---

## User Review Required

> [!IMPORTANT]
> **1. WSL2 Networking Mode Requirement:**  
> By default, WSL2 uses NAT networking. Incoming UDP DDS packets from the Raspberry Pi over Wi-Fi **cannot reach WSL2 ROS 2 nodes** unless WSL2 is configured in **mirrored networking mode**. You must ensure `C:\Users\<User>\.wslconfig` contains:
> ```ini
> [wsl2]
> networkingMode=mirrored
> ```
> *(Requires Windows 11 22H2+).* If using standard NAT on Windows 10/11, CycloneDDS unicast port forwarding or a bridged adapter is required.

> [!WARNING]
> **2. Nav2 Version Incompatibility (Jazzy 1.3 vs Humble 1.1):**  
> `nav2_params.yaml`, `nav2_params_mission.yaml`, and `navigation_slow_cpu_launch.py` in the repository currently contain syntax and packages exclusive to ROS 2 Jazzy (`nav2_route::RouteServer`, `opennav_docking::DockingServer`, `nav2_loopback_sim`, modular `navigators:` list, and `stamp_smoothed_velocity_with_smoothing_time`). Running these on Humble will cause fatal startup crashes. They must be replaced with Humble 1.1 equivalents (`plugin_lib_names`, `default_nav_to_pose_bt_xml`, and stripping missing packages).

> [!IMPORTANT]
> **3. Motor Bridge Pitch IMU Source:**  
> Currently `motor_bridge.py` defaults to subscribing to `/imu/data_lpf` (produced by `sensor_prep`). In this distributed architecture, `sensor_prep` runs on the Laptop. If Wi-Fi jitters > 500 ms, `motor_bridge` detects a stale IMU (`imu_timeout: 0.5s`) and falls back to `FLAT`. We configure `motor_bridge` on the Pi to subscribe directly to local `/imu/data` (from `imu_filter_madgwick` running locally on the Pi), making motor ramp detection and pitch feedforward **100% resilient to Wi-Fi drops**.

---

## Deliverable 1: Precise List of Files, Lines, and Changes for the Split

### Summary of Node Distribution

| Node Name | Package | Source Host / OS | Destination Host / OS | Topic Inputs | Topic Outputs |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `sllidar_node` | `sllidar_ros2` | Pi (Jazzy) | **Pi (Jazzy)** | USB `/dev/ttyUSB0` | `/scan` |
| `imu_serial_node` | `robot_slam` | Pi (Jazzy) | **Pi (Jazzy)** | USB `/dev/ttyACM1` | `/imu/data_raw` |
| `imu_filter_madgwick_node` | `imu_filter_madgwick` | Pi (Jazzy) | **Pi (Jazzy)** | `/imu/data_raw` | `/imu/data` |
| `pi_cam_node` | `pi_cam` | Pi (Jazzy) | **Pi (Jazzy)** | CSI / GStreamer | `/camera/image_raw/compressed` |
| `motor_bridge` | `motor_teleop` | Pi (Jazzy) | **Pi (Jazzy)** | `/cmd_vel`, `/imu/data` | USB `/dev/pico`, `/encoders`, `/surface_state` |
| `static_transform_publisher` | `tf2_ros` | Pi / Laptop | **Laptop (Humble)** | Launch arguments | `/tf_static` (`base_link` -> `laser`, `imu_link`, `camera`) |
| `sensor_prep` | `robot_slam` | Pi (Jazzy) | **Laptop (Humble)** | `/scan`, `/imu/data`, `/imu/data_raw` | `/scan_filtered`, `/imu/data_lpf` |
| `cartographer_node` | `cartographer_ros` | Pi (Jazzy) | **Laptop (Humble)** | `/scan_filtered`, `/imu/data` | `/tf` (`map` -> `odom` -> `base_link`), Submaps |
| `occupancy_grid_node` | `cartographer_ros` | Pi (Jazzy) | **Laptop (Humble)** | Cartographer submaps | `/map` |
| `rf2o_laser_odometry` | `rf2o_laser_odometry` | Pi (Jazzy) | **Laptop (Humble)** | `/scan_filtered` | `/odom_rf2o` (`publish_tf: false`) |
| `ekf_filter_node` | `robot_localization` | Pi (Jazzy) | **Laptop (Humble)** | `/odom_rf2o`, `/imu/data_lpf` | `/odom` (`publish_tf: false`) |
| `twist_mux` | `twist_mux` | Pi (Jazzy) | **Laptop (Humble)** | `/cmd_vel_teleop`, `/cmd_vel_nav` | `/cmd_vel_mux` |
| `velocity_smoother` | `nav2_velocity_smoother`| Pi (Jazzy) | **Laptop (Humble)** | `/cmd_vel_mux` | `/cmd_vel_smoothed` |
| `collision_monitor` | `nav2_collision_monitor`| Pi (Jazzy) | **Laptop (Humble)** | `/cmd_vel_smoothed`, `/scan_filtered` | `/cmd_vel` |
| `nav2_controller` | `nav2_controller` | Pi (Jazzy) | **Laptop (Humble)** | `/odom`, `/scan_filtered`, `/plan` | `/cmd_vel_nav` |
| `nav2_planner` | `nav2_planner` | Pi (Jazzy) | **Laptop (Humble)** | `/map`, `/odom`, `/scan_filtered` | `/plan` |
| `nav2_behaviors` | `nav2_behaviors` | Pi (Jazzy) | **Laptop (Humble)** | `/odom`, Costmaps | `/cmd_vel_nav` |
| `nav2_bt_navigator` | `nav2_bt_navigator` | Pi (Jazzy) | **Laptop (Humble)** | Action goals | Behavior Tree execution |
| `coverage_mission` / `competition_mission` | `robot_slam` | Pi (Jazzy) | **Laptop (Humble)** | `/scan_filtered`, `/map` | Nav2 action goals, `/speed_limit` |
| `classify_service` | `waste_classifier` | Pi (Jazzy) | **Laptop (Humble)** | `/camera/image_raw/compressed` | `/classify` service |
| `rviz2` | `rviz2` | Pi (Jazzy) | **Laptop (Humble)** | All topics / TF | Operator display |

---

### Detailed File & Line Modification Audit

#### 1. `ros2_ws/src/robot_slam/robot_slam/stack_common.py`
- [link to stack_common.py](file:///C:/Users/Aljamea%20Nairobi/Documents/hardleel-pi-repo/.claude/worktrees/laptop-brain/ros2_ws/src/robot_slam/robot_slam/stack_common.py#L19-L49)
- **Lines 19-20 & 47-48**: Static TFs (`base_to_laser`, `base_to_imu`) are embedded inside `lidar(lidar_port)`.  
  *Change*: Extract static transforms into an independent `transforms()` function callable by the laptop launch without launching the hardware lidar. Add `base_to_camera` transform (`0.10, 0.0, 0.20, 0, 0, 0`).
- **Lines 40-50 (`lidar` function)**: Launches `sllidar_a1_launch.py`.  
  *Change*: Move execution to Pi only (`robot_hw.launch.py`).
- **Lines 52-64 (`imu_and_prep` function)**: Combines `imu_serial_node` + `imu_filter_madgwick_node` + `sensor_prep`.  
  *Change*: Split into `imu_hw(imu_port)` (for Pi) and `sensor_prep_node(use_sim_time)` (for Laptop).
- **Lines 66-80 (`cartographer` function)**: Lines 75 & 78 hardcode `parameters=[{'use_sim_time': False}]`.  
  *Change*: Add `use_sim_time=False` parameter to `cartographer()` function to accept launch configurations.
- **Lines 83-98 (`safety_chain` function)**:  
  *Change*: Runs on laptop. Point parameter file to Humble-compatible YAML.
- **Lines 101-115 (`nav2_nodes` function)**:  
  *Change*: Ensure node list is Humble-compatible (contains controller, smoother, planner, behavior, bt_navigator, waypoint_follower; excludes route_server and docking_server).
- **Lines 117-129 (`odom_for_nav2` function)**:  
  *Change*: Parameterize `use_sim_time` for `rf2o_laser_odometry` and `robot_localization`.

#### 2. `ros2_ws/src/robot_slam/launch/mapping_stack.launch.py`
- [link to mapping_stack.launch.py](file:///C:/Users/Aljamea%20Nairobi/Documents/hardleel-pi-repo/.claude/worktrees/laptop-brain/ros2_ws/src/robot_slam/launch/mapping_stack.launch.py#L25-L38)
- **Lines 29-37**: Launches lidar, imu, cartographer, safety chain, and motor bridge on a single host.  
  *Change*: Superseded by the two-file architecture: `robot_hw.launch.py` on Pi and `brain.launch.py mode:=mapping` on Laptop.

#### 3. `ros2_ws/src/robot_slam/launch/mission_stack.launch.py`
- [link to mission_stack.launch.py](file:///C:/Users/Aljamea%20Nairobi/Documents/hardleel-pi-repo/.claude/worktrees/laptop-brain/ros2_ws/src/robot_slam/launch/mission_stack.launch.py#L32-L48)
- **Lines 32-48**: Launches lidar, imu, cartographer localization, odom, safety chain, nav2, motor bridge, and coverage mission.  
  *Change*: Superseded by `robot_hw.launch.py` on Pi and `brain.launch.py mode:=mission` on Laptop.

#### 4. `ros2_ws/src/robot_slam/launch/nav.launch.py`
- [link to nav.launch.py](file:///C:/Users/Aljamea%20Nairobi/Documents/hardleel-pi-repo/.claude/worktrees/laptop-brain/ros2_ws/src/robot_slam/launch/nav.launch.py#L65-L99)
- **Line 65**: `parameters=[{'laser_scan_topic': '/scan', ...}]` -> **BUG**: Reading raw `/scan` causes odometry slippage on ramps. Must be `/scan_filtered`.
- **Line 76**: `ekf_node` does not override `publish_tf: False` -> **BUG**: Causes conflicting TF broadcast with Cartographer.
- **Lines 82 & 93**: Includes `localization_slow_cpu_launch.py` and `navigation_slow_cpu_launch.py`.

#### 5. `ros2_ws/src/robot_slam/launch/navigation_slow_cpu_launch.py`
- [link to navigation_slow_cpu_launch.py](file:///C:/Users/Aljamea%20Nairobi/Documents/hardleel-pi-repo/.claude/worktrees/laptop-brain/ros2_ws/src/robot_slam/launch/navigation_slow_cpu_launch.py#L45-L56)
- **Lines 49 & 55**: `'route_server'` and `'docking_server'` in `lifecycle_nodes`. **Jazzy only**. Must be removed for Humble.
- **Lines 164-173**: Node `nav2_route::route_server` (fails to find package in Humble). Remove.
- **Lines 231-240**: Node `opennav_docking::docking_server` (fails to find package in Humble). Remove.
- **Lines 281-286 & 324-329**: Composable variants of above. Remove.

#### 6. `ros2_ws/src/robot_slam/config/nav2_params.yaml` & `nav2_params_mission.yaml`
- [link to nav2_params.yaml](file:///C:/Users/Aljamea%20Nairobi/Documents/hardleel-pi-repo/.claude/worktrees/laptop-brain/ros2_ws/src/robot_slam/config/nav2_params.yaml#L42-L61)
- **Line 42**: `scan_topic: scan` in `amcl`. Change to `/scan_filtered`.
- **Lines 52-61**: `bt_navigator` parameters:
  ```yaml
  # JAZZY 1.3 SYNTAX (INCOMPATIBLE WITH HUMBLE):
  navigators:
  - navigate_to_pose
  - navigate_through_poses
  navigate_to_pose:
    plugin: nav2_bt_navigator::NavigateToPoseNavigator
  navigate_through_poses:
    plugin: nav2_bt_navigator::NavigateThroughPosesNavigator
  error_code_names:
  - compute_path_error_code
  - follow_path_error_code
  ```
  **Must be replaced with Humble 1.1 syntax**:
  ```yaml
  default_nav_to_pose_bt_xml: "navigate_w_replanning_and_recovery.xml"
  default_nav_through_poses_bt_xml: "navigate_through_poses_w_replanning_and_recovery.xml"
  plugin_lib_names:
  - nav2_compute_path_to_pose_action_bt_node
  - nav2_compute_path_through_poses_action_bt_node
  - nav2_smooth_path_action_bt_node
  - nav2_follow_path_action_bt_node
  - nav2_spin_action_bt_node
  - nav2_wait_action_bt_node
  - nav2_back_up_action_bt_node
  - nav2_clear_costmap_service_bt_node
  - nav2_is_stuck_condition_bt_node
  - nav2_goal_reached_condition_bt_node
  - nav2_goal_updated_condition_bt_node
  - nav2_initial_pose_received_condition_bt_node
  - nav2_reinitialize_global_localization_service_bt_node
  - nav2_rate_controller_bt_node
  - nav2_distance_controller_bt_node
  - nav2_speed_controller_bt_node
  - nav2_truncate_path_action_bt_node
  - nav2_goal_updater_node_bt_node
  - nav2_pipeline_sequence_bt_node
  - nav2_round_robin_bt_node
  - nav2_transform_available_condition_bt_node
  - nav2_time_expired_condition_bt_node
  - nav2_distance_traveled_condition_bt_node
  - nav2_single_trigger_bt_node
  - nav2_is_battery_low_condition_bt_node
  - nav2_navigate_through_poses_action_bt_node
  - nav2_navigate_to_pose_action_bt_node
  - nav2_remove_passed_goals_action_bt_node
  - nav2_planner_selector_bt_node
  - nav2_controller_selector_bt_node
  - nav2_goal_checker_selector_bt_node
  ```
- **Lines 251-273**: `route_server:` configuration block. **Jazzy only**. Remove.
- **Line 277**: `stamp_smoothed_velocity_with_smoothing_time: false` in `velocity_smoother`. **Jazzy only**. Remove.
- **Lines 303-332**: `collision_monitor:` block inside `nav2_params*.yaml`. Line 318 uses `action_type: approach` and line 329 uses `topic: scan` (unfiltered). Collision monitor should be cleanly separated in `collision_monitor.yaml`.
- **Lines 333-374**: `docking_server:` block (`opennav_docking`). **Jazzy only**. Remove.
- **Lines 375-386**: `loopback_simulator:` block. **Jazzy only**. Remove.

#### 7. `ros2_ws/src/robot_slam/config/collision_monitor.yaml`
- [link to collision_monitor.yaml](file:///C:/Users/Aljamea%20Nairobi/Documents/hardleel-pi-repo/.claude/worktrees/laptop-brain/ros2_ws/src/robot_slam/config/collision_monitor.yaml#L9-L77)
- Compatible with Humble Nav2 1.1! Uses `type: circle` with `action_type: stop` and `action_type: slowdown`, which are fully supported in Humble.
- Input topic is `cmd_vel_in_topic: cmd_vel_smoothed`, output topic is `cmd_vel_out_topic: cmd_vel`, and observation source is `/scan_filtered`. No syntax changes required.

#### 8. `ros2_ws/src/motor_teleop/motor_teleop/motor_bridge.py`
- [link to motor_bridge.py](file:///C:/Users/Aljamea%20Nairobi/Documents/hardleel-pi-repo/.claude/worktrees/laptop-brain/ros2_ws/src/motor_teleop/motor_teleop/motor_bridge.py#L44-L63)
- **Line 44**: `self.declare_parameter('port', '/dev/pico')` (Matches hardware udev symlink).
- **Line 62**: `self.declare_parameter('imu_topic', '/imu/data_lpf')`  
  *Change*: Default to `/imu/data` or pass `imu_topic:='/imu/data'` in `robot_hw.launch.py`. This ensures motor pitch feedforward and ramp arming use the local 100 Hz Madgwick stream on the Pi rather than routing through the laptop over Wi-Fi.

#### 9. `pi_cam/pi_cam_node.py`
- [link to pi_cam_node.py](file:///C:/Users/Aljamea%20Nairobi/Documents/hardleel-pi-repo/.claude/worktrees/laptop-brain/pi_cam/pi_cam_node.py#L19-L20)
- **Line 20**: `self.pub = self.create_publisher(CompressedImage, "/camera/image_raw/compressed", QoSProfile(depth=1))`  
  *Change*: Explicitly configure `ReliabilityPolicy.BEST_EFFORT` on the publisher. Default ROS 2 QoS is RELIABLE, which introduces TCP-like congestion collapse and queue delay over Wi-Fi when streaming video.

---

## Deliverable 2: Static TF Publishing & Ownership Analysis

### Current Status in Repo
Static transforms are scattered across multiple launch files:
- `stack_common.py` (lines 47-48): `base_to_laser` and `base_to_imu` published inside `lidar()`.
- `carto_mapping.launch.py` (lines 35-38): publishes `base_to_laser` only.
- `nav.launch.py` (lines 53-54): publishes `base_to_laser` and `base_to_imu`.
- `imu.launch.py` (lines 58-65): publishes `base_to_imu` only.
- No file publishes a static transform for `camera` (`base_link -> camera`).

### Who Should Own Static TFs?
**The Laptop (WSL2 Humble) MUST own and publish all static transforms.**

#### Justification:
1. **DDS / Transient Local Wi-Fi Hazard:** In ROS 2, `static_transform_publisher` publishes to `/tf_static` with `transient_local` durability. Across a wireless boundary (especially between different ROS 2 distributions—Jazzy on Pi and Humble on Laptop—using CycloneDDS), late-joining nodes or restarted nodes on the laptop frequently miss or experience seconds-long delays in receiving latched `/tf_static` messages.
2. **Zero TF Need on the Pi:**
   - `sllidar_node` stamps messages with `frame_id: 'laser'`. It never looks up TF.
   - `imu_serial_node` stamps messages with `frame_id: 'imu_link'`. It never looks up TF.
   - `imu_filter_madgwick` runs with `publish_tf: false`. It never looks up TF.
   - `motor_bridge` processes `/cmd_vel` without TF.
   - `pi_cam_node` stamps frames with `frame_id: 'camera'`. It never looks up TF.  
   **Not a single node on the Raspberry Pi consumes TF.**
3. **All TF Consumers are on the Laptop:**
   `sensor_prep`, Cartographer, `rf2o_laser_odometry`, `robot_localization` EKF, Nav2 local & global costmaps, `collision_monitor`, and `rviz2` all live on the Laptop. Publishing `/tf_static` locally on the laptop guarantees 0 ms latency, instantaneous delivery over local IPC/shared memory, and immunity to Wi-Fi dropouts.
4. **Simulation Parity:** When running in simulation (`sim:=true`), the laptop's `robot_state_publisher` generates the TF tree. Keeping TF ownership on the laptop ensures consistent behavior between physical and simulated runs.

#### Concrete Static TF Definitions for Laptop:
```python
# In brain.launch.py (Laptop):
static_tf(0.0, 0.0, 0.15, 0.0, 'base_link', 'laser', 'base_to_laser'),
static_tf(0.0, 0.0, 0.05, 0.0, 'base_link', 'imu_link', 'base_to_imu'),
static_tf(0.10, 0.0, 0.20, 0.0, 'base_link', 'camera', 'base_to_camera'),
```

---

## Deliverable 3: Bugs, Inconsistencies, and Hazards Found

### 1. Topic Inconsistency: `/scan` vs `/scan_filtered`
- **Design Invariant:** On ramps, raw `/scan` detects the floor slope as an obstacle, corrupting the map and triggering false emergency stops. `sensor_prep.py` removes ground hits and overhead hits above `arm_deg` (2.0° tilt), outputting `/scan_filtered`.
- **Bugs Found:**
  - `nav2_params.yaml` line 329 & `nav2_params_mission.yaml` line 329: `collision_monitor` is set to `topic: scan` (unfiltered)!
  - `nav2_params.yaml` line 42 & `nav2_params_mission.yaml` line 42: `amcl: scan_topic: scan` (unfiltered).
  - `competition_mission.py` line 99: `self.create_subscription(LaserScan, 'scan', ...)` subscribes to relative unfiltered `scan` instead of `/scan_filtered`.
  - `nav.launch.py` line 65: `rf2o_laser_odometry` is configured with `laser_scan_topic: '/scan'` instead of `'/scan_filtered'`.
  - `slam_toolbox.yaml` line 6: `scan_topic: /scan`.

### 2. QoS Policy Mismatches Across DDS
- `sllidar_node.cpp` line 62 publishes `/scan` with `rclcpp::QoS(rclcpp::KeepLast(10))` -> **RELIABLE**.
- `sensor_prep.py` lines 80 & 93-94 publishes `/scan_filtered` and `/imu/data_lpf` with `ReliabilityPolicy.BEST_EFFORT`.
- `pi_cam_node.py` line 20 publishes `/camera/image_raw/compressed` with default QoS -> **RELIABLE** (causes packet stalling on Wi-Fi).
- **Fix:** Standardize all high-rate sensor streams (`/scan`, `/scan_filtered`, `/imu/*`, `/camera/*`) to use `SensorDataQoS` (Best Effort, Volatile, Depth 5-10) so packet loss does not trigger blocking retransmissions over Wi-Fi.

### 3. TF Multi-Publisher Collision (`odom` -> `base_link`)
- In `cartographer_2d.lua` (lines 10-11):
  ```lua
  odom_frame = "odom"
  provide_odom_frame = true
  ```
  Cartographer publishes `map -> odom` **AND** `odom -> base_link`.
- In `ekf_nav.yaml` (lines 6, 10-12):
  ```yaml
  publish_tf: true
  odom_frame: odom
  base_link_frame: base_link
  world_frame: odom
  ```
  `ekf_filter_node` ALSO publishes `odom -> base_link`.
- In `nav.launch.py` line 76: `ekf_filter_node` is launched without overriding `publish_tf`. Both Cartographer and EKF broadcast competing transforms for `odom -> base_link`, causing extreme TF jitter in RViz and Nav2.
- **Fix:** Cartographer must remain the single authoritative publisher of `odom -> base_link` (or `provide_odom_frame = false` if EKF is used as primary odom). In `stack_common.py` line 127, `publish_tf: False` was correctly overridden for EKF; this must be strictly maintained in `brain.launch.py`.

### 4. Hardcoded `use_sim_time: False` Breaking Simulation
- In `stack_common.py` lines 75 & 78, `cartographer_node` and `occupancy_grid_node` are hardcoded to `parameters=[{'use_sim_time': False}]`.
- In `carto_mapping.launch.py` lines 56 & 64, hardcoded `{'use_sim_time': False}`.
- In `nav.launch.py` lines 86 & 96, hardcoded `'use_sim_time': 'false'`.
- When launching in WSL with Gazebo (`sim:=true`), clock messages (`/clock`) are ignored by Cartographer and Nav2, stalling all timers and path planners.

### 5. Missing Camera Static Transform
- No transform exists for `base_link -> camera`. Any visual detection node attempting to project detected waste or offloading bays into 3D world coordinates (`tf2_ros::Buffer::lookupTransform("map", "camera", ...)`) will throw a `LookupTransformException`.

---

## Deliverable 4: Minimal File Layout & Launch Design

### Target File Structure
```
ros2_ws/src/robot_slam/
├── config/
│   ├── cartographer_2d.lua                  # Mapping config
│   ├── cartographer_2d_localization.lua     # Pure localization config
│   ├── collision_monitor.yaml               # Mapping & Mission safety zones (Humble-ready)
│   ├── coverage_mission.yaml                # Coverage waypoints & keepout
│   ├── ekf_nav.yaml                         # EKF sensor fusion (publish_tf: false)
│   ├── nav2_params_humble.yaml              # Humble 1.1 Nav2 config (NEW)
│   └── twist_mux.yaml                       # Teleop vs Nav priority
├── launch/
│   ├── brain.launch.py                      # Laptop master launch (NEW)
│   └── robot_hw.launch.py                   # Pi 4 master launch (NEW)
├── robot_slam/
│   ├── competition_core.py                  # State machine logic
│   ├── competition_mission.py               # Autonomous mission node
│   ├── coverage_mission.py                  # Coverage driver
│   ├── imu_serial_node.py                   # Nano IMU driver
│   ├── sensor_prep.py                       # Filtered scan & IMU LPF
│   └── stack_common.py                      # Shared builders & helpers
└── package.xml
```

---

### Implementation of `robot_hw.launch.py` (Raspberry Pi 4)
*Location: `ros2_ws/src/robot_slam/launch/robot_hw.launch.py`*

```python
"""Hardware drivers master launch for robot 'hardleel' on Raspberry Pi 4 (ROS 2 Jazzy).

Launches ONLY:
1. RPLIDAR A1 driver (/scan)
2. Arduino Nano 33 BLE Sense IMU driver (/imu/data_raw)
3. Madgwick orientation filter (/imu/data)
4. Pi Camera node (/camera/image_raw/compressed)
5. Pico motor bridge (/cmd_vel -> motors, /encoders, /surface_state)

Usage:
    ros2 launch robot_slam robot_hw.launch.py
    ros2 launch robot_slam robot_hw.launch.py camera:=false motor_bridge:=false
"""
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def find_nano():
    found = sorted(glob.glob('/dev/serial/by-id/usb-Arduino*'))
    return found[0] if found else '/dev/ttyACM1'


def generate_launch_description():
    sllidar_pkg = get_package_share_directory('sllidar_ros2')
    motor_pkg = get_package_share_directory('motor_teleop')

    return LaunchDescription([
        # Serial port arguments
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0', description='RPLIDAR serial port'),
        DeclareLaunchArgument('imu_port', default_value=find_nano(), description='Arduino Nano serial port'),
        DeclareLaunchArgument('pico_port', default_value='/dev/pico', description='Raspberry Pi Pico serial port'),
        DeclareLaunchArgument('motor_bridge', default_value='true', description='Enable motor driver bridge'),
        DeclareLaunchArgument('camera', default_value='true', description='Enable Pi Camera compressed stream'),
        DeclareLaunchArgument('cam_fps', default_value='15.0', description='Camera frame rate'),

        # 1. RPLIDAR A1 Driver -> /scan (Sensitivity mode, 8k samples/s, ~1400 pts/rev)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sllidar_pkg, 'launch', 'sllidar_a1_launch.py')),
            launch_arguments={
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': '115200',
                'frame_id': 'laser',
                'angle_compensate': 'true',
                'scan_mode': 'Sensitivity',
            }.items(),
        ),

        # 2. Arduino Nano IMU -> /imu/data_raw (100 Hz raw gyro + accel)
        Node(
            package='robot_slam',
            executable='imu_serial_node',
            name='imu_serial_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('imu_port'),
                'frame_id': 'imu_link',
                'gyro_calib_s': 5.0,
            }],
        ),

        # 3. Madgwick Filter -> /imu/data (fused orientation, low gain for gravel rejection)
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick_node',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
                'gain': 0.02,
            }],
            remappings=[
                ('imu/data_raw', '/imu/data_raw'),
                ('imu/data', '/imu/data'),
            ],
        ),

        # 4. Pi Camera Driver -> /camera/image_raw/compressed
        Node(
            package='pi_cam',
            executable='pi_cam_node',
            name='pi_cam',
            output='screen',
            parameters=[{
                'fps': LaunchConfiguration('cam_fps'),
                'jpeg_quality': 75,
            }],
            condition=IfCondition(LaunchConfiguration('camera')),
        ),

        # 5. Pico Motor Bridge -> /cmd_vel (listens to collision_monitor output from laptop)
        # Note: imu_topic points to local /imu/data for zero-latency slope feedforward
        Node(
            package='motor_teleop',
            executable='motor_bridge',
            name='motor_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('pico_port'),
                'send_rate': 20.0,
                'imu_topic': '/imu/data',
                'max_pwm': 255,
                'flat_max_pwm': 220,
                'sand_max_pwm': 150,
                'gravel_max_pwm': 180,
            }],
            condition=IfCondition(LaunchConfiguration('motor_bridge')),
        ),
    ])
```

---

### Implementation of `brain.launch.py` (Laptop WSL2)
*Location: `ros2_ws/src/robot_slam/launch/brain.launch.py`*

```python
"""Master brain launch for robot 'hardleel' on Laptop WSL2 (ROS 2 Humble).

Runs everything except hardware drivers:
- Static TFs (base_link -> laser, imu_link, camera)
- sensor_prep (/scan_filtered, /imu/data_lpf)
- Cartographer (mapping or pure localization)
- rf2o + EKF odometry (/odom)
- Nav2 Humble stack
- Safe command chain (twist_mux -> velocity_smoother -> collision_monitor -> /cmd_vel)
- Autonomy mission & classifier
- RViz2

Usage:
    ros2 launch robot_slam brain.launch.py mode:=mapping
    ros2 launch robot_slam brain.launch.py mode:=mission pbstream:=/path/to/arena.pbstream
    ros2 launch robot_slam brain.launch.py mode:=mission pbstream:=... run_mission:=true
    ros2 launch robot_slam brain.launch.py mode:=mapping sim:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg = get_package_share_directory('robot_slam')
    cfg = os.path.join(pkg, 'config')

    mode = LaunchConfiguration('mode')
    sim = LaunchConfiguration('sim')
    pbstream = LaunchConfiguration('pbstream')
    run_mission = LaunchConfiguration('run_mission')
    rviz = LaunchConfiguration('rviz')
    classifier = LaunchConfiguration('classifier')

    is_mapping = EqualsSubstitution(mode, 'mapping')
    is_mission = EqualsSubstitution(mode, 'mission')
    is_real = UnlessCondition(sim)

    nav2_params = os.path.join(cfg, 'nav2_params_humble.yaml')

    return LaunchDescription([
        # Core arguments
        DeclareLaunchArgument('mode', default_value='mapping', description='Stack mode: mapping | mission'),
        DeclareLaunchArgument('sim', default_value='false', description='Simulation mode (Gazebo clock)'),
        DeclareLaunchArgument('pbstream', default_value='', description='Cartographer map pbstream for localization'),
        DeclareLaunchArgument('run_mission', default_value='false', description='Auto-start mission execution'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('classifier', default_value='false', description='Launch waste classifier service'),

        # Propagate use_sim_time globally across all nodes
        SetParameter('use_sim_time', sim),

        # 1. Static Transforms (Owned exclusively by Laptop when connected to real robot)
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.15', '--yaw', '0.0', '--pitch', '0', '--roll', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser'],
            condition=is_real,
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='base_to_imu',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.05', '--yaw', '0.0', '--pitch', '0', '--roll', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
            condition=is_real,
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='base_to_camera',
            arguments=['--x', '0.10', '--y', '0.0', '--z', '0.20', '--yaw', '0.0', '--pitch', '0', '--roll', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'camera'],
            condition=is_real,
        ),

        # 2. Sensor Preparation: tilt gating & IMU low-pass filter
        Node(
            package='robot_slam',
            executable='sensor_prep',
            name='sensor_prep',
            output='screen',
            parameters=[{
                'lidar_height': 0.15,
                'robot_height': 0.35,
                'ground_margin': 0.03,
                'arm_deg': 2.0,
                'release_deg': 1.0,
            }],
        ),

        # 3. Cartographer SLAM (Mapping mode)
        Node(
            package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', cfg,
                '-configuration_basename', 'cartographer_2d.lua',
            ],
            remappings=[('scan', '/scan_filtered'), ('imu', '/imu/data')],
            condition=IfCondition(is_mapping),
        ),

        # 3b. Cartographer Pure Localization (Mission mode)
        Node(
            package='cartographer_ros', executable='cartographer_node', name='cartographer_node_loc',
            output='screen',
            arguments=[
                '-configuration_directory', cfg,
                '-configuration_basename', 'cartographer_2d_localization.lua',
                '-load_state_filename', pbstream,
                '-load_frozen_state=true',
            ],
            remappings=[('scan', '/scan_filtered'), ('imu', '/imu/data')],
            condition=IfCondition(is_mission),
        ),

        # Cartographer Occupancy Grid -> /map
        Node(
            package='cartographer_ros', executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node', output='screen',
            arguments=['-resolution', '0.05', '-publish_period_sec', '0.5'],
        ),

        # 4. Odometry for Nav2 (rf2o + EKF) - Mission mode
        Node(
            package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node', name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan_filtered',
                'odom_topic': '/odom_rf2o',
                'publish_tf': False,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 10.0,
            }],
            condition=IfCondition(is_mission),
        ),
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(cfg, 'ekf_nav.yaml'), {'publish_tf': False}],
            remappings=[('odometry/filtered', '/odom')],
            condition=IfCondition(is_mission),
        ),

        # 5. Safe Command Chain: twist_mux -> velocity_smoother -> collision_monitor -> /cmd_vel
        Node(
            package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
            parameters=[os.path.join(cfg, 'twist_mux.yaml')],
            remappings=[('cmd_vel_out', 'cmd_vel_mux')],
        ),
        Node(
            package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
            output='screen',
            parameters=[nav2_params],
            remappings=[('cmd_vel', 'cmd_vel_mux'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')],
        ),
        Node(
            package='nav2_collision_monitor', executable='collision_monitor',
            name=PythonExpression(["'collision_monitor_mapping' if '", mode, "' == 'mapping' else 'collision_monitor_mission'"]),
            output='screen',
            parameters=[os.path.join(cfg, 'collision_monitor.yaml')],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_safety',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'velocity_smoother',
                    PythonExpression(["'collision_monitor_mapping' if '", mode, "' == 'mapping' else 'collision_monitor_mission'"]),
                ],
            }],
        ),

        # 6. Nav2 Navigation Stack (Mission mode only, Humble-configured)
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[nav2_params], remappings=[('cmd_vel', 'cmd_vel_nav')],
             condition=IfCondition(is_mission)),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server',
             output='screen', parameters=[nav2_params], condition=IfCondition(is_mission)),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[nav2_params], condition=IfCondition(is_mission)),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=[nav2_params], remappings=[('cmd_vel', 'cmd_vel_nav')],
             condition=IfCondition(is_mission)),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[nav2_params], condition=IfCondition(is_mission)),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
             output='screen', parameters=[nav2_params], condition=IfCondition(is_mission)),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen',
             parameters=[{
                 'autostart': True,
                 'node_names': ['controller_server', 'smoother_server', 'planner_server',
                                'behavior_server', 'bt_navigator', 'waypoint_follower'],
                 'bond_timeout': 10.0,
             }],
             condition=IfCondition(is_mission)),

        # 7. Autonomy & Perception
        TimerAction(
            period=15.0,
            condition=IfCondition(PythonExpression(["'", mode, "' == 'mission' and '", run_mission, "' == 'true'"])),
            actions=[
                Node(package='robot_slam', executable='competition_mission', name='competition_mission',
                     output='screen', parameters=[{'auto_start': True}]),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('waste_classifier'), 'launch', 'classify.launch.py')),
            condition=IfCondition(classifier),
        ),

        # 8. RViz2
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(cfg, 'nav2_default_view.rviz')],
            condition=IfCondition(rviz),
        ),
    ])
```

---

## Deliverable 5: CycloneDDS Network Configuration

### Common Network Topology
- **Subnet:** `192.168.1.0/24` (or router DHCP subnet)
- **Raspberry Pi IP:** e.g., `192.168.1.50`
- **Laptop WSL IP:** e.g., `192.168.1.100` (Mirrored mode uses Windows host IP)
- **`ROS_DOMAIN_ID`:** `7`

### `cyclonedds.xml` Configuration (Both Machines)
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
        </General>
        <Discovery>
            <Peers>
                <!-- Add static peer IPs to guarantee discovery even if Wi-Fi drops multicast -->
                <Peer Address="192.168.1.50"/>
                <Peer Address="192.168.1.100"/>
            </Peers>
            <ParticipantIndex>auto</ParticipantIndex>
        </Discovery>
        <Internal>
            <Watermarks>
                <WhcHigh>500kB</WhcHigh>
            </Watermarks>
        </Internal>
    </Domain>
</CycloneDDS>
```

### Environment Setup
Add to `~/.bashrc` on both Pi and WSL:
```bash
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
```

---

## Verification Plan

### 1. Automated Syntax & Compilation Tests
```bash
# In WSL2:
cd ~/hardleel-pi-repo/ros2_ws
colcon build --packages-select robot_slam motor_teleop waste_classifier
colcon test --packages-select robot_slam
```

### 2. Standalone Pi Hardware Bringup
```bash
# On Raspberry Pi 4:
ros2 launch robot_slam robot_hw.launch.py

# Verify hardware topics from Laptop WSL:
ros2 topic hz /scan                              # Expect ~5.5 Hz, >= 1300 pts/rev
ros2 topic hz /imu/data_raw                      # Expect 100 Hz
ros2 topic hz /imu/data                          # Expect 100 Hz
ros2 topic hz /camera/image_raw/compressed       # Expect 10-15 Hz
```

### 3. Distributed Mapping Verification
```bash
# On Laptop WSL:
ros2 launch robot_slam brain.launch.py mode:=mapping

# Verify:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop -p speed:=0.15
# Observe /cmd_vel published to Pi, motors turning, Cartographer submaps rendering in RViz.
```

### 4. Safety Chain & Obstacle Avoidance Verification
- Put hand 0.25 m in front of RPLIDAR during teleop -> `/cmd_vel` drops to 0.0 (StopZone triggered).
- Put hand 0.40 m in front -> velocity scales down by 40% (SlowZone triggered).
- Tilt robot nose-down > 2.0° -> `/scan_filtered` gates out floor returns; Cartographer does not corrupt submap.

### 5. Distributed Mission Verification
```bash
# Save mapping pbstream:
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/user/maps/field.pbstream'}"

# Launch mission mode:
ros2 launch robot_slam brain.launch.py mode:=mission pbstream:=/home/user/maps/field.pbstream run_mission:=true
# Verify Nav2 lifecycle states transition to active, pure localization tracks pose, robot navigates to screen.
```
