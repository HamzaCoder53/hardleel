include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",     -- the IMU frame, because the IMU is used
  published_frame = "odom",        -- odom->base_link comes from your EKF, so publish map->odom
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = false,   -- 19 Nov: true crashed Cartographer (imu_tracker.cc:40 "time_ <= time": IMU messages fed unordered)
  use_odometry = false,            -- no wheel odometry
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,  -- try 2 if you rotate fast: splits the ~180 ms A1 scan
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,     -- was 0.3 (CPU/Wi-Fi, 15 Nov)
  pose_publish_period_sec = 2e-2,      -- 50 Hz, was 5e-3 = 200 Hz (CPU/TF traffic, 15 Nov)
  trajectory_publish_period_sec = 100e-3,   -- RViz path only, was 30e-3
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 0.25,   -- 19 Nov: at 92 Hz Cartographer (ROS 2 node) reordered IMU msgs under load -> "Non-sorted data added to queue (0, imu)" crash loops (14 in 60 s); 0.25 (~23 Hz): 0 in 90 s
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = true   -- back on 19 Nov: the IMU reader nap (stamp jitter 0.8-21.6 ms) was the crash cause; 0 crashes in 120 s after f266131
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true   -- helps with no odometry
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1                -- the A1 gives one full scan per message
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.           -- was 0.2: don't add nodes every 0.2 s while parked (drift fix, 15 Nov)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45

-- Pose graph: this is the ONLY thing that removes accumulated drift.
-- Previous values (min_score 0.65 / sampling 0.2 / every 40) rejected or
-- skipped nearly every loop closure, so drift was never corrected (drift fix, 15 Nov).
POSE_GRAPH.optimize_every_n_nodes = 30                             -- was 40: optimize ~once per finished submap
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3                 -- was 0.2: 0.2 skips 80% of loop candidates
POSE_GRAPH.constraint_builder.min_score = 0.55                     -- was 0.65: 0.65 rejects almost every match; raise to 0.6 only if you see false closures (map snaps/warps)
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6  -- only used in pure-localization mode, not mapping
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.        -- raise if your loops are longer than 15 m
POSE_GRAPH.constraint_builder.log_matches = true                   -- watch the log: revisit old areas, expect "Found connection" lines with score >= 0.55

return options
