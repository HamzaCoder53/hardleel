-- Cartographer for the ENCODER + LIDAR stack, no IMU (pi_encoder_mapping.launch.py, 22 Nov).
-- Odometry: esp32_encoder_bridge publishes odom->base_link from the wheel encoders (4x CHR-GM25-370, 80 mm
-- wheels, 748 counts per wheel turn). Skid steer: encoder DISTANCE is good, encoder ROTATION slips - so the
-- odometry rotation is trusted less and the lidar scan matching corrects the heading.
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",    -- no IMU: track the robot centre
  published_frame = "odom",        -- odom->base_link comes from the encoder bridge, Cartographer adds map->odom
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = false,   -- 19 Nov: true crashed Cartographer (data fed unordered)
  use_odometry = true,             -- wheel encoders on /odom
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 2e-2,
  trajectory_publish_period_sec = 100e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = false                          -- this stack has no IMU
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true   -- corrects encoder heading slip
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45

-- Global SLAM (loop closure), same as cartographer_2d.lua
POSE_GRAPH.optimize_every_n_nodes = 30
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.min_score = 0.65                     -- 0.55 gave false closures in the maze (19 Nov)
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.log_matches = true
-- How much the optimisation believes the wheel odometry between scans (defaults 1e5 / 1e5):
-- distance stays trusted, rotation much less (skid steer slips when turning).
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e3

return options
