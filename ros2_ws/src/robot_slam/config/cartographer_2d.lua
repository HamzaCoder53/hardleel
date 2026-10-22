include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",          -- Match the frame_id of /imu/data
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,            -- Cartographer provides map -> odom -> base_link
  publish_frame_projected_to_2d = true, -- Locks base_link to 2D plane despite ramp pitch
  use_pose_extrapolator = true,
  use_odometry = false,                 -- No physical wheel encoders
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 20e-3,      -- 50 Hz pose output
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 3  -- Leaves 1 core on Pi 4 for OS + safety node

-- Sensor & Range Bounds (Matched to 4.87m x 2.44m Arena)
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.18
TRAJECTORY_BUILDER_2D.max_range = 5.50  -- Arena diagonal is 5.44m; ignores outer noise
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 2.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- Voxel Filter
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 100

-- IMU Gravity Smoothing (Increased to reject gravel vibration)
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 20.0

-- MANDATORY WITHOUT ENCODERS: Correlative Scan Matcher finds global local-maxima first
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(25.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres Scan Matcher: Trust LiDAR geometry heavily + IMU yaw, minimal translation prior
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 0.5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 15.0

-- Motion Filter: Avoid inserting duplicate scans when stationary
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 2.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.5)

-- Submaps: Small submaps (35 scans = ~6.3s at 5.5 Hz) close loops fast in small arenas
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.03
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.58
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.47

-- Global SLAM / Loop Closure Tuning
POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66
POSE_GRAPH.optimization_problem.huber_scale = 5e1

return options
