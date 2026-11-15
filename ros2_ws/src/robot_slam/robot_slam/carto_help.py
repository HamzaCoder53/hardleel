"""What each Cartographer value in cartographer_2d.lua does (shown in the tuning GUI's Cartographer tab).
'LEAVE' = describes the robot's sensors/frames; changing it breaks mapping on this robot."""

CARTO_HELP = {
    # ---- options: what the robot has and how often things are published --------------------------
    'options.provide_odom_frame':
        'LEAVE false. true = Cartographer publishes odom->base_link itself; here the EKF does that.',
    'options.publish_frame_projected_to_2d':
        'Keeps the published pose flat (no roll/pitch from the IMU). Keep true for a ground robot.',
    'options.use_pose_extrapolator':
        'Publish poses predicted between scans (smoother TF). false = only at scan times (jumpier).',
    'options.use_odometry':
        'Use /odom in the SLAM itself. false now (no wheel encoders); true only if odometry is trustworthy.',
    'options.use_nav_sat': 'LEAVE false - no GPS.',
    'options.use_landmarks': 'LEAVE false - no landmark detector.',
    'options.num_laser_scans': 'LEAVE 1 - one 2D lidar on /scan.',
    'options.num_multi_echo_laser_scans': 'LEAVE 0 - the A1 is not a multi-echo lidar.',
    'options.num_subdivisions_per_laser_scan':
        'Split each scan into N parts to correct motion during the sweep. 1 is fine at low speed.',
    'options.num_point_clouds': 'LEAVE 0 - no 3D sensor.',
    'options.lookup_transform_timeout_sec':
        'How long to wait for a transform (TF). Raise (0.3-0.5) if the log says "lookup would require extrapolation".',
    'options.submap_publish_period_sec':
        'How often submaps are sent out (s). Lower = smoother map updates, more CPU and Wi-Fi traffic.',
    'options.pose_publish_period_sec':
        'How often the robot pose / map->odom is published (s). 5e-3 = 200 Hz.',
    'options.trajectory_publish_period_sec': 'How often the drawn path (trajectory) is published (s).',
    'options.rangefinder_sampling_ratio':
        'Fraction of scans used (1.0 = all, 0.5 = every 2nd). Lower to save CPU; worse map when moving fast.',
    'options.odometry_sampling_ratio': 'Fraction of odometry messages used (only matters with use_odometry).',
    'options.fixed_frame_pose_sampling_ratio': 'Unused (no GPS). Leave 1.',
    'options.imu_sampling_ratio': 'Fraction of IMU messages used (1.0 = all 93 Hz).',
    'options.landmarks_sampling_ratio': 'Unused (no landmarks). Leave 1.',
    'MAP_BUILDER.use_trajectory_builder_2d': 'LEAVE true - 2D mapping.',

    # ---- local SLAM: how each scan is matched and inserted ---------------------------------------
    'TRAJECTORY_BUILDER_2D.min_range':
        'Lidar points closer than this (m) are ignored - hides the robot\'s own body/cables. Raise if parts '
        'of the robot show up as walls.',
    'TRAJECTORY_BUILDER_2D.max_range':
        'Points farther than this (m) are ignored. The arena is ~6 m; far points are noisy. 6-8 is good.',
    'TRAJECTORY_BUILDER_2D.missing_data_ray_length':
        'When a beam returns nothing, this length (m) of free space is drawn along it. Lower if open '
        'areas get falsely cleared.',
    'TRAJECTORY_BUILDER_2D.use_imu_data':
        'Use the IMU gyro for rotation between scans. Keep true: it prevents smearing when turning.',
    'TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching':
        'Brute-force search for the best scan fit before fine matching. Needed without odometry; costs CPU.',
    'TRAJECTORY_BUILDER_2D.num_accumulated_range_data':
        'Scans merged into one before matching. 1 for the A1 (each message is a full 360 deg scan).',
    'TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters':
        'A scan is only added after moving at least this far (m). Lower = more detail and CPU.',
    'TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians':
        'A scan is only added after turning at least this much. Lower = better maps while turning, more CPU.',
    'TRAJECTORY_BUILDER_2D.submaps.num_range_data':
        'Scans per submap. Smaller = more, shorter submaps (less drift inside each, more to optimize). 30-90.',

    # ---- global SLAM: loop closure / fixing drift ------------------------------------------------
    'POSE_GRAPH.optimize_every_n_nodes':
        'Run the global optimization (loop closure) after this many new scans. Lower = drift fixed sooner, '
        'more CPU (0 = off).',
    'POSE_GRAPH.constraint_builder.sampling_ratio':
        'Fraction of scans tried as loop-closure candidates. Higher = finds loops more often, more CPU.',
    'POSE_GRAPH.constraint_builder.min_score':
        'How good a match must be to count as a loop closure (0-1). Higher = fewer false loops (doubled/bent '
        'walls); lower = closes more loops.',
    'POSE_GRAPH.constraint_builder.global_localization_min_score':
        'Same, for matching against another trajectory / a loaded map (0-1).',
}
