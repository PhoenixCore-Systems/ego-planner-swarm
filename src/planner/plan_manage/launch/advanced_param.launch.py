import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # LaunchConfigurations
    map_size_x = LaunchConfiguration('map_size_x_', default=42.0)
    map_size_y = LaunchConfiguration('map_size_y_', default=30.0)
    map_size_z = LaunchConfiguration('map_size_z_', default=5.0)
    virtual_ceil_height = LaunchConfiguration('virtual_ceil_height', default=2.9)
    
    odometry_topic = LaunchConfiguration('odometry_topic', default='odom')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='camera_pose')
    depth_topic = LaunchConfiguration('depth_topic', default='depth_image')
    cloud_topic = LaunchConfiguration('cloud_topic', default='cloud')
    
    cx = LaunchConfiguration('cx', default=321.04638671875)
    cy = LaunchConfiguration('cy', default=243.44969177246094)
    fx = LaunchConfiguration('fx', default=387.229248046875)
    fy = LaunchConfiguration('fy', default=387.229248046875)
    
    max_vel = LaunchConfiguration('max_vel', default=0.7)
    max_acc = LaunchConfiguration('max_acc', default=1.0)
    planning_horizon = LaunchConfiguration('planning_horizon', default=7.5)
    obstacles_inflation = LaunchConfiguration('obstacles_inflation', default=0.55)
    collision_dist0 = LaunchConfiguration('collision_dist0', default=1.0)
    swarm_clearance = LaunchConfiguration('swarm_clearance', default=1.0)
    tracking_error_monitor_enabled = LaunchConfiguration(
        'tracking_error_monitor_enabled', default=True)
    tracking_error_soft_threshold = LaunchConfiguration(
        'tracking_error_soft_threshold', default=0.5)
    tracking_error_hard_threshold = LaunchConfiguration(
        'tracking_error_hard_threshold', default=1.0)
    tracking_error_soft_duration = LaunchConfiguration(
        'tracking_error_soft_duration', default=0.25)
    tracking_error_hard_duration = LaunchConfiguration(
        'tracking_error_hard_duration', default=0.10)
    stall_commanded_speed = LaunchConfiguration(
        'stall_commanded_speed', default=0.15)
    stall_measured_speed = LaunchConfiguration(
        'stall_measured_speed', default=0.05)
    stall_duration = LaunchConfiguration('stall_duration', default=2.0)
    goal_tolerance = LaunchConfiguration('goal_tolerance', default=0.3)
    goal_velocity_tolerance = LaunchConfiguration(
        'goal_velocity_tolerance', default=0.2)
    terrain_ref_enabled = LaunchConfiguration('terrain_ref_enabled', default=True)
    terrain_ref_lambda = LaunchConfiguration('terrain_ref_lambda', default=4.0)
    terrain_band_lambda = LaunchConfiguration('terrain_band_lambda', default=12.0)
    terrain_min_confidence = LaunchConfiguration('terrain_min_confidence', default=0.20)
    terrain_max_profile_age_sec = LaunchConfiguration('terrain_max_profile_age_sec', default=0.75)
    terrain_max_lateral_m = LaunchConfiguration('terrain_max_lateral_m', default=1.5)
    terrain_agl_tolerance_below = LaunchConfiguration('terrain_agl_tolerance_below', default=0.35)
    terrain_agl_tolerance_above = LaunchConfiguration('terrain_agl_tolerance_above', default=0.60)
    terrain_agl_floor_soft = LaunchConfiguration('terrain_agl_floor_soft', default=1.0)
    
    point_num = LaunchConfiguration('point_num', default=1)
    point0_x = LaunchConfiguration('point0_x', default=0.0)
    point0_y = LaunchConfiguration('point0_y', default=0.0)
    point0_z = LaunchConfiguration('point0_z', default=0.0)
    point1_x = LaunchConfiguration('point1_x', default=10.0)
    point1_y = LaunchConfiguration('point1_y', default=10.0)
    point1_z = LaunchConfiguration('point1_z', default=0.0)
    point2_x = LaunchConfiguration('point2_x', default=20.0)
    point2_y = LaunchConfiguration('point2_y', default=20.0)
    point2_z = LaunchConfiguration('point2_z', default=1.0)
    point3_x = LaunchConfiguration('point3_x', default=-10.0)
    point3_y = LaunchConfiguration('point3_y', default=-10.0)
    point3_z = LaunchConfiguration('point3_z', default=1.0)
    point4_x = LaunchConfiguration('point4_x', default=30.0)
    point4_y = LaunchConfiguration('point4_y', default=30.0)
    point4_z = LaunchConfiguration('point4_z', default=1.0)

    flight_type = LaunchConfiguration('flight_type', default=2)
    use_distinctive_trajs = LaunchConfiguration('use_distinctive_trajs', default=True)
    
    obj_num_set = LaunchConfiguration('obj_num_set', default=10)

    drone_id = LaunchConfiguration('drone_id', default=0)
    planning_frame = LaunchConfiguration('planning_frame')
    observed_space_enabled = LaunchConfiguration(
        'observed_space_enabled', default=False)
    observed_space_timeout = LaunchConfiguration(
        'observed_space_timeout', default=0.5)
    observed_space_retention = LaunchConfiguration(
        'observed_space_retention', default=0.5)
    observed_space_clearance = LaunchConfiguration(
        'observed_space_clearance', default=0.0)
    observed_space_ray_dilation_voxels = LaunchConfiguration(
        'observed_space_ray_dilation_voxels', default=0)
    observed_space_seed_radius = LaunchConfiguration(
        'observed_space_seed_radius', default=0.45)
    observed_space_min_update_interval = LaunchConfiguration(
        'observed_space_min_update_interval', default=0.0)
    observed_space_resolution = LaunchConfiguration(
        'observed_space_resolution', default=0.0)
    observed_space_target_search_half_angle_deg = LaunchConfiguration(
        'observed_space_target_search_half_angle_deg', default=60.0)
    observed_space_target_search_steps = LaunchConfiguration(
        'observed_space_target_search_steps', default=3)
    observed_space_target_margin = LaunchConfiguration(
        'observed_space_target_margin', default=0.0)
    observed_space_validation_step = LaunchConfiguration(
        'observed_space_validation_step', default=0.05)
    local_update_range_xy = LaunchConfiguration(
        'local_update_range_xy', default=5.5)
    local_update_range_z = LaunchConfiguration(
        'local_update_range_z', default=4.5)
    max_ray_length = LaunchConfiguration('max_ray_length', default=4.5)
    visibility_cloud_topic = LaunchConfiguration(
        'visibility_cloud_topic', default='/planning/visibility_endpoints')
    visibility_origin_topic = LaunchConfiguration(
        'visibility_origin_topic', default='/planning/visibility_origin')

    # DeclareLaunchArguments
    map_size_x_arg = DeclareLaunchArgument('map_size_x_', default_value=map_size_x, description='Map size along X')
    map_size_y_arg = DeclareLaunchArgument('map_size_y_', default_value=map_size_y, description='Map size along Y')
    map_size_z_arg = DeclareLaunchArgument('map_size_z_', default_value=map_size_z, description='Map size along Z')
    virtual_ceil_height_arg = DeclareLaunchArgument(
        'virtual_ceil_height',
        default_value=virtual_ceil_height,
        description='Occupied virtual ceiling height in the map frame',
    )
    odometry_topic_arg = DeclareLaunchArgument('odometry_topic', default_value=odometry_topic, description='Odometry topic')
    camera_pose_topic_arg = DeclareLaunchArgument('camera_pose_topic', default_value=camera_pose_topic, description='Camera pose topic')
    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value=depth_topic, description='Depth topic')
    cloud_topic_arg = DeclareLaunchArgument('cloud_topic', default_value=cloud_topic, description='Point cloud topic')
    cx_arg = DeclareLaunchArgument('cx', default_value=cx, description='Camera intrinsic cx')
    cy_arg = DeclareLaunchArgument('cy', default_value=cy, description='Camera intrinsic cy')
    fx_arg = DeclareLaunchArgument('fx', default_value=fx, description='Camera intrinsic fx')
    fy_arg = DeclareLaunchArgument('fy', default_value=fy, description='Camera intrinsic fy')
    max_vel_arg = DeclareLaunchArgument('max_vel', default_value=max_vel, description='Maximum velocity')
    max_acc_arg = DeclareLaunchArgument('max_acc', default_value=max_acc, description='Maximum acceleration')
    planning_horizon_arg = DeclareLaunchArgument('planning_horizon', default_value=planning_horizon, description='Planning horizon')
    obstacles_inflation_arg = DeclareLaunchArgument(
        'obstacles_inflation',
        default_value=obstacles_inflation,
        description='Inflation radius for local map obstacles',
    )
    collision_dist0_arg = DeclareLaunchArgument(
        'collision_dist0',
        default_value=collision_dist0,
        description='EGO collision cost target distance',
    )
    swarm_clearance_arg = DeclareLaunchArgument(
        'swarm_clearance',
        default_value=swarm_clearance,
        description='EGO swarm clearance distance',
    )
    tracking_error_monitor_enabled_arg = DeclareLaunchArgument(
        'tracking_error_monitor_enabled',
        default_value=tracking_error_monitor_enabled,
        description='Enable odometry-versus-nominal trajectory divergence monitoring',
    )
    tracking_error_soft_threshold_arg = DeclareLaunchArgument(
        'tracking_error_soft_threshold',
        default_value=tracking_error_soft_threshold,
        description='Position error that requests a measured-state replan',
    )
    tracking_error_hard_threshold_arg = DeclareLaunchArgument(
        'tracking_error_hard_threshold',
        default_value=tracking_error_hard_threshold,
        description='Position error that latches an emergency stop until a new goal',
    )
    tracking_error_soft_duration_arg = DeclareLaunchArgument(
        'tracking_error_soft_duration',
        default_value=tracking_error_soft_duration,
        description='Time above the soft tracking threshold before replanning',
    )
    tracking_error_hard_duration_arg = DeclareLaunchArgument(
        'tracking_error_hard_duration',
        default_value=tracking_error_hard_duration,
        description='Time above the hard tracking threshold before emergency stop',
    )
    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value=goal_tolerance,
        description='Measured 3D distance required for goal completion',
    )
    goal_velocity_tolerance_arg = DeclareLaunchArgument(
        'goal_velocity_tolerance',
        default_value=goal_velocity_tolerance,
        description='Measured speed required for goal completion',
    )
    terrain_ref_enabled_arg = DeclareLaunchArgument(
        'terrain_ref_enabled',
        default_value=terrain_ref_enabled,
        description='Enable terrain-reference Z optimization',
    )
    terrain_ref_lambda_arg = DeclareLaunchArgument(
        'terrain_ref_lambda',
        default_value=terrain_ref_lambda,
        description='Terrain Z reference cost weight',
    )
    terrain_band_lambda_arg = DeclareLaunchArgument(
        'terrain_band_lambda',
        default_value=terrain_band_lambda,
        description='Terrain AGL band cost weight',
    )
    terrain_min_confidence_arg = DeclareLaunchArgument(
        'terrain_min_confidence',
        default_value=terrain_min_confidence,
        description='Minimum terrain profile confidence for optimizer samples',
    )
    terrain_max_profile_age_arg = DeclareLaunchArgument(
        'terrain_max_profile_age_sec',
        default_value=terrain_max_profile_age_sec,
        description='Maximum profile age for hard-floor validation; current stale behavior fails open',
    )
    terrain_max_lateral_arg = DeclareLaunchArgument(
        'terrain_max_lateral_m',
        default_value=terrain_max_lateral_m,
        description='Maximum lateral distance from a terrain profile sample',
    )
    terrain_agl_below_arg = DeclareLaunchArgument(
        'terrain_agl_tolerance_below',
        default_value=terrain_agl_tolerance_below,
        description='Allowed distance below desired terrain AGL',
    )
    terrain_agl_above_arg = DeclareLaunchArgument(
        'terrain_agl_tolerance_above',
        default_value=terrain_agl_tolerance_above,
        description='Allowed distance above desired terrain AGL',
    )
    terrain_agl_floor_soft_arg = DeclareLaunchArgument(
        'terrain_agl_floor_soft',
        default_value=terrain_agl_floor_soft,
        description='Absolute AGL hard-floor for terrain veto (recovery-aware)',
    )
    
    point_num_arg = DeclareLaunchArgument('point_num', default_value=point_num, description='Number of waypoints')
    point0_x_arg = DeclareLaunchArgument('point0_x', default_value=point0_x, description='Waypoint 0 X coordinate')
    point0_y_arg = DeclareLaunchArgument('point0_y', default_value=point0_y, description='Waypoint 0 Y coordinate')
    point0_z_arg = DeclareLaunchArgument('point0_z', default_value=point0_z, description='Waypoint 0 Z coordinate')
    point1_x_arg = DeclareLaunchArgument('point1_x', default_value=point1_x, description='Waypoint 1 X coordinate')
    point1_y_arg = DeclareLaunchArgument('point1_y', default_value=point1_y, description='Waypoint 1 Y coordinate')
    point1_z_arg = DeclareLaunchArgument('point1_z', default_value=point1_z, description='Waypoint 1 Z coordinate')
    point2_x_arg = DeclareLaunchArgument('point2_x', default_value=point2_x, description='Waypoint 2 X coordinate')
    point2_y_arg = DeclareLaunchArgument('point2_y', default_value=point2_y, description='Waypoint 2 Y coordinate')
    point2_z_arg = DeclareLaunchArgument('point2_z', default_value=point2_z, description='Waypoint 2 Z coordinate')
    point3_x_arg = DeclareLaunchArgument('point3_x', default_value=point3_x, description='Waypoint 3 X coordinate')
    point3_y_arg = DeclareLaunchArgument('point3_y', default_value=point3_y, description='Waypoint 3 Y coordinate')
    point3_z_arg = DeclareLaunchArgument('point3_z', default_value=point3_z, description='Waypoint 3 Z coordinate')
    point4_x_arg = DeclareLaunchArgument('point4_x', default_value=point4_x, description='Waypoint 4 X coordinate')
    point4_y_arg = DeclareLaunchArgument('point4_y', default_value=point4_y, description='Waypoint 4 Y coordinate')
    point4_z_arg = DeclareLaunchArgument('point4_z', default_value=point4_z, description='Waypoint 4 Z coordinate')
    
    flight_type_arg = DeclareLaunchArgument('flight_type', default_value=flight_type, description='flight_type')
    use_distinctive_trajs_arg = DeclareLaunchArgument('use_distinctive_trajs', default_value=use_distinctive_trajs, description='Use distinctive trajectories')
    obj_num_set_arg = DeclareLaunchArgument('obj_num_set', default_value=obj_num_set, description='Number of objects')
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    planning_frame_arg = DeclareLaunchArgument(
        'planning_frame',
        default_value='world',
        description='Frame used for EGO planning-map and visualization outputs',
    )
    observed_space_enabled_arg = DeclareLaunchArgument(
        'observed_space_enabled',
        default_value=observed_space_enabled,
        description='Opt in to fail-closed recent observed-space planning',
    )
    observed_space_timeout_arg = DeclareLaunchArgument(
        'observed_space_timeout',
        default_value=observed_space_timeout,
        description='Maximum visibility stream age before emergency stop',
    )
    observed_space_retention_arg = DeclareLaunchArgument(
        'observed_space_retention',
        default_value=observed_space_retention,
        description='Recent visibility-ray history retained as observed free',
    )
    observed_space_clearance_arg = DeclareLaunchArgument(
        'observed_space_clearance',
        default_value=observed_space_clearance,
        description='Physical clearance required inside observed free space',
    )
    observed_space_ray_dilation_arg = DeclareLaunchArgument(
        'observed_space_ray_dilation_voxels',
        default_value=observed_space_ray_dilation_voxels,
        description='Raster-only visibility ray tolerance in voxels (0 or 1)',
    )
    observed_space_seed_radius_arg = DeclareLaunchArgument(
        'observed_space_seed_radius',
        default_value=observed_space_seed_radius,
        description='Observed seed sphere around each sensor origin',
    )
    observed_space_target_search_half_angle_deg_arg = DeclareLaunchArgument(
        'observed_space_target_search_half_angle_deg',
        default_value=observed_space_target_search_half_angle_deg,
        description='Half-angle for the observed-space local target direction search (deg, 0 = straight ray only)')
    observed_space_target_search_steps_arg = DeclareLaunchArgument(
        'observed_space_target_search_steps',
        default_value=observed_space_target_search_steps,
        description='Yaw offsets tried per side during the local target direction search')
    observed_space_resolution_arg = DeclareLaunchArgument(
        'observed_space_resolution',
        default_value=observed_space_resolution,
        description='Visibility mask voxel size (m, 0 = grid_map/resolution); must exceed LiDAR beam separation at the planning horizon')
    observed_space_min_update_interval_arg = DeclareLaunchArgument(
        'observed_space_min_update_interval',
        default_value=observed_space_min_update_interval,
        description='Minimum seconds between integrated visibility frames (0 = every frame)')
    stall_commanded_speed_arg = DeclareLaunchArgument(
        'stall_commanded_speed',
        default_value=stall_commanded_speed,
        description='Commanded speed above which a stalled vehicle is judged (m/s)')
    stall_measured_speed_arg = DeclareLaunchArgument(
        'stall_measured_speed',
        default_value=stall_measured_speed,
        description='Measured speed below which the vehicle counts as stalled (m/s)')
    stall_duration_arg = DeclareLaunchArgument(
        'stall_duration',
        default_value=stall_duration,
        description='Seconds of commanded-but-not-moving before latching HOLD')
    observed_space_target_margin_arg = DeclareLaunchArgument(
        'observed_space_target_margin',
        default_value=observed_space_target_margin,
        description='Distance to back off from an unknown local frontier',
    )
    observed_space_validation_step_arg = DeclareLaunchArgument(
        'observed_space_validation_step',
        default_value=observed_space_validation_step,
        description='Spatial sampling step for full B-spline safety validation',
    )
    local_update_range_xy_arg = DeclareLaunchArgument(
        'local_update_range_xy',
        default_value=local_update_range_xy,
        description='Independent-cloud local map half-range in X and Y',
    )
    local_update_range_z_arg = DeclareLaunchArgument(
        'local_update_range_z',
        default_value=local_update_range_z,
        description='Independent-cloud local map half-range in Z',
    )
    max_ray_length_arg = DeclareLaunchArgument(
        'max_ray_length',
        default_value=max_ray_length,
        description='Maximum visibility ray length',
    )
    visibility_cloud_topic_arg = DeclareLaunchArgument(
        'visibility_cloud_topic',
        default_value=visibility_cloud_topic,
        description='Measured visibility endpoint cloud',
    )
    visibility_origin_topic_arg = DeclareLaunchArgument(
        'visibility_origin_topic',
        default_value=visibility_origin_topic,
        description='Same-stamp visibility sensor origin',
    )

    # Ego Planner Node
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name=['drone_', drone_id, '_ego_planner_node'],
        output='screen',
        remappings=[
            ('odom_world', ['drone_', drone_id, '_', odometry_topic]),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']),
            ('planning/cancel', ['drone_', drone_id, '_planning/cancel']),
            ('planning/data_display', ['drone_', drone_id, '_planning/data_display']),
            ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),
            
            ('goal_point', ['drone_', drone_id, '_plan_vis/goal_point']),
            ('global_list', ['drone_', drone_id, '_plan_vis/global_list']),
            ('init_list', ['drone_', drone_id, '_plan_vis/init_list']),
            ('optimal_list', ['drone_', drone_id, '_plan_vis/optimal_list']),
            ('a_star_list', ['drone_', drone_id, '_plan_vis/a_star_list']),
            
            ('grid_map/odom', ['drone_', drone_id, '_', odometry_topic]),
            ('grid_map/cloud', ['drone_', drone_id, '_', cloud_topic]),
            ('grid_map/visibility_cloud', visibility_cloud_topic),
            ('grid_map/visibility_origin', visibility_origin_topic),
            ('grid_map/pose', ['drone_', drone_id, '_', camera_pose_topic]),
            ('grid_map/depth', ['drone_', drone_id, '_', depth_topic]),
            ('grid_map/occupancy_inflate', ['drone_', drone_id, '_grid/grid_map/occupancy_inflate'])
        ],
        parameters=[
            {'fsm/flight_type': flight_type},
            {'fsm/thresh_replan_time': 1.0},
            {'fsm/thresh_no_replan_meter': 1.0},
            {'fsm/planning_horizon': planning_horizon},
            {'fsm/planning_horizen_time': 5.0},
            {'fsm/emergency_time': 1.0},
            {'fsm/realworld_experiment': False},
            {'fsm/fail_safe': True},
            {'fsm/tracking_error_monitor_enabled': tracking_error_monitor_enabled},
            {'fsm/tracking_error_soft_threshold': tracking_error_soft_threshold},
            {'fsm/tracking_error_hard_threshold': tracking_error_hard_threshold},
            {'fsm/tracking_error_soft_duration': tracking_error_soft_duration},
            {'fsm/tracking_error_hard_duration': tracking_error_hard_duration},
            {'fsm/goal_tolerance': goal_tolerance},
            {'fsm/goal_velocity_tolerance': goal_velocity_tolerance},
            {'fsm/stall_commanded_speed': stall_commanded_speed},
            {'fsm/stall_measured_speed': stall_measured_speed},
            {'fsm/stall_duration': stall_duration},
            {'fsm/observed_space_target_margin': observed_space_target_margin},
            {'fsm/observed_space_target_search_half_angle_deg': observed_space_target_search_half_angle_deg},
            {'fsm/observed_space_target_search_steps': observed_space_target_search_steps},

            {'fsm/waypoint_num': point_num},
            {'fsm/waypoint0_x': point0_x},
            {'fsm/waypoint0_y': point0_y},
            {'fsm/waypoint0_z': point0_z},
            {'fsm/waypoint1_x': point1_x},
            {'fsm/waypoint1_y': point1_y},
            {'fsm/waypoint1_z': point1_z},
            {'fsm/waypoint2_x': point2_x},
            {'fsm/waypoint2_y': point2_y},
            {'fsm/waypoint2_z': point2_z},
            {'fsm/waypoint3_x': point3_x},
            {'fsm/waypoint3_y': point3_y},
            {'fsm/waypoint3_z': point3_z},
            {'fsm/waypoint4_x': point4_x},
            {'fsm/waypoint4_y': point4_y},
            {'fsm/waypoint4_z': point4_z},
            
            {'grid_map/resolution': 0.1},
            {'grid_map/map_size_x': map_size_x},
            {'grid_map/map_size_y': map_size_y},
            {'grid_map/map_size_z': map_size_z},
            {'grid_map/local_update_range_x': local_update_range_xy},
            {'grid_map/local_update_range_y': local_update_range_xy},
            {'grid_map/local_update_range_z': local_update_range_z},
            {'grid_map/obstacles_inflation': obstacles_inflation},
            {'grid_map/local_map_margin': 10},
            {'grid_map/ground_height': -0.01},
            # camera parameter
            {'grid_map/cx': cx},
            {'grid_map/cy': cy},
            {'grid_map/fx': fx},
            {'grid_map/fy': fy},
            # depth filter
            {'grid_map/use_depth_filter': True},
            {'grid_map/depth_filter_tolerance': 0.15},
            {'grid_map/depth_filter_maxdist': 5.0},
            {'grid_map/depth_filter_mindist': 0.2},
            {'grid_map/depth_filter_margin': 2},
            {'grid_map/k_depth_scaling_factor': 1000.0},
            {'grid_map/skip_pixel': 2},
            # local fusion
            {'grid_map/p_hit': 0.65},
            {'grid_map/p_miss': 0.35},
            {'grid_map/p_min': 0.12},
            {'grid_map/p_max': 0.90},
            {'grid_map/p_occ': 0.80},
            {'grid_map/min_ray_length': 0.1},
            {'grid_map/max_ray_length': max_ray_length},
            {'grid_map/observed_space_enabled': observed_space_enabled},
            {'grid_map/observed_space_timeout': observed_space_timeout},
            {'grid_map/observed_space_retention': observed_space_retention},
            {'grid_map/observed_space_clearance': observed_space_clearance},
            {'grid_map/observed_space_ray_dilation_voxels': observed_space_ray_dilation_voxels},
            {'grid_map/observed_space_seed_radius': observed_space_seed_radius},
            {'grid_map/observed_space_min_update_interval': observed_space_min_update_interval},
            {'grid_map/observed_space_resolution': observed_space_resolution},
            
            {'grid_map/virtual_ceil_height': virtual_ceil_height},
            {'grid_map/visualization_truncate_height': 1.8},
            {'grid_map/show_occ_time': False},
            {'grid_map/pose_type': 1},
            {'grid_map/frame_id': planning_frame},
            {'visualization/frame_id': planning_frame},
            # planner manager
            {'manager/max_vel': max_vel},
            {'manager/max_acc': max_acc},
            {'manager/max_jerk': 4.0},
            {'manager/control_points_distance': 0.4},
            {'manager/feasibility_tolerance': 0.05},
            {'manager/planning_horizon': planning_horizon},
            {'manager/use_distinctive_trajs': use_distinctive_trajs},
            {'manager/drone_id': drone_id},
            {'manager/observed_space_validation_step': observed_space_validation_step},
            # Trajectory optimization parameters
            {'optimization/lambda_smooth': 1.0},
            {'optimization/lambda_collision': 0.5},
            {'optimization/lambda_feasibility': 0.1},
            {'optimization/lambda_fitness': 1.0},
            {'optimization/dist0': collision_dist0},
            {'optimization/swarm_clearance': swarm_clearance},
            {'optimization/max_vel': max_vel},
            {'optimization/max_acc': max_acc},
            {'optimization/terrain_ref_enabled': terrain_ref_enabled},
            {'optimization/terrain_ref_lambda': terrain_ref_lambda},
            {'optimization/terrain_band_lambda': terrain_band_lambda},
            {'optimization/terrain_min_confidence': terrain_min_confidence},
            {'optimization/terrain_max_profile_age_sec': terrain_max_profile_age_sec},
            {'optimization/terrain_max_lateral_m': terrain_max_lateral_m},
            {'optimization/terrain_agl_tolerance_below': terrain_agl_tolerance_below},
            {'optimization/terrain_agl_tolerance_above': terrain_agl_tolerance_above},
            {'optimization/terrain_agl_floor_soft': terrain_agl_floor_soft},

            # B-Spline parameters
            {'bspline/limit_vel': max_vel},
            {'bspline/limit_acc': max_acc},
            {'bspline/limit_ratio': 1.1},

            # Object prediction parameters
            {'prediction/obj_num': obj_num_set},
            {'prediction/lambda': 1.0},
            {'prediction/predict_rate': 1.0}
        ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()

    # Add LaunchArguments
    ld.add_action(map_size_x_arg)
    ld.add_action(map_size_y_arg)
    ld.add_action(map_size_z_arg)
    ld.add_action(virtual_ceil_height_arg)
    ld.add_action(odometry_topic_arg)
    ld.add_action(camera_pose_topic_arg)
    ld.add_action(depth_topic_arg)
    ld.add_action(cloud_topic_arg)
    ld.add_action(cx_arg)
    ld.add_action(cy_arg)
    ld.add_action(fx_arg)
    ld.add_action(fy_arg)
    ld.add_action(max_vel_arg)
    ld.add_action(max_acc_arg)
    ld.add_action(planning_horizon_arg)
    ld.add_action(obstacles_inflation_arg)
    ld.add_action(collision_dist0_arg)
    ld.add_action(swarm_clearance_arg)
    ld.add_action(tracking_error_monitor_enabled_arg)
    ld.add_action(tracking_error_soft_threshold_arg)
    ld.add_action(tracking_error_hard_threshold_arg)
    ld.add_action(tracking_error_soft_duration_arg)
    ld.add_action(tracking_error_hard_duration_arg)
    ld.add_action(goal_tolerance_arg)
    ld.add_action(goal_velocity_tolerance_arg)
    ld.add_action(terrain_ref_enabled_arg)
    ld.add_action(terrain_ref_lambda_arg)
    ld.add_action(terrain_band_lambda_arg)
    ld.add_action(terrain_min_confidence_arg)
    ld.add_action(terrain_max_profile_age_arg)
    ld.add_action(terrain_max_lateral_arg)
    ld.add_action(terrain_agl_below_arg)
    ld.add_action(terrain_agl_above_arg)
    ld.add_action(terrain_agl_floor_soft_arg)

    ld.add_action(point_num_arg)
    ld.add_action(point0_x_arg)
    ld.add_action(point0_y_arg)
    ld.add_action(point0_z_arg)
    ld.add_action(point1_x_arg)
    ld.add_action(point1_y_arg)
    ld.add_action(point1_z_arg)
    ld.add_action(point2_x_arg)
    ld.add_action(point2_y_arg)
    ld.add_action(point2_z_arg)
    ld.add_action(point3_x_arg)
    ld.add_action(point3_y_arg)
    ld.add_action(point3_z_arg)
    ld.add_action(point4_x_arg)
    ld.add_action(point4_y_arg)
    ld.add_action(point4_z_arg)
    
    ld.add_action(flight_type_arg)
    ld.add_action(use_distinctive_trajs_arg)
    ld.add_action(obj_num_set_arg)
    ld.add_action(drone_id_arg)
    ld.add_action(planning_frame_arg)
    ld.add_action(observed_space_enabled_arg)
    ld.add_action(observed_space_timeout_arg)
    ld.add_action(observed_space_retention_arg)
    ld.add_action(observed_space_clearance_arg)
    ld.add_action(observed_space_ray_dilation_arg)
    ld.add_action(observed_space_seed_radius_arg)
    ld.add_action(observed_space_target_search_half_angle_deg_arg)
    ld.add_action(observed_space_target_search_steps_arg)
    ld.add_action(observed_space_resolution_arg)
    ld.add_action(observed_space_min_update_interval_arg)
    ld.add_action(stall_commanded_speed_arg)
    ld.add_action(stall_measured_speed_arg)
    ld.add_action(stall_duration_arg)
    ld.add_action(observed_space_target_margin_arg)
    ld.add_action(observed_space_validation_step_arg)
    ld.add_action(local_update_range_xy_arg)
    ld.add_action(local_update_range_z_arg)
    ld.add_action(max_ray_length_arg)
    ld.add_action(visibility_cloud_topic_arg)
    ld.add_action(visibility_origin_topic_arg)


    # Add Node
    ld.add_action(ego_planner_node)

    return ld
