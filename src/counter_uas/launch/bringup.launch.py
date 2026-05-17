from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = PathJoinSubstitution(
        [
            FindPackageShare('counter_uas'),
            'config',
            LaunchConfiguration('counter_uas_config'),
        ],
    )
    params = [ParameterFile(config_file, allow_substs=True)]

    gazebo_target = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('gazebo_target_sim'),
                        'launch',
                        'gazebo_target.launch.py',
                    ],
                ),
            ],
        ),
        launch_arguments={
            'intercept_measurement_source': LaunchConfiguration('intercept_measurement_source'),
            'fused_detections_topic': LaunchConfiguration('fused_detections_topic'),
            'tracks_topic': LaunchConfiguration('tracks_topic'),
            'tracks_state_topic': LaunchConfiguration('tracks_state_topic'),
            'use_noisy_measurement': LaunchConfiguration('use_noisy_measurement'),
            'ground_truth_topic': LaunchConfiguration('ground_truth_topic'),
            'noisy_topic': LaunchConfiguration('noisy_topic'),
            'noise_seed': LaunchConfiguration('noise_seed'),
            'noise_rate_hz': LaunchConfiguration('noise_rate_hz'),
            'ghost_detection_prob': LaunchConfiguration('ghost_detection_prob'),
            'ghost_placement_mode': LaunchConfiguration('ghost_placement_mode'),
            'ghost_near_threshold_xy_min_m': LaunchConfiguration('ghost_near_threshold_xy_min_m'),
            'ghost_near_threshold_xy_max_m': LaunchConfiguration('ghost_near_threshold_xy_max_m'),
            'ghost_persistence_ticks': LaunchConfiguration('ghost_persistence_ticks'),
            'fragmentation_staggered_enabled': LaunchConfiguration('fragmentation_staggered_enabled'),
            'fragmentation_stagger_cycle_ticks': LaunchConfiguration('fragmentation_stagger_cycle_ticks'),
            'fragmentation_stagger_phase_ticks': LaunchConfiguration('fragmentation_stagger_phase_ticks'),
            'fragmentation_stagger_gap_ticks': LaunchConfiguration('fragmentation_stagger_gap_ticks'),
            'target_start_x_m': LaunchConfiguration('target_start_x_m'),
            'target_start_y_m': LaunchConfiguration('target_start_y_m'),
            'target_start_z_m': LaunchConfiguration('target_start_z_m'),
            # Forward through so one ``ros2 launch counter_uas bringup`` line can toggle
            # Gazebo GUI and RViz (defaults match gazebo_target.launch.py).
            'use_gazebo_gui': LaunchConfiguration('use_gazebo_gui'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'counter_uas_config',
                default_value='config.yaml',
                description=(
                    'YAML under share/counter_uas/config/ (e.g. config.yaml, config_gazebo_counter_uas.yaml, '
                    'config_lab_toy.yaml).'
                ),
            ),
            DeclareLaunchArgument(
                'intercept_measurement_source',
                default_value='tracks_state',
                description=(
                    'interception_logic_node input: ground_truth | fused | tracks | tracks_state '
                    '(full stack defaults to tracks_state).'
                ),
            ),
            DeclareLaunchArgument(
                'fused_detections_topic',
                default_value='/fused_detections',
                description='Point topic when intercept_measurement_source:=fused.',
            ),
            DeclareLaunchArgument(
                'tracks_topic',
                default_value='/tracks',
                description='Point topic when intercept_measurement_source:=tracks.',
            ),
            DeclareLaunchArgument(
                'tracks_state_topic',
                default_value='/tracks/state',
                description='Odometry topic when intercept_measurement_source:=tracks_state.',
            ),
            DeclareLaunchArgument(
                'use_noisy_measurement',
                default_value='false',
                description='Forwarded to gazebo_target.launch.py; enables noisy_measurement_node when true.',
            ),
            DeclareLaunchArgument(
                'ground_truth_topic',
                default_value='/drone/position',
                description='Ground-truth target Point topic from gazebo_target.launch.py.',
            ),
            DeclareLaunchArgument(
                'noisy_topic',
                default_value='/drone/position_noisy',
                description='Noisy measurement Point topic from gazebo_target.launch.py.',
            ),
            DeclareLaunchArgument(
                'sensor_input_topic',
                default_value='/drone/position',
                description=(
                    'Input topic for radar/camera sensing path. Leave at /drone/position for legacy behavior; '
                    'set to /drone/position_noisy to propagate additive ambiguity overlays into /fused_detections and /tracks/state.'
                ),
            ),
            DeclareLaunchArgument(
                'noise_seed',
                default_value='0',
                description='Forwarded noise seed for deterministic noisy_measurement overlay runs.',
            ),
            DeclareLaunchArgument(
                'noise_rate_hz',
                default_value='10.0',
                description='Forwarded noisy_measurement publish cadence (Hz).',
            ),
            DeclareLaunchArgument(
                'ghost_detection_prob',
                default_value='0.0',
                description='Forwarded noisy_measurement ghost injection probability.',
            ),
            DeclareLaunchArgument(
                'ghost_placement_mode',
                default_value='broad',
                description='Forwarded ghost placement mode: broad or near_threshold.',
            ),
            DeclareLaunchArgument(
                'ghost_near_threshold_xy_min_m',
                default_value='18.0',
                description='Forwarded minimum XY offset (m) for near-threshold ghost placement.',
            ),
            DeclareLaunchArgument(
                'ghost_near_threshold_xy_max_m',
                default_value='24.0',
                description='Forwarded maximum XY offset (m) for near-threshold ghost placement.',
            ),
            DeclareLaunchArgument(
                'ghost_persistence_ticks',
                default_value='1',
                description='Forwarded number of adjacent noisy ticks to repeat a sampled ghost offset.',
            ),
            DeclareLaunchArgument(
                'fragmentation_staggered_enabled',
                default_value='false',
                description='Forwarded deterministic staggered fragmentation toggle.',
            ),
            DeclareLaunchArgument(
                'fragmentation_stagger_cycle_ticks',
                default_value='6',
                description='Forwarded stagger cycle length (ticks).',
            ),
            DeclareLaunchArgument(
                'fragmentation_stagger_phase_ticks',
                default_value='1',
                description='Forwarded stagger phase offset (ticks).',
            ),
            DeclareLaunchArgument(
                'fragmentation_stagger_gap_ticks',
                default_value='2',
                description='Forwarded staggered fragmentation gap length (ticks).',
            ),
            DeclareLaunchArgument(
                'target_start_x_m',
                default_value='-2400.0',
                description='Forwarded target sphere initial X (m) for reachable bringup profiles.',
            ),
            DeclareLaunchArgument(
                'target_start_y_m',
                default_value='1680.0',
                description='Forwarded target sphere initial Y (m) for reachable bringup profiles.',
            ),
            DeclareLaunchArgument(
                'target_start_z_m',
                default_value='1650.0',
                description='Forwarded target sphere initial Z (m) for reachable bringup profiles.',
            ),
            DeclareLaunchArgument(
                'use_gazebo_gui',
                default_value='true',
                description=(
                    'Passed to gazebo_target.launch.py — false runs gz server-only (-s); '
                    'true opens the 3D client (needs DISPLAY/Wayland).'
                ),
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='false',
                description=(
                    'Passed to gazebo_target.launch.py — if true, starts rviz2 with '
                    'gazebo_target_sim/rviz/hit_overlay.rviz.'
                ),
            ),
            DeclareLaunchArgument(
                'enable_lifecycle_observer',
                default_value='false',
                description=(
                    'Default-off passive observer for /tracks/state and optional selection evidence in the '
                    'real bringup topology.'
                ),
            ),
            gazebo_target,
            Node(
                package='radar_sim',
                executable='radar_sim_node',
                name='radar_sim_node',
                output='screen',
                parameters=params,
                remappings=[('/drone/position', LaunchConfiguration('sensor_input_topic'))],
            ),
            Node(
                package='camera_sim',
                executable='camera_sim_node',
                name='camera_sim_node',
                output='screen',
                parameters=params,
                remappings=[('/drone/position', LaunchConfiguration('sensor_input_topic'))],
            ),
            Node(
                package='fusion',
                executable='fusion_node',
                name='fusion_node',
                output='screen',
                parameters=params,
            ),
            Node(
                package='tracking',
                executable='tracking_node',
                name='tracking_node',
                output='screen',
                parameters=params,
            ),
            Node(
                package='threat_assessment',
                executable='threat_assessment_node',
                name='threat_assessment_node',
                output='screen',
                parameters=params,
            ),
            Node(
                package='visualization',
                executable='viz_node',
                name='viz_node',
                output='screen',
                parameters=params,
            ),
            Node(
                package='counter_uas',
                executable='lifecycle_observer_node',
                name='lifecycle_observer_node',
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_lifecycle_observer')),
                parameters=params,
            ),
        ],
    )
