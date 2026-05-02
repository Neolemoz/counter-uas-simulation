from launch import LaunchDescription
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
                default_value='tracks',
                description=(
                    'interception_logic_node input: ground_truth | fused | tracks | tracks_state '
                    '(full stack defaults to tracks).'
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
            gazebo_target,
            Node(
                package='radar_sim',
                executable='radar_sim_node',
                name='radar_sim_node',
                output='screen',
                parameters=params,
            ),
            Node(
                package='camera_sim',
                executable='camera_sim_node',
                name='camera_sim_node',
                output='screen',
                parameters=params,
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
        ],
    )
