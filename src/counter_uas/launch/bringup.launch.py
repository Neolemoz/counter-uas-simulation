from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution(
        [FindPackageShare('counter_uas'), 'config', 'config.yaml'],
    )
    params = [ParameterFile(config, allow_substs=True)]

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
    )

    return LaunchDescription(
        [
            # Synthetic kinematic target replaced by Gazebo (see gazebo_target_sim).
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
                package='visualization',
                executable='viz_node',
                name='viz_node',
                output='screen',
                parameters=params,
            ),
        ],
    )
