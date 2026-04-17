from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution(
        [FindPackageShare('counter_uas'), 'config', 'config.yaml'],
    )
    params = [ParameterFile(config, allow_substs=True)]

    return LaunchDescription(
        [
            Node(
                package='world_sim',
                executable='world_sim_node',
                name='world_sim_node',
                output='screen',
                parameters=params,
            ),
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
