"""Start gz sim + target / multi-interceptor kinematics; /drone/position from target_controller (ground truth)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _gz_target_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world_name').perform(context)
    world_file = PathJoinSubstitution(
        [FindPackageShare('gazebo_target_sim'), 'worlds', 'target_sphere.sdf'],
    ).perform(context)

    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
    )

    controller = Node(
        package='gazebo_target_sim',
        executable='target_controller_node',
        name='target_controller_node',
        output='screen',
        parameters=[
            {
                'world_name': world_name,
                'model_name': 'sphere_target',
                'rate_hz': 10.0,
                'center_x': 0.0,
                'center_y': 0.0,
                'radius_m': 10.0,
                'orbit_speed': 3.0,
                'z_base_m': 5.0,
                'z_amplitude_m': 0.5,
                'z_omega_rad_s': 0.55,
                'publish_drone_position': True,
                'drone_position_topic': '/drone/position',
                'stop_topic': '/target/stop',
            },
        ],
    )

    interceptors_cfg = [
        ('interceptor_0', -15.0, 0.0, 0.0),
        ('interceptor_1', 12.0, -12.0, 0.0),
        ('interceptor_2', -5.0, 12.0, 0.0),
    ]
    interceptor_nodes = []
    for idx, (model_name, ox, oy, oz) in enumerate(interceptors_cfg):
        interceptor_nodes.append(
            Node(
                package='gazebo_target_sim',
                executable='interceptor_controller_node',
                name=f'interceptor_ctrl_{idx}',
                output='screen',
                parameters=[
                    {
                        'world_name': world_name,
                        'model_name': model_name,
                        'rate_hz': 10.0,
                        'origin_x': ox,
                        'origin_y': oy,
                        'origin_z': oz,
                        'vel_x': 0.0,
                        'vel_y': 0.0,
                        'vel_z': 0.0,
                        'cmd_velocity_topic': f'/{model_name}/cmd_velocity',
                        'selected_id_topic': '/interceptor/selected_id',
                        'cmd_timeout_s': 0.75,
                        'max_speed_m_s': 8.0,
                        'position_topic': f'/{model_name}/position',
                        'marker_topic': f'/{model_name}/marker',
                        'publish_marker': True,
                    },
                ],
            ),
        )

    interception = Node(
        package='gazebo_target_sim',
        executable='interception_logic_node',
        name='interception_logic_node',
        output='screen',
        parameters=[
            {
                'rate_hz': 20.0,
                'closing_speed_m_s': 4.5,
                'max_speed_m_s': 7.5,
                'min_distance_m': 0.45,
                'log_period_s': 1.0,
                'selection_log_period_s': 2.0,
                'selection_margin_s': 0.3,
                'switch_window_s': 2.0,
                'lost_timeout_s': 1.5,
                'reacquire_confirm_s': 0.25,
                'max_intercept_time_s': 90.0,
                'min_intercept_time_s': 0.02,
                'target_topic': '/drone/position',
                'selected_id_topic': '/interceptor/selected_id',
                'lock_selected_after_first': True,
                'hit_threshold_m': 7.5,
                'stop_topic': '/target/stop',
                'interceptor_ids': ['interceptor_0', 'interceptor_1', 'interceptor_2'],
                'use_pn_refinement': True,
                'pn_navigation_constant': 3.0,
                'pn_blend_gain': 0.22,
                'pn_min_closing_speed_m_s': 0.15,
            },
        ],
    )

    delayed = TimerAction(
        period=3.0,
        actions=[controller, *interceptor_nodes, interception],
    )
    return [gz, delayed]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world_name',
                default_value='counter_uas_target',
                description='Must match <world name="..."> in target_sphere.sdf.',
            ),
            DeclareLaunchArgument(
                'model_child_frame',
                default_value='',
                description='Unused: /drone/position is published by target_controller_node (ground truth).',
            ),
            OpaqueFunction(function=_gz_target_setup),
        ],
    )
