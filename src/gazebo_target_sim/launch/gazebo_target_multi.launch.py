"""Three hostile spheres + TTI bipartite assignment; /drone_i/position and per-target stop topics."""

from __future__ import annotations

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _target_speed_peak_m_s(vx: float, vz_approach: float, vz_dive: float) -> float:
    return max(math.hypot(vx, vz_approach), math.hypot(vx, vz_dive))


def _interception_from_target(
    vx: float,
    vz_approach: float,
    vz_dive: float,
    *,
    closing_margin_over_target_m_s: float,
    vmax_above_closing_m_s: float,
    interceptor_above_guidance_vmax_m_s: float,
) -> tuple[float, float, float, float]:
    v_peak = _target_speed_peak_m_s(vx, vz_approach, vz_dive)
    closing = max(8.0, v_peak + closing_margin_over_target_m_s)
    vmax = closing + vmax_above_closing_m_s
    interceptor_max = vmax + interceptor_above_guidance_vmax_m_s
    return v_peak, closing, vmax, interceptor_max


def _vel_smooth_alpha(interceptor_max_m_s: float, v_target_peak: float) -> float:
    headroom = max(0.0, interceptor_max_m_s - v_target_peak)
    a = 0.38 + 0.035 * headroom
    return float(min(0.88, max(0.32, a)))


TGT_APPROACH_VX_M_S = 0.0
TGT_APPROACH_VZ_M_S = -0.28
TGT_DIVE_VZ_M_S = -0.45
TGT_T_DIV_S = 28.0
STRIKE_SHELL_HALF_WIDTH_M = 10.0
CLOSING_MARGIN_OVER_TARGET_M_S = 6.0
VMAX_ABOVE_CLOSING_M_S = 1.0
INTERCEPTOR_ABOVE_GUIDANCE_VMAX_M_S = 3.0
DOME_OUTER_M = 36.0
DOME_MIDDLE_M = 26.0
DOME_INNER_M = 17.0
INTERCEPTOR_GROUND_Z_M = 0.0


def _gz_multi_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world_name').perform(context)
    world_file = PathJoinSubstitution(
        [FindPackageShare('gazebo_target_sim'), 'worlds', 'target_sphere.sdf'],
    ).perform(context)

    _v_peak, guidance_closing, guidance_vmax, interceptor_vmax = _interception_from_target(
        TGT_APPROACH_VX_M_S,
        TGT_APPROACH_VZ_M_S,
        TGT_DIVE_VZ_M_S,
        closing_margin_over_target_m_s=CLOSING_MARGIN_OVER_TARGET_M_S,
        vmax_above_closing_m_s=VMAX_ABOVE_CLOSING_M_S,
        interceptor_above_guidance_vmax_m_s=INTERCEPTOR_ABOVE_GUIDANCE_VMAX_M_S,
    )
    vel_smooth_alpha = _vel_smooth_alpha(interceptor_vmax, _v_peak)

    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_clock_bridge',
        output='screen',
        arguments=[
            f'/world/{world_name}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]',
        ],
    )

    targets_cfg = [
        ('sphere_target_0', 0.0, 0.0, 36.0, '/drone_0/position', '/target_0/stop'),
        ('sphere_target_1', -6.0, 5.0, 34.0, '/drone_1/position', '/target_1/stop'),
        ('sphere_target_2', 6.0, -5.0, 32.0, '/drone_2/position', '/target_2/stop'),
    ]
    target_nodes = []
    for idx, (model_name, sx, sy, sz, pos_topic, stop_topic) in enumerate(targets_cfg):
        target_nodes.append(
            Node(
                package='gazebo_target_sim',
                executable='target_controller_node',
                name=f'target_controller_{idx}',
                output='screen',
                parameters=[
                    {
                        'world_name': world_name,
                        'model_name': model_name,
                        'rate_hz': 10.0,
                        'start_x_m': sx,
                        'start_y_m': sy,
                        'start_z_m': sz,
                        't_dive_s': TGT_T_DIV_S,
                        'approach_speed': TGT_APPROACH_VX_M_S,
                        'approach_vz': TGT_APPROACH_VZ_M_S,
                        'dive_speed': TGT_DIVE_VZ_M_S,
                        'log_period_s': 1.0,
                        'publish_drone_position': True,
                        'drone_position_topic': pos_topic,
                        'stop_topic': stop_topic,
                        'explode_on_hit': True,
                        'explosion_fade_s': 4.0,
                        'service_timeout_ms': 5000,
                    },
                ],
            ),
        )

    interceptors_cfg = [
        ('interceptor_0', -5.0, 0.0, INTERCEPTOR_GROUND_Z_M),
        ('interceptor_1', 4.0, -4.0, INTERCEPTOR_GROUND_Z_M),
        ('interceptor_2', -4.0, 5.0, INTERCEPTOR_GROUND_Z_M),
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
                        'assigned_target_topic': f'/{model_name}/assigned_target',
                        'cmd_timeout_s': 0.75,
                        'max_speed_m_s': interceptor_vmax,
                        'position_topic': f'/{model_name}/position',
                        'marker_topic': f'/{model_name}/marker',
                        'publish_marker': True,
                        'marker_scale': 1.15,
                        'vel_smooth_alpha': max(vel_smooth_alpha, 0.88),
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
                'rate_hz': 10.0,
                'closing_speed_m_s': guidance_closing,
                'max_speed_m_s': guidance_vmax,
                'min_distance_m': 0.22,
                'log_period_s': 1.0,
                'selection_log_period_s': 2.0,
                'selection_algo_verbose': False,
                'selection_algo_period_s': 1.0,
                'selection_margin_s': 0.3,
                'switch_window_s': 2.0,
                'lost_timeout_s': 1.5,
                'reacquire_confirm_s': 0.25,
                'max_intercept_time_s': 90.0,
                'min_intercept_time_s': 0.02,
                'target_topic': '/drone/position',
                'selected_id_topic': '/interceptor/selected_id',
                'lock_selected_after_first': False,
                'hit_threshold_m': 4.5,
                'hit_min_target_z_m': 0.5,
                'aim_strike_on_mid_shell': True,
                'target_velocity_smooth_alpha': 0.52,
                'pursuit_lead_blend': 0.28,
                'world_name': world_name,
                # Continuous engagement: do not freeze sim after a kill; other tracks keep running.
                'pause_gz_on_hit': False,
                'strike_shell_half_width_m': STRIKE_SHELL_HALF_WIDTH_M,
                'stop_topic': '/target/stop',
                'interceptor_ids': ['interceptor_0', 'interceptor_1', 'interceptor_2'],
                'use_pn_refinement': False,
                'pn_navigation_constant': 3.0,
                'pn_blend_gain': 0.0,
                'pn_min_closing_speed_m_s': 0.15,
                'naive_lead_time_s': 0.85,
                'dome_enabled': True,
                'dome_center_x': 0.0,
                'dome_center_y': 0.0,
                'dome_center_z': 0.0,
                'dome_outer_m': DOME_OUTER_M,
                'dome_middle_m': DOME_MIDDLE_M,
                'dome_inner_m': DOME_INNER_M,
                'reset_lock_when_outside_dome': True,
                'dome_selection_mode': 'nearest',
                'publish_dome_rviz_marker': True,
                'multi_target_enabled': True,
                'multi_target_labels': ['target_0', 'target_1', 'target_2'],
                'multi_target_position_topics': [
                    '/drone_0/position',
                    '/drone_1/position',
                    '/drone_2/position',
                ],
                'multi_target_stop_topics': [
                    '/target_0/stop',
                    '/target_1/stop',
                    '/target_2/stop',
                ],
                'assignment_print_period_s': 1.0,
                'threat_weight_dist': 1.0,
                'threat_weight_vz': 0.05,
                'threat_weight_tti': 1.0,
                'threat_distance_eps_m': 1.0,
                'threat_tti_fallback_s': 120.0,
                'threat_dive_speed_threshold_m_s': 0.35,
                'threat_dive_boost': 6.0,
                'threat_critical_radius_m': 14.0,
                'threat_critical_boost': 50.0,
                'assignment_stability_enabled': True,
                'assignment_switch_tti_margin_s': 1.2,
            },
        ],
    )

    delayed = TimerAction(
        period=0.5,
        actions=[*target_nodes, *interceptor_nodes, interception],
    )
    return [gz, clock_bridge, delayed]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world_name',
                default_value='counter_uas_target',
                description='Must match <world name="..."> in target_sphere.sdf.',
            ),
            OpaqueFunction(function=_gz_multi_setup),
        ],
    )
