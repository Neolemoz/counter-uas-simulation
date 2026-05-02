"""Three hostile spheres + TTI bipartite assignment; /drone_i/position and per-target stop topics."""

from __future__ import annotations

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, SetEnvironmentVariable, TimerAction
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


def _get_gz_ip() -> str:
    import socket as _s

    try:
        s = _s.socket(_s.AF_INET, _s.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        if not ip.startswith('127.') and not ip.startswith('169.254.'):
            return ip
    except Exception:
        pass
    return '127.0.0.1'


def _apply_gz_ip_from_launch(context, *args, **kwargs):
    v = str(LaunchConfiguration('gz_transport_ip').perform(context)).strip()
    if not v or v.lower() == 'auto':
        ip = _get_gz_ip()
    else:
        ip = v
    return [
        SetEnvironmentVariable('GZ_IP', ip),
        LogInfo(
            msg=(
                f'gazebo_target_multi: GZ_IP={ip} '
                f'(gz_transport_ip={"auto" if not v or v.lower() == "auto" else v}).'
            )
        ),
    ]


TGT_APPROACH_VX_M_S = 38.0
TGT_LOS_CLOSING_SPEED_M_S = 38.0
TGT_LOS_DIVE_GAIN = 1.35
TGT_APPROACH_VZ_M_S = -2.0
TGT_DIVE_VZ_M_S = -5.0
TGT_T_DIV_S = 40.0
STRIKE_SHELL_HALF_WIDTH_M = 75.0
CLOSING_MARGIN_OVER_TARGET_M_S = 18.0
VMAX_ABOVE_CLOSING_M_S = 8.0
INTERCEPTOR_ABOVE_GUIDANCE_VMAX_M_S = 14.0
DOME_OUTER_M = 6000.0
DOME_MIDDLE_M = 3000.0
DOME_INNER_M = 1200.0
DOME_OUTER_HYSTERESIS_M = 150.0
INTERCEPTOR_GROUND_Z_M = 0.0


def _gz_multi_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world_name').perform(context)
    use_noisy = str(LaunchConfiguration('use_noisy_measurement').perform(context)).strip().lower() in (
        '1',
        'true',
        'yes',
        'on',
    )
    noise_std_m = float(LaunchConfiguration('noise_std_m').perform(context))
    dropout_prob = float(LaunchConfiguration('dropout_prob').perform(context))
    noise_seed = int(float(LaunchConfiguration('noise_seed').perform(context)))
    noise_rate_hz = float(LaunchConfiguration('noise_rate_hz').perform(context))
    noisy_suffix = str(LaunchConfiguration('noisy_suffix').perform(context)).strip() or '_noisy'
    world_file = PathJoinSubstitution(
        [FindPackageShare('gazebo_target_sim'), 'worlds', 'target_sphere_multi.sdf'],
    ).perform(context)

    eng_preset = str(LaunchConfiguration('cuas_engagement_preset').perform(context)).strip().lower()
    if eng_preset in ('doctrine', 'mid', 'select', 'layer2', 'commit'):
        guidance_layer = 'select'
        engagement_layer = 'select'
    else:
        guidance_layer = 'detect'
        engagement_layer = 'detect'

    tgt_los = float(LaunchConfiguration('target_los_closing_m_s').perform(context))
    tgt_dive_gain = float(LaunchConfiguration('target_los_dive_gain').perform(context))
    tgt_vx = float(LaunchConfiguration('target_approach_vx_m_s').perform(context))
    tgt_vz_ap = float(LaunchConfiguration('target_approach_vz_m_s').perform(context))
    tgt_vz_dive = float(LaunchConfiguration('target_dive_vz_m_s').perform(context))
    closing_margin = float(LaunchConfiguration('closing_margin_over_target_m_s').perform(context))
    vmax_above = float(LaunchConfiguration('vmax_above_closing_m_s').perform(context))
    interceptor_headroom = float(LaunchConfiguration('interceptor_above_guidance_vmax_m_s').perform(context))
    use_rviz = str(LaunchConfiguration('use_rviz').perform(context)).strip().lower() in (
        '1',
        'true',
        'yes',
        'on',
    )

    _v_peak, guidance_closing, guidance_vmax, interceptor_vmax = _interception_from_target(
        tgt_vx,
        tgt_vz_ap,
        tgt_vz_dive,
        closing_margin_over_target_m_s=closing_margin,
        vmax_above_closing_m_s=vmax_above,
        interceptor_above_guidance_vmax_m_s=interceptor_headroom,
    )
    vel_smooth_alpha = _vel_smooth_alpha(interceptor_vmax, _v_peak)

    heatmap_export_dir = str(
        LaunchConfiguration('intercept_heatmap_prob_export_dir').perform(context),
    ).strip()
    heatmap_export_stamp = str(
        LaunchConfiguration('intercept_heatmap_prob_export_stamp_files').perform(context),
    ).strip().lower() in ('1', 'true', 'yes', 'on')
    heatmap_prob_use_cmd_vel = str(
        LaunchConfiguration('intercept_heatmap_prob_use_cmd_vel').perform(context),
    ).strip().lower() in ('1', 'true', 'yes', 'on')
    meas_src = str(LaunchConfiguration('intercept_measurement_source').perform(context)).strip().lower()
    fused_topic_arg = str(LaunchConfiguration('fused_detections_topic').perform(context)).strip() or '/fused_detections'
    tracks_topic_arg = str(LaunchConfiguration('tracks_topic').perform(context)).strip() or '/tracks'

    dome_hyst = float(LaunchConfiguration('dome_outer_hysteresis_m').perform(context))
    tv_alpha = float(LaunchConfiguration('target_velocity_smooth_alpha').perform(context))
    dome_en = str(LaunchConfiguration('dome_enabled').perform(context)).strip().lower() in (
        '1',
        'true',
        'yes',
        'on',
    )

    use_gui = str(LaunchConfiguration('use_gazebo_gui').perform(context)).strip().lower() in (
        '1', 'true', 'yes', 'on',
    )
    use_shipped_gui = str(LaunchConfiguration('gz_use_shipped_gui_config').perform(context)).strip().lower() in (
        '1',
        'true',
        'yes',
        'on',
    )
    if use_gui:
        gz_cmd: list[str] = ['gz', 'sim', '-r']
        if use_shipped_gui:
            _gui = os.path.join(
                get_package_share_directory('gazebo_target_sim'),
                'gui',
                'cuas_gz_gui.config',
            )
            gz_cmd.extend(['--gui-config', _gui])
        gz_cmd.append(world_file)
    else:
        gz_cmd = ['gz', 'sim', '-s', '-r', world_file]
    gz_ip_child = str(LaunchConfiguration('gz_transport_ip').perform(context)).strip()
    if not gz_ip_child or gz_ip_child.lower() == 'auto':
        gz_ip_child = _get_gz_ip()
    gz = ExecuteProcess(
        cmd=gz_cmd,
        output='screen',
        additional_env={'GZ_IP': gz_ip_child},
    )
    gz_argv_log = LogInfo(
        msg=(
            f'[gazebo_target_multi] gz sim command: {" ".join(gz_cmd)}'
            f' | GZ_IP={gz_ip_child}'
        ),
    )
    gz_empty_tree_hint = LogInfo(
        msg=(
            '[gazebo_target_multi] Empty Entity tree usually means Gazebo was opened without this launch '
            '(e.g. app menu). Use: ros2 launch gazebo_target_sim gazebo_target_multi.launch.py. '
            'If still gray: mv ~/.gz/sim ~/.gz/sim.bak && relaunch. Press Play if paused.'
        ),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_clock_bridge',
        output='screen',
        arguments=[
            f'/world/{world_name}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[(f'/world/{world_name}/clock', '/clock')],
    )

    map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    targets_cfg = [
        ('sphere_target_0', 0.0, 0.0, 36.0, '/drone_0/position', '/target_0/stop'),
        ('sphere_target_1', -6.0, 5.0, 34.0, '/drone_1/position', '/target_1/stop'),
        ('sphere_target_2', 6.0, -5.0, 32.0, '/drone_2/position', '/target_2/stop'),
    ]
    target_nodes = []
    noise_nodes = []
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
                        'approach_speed': tgt_vx,
                        'approach_vz': tgt_vz_ap,
                        'dive_speed': tgt_vz_dive,
                        'log_period_s': 1.0,
                        'publish_drone_position': True,
                        'drone_position_topic': pos_topic,
                        'stop_topic': stop_topic,
                        'explode_on_hit': True,
                        'explosion_fade_s': 12.0,
                        'remove_model_on_hit': False,
                        'service_timeout_ms': 5000,
                        'reset_on_sim_clock_rewind': use_gui,
                        'attack_los_dive_gain': tgt_dive_gain,
                    },
                ],
            ),
        )
        if use_noisy:
            noise_nodes.append(
                Node(
                    package='gazebo_target_sim',
                    executable='noisy_measurement_node',
                    name=f'noisy_measurement_{idx}',
                    output='screen',
                    parameters=[
                        {
                            'input_topic': pos_topic,
                            'output_topic': f'{pos_topic}{noisy_suffix}',
                            'rate_hz': noise_rate_hz,
                            'noise_std_m': noise_std_m,
                            'dropout_prob': dropout_prob,
                            'seed': noise_seed + idx,
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
                        'reset_on_sim_clock_rewind': use_gui,
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
                'max_speed_m_s': interceptor_vmax,
                'adaptive_speed_enabled': True,
                'adaptive_speed_gain': 0.25,
                'guidance_start_layer': guidance_layer,
                'engagement_layer': engagement_layer,
                'hit_outer_layer_only': False,
                'min_distance_m': 0.22,
                'log_period_s': 1.0,
                'selection_log_period_s': 2.0,
                'selection_algo_verbose': False,
                'selection_algo_period_s': 1.0,
                'selection_margin_s': 0.3,
                'switch_window_s': 2.0,
                'lost_timeout_s': 1.5,
                'reacquire_confirm_s': 0.25,
                'max_intercept_time_s': 300.0,
                'min_intercept_time_s': 0.02,
                'target_topic': '/drone/position',
                'intercept_measurement_source': meas_src,
                'fused_detections_topic': fused_topic_arg,
                'tracks_topic': tracks_topic_arg,
                'selected_id_topic': '/interceptor/selected_id',
                'lock_selected_after_first': False,
                'hit_threshold_m': 4.5,
                'hit_min_target_z_m': 0.5,
                'aim_strike_on_mid_shell': True,
                'target_velocity_smooth_alpha': tv_alpha,
                'pursuit_lead_blend': 0.28,
                'world_name': world_name,
                # Continuous engagement: do not freeze sim after a kill; other tracks keep running.
                'pause_gz_on_hit': False,
                'strike_shell_half_width_m': STRIKE_SHELL_HALF_WIDTH_M,
                'stop_topic': '/target/stop',
                # Phase 2 → Phase 3 gating (same semantics as single-target launch).
                'sensing_gate_enabled': False,
                'radar_position_x': 0.0,
                'radar_position_y': 0.0,
                'radar_position_z': 0.0,
                'radar_range_m': 6500.0,
                'tracking_delay_s': 2.0,
                'feasibility_based_engagement': True,
                'classification_gating_enabled': True,
                'classification_confidence_threshold': 0.55,
                'classification_placeholder_confidence': 1.0,
                'feasibility_log_period_s': 1.0,
                'interceptor_max_speed_m_s': interceptor_vmax,
                'interceptor_ids': ['interceptor_0', 'interceptor_1', 'interceptor_2'],
                'use_pn_refinement': False,
                'pn_navigation_constant': 3.0,
                'pn_blend_gain': 0.0,
                'pn_min_closing_speed_m_s': 0.15,
                'naive_lead_time_s': 0.85,
                'dome_enabled': dome_en,
                'dome_center_x': 0.0,
                'dome_center_y': 0.0,
                'dome_center_z': 0.0,
                'dome_outer_m': DOME_OUTER_M,
                'dome_outer_hysteresis_m': dome_hyst,
                'dome_middle_m': DOME_MIDDLE_M,
                'dome_inner_m': DOME_INNER_M,
                'reset_lock_when_outside_dome': True,
                'dome_selection_mode': 'nearest',
                'publish_dome_rviz_marker': True,
                'publish_intercept_markers': True,
                'cuas_trails_enabled': True,
                'cuas_trail_hostile_rgba': [1.0, 0.2, 0.06, 0.96],
                'intercept_math_trail_target_rgba': [1.0, 0.22, 0.12, 0.95],
                'publish_hit_markers': True,
                'hit_marker_topic': '/interception/hit_markers',
                'cuas_trail_max_points': 12000,
                'cuas_intercept_predict_viz_enabled': True,
                'cuas_intercept_predict_text_enabled': True,
                'cuas_intercept_line_dashed': True,
                'multi_target_enabled': True,
                'multi_target_labels': ['target_0', 'target_1', 'target_2'],
                'multi_target_position_topics': [
                    f'/drone_0/position{noisy_suffix}' if use_noisy else '/drone_0/position',
                    f'/drone_1/position{noisy_suffix}' if use_noisy else '/drone_1/position',
                    f'/drone_2/position{noisy_suffix}' if use_noisy else '/drone_2/position',
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
                'intercept_heatmap_prob_export_dir': heatmap_export_dir,
                'intercept_heatmap_prob_export_stamp_files': heatmap_export_stamp,
                'intercept_heatmap_prob_use_cmd_vel': heatmap_prob_use_cmd_vel,
                'intercept_heatmap_prob_use_kinematic_rollout': True,
                'intercept_heatmap_prob_rollout_dt_s': 0.05,
                'intercept_heatmap_prob_rollout_mc_cap': 12,
                'intercept_heatmap_prob_use_cell_los_velocity': True,
                'intercept_heatmap_prob_target_los_speed_m_s': tgt_los if tgt_los > 0.0 else tgt_vx,
                'intercept_heatmap_prob_pos_sigma_m': 22.0,
                'intercept_heatmap_prob_vel_sigma_m_s': 1.35,
                'intercept_heatmap_prob_delay_jitter_s': 0.28,
                'reset_on_sim_clock_rewind': use_gui,
            },
        ],
    )

    delayed = TimerAction(
        period=0.5,
        actions=[*target_nodes, *noise_nodes, *interceptor_nodes, interception],
    )
    rviz_nodes = []
    if use_rviz:
        rviz_cfg = PathJoinSubstitution(
            [FindPackageShare('gazebo_target_sim'), 'rviz', 'hit_overlay.rviz'],
        ).perform(context)
        rviz_nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_hit_overlay',
                output='screen',
                arguments=['-d', rviz_cfg],
            ),
        )
    return [gz_argv_log, gz_empty_tree_hint, gz, clock_bridge, map_tf, *rviz_nodes, delayed]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    'gazebo_target_multi: Same visualization as gazebo_target — '
                    'Gazebo 3D or RViz /danger_zone/dome, /interception/markers, /interception/hit_markers. '
                    'use_rviz:=true loads hit_overlay.rviz.'
                ),
            ),
            DeclareLaunchArgument(
                'cuas_engagement_preset',
                default_value='far_ram',
                description=(
                    'far_ram = detect/detect (early engagement); doctrine = select/select (commit at r_mid).'
                ),
            ),
            DeclareLaunchArgument(
                'use_gazebo_gui',
                default_value='true',
                description='If false, gz sim runs server-only (-s) with no GUI window.',
            ),
            DeclareLaunchArgument(
                'gz_use_shipped_gui_config',
                default_value='true',
                description='If true in GUI mode, use share/gazebo_target_sim/gui/cuas_gz_gui.config (--gui-config).',
            ),
            DeclareLaunchArgument(
                'gz_transport_ip',
                default_value='127.0.0.1',
                description='GZ_IP for Gazebo Transport (127.0.0.1 for single-host headless; auto = UDP discovery).',
            ),
            DeclareLaunchArgument(
                'world_name',
                default_value='counter_uas_target',
                description='Must match <world name="..."> in worlds/target_sphere_multi.sdf.',
            ),
            DeclareLaunchArgument(
                'use_noisy_measurement',
                default_value='false',
                description='If true, route each /drone_i/position through noisy_measurement_node.',
            ),
            DeclareLaunchArgument(
                'noisy_suffix',
                default_value='_noisy',
                description='Suffix appended to each /drone_i/position topic for noisy measurements.',
            ),
            DeclareLaunchArgument(
                'noise_std_m',
                default_value='0.0',
                description='Gaussian noise std-dev (meters) applied to x/y/z.',
            ),
            DeclareLaunchArgument(
                'dropout_prob',
                default_value='0.0',
                description='Probability to drop a measurement publish tick (0..1).',
            ),
            DeclareLaunchArgument(
                'noise_seed',
                default_value='0',
                description='Seed for deterministic noise/dropout (per-target seeds use seed+idx).',
            ),
            DeclareLaunchArgument(
                'noise_rate_hz',
                default_value='10.0',
                description='Publish rate (Hz) for noisy measurement topics.',
            ),
            DeclareLaunchArgument(
                'intercept_heatmap_prob_export_dir',
                default_value='',
                description=(
                    'If non-empty: heatmap export on. At startup, removes stale intercept_heatmap_prob_*.csv/.svg '
                    '(timestamped clutter); retains intercept_heatmap_prob_latest.* ; each tick overwrites latest.'
                ),
            ),
            DeclareLaunchArgument(
                'intercept_heatmap_prob_export_stamp_files',
                default_value='false',
                description=(
                    'If true, also overwrites intercept_heatmap_prob_stamped.csv/.svg each update (fixed names).'
                ),
            ),
            DeclareLaunchArgument(
                'intercept_heatmap_prob_use_cmd_vel',
                default_value='false',
                description=(
                    'If true, prob heatmap MC uses last guidance cmd for all cells (legacy). '
                    'Default false: v_i_max toward each cell p_hit.'
                ),
            ),
            DeclareLaunchArgument(
                'intercept_measurement_source',
                default_value='ground_truth',
                description=(
                    'Single-target only; ignored when multi_target_enabled. fused/tracks select alternate Point topics.'
                ),
            ),
            DeclareLaunchArgument(
                'fused_detections_topic',
                default_value='/fused_detections',
                description='Point topic when intercept_measurement_source:=fused (single-target).',
            ),
            DeclareLaunchArgument(
                'tracks_topic',
                default_value='/tracks',
                description='Point topic when intercept_measurement_source:=tracks (single-target).',
            ),
            DeclareLaunchArgument(
                'dome_outer_hysteresis_m',
                default_value=str(DOME_OUTER_HYSTERESIS_M),
                description='Outer-dome boundary hysteresis (m) to reduce selection chatter.',
            ),
            DeclareLaunchArgument(
                'target_velocity_smooth_alpha',
                default_value='0.38',
                description='EMA alpha for target velocity smoothing.',
            ),
            DeclareLaunchArgument(
                'dome_enabled',
                default_value='true',
                description='If false, disables nested policy domes.',
            ),
            DeclareLaunchArgument(
                'target_los_closing_m_s',
                default_value=str(TGT_LOS_CLOSING_SPEED_M_S),
                description='Reference LOS speed for heatmap MC (m/s) when cell LOS velocity is used.',
            ),
            DeclareLaunchArgument(
                'target_los_dive_gain',
                default_value=str(TGT_LOS_DIVE_GAIN),
                description='LOS target vertical emphasis (1.0 = true LOS toward origin).',
            ),
            DeclareLaunchArgument(
                'target_approach_vx_m_s',
                default_value=str(TGT_APPROACH_VX_M_S),
                description='Hostile horizontal approach speed (m/s).',
            ),
            DeclareLaunchArgument(
                'target_approach_vz_m_s',
                default_value=str(TGT_APPROACH_VZ_M_S),
                description='Hostile approach vertical speed (m/s, <=0).',
            ),
            DeclareLaunchArgument(
                'target_dive_vz_m_s',
                default_value=str(TGT_DIVE_VZ_M_S),
                description='Hostile dive vertical speed (m/s, <=0).',
            ),
            DeclareLaunchArgument(
                'closing_margin_over_target_m_s',
                default_value=str(CLOSING_MARGIN_OVER_TARGET_M_S),
                description='Guidance closing_speed >= target_peak + this margin (m/s).',
            ),
            DeclareLaunchArgument(
                'vmax_above_closing_m_s',
                default_value=str(VMAX_ABOVE_CLOSING_M_S),
                description='Guidance max_speed closing_speed + this (m/s).',
            ),
            DeclareLaunchArgument(
                'interceptor_above_guidance_vmax_m_s',
                default_value=str(INTERCEPTOR_ABOVE_GUIDANCE_VMAX_M_S),
                description='Interceptor max_speed >= guidance max + this (m/s).',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='false',
                description='If true, starts rviz2 with share/gazebo_target_sim/rviz/hit_overlay.rviz.',
            ),
            OpaqueFunction(function=_apply_gz_ip_from_launch),
            OpaqueFunction(function=_gz_multi_setup),
        ],
    )
