"""Start gz sim + one hostile sphere (sphere_target_0) / multi-interceptor kinematics; /drone/position = ground truth."""

from __future__ import annotations

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _get_gz_ip() -> str:
    """Return the best local IP for gz transport.

    Uses a UDP socket trick (connect to a public IP without sending data)
    to discover the real outbound interface IP.  This works in containers,
    sandboxes, and machines with multiple NICs.  Falls back to 127.0.0.1
    which at minimum prevents the gz transport SIGSEGV when no interface
    is reachable.
    """
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
    """Set GZ_IP before gz/ bridges start: default loopback for stable single-host headless."""
    v = str(LaunchConfiguration('gz_transport_ip').perform(context)).strip()
    if not v or v.lower() == 'auto':
        ip = _get_gz_ip()
    else:
        ip = v
    return [
        SetEnvironmentVariable('GZ_IP', ip),
        LogInfo(
            msg=(
                f'gazebo_target_sim: GZ_IP={ip} '
                f'(gz_transport_ip={"auto" if not v or v.lower() == "auto" else v}).'
            )
        ),
    ]


# -----------------------------------------------------------------------------
# Scenario tuning (แก้ที่นี่): เปลี่ยนความเร็วโดรนโจมตี → guidance + interceptor คำนวณตาม
# -----------------------------------------------------------------------------

def _target_speed_peak_m_s(vx: float, vz_approach: float, vz_dive: float) -> float:
    """ขนาดความเร็วสูงสุดของ traject โจมตี (ช่วง approach vs dive)."""
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
    """
    คืน (v_target_peak, guidance_closing, guidance_max_speed, interceptor_max_speed).

    interception_logic ใช้ closing / max_speed สั่งทิศและ clamp;
    interceptor ต้อง max_speed >= คำสั่งที่ออกจริง (≈ min(closing, vmax)) + หัวเหลือเล็กน้อย.
    """
    v_peak = _target_speed_peak_m_s(vx, vz_approach, vz_dive)
    closing = max(8.0, v_peak + closing_margin_over_target_m_s)
    vmax = closing + vmax_above_closing_m_s
    interceptor_max = vmax + interceptor_above_guidance_vmax_m_s
    return v_peak, closing, vmax, interceptor_max


def _vel_smooth_alpha(interceptor_max_m_s: float, v_target_peak: float) -> float:
    """อัตราตาม headroom — ยิ่งเร็วกว่าเป้ามาก ยิ่งโน้มคำสั่งไวขึ้น."""
    headroom = max(0.0, interceptor_max_m_s - v_target_peak)
    a = 0.38 + 0.035 * headroom
    return float(min(0.88, max(0.32, a)))


# -----------------------------------------------------------------------------
# Realism baseline (slide-friendly SI, defense-in-depth domes):
# - Hostile sUAS: ~15–40+ m/s; default LOS closing **38 m/s** (~137 km/h) — fast but credible.
# - Radar gate: **6.5 km** effective range (can exceed r_outer policy dome).
# - Nested domes (≈1 : 2.5 : 5): inner = last-chance / critical, mid = commit,
#   outer = detect aligned with radar envelope — r_outer 6, r_mid 3, r_inner 1.2 km.
# -----------------------------------------------------------------------------

# ลู่บินโจมตี (ต้องตรงกับค่าที่ส่งให้ target_controller_node)
# เริ่มห่าง ~2.95 km slant — ใกล้พอให้ถึง strike shell ในไม่กี่นาทีเมื่อ headless CI
TGT_START_X_M = -2400.0
TGT_START_Y_M = 1680.0
TGT_START_Z_M = 240.0
# Horizontal speed toward asset center (m/s) → `approach_speed` on target_controller_node
TGT_APPROACH_VX_M_S = 38.0
# แนวดิ่งลง (m/s, <=0)
TGT_APPROACH_VZ_M_S = -2.0
TGT_DIVE_VZ_M_S = -5.0
TGT_T_DIV_S = 40.0
# แถบชนรอบ r_mid: |dist_to_center - dome_middle_m| ≤ half_width (เปลือกบางรอบขอบ detect/select)
STRIKE_SHELL_HALF_WIDTH_M = 75.0

# ไล่เร็วกว่าเป้า (peak) อย่างน้อยเท่าใด → closing_speed; vmax = closing + เล็กน้อย (clamp ใน node)
# เลขชุดนี้ตั้งให้สอดคล้อง km-scale + target LOS ~30 m/s: เพิ่ม margin เพื่อให้ปิดระยะทันก่อนเป้าลงต่ำ
CLOSING_MARGIN_OVER_TARGET_M_S = 18.0
VMAX_ABOVE_CLOSING_M_S = 8.0
# แรงดันจริงของ interceptor ต้องเหนือค่า max ที่ guidance คิดไว้ (สอดคล้อง max_speed บน controller)
INTERCEPTOR_ABOVE_GUIDANCE_VMAX_M_S = 14.0

# โดม 3 ชั้น (m) — ตัวเลขนี้ต้องตรงกับ worlds/target_sphere.sdf danger_dome_* radius
DOME_OUTER_M  = 6000.0
DOME_MIDDLE_M = 3000.0
DOME_INNER_M  = 1200.0
DOME_OUTER_HYSTERESIS_M = 150.0

# โดรนสะกัด: z=0 ให้ฐานโมเดลอยู่ชิดพื้น (collision box ~0–0.45 m — ตรงกับ target_sphere.sdf)
INTERCEPTOR_GROUND_Z_M = 0.0

# เป้า: วงโคจร XY + ลงช้า (spiral จากฟ้า) แล้วค่อย LOS เข้า (0,0,0) — ทดสอบ detect/select/strike + intercept
TGT_ORBIT_ENABLED = False
TGT_ORBIT_TURNS = 1.0
TGT_ORBIT_DURATION_S = 20.0
TGT_ORBIT_Z_HOLD = False
# ระหว่าง orbit: ลงเบาๆ (m/s, <=0); ไม่ใช้ dive_speed เพื่อไม่ให้โหนงเกินก่อนจบวง
TGT_ORBIT_DESCENT_M_S = -0.12
TGT_ATTACK_XY_SNAP_M = 0.45
# โจมตีแบบ LOS สู่ (0,0,0) — เส้นทางเฉียงลงทั้งเส้น (ไม่แยก “ราบแล้วดิ่งตรง”)
TGT_ATTACK_LOS_TO_ORIGIN = True
# 0 = ให้ target_controller ใช้ hypot(|approach_speed|, |dive_speed|)
TGT_LOS_CLOSING_SPEED_M_S = 38.0
# >1.0 = steeper LOS-style dive (stronger vertical closing vs horizontal; 1.0 = true 3D LOS).
TGT_LOS_DIVE_GAIN = 1.35


def _gz_target_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world_name').perform(context)
    tgt_sx = float(LaunchConfiguration('target_start_x_m').perform(context))
    tgt_sy = float(LaunchConfiguration('target_start_y_m').perform(context))
    tgt_sz = float(LaunchConfiguration('target_start_z_m').perform(context))
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
    noisy_topic = str(LaunchConfiguration('noisy_topic').perform(context)).strip() or '/drone/position_noisy'
    gt_topic = str(LaunchConfiguration('ground_truth_topic').perform(context)).strip() or '/drone/position'
    target_topic_for_guidance = noisy_topic if use_noisy else gt_topic
    meas_src = str(LaunchConfiguration('intercept_measurement_source').perform(context)).strip().lower()
    fused_topic_arg = str(LaunchConfiguration('fused_detections_topic').perform(context)).strip() or '/fused_detections'
    tracks_topic_arg = str(LaunchConfiguration('tracks_topic').perform(context)).strip() or '/tracks'
    tracks_state_topic_arg = str(LaunchConfiguration('tracks_state_topic').perform(context)).strip() or '/tracks/state'
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
    world_file = PathJoinSubstitution(
        [FindPackageShare('gazebo_target_sim'), 'worlds', 'target_sphere.sdf'],
    ).perform(context)

    eng_preset = str(LaunchConfiguration('cuas_engagement_preset').perform(context)).strip().lower()
    if eng_preset in ('doctrine', 'mid', 'select', 'layer2', 'commit'):
        guidance_layer = 'select'
        engagement_layer = 'select'
    else:
        # far_ram (default), detect, far, outer — allow kinetic HIT in detect annulus (r_mid < d <= r_outer).
        guidance_layer = 'detect'
        engagement_layer = 'detect'

    if TGT_ATTACK_LOS_TO_ORIGIN:
        los_eff = tgt_los if tgt_los > 0.0 else math.hypot(tgt_vx, abs(tgt_vz_dive))
        heatmap_los_speed = los_eff
        _v_peak, guidance_closing, guidance_vmax, interceptor_vmax = _interception_from_target(
            los_eff,
            0.0,
            0.0,
            closing_margin_over_target_m_s=closing_margin,
            vmax_above_closing_m_s=vmax_above,
            interceptor_above_guidance_vmax_m_s=interceptor_headroom,
        )
    else:
        heatmap_los_speed = tgt_vx
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

    use_gui = str(LaunchConfiguration('use_gazebo_gui').perform(context)).strip().lower() in (
        '1', 'true', 'yes', 'on',
    )
    # Guidance accel envelope (km-scale interceptor needs realistic small-missile acceleration —
    # the legacy 3 m/s² default produces a 26 s ramp from rest to 78 m/s, which the operator
    # observes as "the interceptor is not engaging" while the attacker closes inbound).
    interceptor_max_accel = float(
        LaunchConfiguration('interceptor_max_accel_m_s2').perform(context),
    )
    interceptor_turn_rate = float(
        LaunchConfiguration('interceptor_max_turn_rate_rad_s').perform(context),
    )
    dome_hyst = float(LaunchConfiguration('dome_outer_hysteresis_m').perform(context))
    tv_alpha = float(LaunchConfiguration('target_velocity_smooth_alpha').perform(context))
    dome_en = str(LaunchConfiguration('dome_enabled').perform(context)).strip().lower() in (
        '1',
        'true',
        'yes',
        'on',
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
            f'[gazebo_target_sim] gz sim command: {" ".join(gz_cmd)}'
            f' | GZ_IP={gz_ip_child}'
        ),
    )
    gz_empty_tree_hint = LogInfo(
        msg=(
            '[gazebo_target_sim] ถ้า Gazebo เปิดแต่ Entity tree ว่าง: colcon build อย่างเดียวไม่โหลด world '
            '— ต้องรัน ros2 launch gazebo_target_sim gazebo_target.launch.py '
            '(อย่าเปิดแอป Gazebo Sim จากเมนูคนเดียว). '
            'ถ้ายังเทา: สำรองแล้วลบ ~/.gz/sim แล้ว launch ใหม่; กด Play ถ้าซิมหยุด'
        ),
    )

    # แปลงเวลาจำลองจาก Gazebo -> /clock เพื่อให้โหนดจับ "reset" (เวลาย้อน) แล้วเคลียร์สถานะ ROS
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

    # RViz needs at least one TF so Fixed Frame "map" resolves (markers use frame_id map).
    map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    controller = Node(
        package='gazebo_target_sim',
        executable='target_controller_node',
        name='target_controller_node',
        output='screen',
        parameters=[
            {
                'world_name': world_name,
                'model_name': 'sphere_target_0',
                'rate_hz': 10.0,
                'start_x_m': tgt_sx,
                'start_y_m': tgt_sy,
                'start_z_m': tgt_sz,
                't_dive_s': TGT_T_DIV_S,
                'approach_speed': tgt_vx,
                'approach_vz': tgt_vz_ap,
                'dive_speed': tgt_vz_dive,
                'log_period_s': 1.0,
                'publish_drone_position': True,
                'drone_position_topic': gt_topic,
                'stop_topic': '/target/stop',
                # Block spurious /target/stop during startup (stale session / echo subscribers).
                # Reduced from 8s: pairing with VOLATILE stop + interception repeat bursts avoids missing HIT explosions.
                'ignore_stop_true_first_s': 2.0,
                'explode_on_hit': True,
                'explosion_fade_s': 12.0,
                # False = ซ่อนเป้าด้วย set_pose (ไม่ gz remove) — world ยังมี sphere_target_0..2 ครบหลังชน/รีเซ็ต
                'remove_model_on_hit': False,
                # gz remove/create บางครั้งใช้เวลานานกว่า set_pose
                'service_timeout_ms': 5000,
                'orbit_enabled': TGT_ORBIT_ENABLED,
                'orbit_turns': TGT_ORBIT_TURNS,
                'orbit_duration_s': TGT_ORBIT_DURATION_S,
                'orbit_z_hold': TGT_ORBIT_Z_HOLD,
                'orbit_descent_m_s': TGT_ORBIT_DESCENT_M_S,
                'attack_xy_snap_m': TGT_ATTACK_XY_SNAP_M,
                'attack_los_to_origin': TGT_ATTACK_LOS_TO_ORIGIN,
                'los_closing_speed_m_s': tgt_los,
                'attack_los_dive_gain': tgt_dive_gain,
                'reset_on_sim_clock_rewind': use_gui,
            },
        ],
    )

    noise_layer = None
    if use_noisy:
        noise_layer = Node(
            package='gazebo_target_sim',
            executable='noisy_measurement_node',
            name='noisy_measurement_node',
            output='screen',
            parameters=[
                {
                    'input_topic': gt_topic,
                    'output_topic': noisy_topic,
                    'rate_hz': noise_rate_hz,
                    'noise_std_m': noise_std_m,
                    'dropout_prob': dropout_prob,
                    'seed': noise_seed,
                },
            ],
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
                        'cmd_timeout_s': 0.75,
                        'max_speed_m_s': interceptor_vmax,
                        'position_topic': f'/{model_name}/position',
                        'marker_topic': f'/{model_name}/marker',
                        'publish_marker': True,
                        'marker_scale': 1.15,
                        'vel_smooth_alpha': max(vel_smooth_alpha, 0.55),
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
                # Match interceptor_controller tick (10 Hz) to avoid chattering between guidance steps.
                'rate_hz': 10.0,
                # Per-cycle Δv = max_accel * dt.  Default 3 m/s² @ 20 Hz = 0.15 m/s/cycle,
                # which makes the interceptor "appear stationary" for ~26 s before reaching
                # v_max=78 m/s.  30 m/s² (~3 g) is a realistic small-missile envelope and
                # cuts the ramp under 3 s without breaking physical plausibility.
                'max_acceleration_m_s2': interceptor_max_accel,
                'max_turn_rate_rad_s': interceptor_turn_rate,
                'closing_speed_m_s': guidance_closing,
                # Use same cap as interceptor_controller (interceptor_vmax), not guidance_vmax —
                # otherwise logic clamps cmd at ~47 m/s while physics allows ~57+ m/s (never "runs out" headroom).
                'max_speed_m_s': interceptor_vmax,
                # Let guidance raise speed toward max_speed when far away (helps climb/catch-up).
                'adaptive_speed_enabled': True,
                'adaptive_speed_gain': 0.25,
                # doctrine: guidance_start_layer=select, engagement_layer=select (commit near r_mid).
                # far_ram (cuas_engagement_preset default): detect/detect — ram + HIT allowed from outer annulus.
                'guidance_start_layer': guidance_layer,
                'engagement_layer': engagement_layer,
                'hit_outer_layer_only': False,
                'min_distance_m': 0.22,
                'log_period_s': 1.0,
                'intercept_debug': True,
                'metrics_log_period_s': 1.0,
                'publish_intercept_markers': True,
                # Presentation (RViz /interception/markers): hostile red trail, interceptor blue trail, predicted intercept.
                'cuas_trails_enabled': True,
                'cuas_trail_hostile_rgba': [1.0, 0.2, 0.06, 0.96],
                'intercept_math_trail_target_rgba': [1.0, 0.22, 0.12, 0.95],
                'publish_hit_markers': True,
                'hit_marker_topic': '/interception/hit_markers',
                'cuas_trail_max_points': 12000,
                'cuas_intercept_predict_viz_enabled': True,
                'cuas_intercept_predict_text_enabled': True,
                'cuas_intercept_line_dashed': True,
                'selection_log_period_s': 2.0,
                # พิมพ์สมการ TTI + กฎเลือกโดรนลง terminal เป็นระยะ (ดู selection_algo_period_s)
                'selection_algo_verbose': True,
                'selection_algo_period_s': 1.0,
                'selection_margin_s': 0.3,
                'switch_window_s': 2.0,
                'lost_timeout_s': 1.5,
                'reacquire_confirm_s': 0.25,
                # Predictive intercept enabled: solver finds intercept in [min, max] second window.
                # Keep min small: 0.5 s falsely marked near-contact (t_hit≈0.47) infeasible and cleared selection before HIT.
                # max=15 rejects spurious far-future solutions (e.g. t_go=90s or t_go=12s
                # when interceptor has overshot and solver finds a "come-back" trajectory).
                # km-scale dome: allow longer predictive intercept horizon (solver rejects > max).
                'max_intercept_time_s': 300.0,
                'min_intercept_time_s': 0.02,
                'target_topic': target_topic_for_guidance,
                'intercept_measurement_source': meas_src,
                'fused_detections_topic': fused_topic_arg,
                'tracks_topic': tracks_topic_arg,
                'tracks_state_topic': tracks_state_topic_arg,
                'selected_id_topic': '/interceptor/selected_id',
                # Allow re-assignment so the unit closest to a kill window stays committed (see TTI selection).
                'lock_selected_after_first': False,
                # hit_threshold_m: 3-D radius for HIT detection.
                # 1.0 m = tight enough to look like near-physical contact visually,
                # wide enough to register before the target descends below air_ok altitude.
                # (Previous 1.5 m was visually too far; 0.5 m was too tight — target hit
                # ground before interceptor could close that gap.)
                'hit_threshold_m': 1.0,
                # Ground-start interceptors: default node param 1.5 m blocked HIT before climb; relax for kinetic closure.
                'hit_min_interceptor_z_m': 0.05,
                'hit_min_interceptor_travel_m': 1.0,
                # Allow HIT at any altitude down to ground: target drops to z≈0 very fast
                # (LOS 5 m/s, z-component ≈ 3.7 m/s → z=0 in ~10 s).  With 0.5 m the
                # window closed before the interceptor could get within hit_threshold.
                'hit_min_target_z_m': -1.0,
                # Aim directly at the target (not the mid-shell boundary point).
                # aim_mid_shell causes interceptor to overshoot when target is already inside the shell.
                'aim_strike_on_mid_shell': False,
                'target_velocity_smooth_alpha': tv_alpha,
                'pursuit_lead_blend': 0.28,
                'world_name': world_name,
                'pause_gz_on_hit': False,
                'strike_shell_half_width_m': STRIKE_SHELL_HALF_WIDTH_M,
                'stop_topic': '/target/stop',
                'stop_signal_repeat_duration_s': 15.0,
                'stop_signal_repeat_period_s': 0.35,
                'hit_marker_duration_s': 10.0,
                'hit_marker_outer_diameter_m': 760.0,
                # Default off: gate + 2 s tracking delay after false /clock reorder "resets" blocked guidance for long stretches.
                # Enable for Phase 2→3 integration tests: sensing_gate_enabled:=true via override if needed.
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
                # Direct collision course; PN off (was causing lag vs fast target).
                'use_pn_refinement': False,
                'pn_navigation_constant': 3.0,
                'pn_blend_gain': 0.0,
                'pn_min_closing_speed_m_s': 0.15,
                'naive_lead_time_s': 0.85,
                # Ram/HIT only in strike shell |d - dome_middle_m| ≤ strike_shell_half_width_m. Radii match SDF.
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
                'intercept_heatmap_prob_export_dir': heatmap_export_dir,
                'intercept_heatmap_prob_export_stamp_files': heatmap_export_stamp,
                'intercept_heatmap_prob_use_cmd_vel': heatmap_prob_use_cmd_vel,
                'intercept_heatmap_prob_use_kinematic_rollout': True,
                'intercept_heatmap_prob_rollout_dt_s': 0.05,
                'intercept_heatmap_prob_rollout_mc_cap': 12,
                'intercept_heatmap_prob_use_cell_los_velocity': True,
                'intercept_heatmap_prob_target_los_speed_m_s': heatmap_los_speed,
                # Heatmap MC: align with 10 Hz loop + velocity EMA + tracking-delay dispersion (tune vs Gazebo batch).
                'intercept_heatmap_prob_pos_sigma_m': 22.0,
                'intercept_heatmap_prob_vel_sigma_m_s': 1.35,
                'intercept_heatmap_prob_delay_jitter_s': 0.28,
                'reset_on_sim_clock_rewind': use_gui,
            },
        ],
    )

    # Gazebo must be up before gz service calls. Keep this short: sphere_target is dynamic with
    # gravity — a multi-second delay lets it fall underground before set_pose runs (no visible target).
    delayed = TimerAction(
        period=0.5,
        actions=[controller, *( [noise_layer] if noise_layer is not None else [] ), *interceptor_nodes, interception],
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
            DeclareLaunchArgument(
                'use_gazebo_gui',
                default_value='true',
                description='If false, runs gz in server-only mode (-s): no 3D window. Use true on a machine with DISPLAY/Wayland.',
            ),
            DeclareLaunchArgument(
                'interceptor_max_accel_m_s2',
                default_value='30.0',
                description=(
                    'Interceptor longitudinal acceleration limit (m/s²). Default 30 (~3 g) for the '
                    'km-scale config; lower for lab toy.  Must be ≥ ~ v_max / desired_ramp_seconds.'
                ),
            ),
            DeclareLaunchArgument(
                'interceptor_max_turn_rate_rad_s',
                default_value='2.5',
                description=(
                    'Interceptor turn-rate limit (rad/s). Default 2.5 (~143 deg/s).  Lower values '
                    'make heading changes smoother but reduce ability to track manoeuvring targets.'
                ),
            ),
            DeclareLaunchArgument(
                'gz_use_shipped_gui_config',
                default_value='true',
                description=(
                    'If true (GUI mode only), passes --gui-config to use share/gazebo_target_sim/gui/cuas_gz_gui.config '
                    'so the scene/Entity tree are not broken by a stale ~/.gz/sim client layout. Set false to use only the SDF <gui> block.'
                ),
            ),
            DeclareLaunchArgument(
                'gz_transport_ip',
                default_value='127.0.0.1',
                description=(
                    'GZ_IP for Gazebo Transport. Default 127.0.0.1 for single-host headless; '
                    'use auto for legacy UDP discovery to outbound interface; or set an explicit LAN IP.'
                ),
            ),
            DeclareLaunchArgument(
                'world_name',
                default_value='counter_uas_target',
                description='Must match <world name="..."> in target_sphere.sdf.',
            ),
            DeclareLaunchArgument(
                'use_noisy_measurement',
                default_value='false',
                description='If true, route interception input through noisy_measurement_node.',
            ),
            DeclareLaunchArgument(
                'ground_truth_topic',
                default_value='/drone/position',
                description='Ground-truth target Point topic from the simulator/controller.',
            ),
            DeclareLaunchArgument(
                'noisy_topic',
                default_value='/drone/position_noisy',
                description='Noisy measurement Point topic published by noisy_measurement_node.',
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
                description='Seed for deterministic noise/dropout.',
            ),
            DeclareLaunchArgument(
                'noise_rate_hz',
                default_value='10.0',
                description='Publish rate (Hz) for noisy measurement topic.',
            ),
            DeclareLaunchArgument(
                'cuas_engagement_preset',
                default_value='far_ram',
                description=(
                    'C-UAS layer policy: far_ram (default) = detect/detect — early guidance + HIT in outer annulus; '
                    'doctrine = select/select — commit and HIT only inside r_mid (≈3 km).'
                ),
            ),
            DeclareLaunchArgument(
                'model_child_frame',
                default_value='',
                description='Unused: /drone/position is published by target_controller_node (ground truth).',
            ),
            DeclareLaunchArgument(
                'target_start_x_m',
                default_value=str(TGT_START_X_M),
                description='Target sphere initial X (m, map). Override for heatmap / validation trials.',
            ),
            DeclareLaunchArgument(
                'target_start_y_m',
                default_value=str(TGT_START_Y_M),
                description='Target sphere initial Y (m, map).',
            ),
            DeclareLaunchArgument(
                'target_start_z_m',
                default_value=str(TGT_START_Z_M),
                description='Target sphere initial Z (m, map).',
            ),
            DeclareLaunchArgument(
                'intercept_heatmap_prob_export_dir',
                default_value='',
                description=(
                    'If non-empty: enables prob heatmap export. At node startup, deletes prior '
                    'intercept_heatmap_prob_*.csv/.svg in that directory (stale timestamped runs) '
                    'and keeps intercept_heatmap_prob_latest.* ; each heatmap update overwrites latest.csv/.svg.'
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
                    'If true, probability heatmap MC extrapolates along last guidance cmd for all grid cells '
                    '(legacy; often drives P_hit→0 off-boresight). Default false: v_i_max toward each cell p_hit.'
                ),
            ),
            DeclareLaunchArgument(
                'intercept_measurement_source',
                default_value='ground_truth',
                description=(
                    'interception_logic_node target Point stream, ground_truth uses target_topic (GT or noisy); '
                    'fused → fused_detections_topic; tracks → tracks_topic.'
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
                'dome_outer_hysteresis_m',
                default_value=str(DOME_OUTER_HYSTERESIS_M),
                description='Outer-dome boundary hysteresis (m) to reduce selected_id chatter and STANDBY/LAUNCH spam.',
            ),
            DeclareLaunchArgument(
                'target_velocity_smooth_alpha',
                default_value='0.38',
                description='EMA alpha for target velocity smoothing (lower reduces spikes; slightly more lag).',
            ),
            DeclareLaunchArgument(
                'dome_enabled',
                default_value='true',
                description='If false, disables nested policy domes (simpler TTI selection for debugging).',
            ),
            DeclareLaunchArgument(
                'target_los_closing_m_s',
                default_value=str(TGT_LOS_CLOSING_SPEED_M_S),
                description='LOS attack closing speed (m/s) when SDF uses attack_los_to_origin; heatmap uses this in LOS mode.',
            ),
            DeclareLaunchArgument(
                'target_los_dive_gain',
                default_value=str(TGT_LOS_DIVE_GAIN),
                description=(
                    'LOS attack: vertical emphasis (1.0 = true 3D toward origin; '
                    'e.g. 1.35 = steeper dive). Passed to target_controller attack_los_dive_gain.'
                ),
            ),
            DeclareLaunchArgument(
                'target_approach_vx_m_s',
                default_value=str(TGT_APPROACH_VX_M_S),
                description='Hostile horizontal approach speed (m/s) for target_controller decoupled mode / approach_speed.',
            ),
            DeclareLaunchArgument(
                'target_approach_vz_m_s',
                default_value=str(TGT_APPROACH_VZ_M_S),
                description='Hostile approach vertical speed (m/s, <=0).',
            ),
            DeclareLaunchArgument(
                'target_dive_vz_m_s',
                default_value=str(TGT_DIVE_VZ_M_S),
                description='Hostile dive vertical speed (m/s, <=0) after t_dive.',
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
                description='interceptor_controller max_speed >= guidance max + this headroom (m/s).',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='false',
                description='If true, starts rviz2 with share/gazebo_target_sim/rviz/hit_overlay.rviz.',
            ),
            OpaqueFunction(function=_apply_gz_ip_from_launch),
            LogInfo(
                msg=(
                    'gazebo_target_sim: Visualization notes — '
                    'Domes + hostile + interceptors in the Gazebo window; RViz topic /interception/markers adds '
                    'red hostile trail, blue interceptor trails, and (optional) predicted intercept sphere + dashed line '
                    '(cuas_* params on interception_logic_node). /interception/hit_markers shows the HIT pulse. '
                    'Optional: use_rviz:=true opens rviz/hit_overlay.rviz. Fixed frame: map.'
                ),
            ),
            OpaqueFunction(function=_gz_target_setup),
        ],
    )
