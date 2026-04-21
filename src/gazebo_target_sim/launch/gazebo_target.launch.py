"""Start gz sim + one hostile sphere (sphere_target_0) / multi-interceptor kinematics; /drone/position = ground truth."""

from __future__ import annotations

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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


# ลู่บินโจมตี (ต้องตรงกับค่าที่ส่งให้ target_controller_node)
# เริ่มสูง + offset XY → วงโคจร + spiral ลงเบาๆ → โจมตี LOS สู่ (0,0,0) (เส้นทางเฉียง 3D สมจริง)
TGT_START_X_M = -11.0
TGT_START_Y_M = 9.0
TGT_START_Z_M = 40.0
# Horizontal speed toward asset center (m/s) → `approach_speed` on target_controller_node
TGT_APPROACH_VX_M_S = 3.0
# ช้าลงให้มองทัน — ถ้ายังเร็ว ลดเลขต่อ (ใกล้ 0 ยิ่งช้า)
TGT_APPROACH_VZ_M_S = -0.28
TGT_DIVE_VZ_M_S = -0.45
TGT_T_DIV_S = 28.0
# ครึ่งควหนาแถบชนรอบ r_mid: เดิม 3 m ทำให้อยู่ชั้น detect นานแล้วตัด cmd=0 — โดรนเลยไม่บินไล่
STRIKE_SHELL_HALF_WIDTH_M = 10.0

# ไล่เร็วกว่าเป้า (peak) อย่างน้อยเท่าใด → closing_speed; vmax = closing + เล็กน้อย (clamp ใน node)
CLOSING_MARGIN_OVER_TARGET_M_S = 6.0
VMAX_ABOVE_CLOSING_M_S = 1.0
# แรงดันจริงของ interceptor ต้องเหนือ vmax ของ guidance เล็กน้อย (ไม่ให้โดนตัดที่ 8–18 แบบคงที่)
INTERCEPTOR_ABOVE_GUIDANCE_VMAX_M_S = 3.0

# โดม 3 ชั้น (m) — ต้องตรงกับ danger_dome_* ใน target_sphere.sdf (โดมเล็กลง มอง overview + โดรนในจอชัดขึ้น)
DOME_OUTER_M = 36.0
DOME_MIDDLE_M = 26.0
DOME_INNER_M = 17.0

# โดรนสะกัด: z=0 ให้ฐานโมเดลอยู่ชิดพื้น (collision box ~0–0.45 m — ตรงกับ target_sphere.sdf)
INTERCEPTOR_GROUND_Z_M = 0.0

# เป้า: วงโคจร XY + ลงช้า (spiral จากฟ้า) แล้วค่อย LOS เข้า (0,0,0) — ทดสอบ detect/select/strike + intercept
TGT_ORBIT_ENABLED = True
TGT_ORBIT_TURNS = 1.0
TGT_ORBIT_DURATION_S = 38.0
TGT_ORBIT_Z_HOLD = False
# ระหว่าง orbit: ลงเบาๆ (m/s, <=0); ไม่ใช้ dive_speed เพื่อไม่ให้โหนงเกินก่อนจบวง
TGT_ORBIT_DESCENT_M_S = -0.12
TGT_ATTACK_XY_SNAP_M = 0.45
# โจมตีแบบ LOS สู่ (0,0,0) — เส้นทางเฉียงลงทั้งเส้น (ไม่แยก “ราบแล้วดิ่งตรง”)
TGT_ATTACK_LOS_TO_ORIGIN = True
# 0 = ให้ target_controller ใช้ hypot(|approach_speed|, |dive_speed|)
TGT_LOS_CLOSING_SPEED_M_S = 3.8


def _gz_target_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world_name').perform(context)
    world_file = PathJoinSubstitution(
        [FindPackageShare('gazebo_target_sim'), 'worlds', 'target_sphere.sdf'],
    ).perform(context)

    if TGT_ATTACK_LOS_TO_ORIGIN:
        los_eff = (
            TGT_LOS_CLOSING_SPEED_M_S
            if TGT_LOS_CLOSING_SPEED_M_S > 0.0
            else math.hypot(TGT_APPROACH_VX_M_S, abs(TGT_DIVE_VZ_M_S))
        )
        _v_peak, guidance_closing, guidance_vmax, interceptor_vmax = _interception_from_target(
            los_eff,
            0.0,
            0.0,
            closing_margin_over_target_m_s=CLOSING_MARGIN_OVER_TARGET_M_S,
            vmax_above_closing_m_s=VMAX_ABOVE_CLOSING_M_S,
            interceptor_above_guidance_vmax_m_s=INTERCEPTOR_ABOVE_GUIDANCE_VMAX_M_S,
        )
    else:
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

    # แปลงเวลาจำลองจาก Gazebo -> /clock เพื่อให้โหนดจับ "reset" (เวลาย้อน) แล้วเคลียร์สถานะ ROS
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_clock_bridge',
        output='screen',
        arguments=[
            f'/world/{world_name}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]',
        ],
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
                'start_x_m': TGT_START_X_M,
                'start_y_m': TGT_START_Y_M,
                'start_z_m': TGT_START_Z_M,
                't_dive_s': TGT_T_DIV_S,
                'approach_speed': TGT_APPROACH_VX_M_S,
                'approach_vz': TGT_APPROACH_VZ_M_S,
                'dive_speed': TGT_DIVE_VZ_M_S,
                'log_period_s': 1.0,
                'publish_drone_position': True,
                'drone_position_topic': '/drone/position',
                'stop_topic': '/target/stop',
                'explode_on_hit': True,
                'explosion_fade_s': 4.0,
                # gz remove/create บางครั้งใช้เวลานานกว่า set_pose
                'service_timeout_ms': 5000,
                'orbit_enabled': TGT_ORBIT_ENABLED,
                'orbit_turns': TGT_ORBIT_TURNS,
                'orbit_duration_s': TGT_ORBIT_DURATION_S,
                'orbit_z_hold': TGT_ORBIT_Z_HOLD,
                'orbit_descent_m_s': TGT_ORBIT_DESCENT_M_S,
                'attack_xy_snap_m': TGT_ATTACK_XY_SNAP_M,
                'attack_los_to_origin': TGT_ATTACK_LOS_TO_ORIGIN,
                'los_closing_speed_m_s': TGT_LOS_CLOSING_SPEED_M_S,
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
                # Match interceptor_controller tick (10 Hz) to avoid chattering between guidance steps.
                'rate_hz': 10.0,
                'closing_speed_m_s': guidance_closing,
                'max_speed_m_s': guidance_vmax,
                'min_distance_m': 0.22,
                'log_period_s': 1.0,
                'selection_log_period_s': 2.0,
                # พิมพ์สมการ TTI + กฎเลือกโดรนลง terminal เป็นระยะ (ดู selection_algo_period_s)
                'selection_algo_verbose': True,
                'selection_algo_period_s': 1.0,
                'selection_margin_s': 0.3,
                'switch_window_s': 2.0,
                'lost_timeout_s': 1.5,
                'reacquire_confirm_s': 0.25,
                'max_intercept_time_s': 90.0,
                'min_intercept_time_s': 0.02,
                'target_topic': '/drone/position',
                'selected_id_topic': '/interceptor/selected_id',
                'lock_selected_after_first': True,
                # ระยะหยุด/ระเบิด — ให้กว้างพอจับเมื่อเป้ากับสะกัดมาจากมุมต่างกัน
                'hit_threshold_m': 4.5,
                'hit_min_target_z_m': 0.5,
                # พุ่งไปจุดบนเปลือก r_mid (ขอบ L1/L2) ในทิศเดียวกับเป้า — pursuit/lead
                'aim_strike_on_mid_shell': True,
                'target_velocity_smooth_alpha': 0.52,
                'pursuit_lead_blend': 0.28,
                'world_name': world_name,
                'pause_gz_on_hit': True,
                'strike_shell_half_width_m': STRIKE_SHELL_HALF_WIDTH_M,
                'stop_topic': '/target/stop',
                'interceptor_ids': ['interceptor_0', 'interceptor_1', 'interceptor_2'],
                # Direct collision course; PN off (was causing lag vs fast target).
                'use_pn_refinement': False,
                'pn_navigation_constant': 3.0,
                'pn_blend_gain': 0.0,
                'pn_min_closing_speed_m_s': 0.15,
                'naive_lead_time_s': 0.85,
                # Ram/HIT only in strike shell |d - dome_middle_m| ≤ strike_shell_half_width_m. Radii match SDF.
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
            },
        ],
    )

    # Gazebo must be up before gz service calls. Keep this short: sphere_target is dynamic with
    # gravity — a multi-second delay lets it fall underground before set_pose runs (no visible target).
    delayed = TimerAction(
        period=0.5,
        actions=[controller, *interceptor_nodes, interception],
    )
    return [gz, clock_bridge, map_tf, delayed]


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
