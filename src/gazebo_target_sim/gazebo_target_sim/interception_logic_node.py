"""
Multi-interceptor: hysteresis TTI reassignment + predictive intercept + PN on the committed unit only.

**Three-layer dome:** ชนที่ **ขอบระหว่างชั้น 1 (detect) กับ 2 (select)** = เปลือก ``r ≈ dome_middle_m``.
ใช้แถบ ``|d - r_mid| ≤ strike_shell_half_width_m``. นอกแถบ / ชั้นใน = ไม่ ram. Topic ``/danger_zone/layer``.

**Scenario (counter-UAS):** interceptors start near ground; guidance is **3-D** with ``vz`` to climb.
"""

from __future__ import annotations

import math
import shutil
import subprocess
from typing import TYPE_CHECKING

import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker

from gazebo_target_sim.clock_reset import subscribe_sim_time_reset

if TYPE_CHECKING:
    from rclpy.publisher import Publisher


def _norm(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def _dot(ax: float, ay: float, az: float, bx: float, by: float, bz: float) -> float:
    return ax * bx + ay * by + az * bz


def _cross(ax: float, ay: float, az: float, bx: float, by: float, bz: float) -> tuple[float, float, float]:
    return (
        ay * bz - az * by,
        az * bx - ax * bz,
        ax * by - ay * bx,
    )


def _unit(dx: float, dy: float, dz: float, eps: float = 1e-9) -> tuple[float, float, float]:
    n = _norm(dx, dy, dz)
    if n < eps:
        return (0.0, 0.0, 0.0)
    return (dx / n, dy / n, dz / n)


def _solve_intercept_time(
    p_tx: float,
    p_ty: float,
    p_tz: float,
    v_tx: float,
    v_ty: float,
    v_tz: float,
    p_ix: float,
    p_iy: float,
    p_iz: float,
    s_i: float,
    trace: list[str] | None = None,
) -> float | None:
    """
    เวลาปิดระยะเชิงค่าคงที่: หา t>0 น้อยสุดที่ |r0 + v_T t| = s_i t
    โดย r0 = P_T - P_I  ยกกำลังสองได้สมการกำลังสอง a t^2 + b t + c = 0
    a = |v_T|^2 - s_i^2,  b = 2 r0·v_T,  c = |r0|^2
    """
    r0x = p_tx - p_ix
    r0y = p_ty - p_iy
    r0z = p_tz - p_iz
    vv = _dot(v_tx, v_ty, v_tz, v_tx, v_ty, v_tz)
    rv = _dot(r0x, r0y, r0z, v_tx, v_ty, v_tz)
    rr = _dot(r0x, r0y, r0z, r0x, r0y, r0z)

    if trace is not None:
        trace.append(f"    r0 = P_T - P_I = ({r0x:.6f}, {r0y:.6f}, {r0z:.6f})")
        trace.append(f"    |v_T|^2 = {vv:.8f}    r0·v_T = {rv:.8f}    |r0|^2 = {rr:.8f}")
        trace.append(f"    s_i (closing/intercept speed) = {s_i:.6f} m/s")
        trace.append('    Quad from |r0+v_T t|^2 = s_i^2 t^2  =>  a t^2 + b t + c = 0')
        trace.append(f"    a = |v_T|^2 - s_i^2 = {vv:.8f} - {s_i * s_i:.8f}")

    a = vv - s_i * s_i
    b = 2.0 * rv
    c = rr
    if trace is not None:
        trace.append(f"        = {a:.8f}")
        trace.append(f"    b = 2 * r0·v_T = {b:.8f}")
        trace.append(f"    c = |r0|^2 = {c:.8f}")
    eps = 1e-12
    candidates: list[float] = []

    if abs(a) < eps:
        if trace is not None:
            trace.append('    |a| ~ 0  =>  linear: b*t + c = 0  =>  t = -c/b')
        if abs(b) < eps:
            if trace is not None:
                trace.append('    b ~ 0  =>  no positive root')
            return None
        t_lin = -c / b
        if trace is not None:
            trace.append(f'    t_lin = {-c:.8f}/{b:.8f} = {t_lin:.8f}')
        if t_lin > 0.0:
            candidates.append(t_lin)
        else:
            if trace is not None:
                trace.append('    t_lin <= 0  rejected')
    else:
        disc = b * b - 4.0 * a * c
        if trace is not None:
            trace.append(f'    discriminant D = b^2 - 4ac = {disc:.8f}')
        if disc < 0.0:
            if trace is not None:
                trace.append('    D < 0  =>  no real t')
            return None
        sqrt_d = math.sqrt(disc)
        t0 = (-b - sqrt_d) / (2.0 * a)
        t1 = (-b + sqrt_d) / (2.0 * a)
        if trace is not None:
            trace.append(f'    t = (-b ± sqrt(D))/(2a)  =>  {t0:.8f}, {t1:.8f}')
        for t in (t0, t1):
            if t > 0.0:
                candidates.append(float(t))

    if not candidates:
        if trace is not None:
            trace.append('    no positive candidate t')
        return None

    if trace is not None:
        trace.append('    Geometric check: |r0 + v_T t| ≈ s_i * t  (tolerance-scaled)')
    valid: list[float] = []
    for t in candidates:
        hx = p_tx + v_tx * t - p_ix
        hy = p_ty + v_ty * t - p_iy
        hz = p_tz + v_tz * t - p_iz
        lhs = _norm(hx, hy, hz)
        rhs = s_i * t
        tol = max(0.12, 5e-4 * max(lhs, rhs, 1.0))
        ok = _intercept_tolerance(lhs, rhs)
        if trace is not None:
            trace.append(
                f'      t={t:.8f}: |r(t)|={lhs:.6f}  s_i*t={rhs:.6f}  |diff|={abs(lhs - rhs):.6f}  tol={tol:.6f}  ok={ok}',
            )
        if ok:
            valid.append(t)

    if not valid:
        if trace is not None:
            trace.append('    no candidate passed tolerance; fallback min(candidates)')
        if candidates:
            return min(candidates)
        return None
    t_best = min(valid)
    if trace is not None:
        trace.append(f'    => chosen t = min(valid) = {t_best:.8f} s')
    return t_best


def _intercept_tolerance(lhs: float, rhs: float) -> bool:
    """ยอมรับคลาดเคลื่อนเลข — เดิมเทียบ 1e-6 ทำให้ทิ้งคำตอบที่ถูกต้องเชิงพีชคณิตบ่อยเกินไป."""
    tol = max(0.12, 5e-4 * max(lhs, rhs, 1.0))
    return abs(lhs - rhs) <= tol


def _compute_intercept(
    p_tx: float,
    p_ty: float,
    p_tz: float,
    v_tx: float,
    v_ty: float,
    v_tz: float,
    p_ix: float,
    p_iy: float,
    p_iz: float,
    s_i: float,
) -> tuple[float, float, float, float, float, float, float] | None:
    t = _solve_intercept_time(p_tx, p_ty, p_tz, v_tx, v_ty, v_tz, p_ix, p_iy, p_iz, s_i)
    if t is None or not math.isfinite(t):
        return None
    phx = p_tx + v_tx * t
    phy = p_ty + v_ty * t
    phz = p_tz + v_tz * t
    ux, uy, uz = _unit(phx - p_ix, phy - p_iy, phz - p_iz)
    if _norm(ux, uy, uz) < 1e-9:
        return None
    return (t, phx, phy, phz, ux, uy, uz)


def _pn_steering_vector(
    rx: float,
    ry: float,
    rz: float,
    v_rx: float,
    v_ry: float,
    v_rz: float,
    pn_n: float,
    min_vc: float,
) -> tuple[float, float, float, float, bool]:
    rn = _norm(rx, ry, rz)
    if rn < 1e-6:
        return (0.0, 0.0, 0.0, 0.0, False)
    rhx, rhy, rhz = rx / rn, ry / rn, rz / rn
    inv_r2 = 1.0 / (rn * rn + 1e-12)
    lrx, lry, lrz = _cross(rx, ry, rz, v_rx, v_ry, v_rz)
    lrx *= inv_r2
    lry *= inv_r2
    lrz *= inv_r2
    vc = -_dot(rhx, rhy, rhz, v_rx, v_ry, v_rz)
    if vc < min_vc:
        return (0.0, 0.0, 0.0, vc, False)
    sx, sy, sz = _cross(lrx, lry, lrz, rhx, rhy, rhz)
    sx *= pn_n * vc
    sy *= pn_n * vc
    sz *= pn_n * vc
    if _norm(sx, sy, sz) < 1e-12:
        return (0.0, 0.0, 0.0, vc, False)
    return (sx, sy, sz, vc, True)


def _tti_feasible(
    tx: float,
    ty: float,
    tz: float,
    v_tx: float,
    v_ty: float,
    v_tz: float,
    ix: float,
    iy: float,
    iz: float,
    closing: float,
    t_min: float,
    t_max: float,
) -> tuple[bool, float | None]:
    sol = _compute_intercept(tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, closing)
    if sol is None:
        return (False, None)
    t_hit, _phx, _phy, _phz, ux, uy, uz = sol
    if not (t_min <= t_hit <= t_max) or _norm(ux, uy, uz) < 1e-6:
        return (False, None)
    return (True, float(t_hit))


class InterceptionLogicNode(Node):
    """
    Subscribes to ``/drone/position`` and ``/<id>/position`` for each interceptor id.
    Maintains a **committed** interceptor; reassignment uses TTI margin + minimum dwell
    ``switch_window_s`` (seconds since last commit). Forced reassignment when the committed
    unit becomes infeasible.     Publishes ``Vector3`` on ``/<id>/cmd_velocity`` (non-committed = 0).

    Logs ``[min_miss]`` (minimum |P_T-P_I| m this run on active guidance/hit paths) vs ``hit_threshold``.

    **Kinematics:** ``_compute_intercept`` / naive LOS use **x,y,z**; ``cmd_vel`` includes ``vz``.
    When the target is above the interceptor, the commanded direction has **vz > 0** (climb to
    intercept), matching ground-based defenders against an aerial threat.
    """

    def __init__(self) -> None:
        super().__init__('interception_logic_node')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('closing_speed_m_s', 4.5)
        self.declare_parameter('max_speed_m_s', 7.5)
        self.declare_parameter('min_distance_m', 0.45)
        self.declare_parameter('log_period_s', 1.0)
        # [Intercept Debug] line: mode, distance, t_hit, poses, cmd_vel, alignment (LOS vs cmd).
        self.declare_parameter('intercept_debug', True)
        self.declare_parameter('selection_log_period_s', 2.0)
        self.declare_parameter('selection_margin_s', 0.3)
        self.declare_parameter('switch_window_s', 2.0)
        self.declare_parameter('lost_timeout_s', 1.5)
        self.declare_parameter('reacquire_confirm_s', 0.25)
        self.declare_parameter('max_intercept_time_s', 90.0)
        self.declare_parameter('min_intercept_time_s', 0.02)
        self.declare_parameter('target_topic', '/drone/position')
        self.declare_parameter('selected_id_topic', '/interceptor/selected_id')
        self.declare_parameter('lock_selected_after_first', True)
        self.declare_parameter('hit_threshold_m', 1.0)
        # ถ้า >= 0: นับ HIT เฉพาะเมื่อเป้ายังอยู่เหนือระดับนี้ (เมตรใน world) เช่น โจมตีในอากาศในชั้น engage
        self.declare_parameter('hit_min_target_z_m', -1.0)
        self.declare_parameter('stop_topic', '/target/stop')
        self.declare_parameter('interceptor_ids', ['interceptor_0', 'interceptor_1', 'interceptor_2'])
        self.declare_parameter('use_pn_refinement', True)
        self.declare_parameter('pn_navigation_constant', 3.0)
        self.declare_parameter('pn_blend_gain', 0.22)
        self.declare_parameter('pn_min_closing_speed_m_s', 0.15)
        self.declare_parameter('naive_lead_time_s', 0.85)
        # ปรับความเร็วเป้าแบบ EMA — ลดการได้ vz=0 ในช่วงแรกที่ทำให้ predictive พลาด
        self.declare_parameter('target_velocity_smooth_alpha', 0.52)
        # 0=pure pursuit เท่านั้น; 0.2–0.35 = ผสม lead ตาม v สำหรับเป้าเคลื่อนที่
        self.declare_parameter('pursuit_lead_blend', 0.28)
        # Three nested domes (asset-centered): outer = detect, middle = select, inner = engage.
        self.declare_parameter('dome_enabled', True)
        self.declare_parameter('dome_center_x', 0.0)
        self.declare_parameter('dome_center_y', 0.0)
        self.declare_parameter('dome_center_z', 0.0)
        self.declare_parameter('dome_outer_m', 75.0)
        self.declare_parameter('dome_middle_m', 55.0)
        self.declare_parameter('dome_inner_m', 32.0)
        self.declare_parameter('reset_lock_when_outside_dome', True)
        # When dome disabled: 'nearest' or 'tti' for committed selection.
        self.declare_parameter('dome_selection_mode', 'nearest')
        self.declare_parameter('publish_dome_rviz_marker', True)
        self.declare_parameter('dome_marker_frame_id', 'map')
        self.declare_parameter('world_name', 'counter_uas_target')
        self.declare_parameter('pause_gz_on_hit', True)
        # ครึ่งความหนาเปลือกชน (m) รอบ ``dome_middle_m`` — ขอบระหว่าง detect กับ select
        self.declare_parameter('strike_shell_half_width_m', 3.0)
        # ในโหมด strike: los/pursuit ไปยังจุดบนเปลือก r_mid (ขอบ detect/select) ตามทิศเป้า
        self.declare_parameter('aim_strike_on_mid_shell', True)
        # พิมพ์ขั้นตอน + สมการเลือก interceptor / TTI ลง terminal (ควบคุมด้วย selection_algo_period_s)
        self.declare_parameter('selection_algo_verbose', False)
        self.declare_parameter('selection_algo_period_s', 1.0)
        # Multiple hostile tracks: TTI bipartite assignment + per-interceptor ``assigned_target`` string.
        self.declare_parameter('multi_target_enabled', False)
        self.declare_parameter('multi_target_labels', ['target_0', 'target_1', 'target_2'])
        self.declare_parameter(
            'multi_target_position_topics',
            ['/drone_0/position', '/drone_1/position', '/drone_2/position'],
        )
        self.declare_parameter(
            'multi_target_stop_topics',
            ['/target_0/stop', '/target_1/stop', '/target_2/stop'],
        )
        self.declare_parameter('assignment_print_period_s', 1.0)
        # Threat score (multi-target): higher = engage first. threat = w_d/d + w_vz*(-vz) + w_t/tti_best
        self.declare_parameter('threat_weight_dist', 1.0)
        self.declare_parameter('threat_weight_vz', 0.05)
        self.declare_parameter('threat_weight_tti', 1.0)
        self.declare_parameter('threat_distance_eps_m', 1.0)
        self.declare_parameter('threat_tti_fallback_s', 120.0)
        # Dive: if vz < -threat_dive_speed_threshold_m_s, add threat_dive_boost to score.
        self.declare_parameter('threat_dive_speed_threshold_m_s', 0.35)
        self.declare_parameter('threat_dive_boost', 6.0)
        # Inside dome: if distance to asset < threat_critical_radius_m, add threat_critical_boost (highest priority).
        self.declare_parameter('threat_critical_radius_m', 14.0)
        self.declare_parameter('threat_critical_boost', 50.0)
        # Reuse prior interceptor unless greedy TTI improves by more than this margin (seconds).
        self.declare_parameter('assignment_stability_enabled', True)
        self.declare_parameter('assignment_switch_tti_margin_s', 1.2)

        self._closing = max(float(self.get_parameter('closing_speed_m_s').value), 0.1)
        self._vmax = max(float(self.get_parameter('max_speed_m_s').value), self._closing)
        self._r_stop = max(float(self.get_parameter('min_distance_m').value), 0.05)
        self._log_period = max(float(self.get_parameter('log_period_s').value), 0.2)
        self._intercept_debug = bool(self.get_parameter('intercept_debug').value)
        self._sel_log_period = max(float(self.get_parameter('selection_log_period_s').value), 0.5)
        self._t_hit_max = max(float(self.get_parameter('max_intercept_time_s').value), 0.5)
        self._t_hit_min = max(float(self.get_parameter('min_intercept_time_s').value), 1e-4)
        self._use_pn = bool(self.get_parameter('use_pn_refinement').value)
        self._pn_n = max(float(self.get_parameter('pn_navigation_constant').value), 0.1)
        self._pn_blend = max(float(self.get_parameter('pn_blend_gain').value), 0.0)
        self._pn_min_vc = max(float(self.get_parameter('pn_min_closing_speed_m_s').value), 0.0)
        self._naive_lead = max(float(self.get_parameter('naive_lead_time_s').value), 0.0)
        self._tgt_vel_smooth_a = min(max(float(self.get_parameter('target_velocity_smooth_alpha').value), 0.05), 0.98)
        self._pursuit_lead_blend = min(max(float(self.get_parameter('pursuit_lead_blend').value), 0.0), 1.0)
        self._dome_enabled = bool(self.get_parameter('dome_enabled').value)
        self._strike_shell_hw = max(float(self.get_parameter('strike_shell_half_width_m').value), 0.0)
        self._aim_mid_shell = bool(self.get_parameter('aim_strike_on_mid_shell').value)
        self._dome_cx = float(self.get_parameter('dome_center_x').value)
        self._dome_cy = float(self.get_parameter('dome_center_y').value)
        self._dome_cz = float(self.get_parameter('dome_center_z').value)
        r_out = float(self.get_parameter('dome_outer_m').value)
        r_mid = float(self.get_parameter('dome_middle_m').value)
        r_in = float(self.get_parameter('dome_inner_m').value)
        if not (r_in < r_mid < r_out):
            raise ValueError(
                f'dome_inner_m ({r_in}) < dome_middle_m ({r_mid}) < dome_outer_m ({r_out}) required',
            )
        self._r_outer = r_out
        self._r_mid = r_mid
        self._r_inner = r_in
        self._reset_lock_outside = bool(self.get_parameter('reset_lock_when_outside_dome').value)
        self._dome_sel_mode = str(self.get_parameter('dome_selection_mode').value).strip().lower()
        self._pub_dome_marker = bool(self.get_parameter('publish_dome_rviz_marker').value)
        self._dome_frame = str(self.get_parameter('dome_marker_frame_id').value).strip() or 'map'
        self._tti_margin = max(float(self.get_parameter('selection_margin_s').value), 0.0)
        self._switch_window_s = max(float(self.get_parameter('switch_window_s').value), 0.0)
        self._lost_timeout_s = max(float(self.get_parameter('lost_timeout_s').value), 0.0)
        self._reacquire_confirm_s = max(float(self.get_parameter('reacquire_confirm_s').value), 0.0)
        self._sel_algo_verbose = bool(self.get_parameter('selection_algo_verbose').value)
        self._sel_algo_period = max(float(self.get_parameter('selection_algo_period_s').value), 0.05)

        raw_ids = self.get_parameter('interceptor_ids').value
        if isinstance(raw_ids, list) and raw_ids:
            self._ids = [str(x).strip() for x in raw_ids if str(x).strip()]
        else:
            self._ids = ['interceptor_0', 'interceptor_1', 'interceptor_2']

        self._multi_enabled = bool(self.get_parameter('multi_target_enabled').value)
        self._assignment_print_period = max(float(self.get_parameter('assignment_print_period_s').value), 0.2)
        self._w_threat_dist = max(float(self.get_parameter('threat_weight_dist').value), 0.0)
        self._w_threat_vz = max(float(self.get_parameter('threat_weight_vz').value), 0.0)
        self._w_threat_tti = max(float(self.get_parameter('threat_weight_tti').value), 0.0)
        self._threat_dist_eps = max(float(self.get_parameter('threat_distance_eps_m').value), 0.05)
        self._threat_tti_fallback = max(float(self.get_parameter('threat_tti_fallback_s').value), self._t_hit_min)
        self._threat_dive_thr = max(float(self.get_parameter('threat_dive_speed_threshold_m_s').value), 0.01)
        self._threat_dive_boost = max(float(self.get_parameter('threat_dive_boost').value), 0.0)
        self._threat_crit_r = max(float(self.get_parameter('threat_critical_radius_m').value), 0.5)
        self._threat_crit_boost = max(float(self.get_parameter('threat_critical_boost').value), 0.0)
        self._assign_stability = bool(self.get_parameter('assignment_stability_enabled').value)
        self._assign_switch_margin = max(float(self.get_parameter('assignment_switch_tti_margin_s').value), 0.0)
        self._multi_labels: list[str] = []
        self._multi_topics: list[str] = []
        self._multi_stops_list: list[str] = []
        if self._multi_enabled:
            ml = self.get_parameter('multi_target_labels').value
            mt = self.get_parameter('multi_target_position_topics').value
            ms = self.get_parameter('multi_target_stop_topics').value
            if not isinstance(ml, list) or not isinstance(mt, list) or not isinstance(ms, list):
                raise ValueError('multi_target_labels, multi_target_position_topics, multi_target_stop_topics must be lists')
            if len(ml) != len(mt) or len(ml) != len(ms) or len(ml) < 1:
                raise ValueError('multi_target_* lists must be non-empty and equal length')
            self._multi_labels = [str(x).strip() for x in ml]
            self._multi_topics = [str(x).strip() for x in mt]
            self._multi_stops_list = [str(x).strip() for x in ms]

        tgt_topic = str(self.get_parameter('target_topic').value).strip()
        sel_topic = str(self.get_parameter('selected_id_topic').value).strip()
        stop_topic = str(self.get_parameter('stop_topic').value).strip()
        rate = max(float(self.get_parameter('rate_hz').value), 1.0)
        self._lock_after_first = bool(self.get_parameter('lock_selected_after_first').value)
        self._hit_thresh = max(float(self.get_parameter('hit_threshold_m').value), 0.05)
        _min_tz = float(self.get_parameter('hit_min_target_z_m').value)
        self._hit_min_tz: float | None = _min_tz if _min_tz > -0.5 else None

        self._pubs: dict[str, Publisher] = {}
        for iid in self._ids:
            cmd_topic = f'/{iid}/cmd_velocity'
            self._pubs[iid] = self.create_publisher(Vector3, cmd_topic, 10)
        self._pub_selected = self.create_publisher(String, sel_topic, 10)
        _stop_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub_stops: dict[str, Publisher] = {}
        self._pub_assigned: dict[str, Publisher] = {}
        if self._multi_enabled:
            self._pub_stop = None
            for lab, st in zip(self._multi_labels, self._multi_stops_list):
                self._pub_stops[lab] = self.create_publisher(Bool, st, _stop_qos)
            for iid in self._ids:
                self._pub_assigned[iid] = self.create_publisher(String, f'/{iid}/assigned_target', 10)
        else:
            self._pub_stop = self.create_publisher(Bool, stop_topic, _stop_qos)
        self._pub_layer = self.create_publisher(String, '/danger_zone/layer', 10)

        if self._multi_enabled:
            for lab, topic in zip(self._multi_labels, self._multi_topics):
                self.create_subscription(Point, topic, lambda msg, l=lab: self._on_target_multi(l, msg), 10)
        else:
            self.create_subscription(Point, tgt_topic, self._on_target, 10)
        for iid in self._ids:
            pos_topic = f'/{iid}/position'
            self.create_subscription(
                Point,
                pos_topic,
                lambda msg, name=iid: self._on_inter(name, msg),
                10,
            )

        self._target: Point | None = None
        self._targets: dict[str, Point | None] = {}
        self._hits_multi: dict[str, bool] = {}
        self._multi_prev_assign: dict[str, str] = {}
        self._multi_prev_target: dict[str, tuple[float, float, float] | None] = {}
        self._multi_prev_target_time: dict[str, Time | None] = {}
        self._multi_v_smooth: dict[str, tuple[float, float, float]] = {}
        self._last_assignment_print: Time | None = None
        if self._multi_enabled:
            for lab in self._multi_labels:
                self._targets[lab] = None
                self._hits_multi[lab] = False
                self._multi_prev_target[lab] = None
                self._multi_prev_target_time[lab] = None
                self._multi_v_smooth[lab] = (0.0, 0.0, 0.0)
        self._inter_pos: dict[str, Point | None] = {i: None for i in self._ids}
        self._prev_target: tuple[float, float, float] | None = None
        self._prev_target_time: Time | None = None
        self._v_tgt_smooth = (0.0, 0.0, 0.0)
        self._prev_inter_pos: dict[str, tuple[float, float, float] | None] = {i: None for i in self._ids}
        self._prev_inter_time: dict[str, Time | None] = {i: None for i in self._ids}

        self._last_log = self.get_clock().now()
        self._last_sel_log = self.get_clock().now()
        self._t0 = self.get_clock().now()
        self._current_selected_id: str | None = None
        self._committed_since: Time | None = None
        self._lost_since: Time | None = None
        self._reacquire_since: Time | None = None
        self._switch_count = 0
        self._best_id_last: str | None = None
        self._last_hold_log: Time | None = None
        self._locked_selected_id: str | None = None
        self._hit = False
        self._last_layer = ''
        self._world_name_gz = str(self.get_parameter('world_name').value).strip() or 'counter_uas_target'
        self._pause_gz_on_hit = bool(self.get_parameter('pause_gz_on_hit').value)
        self._gz_pause_sent = False
        self._last_algo_log: Time | None = None
        # Closest 3D range |P_T - P_I| seen this run (engagement paths only); reset on sim rewind.
        self._min_miss_distance = float("inf")
        self._last_miss_log = self.get_clock().now()  # same cadence as intercept detail log (log_period_s)

        qos_marker = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub_dome_viz = (
            self.create_publisher(Marker, '/danger_zone/dome', qos_marker) if self._pub_dome_marker else None
        )
        if self._pub_dome_viz is not None:
            self.create_timer(2.0, self._publish_dome_marker)
            self._publish_dome_marker()

        period = 1.0 / rate
        self._timer = self.create_timer(period, self._on_control)
        subscribe_sim_time_reset(self, self._on_gz_sim_reset)
        if self._multi_enabled:
            self.get_logger().info(
                f'Multi-target threat assignment: {self._multi_topics} + '
                f'{[f"/{i}/position" for i in self._ids]} | labels={self._multi_labels} | '
                f'threat_weight_dist={self._w_threat_dist} threat_weight_vz={self._w_threat_vz} '
                f'threat_weight_tti={self._w_threat_tti} | dive_thr={self._threat_dive_thr}m/s '
                f'dive_boost={self._threat_dive_boost} | crit_r={self._threat_crit_r}m crit_boost={self._threat_crit_boost} | '
                f'stability={self._assign_stability} switch_margin_tti={self._assign_switch_margin}s',
            )
        else:
            self.get_logger().info(
                f'Multi-interception (predict + PN, TTI+hysteresis): {tgt_topic} + '
                f'{[f"/{i}/position" for i in self._ids]} -> cmd per id; '
                f'margin={self._tti_margin}s window={self._switch_window_s}s lost_timeout={self._lost_timeout_s}s '
                f'({rate:.0f} Hz) | 3-layer dome: outer={self._r_outer} mid={self._r_mid} inner={self._r_inner} m '
                f'center=({self._dome_cx},{self._dome_cy},{self._dome_cz}) strike_shell=±{self._strike_shell_hw}m@r_mid',
            )
        if self._sel_algo_verbose:
            self.get_logger().info(
                f'Selection algorithm verbose: print every {self._sel_algo_period:.2f}s (TTI equations + rules)',
            )

    def _on_gz_sim_reset(self) -> None:
        """กด Reset ใน Gazebo -> เวลาจำลองย้อน — เคลียร์ HIT/pause/ล็อกให้รันรอบใหม่ได้."""
        self.get_logger().info('Sim reset (/clock rewind): clearing interception state (hit, pause flag, locks).')
        self._hit = False
        self._gz_pause_sent = False
        self._clear_assignments()
        self._locked_selected_id = None
        self._current_selected_id = None
        self._committed_since = None
        self._lost_since = None
        self._reacquire_since = None
        self._switch_count = 0
        self._best_id_last = None
        self._last_layer = ''
        self._v_tgt_smooth = (0.0, 0.0, 0.0)
        self._prev_target = None
        self._prev_target_time = None
        for iid in self._ids:
            self._prev_inter_pos[iid] = None
            self._prev_inter_time[iid] = None
        if self._multi_enabled:
            for lab in self._multi_labels:
                self._hits_multi[lab] = False
                self._targets[lab] = None
                self._multi_prev_target[lab] = None
                self._multi_prev_target_time[lab] = None
                self._multi_v_smooth[lab] = (0.0, 0.0, 0.0)
            self._last_assignment_print = None
            self._multi_prev_assign.clear()
        self._min_miss_distance = float("inf")

    def _publish_stop_signal(self, target_label: str | None) -> None:
        stop = Bool(data=True)
        if target_label is None:
            if self._pub_stop is not None:
                for _ in range(3):
                    self._pub_stop.publish(stop)
        else:
            pub = self._pub_stops.get(target_label)
            if pub is not None:
                for _ in range(3):
                    pub.publish(stop)

    def _in_strike_zone(self, d_threat: float) -> bool:
        """ชน/HIT เฉพาะแถบรอบ ``r_mid`` — ขอบระหว่าง detect กับ select; ถ้า hw=0 ใช้ทั้ง annulus select (เดิม)."""
        if self._strike_shell_hw > 1e-6:
            return abs(d_threat - self._r_mid) <= self._strike_shell_hw
        return self._r_inner < d_threat <= self._r_mid

    def _on_target(self, msg: Point) -> None:
        self._target = msg

    def _on_target_multi(self, label: str, msg: Point) -> None:
        self._targets[label] = msg

    def _on_inter(self, iid: str, msg: Point) -> None:
        self._inter_pos[iid] = msg

    def _record_miss_distance(self, miss_distance: float) -> None:
        """Update minimum range on active guidance / hit-check ticks (single- and multi-target)."""
        if not math.isfinite(miss_distance) or miss_distance < 0.0:
            return
        if miss_distance < self._min_miss_distance:
            self._min_miss_distance = miss_distance

    def _print_miss_distance_lines(self) -> None:
        if self._min_miss_distance < float("inf") and math.isfinite(self._min_miss_distance):
            print(f'[min_miss] = {self._min_miss_distance:.4f} m', flush=True)
            print(f'hit_threshold = {self._hit_thresh:.4f} m', flush=True)
        else:
            print('[min_miss] = n/a', flush=True)
            print(f'hit_threshold = {self._hit_thresh:.4f} m', flush=True)

    def _maybe_emit_miss_distance_log_multi(self) -> None:
        """Periodic miss summary when multi-target mode (no _maybe_detail_log on guidance path)."""
        now = self.get_clock().now()
        if (now - self._last_miss_log).nanoseconds * 1e-9 < self._log_period:
            return
        self._last_miss_log = now
        self._print_miss_distance_lines()

    def _pause_gazebo_world(self) -> None:
        if self._gz_pause_sent or not self._pause_gz_on_hit:
            return
        gz = shutil.which('gz')
        if gz is None:
            self.get_logger().warning('gz CLI not found; cannot pause world (poses already frozen).')
            self._gz_pause_sent = True
            return
        req = 'pause: true\n'
        try:
            r = subprocess.run(
                [
                    gz,
                    'service',
                    '-s',
                    f'/world/{self._world_name_gz}/control',
                    '--reqtype',
                    'gz.msgs.WorldControl',
                    '--reptype',
                    'gz.msgs.Boolean',
                    '--timeout',
                    '3000',
                    '--req',
                    req,
                ],
                capture_output=True,
                text=True,
                timeout=6.0,
                check=False,
            )
            if r.returncode == 0:
                self.get_logger().info('Gazebo simulation paused (freeze frame).')
            else:
                self.get_logger().warning(
                    f'Could not pause Gazebo (rc={r.returncode}) stderr={r.stderr!r}',
                )
        except (OSError, subprocess.TimeoutExpired) as e:
            self.get_logger().warning(f'Gazebo pause failed: {e}')
        self._gz_pause_sent = True

    def _dist_threat(self, tx: float, ty: float, tz: float) -> float:
        return _norm(tx - self._dome_cx, ty - self._dome_cy, tz - self._dome_cz)

    def _threat_score(self, d_origin: float, vz: float, tti_best: float | None) -> float:
        """Higher = more urgent (closer to asset, faster dive, shorter best-case TTI)."""
        d_eff = max(d_origin, self._threat_dist_eps)
        inv_d = 1.0 / d_eff
        if tti_best is not None and math.isfinite(tti_best) and tti_best > 0.0:
            tti_eff = max(tti_best, self._t_hit_min)
        else:
            tti_eff = max(self._threat_tti_fallback, self._t_hit_min)
        inv_tti = 1.0 / tti_eff
        base = self._w_threat_dist * inv_d + self._w_threat_vz * (-vz) + self._w_threat_tti * inv_tti
        if vz < -self._threat_dive_thr:
            base += self._threat_dive_boost
        if d_origin < self._threat_crit_r:
            base += self._threat_crit_boost
        return base

    def _threat_reason_tags(self, d_origin: float, vz: float) -> str:
        parts: list[str] = []
        if d_origin < self._threat_crit_r:
            parts.append('crit')
        if vz < -self._threat_dive_thr:
            parts.append('dive')
        return ','.join(parts) if parts else 'base'

    def _stabilize_multi_assignment(
        self,
        active_t: list[str],
        active_i: list[str],
        cost: list[list[float | None]],
        greedy_assign: dict[str, str],
        threat_ordered_labels: list[str],
    ) -> dict[str, str]:
        """Prefer previous interceptor unless greedy option improves TTI by more than margin."""
        if not self._assign_stability or not active_i:
            self._multi_prev_assign = {k: v for k, v in greedy_assign.items()}
            return greedy_assign

        def tti_for(iid: str, ti: int) -> float | None:
            if iid not in active_i:
                return None
            ii = active_i.index(iid)
            return cost[ti][ii]

        self._multi_prev_assign = {k: v for k, v in self._multi_prev_assign.items() if k in active_t}
        stable: dict[str, str] = {}
        used: set[str] = set()
        margin = self._assign_switch_margin

        for tlabel in threat_ordered_labels:
            ti = active_t.index(tlabel)
            greedy_iid = greedy_assign.get(tlabel)
            prev_iid = self._multi_prev_assign.get(tlabel)
            pick: str | None = None

            if prev_iid and prev_iid in active_i and prev_iid not in used:
                ttp = tti_for(prev_iid, ti)
                ttg = tti_for(greedy_iid, ti) if greedy_iid else None
                if ttp is not None:
                    if ttg is None or ttg >= ttp - margin:
                        pick = prev_iid
            if pick is None and greedy_iid and greedy_iid not in used:
                pick = greedy_iid
            if pick is None and prev_iid and prev_iid in active_i and prev_iid not in used:
                ttp = tti_for(prev_iid, ti)
                if ttp is not None:
                    pick = prev_iid

            if pick:
                stable[tlabel] = pick
                used.add(pick)

        self._multi_prev_assign = dict(stable)
        return stable

    def _layer_from_dist(self, d: float) -> str:
        if not self._dome_enabled:
            return 'engage'
        if d > self._r_outer:
            return 'outside'
        if d > self._r_mid:
            return 'detect'
        if d > self._r_inner:
            return 'select'
        return 'engage'

    def _point_on_mid_shell(self, tx: float, ty: float, tz: float) -> tuple[float, float, float]:
        """
        จุดบนทรงกลมขอบ L1/L2: ทิศเดียวกับเป้า ระยะ r_mid จากศูนย์โดม.
        P_g = C + (P_T - C) * (r_mid / |P_T - C|)
        """
        cx, cy, cz = self._dome_cx, self._dome_cy, self._dome_cz
        dx, dy, dz = tx - cx, ty - cy, tz - cz
        d = _norm(dx, dy, dz)
        if d < 1e-9:
            return (tx, ty, tz)
        s = self._r_mid / d
        return (cx + dx * s, cy + dy * s, cz + dz * s)

    def _clear_assignments(self) -> None:
        self._locked_selected_id = None
        self._current_selected_id = None
        self._committed_since = None

    def _nearest_interceptor_id(self, tx: float, ty: float, tz: float) -> str | None:
        best: str | None = None
        best_d = float('inf')
        for iid in self._ids:
            p = self._inter_pos.get(iid)
            if p is None:
                continue
            d = _norm(tx - p.x, ty - p.y, tz - p.z)
            if d < best_d:
                best_d = d
                best = iid
        return best

    def _publish_dome_marker(self) -> None:
        if self._pub_dome_viz is None:
            return
        stamp = self.get_clock().now().to_msg()
        cx, cy, cz = float(self._dome_cx), float(self._dome_cy), float(self._dome_cz)

        clear = Marker()
        clear.header.frame_id = self._dome_frame
        clear.header.stamp = stamp
        clear.ns = 'danger_zone_layer'
        clear.action = Marker.DELETEALL
        self._pub_dome_viz.publish(clear)

        def _circ_xy(r: float, n: int) -> list[Point]:
            pts: list[Point] = []
            for i in range(n):
                tt = 2.0 * math.pi * i / n
                pts.append(Point(x=cx + r * math.cos(tt), y=cy + r * math.sin(tt), z=cz))
            pts.append(pts[0])
            return pts

        def _circ_xz(r: float, n: int) -> list[Point]:
            pts: list[Point] = []
            for i in range(n):
                tt = 2.0 * math.pi * i / n
                pts.append(Point(x=cx + r * math.cos(tt), y=cy, z=cz + r * math.sin(tt)))
            pts.append(pts[0])
            return pts

        def _circ_yz(r: float, n: int) -> list[Point]:
            pts: list[Point] = []
            for i in range(n):
                tt = 2.0 * math.pi * i / n
                pts.append(Point(x=cx, y=cy + r * math.cos(tt), z=cz + r * math.sin(tt)))
            pts.append(pts[0])
            return pts

        nseg = 56
        layers: list[tuple[float, tuple[float, float, float, float]]] = [
            (self._r_outer, (1.0, 0.35, 0.12, 0.92)),
            (self._r_mid, (1.0, 0.82, 0.12, 0.92)),
            (self._r_inner, (1.0, 0.18, 0.22, 0.95)),
        ]
        mid = 0
        for rad, rgba in layers:
            for circle_pts in (_circ_xy(rad, nseg), _circ_xz(rad, nseg), _circ_yz(rad, nseg)):
                m = Marker()
                m.header.frame_id = self._dome_frame
                m.header.stamp = stamp
                m.ns = 'danger_zone_wireframe'
                m.id = mid
                mid += 1
                m.type = Marker.LINE_STRIP
                m.action = Marker.ADD
                m.pose.orientation.w = 1.0
                m.points = circle_pts
                m.scale.x = 0.25
                m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
                m.lifetime.sec = 0
                self._pub_dome_viz.publish(m)

    def _build_tti_rows(
        self,
        tx: float,
        ty: float,
        tz: float,
        v_tx: float,
        v_ty: float,
        v_tz: float,
    ) -> list[tuple[str, bool, float | None]]:
        rows: list[tuple[str, bool, float | None]] = []
        for iid in self._ids:
            p = self._inter_pos.get(iid)
            if p is None:
                rows.append((iid, False, None))
                continue
            ix, iy, iz = float(p.x), float(p.y), float(p.z)
            ok, tti = _tti_feasible(
                tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz,
                self._closing, self._t_hit_min, self._t_hit_max,
            )
            rows.append((iid, ok, tti))
        return rows

    def _run_ram_guidance(
        self,
        tx: float,
        ty: float,
        tz: float,
        v_tx: float,
        v_ty: float,
        v_tz: float,
        selected: str,
        *,
        strike_ok: bool,
        layer_for_log: str = '',
    ) -> None:
        zero = Vector3()
        p_sel = self._inter_pos.get(selected)
        if p_sel is None:
            self._publish_all({i: zero for i in self._ids})
            return

        ix, iy, iz = float(p_sel.x), float(p_sel.y), float(p_sel.z)
        dist_to_target = _norm(tx - ix, ty - iy, tz - iz)
        self._record_miss_distance(dist_to_target)

        air_ok = self._hit_min_tz is None or tz >= self._hit_min_tz
        if (not self._hit) and dist_to_target < self._hit_thresh and air_ok and strike_ok:
            self._hit = True
            extra = f' layer={layer_for_log}' if layer_for_log else ''
            mm = f'{self._min_miss_distance:.4f} m' if self._min_miss_distance < float("inf") else 'n/a'
            print(
                f'[HIT] {selected}{extra}  min_miss={mm}  hit_threshold = {self._hit_thresh:.4f} m',
                flush=True,
            )
            self._publish_stop_signal(None)
            self._publish_all({i: zero for i in self._ids})
            self._pause_gazebo_world()
            return

        v_ix, v_iy, v_iz = self._estimate_interceptor_vel(selected, ix, iy, iz)
        dist = dist_to_target
        out_map = {i: Vector3() for i in self._ids}

        gx, gy, gz = tx, ty, tz
        if strike_ok and self._dome_enabled and self._aim_mid_shell:
            gx, gy, gz = self._point_on_mid_shell(tx, ty, tz)

        if dist < self._r_stop:
            self._publish_all(out_map)
            self._maybe_detail_log(tx, ty, tz, ix, iy, iz, dist, None, None, None, 0.0, 0.0, 0.0, 'hold', False, 0.0, selected)
            return

        mode = 'naive'
        t_hit: float | None = None
        phx = phy = phz = 0.0
        ux = uy = uz = 0.0

        sol = _compute_intercept(tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, self._closing)
        if sol is not None:
            t_hit, phx, phy, phz, ux, uy, uz = sol
            if self._t_hit_min <= t_hit <= self._t_hit_max and _norm(ux, uy, uz) > 1e-6:
                mode = 'predict'
            else:
                sol = None

        if sol is None or mode != 'predict':
            # Pure pursuit เป็นฐาน; ใน strike + aim_mid_shell ใช้จุดบนเปลือก r_mid เป็นทิศ LOS (ขอบชั้น 1–2)
            losx, losy, losz = _unit(gx - ix, gy - iy, gz - iz)
            bl = self._pursuit_lead_blend
            spv = abs(v_tx) + abs(v_ty) + abs(v_tz)
            if bl > 1e-6 and spv > 0.02:
                lx = tx + v_tx * self._naive_lead
                ly = ty + v_ty * self._naive_lead
                lz = tz + v_tz * self._naive_lead
                lox, loy, loz = _unit(lx - ix, ly - iy, lz - iz)
                if _norm(lox, loy, loz) > 1e-9:
                    ux = (1.0 - bl) * losx + bl * lox
                    uy = (1.0 - bl) * losy + bl * loy
                    uz = (1.0 - bl) * losz + bl * loz
                    un = _norm(ux, uy, uz)
                    if un > 1e-9:
                        ux, uy, uz = ux / un, uy / un, uz / un
                    else:
                        ux, uy, uz = losx, losy, losz
                else:
                    ux, uy, uz = losx, losy, losz
            else:
                ux, uy, uz = losx, losy, losz
            if _norm(ux, uy, uz) < 1e-9:
                ux, uy, uz = losx, losy, losz
            t_hit = None
            phx = phy = phz = float('nan')
            mode = 'pursuit'

        pn_active = False
        vc_log = 0.0
        if self._use_pn and self._pn_blend > 1e-9 and dist > 1e-6:
            # เมื่อ aim shell: ใช้เวกเตอร์สัมพัธ์ไปจุดบนเปลือกเพื่อให้สอดคล้อง pursuit
            px, py, pz = (gx, gy, gz) if (strike_ok and self._dome_enabled and self._aim_mid_shell) else (tx, ty, tz)
            rx, ry, rz = px - ix, py - iy, pz - iz
            v_rx = v_tx - v_ix
            v_ry = v_ty - v_iy
            v_rz = v_tz - v_iz
            sx, sy, sz, vc_log, pn_ok = _pn_steering_vector(
                rx, ry, rz, v_rx, v_ry, v_rz, self._pn_n, self._pn_min_vc,
            )
            if pn_ok:
                bx = ux + self._pn_blend * sx
                by = uy + self._pn_blend * sy
                bz = uz + self._pn_blend * sz
                un = _norm(bx, by, bz)
                if un > 1e-9:
                    ux, uy, uz = bx / un, by / un, bz / un
                    pn_active = True

        vx = ux * self._closing
        vy = uy * self._closing
        vz = uz * self._closing
        vx, vy, vz = self._clamp_speed(vx, vy, vz)
        cmd = Vector3()
        cmd.x, cmd.y, cmd.z = vx, vy, vz
        out_map[selected] = cmd
        self._publish_all(out_map)
        self._maybe_detail_log(
            tx, ty, tz, ix, iy, iz, dist, t_hit, phx, phy, phz, vx, vy, vz, mode, pn_active, vc_log, selected,
        )

    def _estimate_target_vel(self, tx: float, ty: float, tz: float) -> tuple[float, float, float]:
        now = self.get_clock().now()
        if self._prev_target is None or self._prev_target_time is None:
            self._prev_target = (tx, ty, tz)
            self._prev_target_time = now
            return self._v_tgt_smooth
        dt = (now - self._prev_target_time).nanoseconds * 1e-9
        px, py, pz = self._prev_target
        self._prev_target = (tx, ty, tz)
        self._prev_target_time = now
        if dt < 1e-3:
            return self._v_tgt_smooth
        vx = (tx - px) / dt
        vy = (ty - py) / dt
        vz = (tz - pz) / dt
        a = self._tgt_vel_smooth_a
        sx = a * vx + (1.0 - a) * self._v_tgt_smooth[0]
        sy = a * vy + (1.0 - a) * self._v_tgt_smooth[1]
        sz = a * vz + (1.0 - a) * self._v_tgt_smooth[2]
        self._v_tgt_smooth = (sx, sy, sz)
        return self._v_tgt_smooth

    def _estimate_target_vel_multi(self, label: str, tx: float, ty: float, tz: float) -> tuple[float, float, float]:
        now = self.get_clock().now()
        prev = self._multi_prev_target.get(label)
        prev_t = self._multi_prev_target_time.get(label)
        if prev is None or prev_t is None:
            self._multi_prev_target[label] = (tx, ty, tz)
            self._multi_prev_target_time[label] = now
            return self._multi_v_smooth.get(label, (0.0, 0.0, 0.0))
        dt = (now - prev_t).nanoseconds * 1e-9
        px, py, pz = prev
        self._multi_prev_target[label] = (tx, ty, tz)
        self._multi_prev_target_time[label] = now
        if dt < 1e-3:
            return self._multi_v_smooth.get(label, (0.0, 0.0, 0.0))
        vx = (tx - px) / dt
        vy = (ty - py) / dt
        vz = (tz - pz) / dt
        a = self._tgt_vel_smooth_a
        prev_s = self._multi_v_smooth.get(label, (0.0, 0.0, 0.0))
        sx = a * vx + (1.0 - a) * prev_s[0]
        sy = a * vy + (1.0 - a) * prev_s[1]
        sz = a * vz + (1.0 - a) * prev_s[2]
        self._multi_v_smooth[label] = (sx, sy, sz)
        return self._multi_v_smooth[label]

    def _estimate_interceptor_vel(
        self,
        iid: str,
        ix: float,
        iy: float,
        iz: float,
    ) -> tuple[float, float, float]:
        now = self.get_clock().now()
        prev = self._prev_inter_pos[iid]
        prev_t = self._prev_inter_time[iid]
        if prev is None or prev_t is None:
            self._prev_inter_pos[iid] = (ix, iy, iz)
            self._prev_inter_time[iid] = now
            return (0.0, 0.0, 0.0)
        dt = (now - prev_t).nanoseconds * 1e-9
        px, py, pz = prev
        self._prev_inter_pos[iid] = (ix, iy, iz)
        self._prev_inter_time[iid] = now
        if dt < 1e-3:
            return (0.0, 0.0, 0.0)
        return ((ix - px) / dt, (iy - py) / dt, (iz - pz) / dt)

    def _clamp_speed(self, vx: float, vy: float, vz: float) -> tuple[float, float, float]:
        n = _norm(vx, vy, vz)
        if n < 1e-9 or n <= self._vmax:
            return (vx, vy, vz)
        s = self._vmax / n
        return (vx * s, vy * s, vz * s)

    def _publish_all(self, velocities: dict[str, Vector3]) -> None:
        for iid in self._ids:
            self._pubs[iid].publish(velocities.get(iid, Vector3()))

    def _time_since_commit_s(self, now: Time) -> float:
        if self._committed_since is None:
            return float('inf')
        return (now - self._committed_since).nanoseconds * 1e-9

    def _apply_commit_change(self, old: str | None, new: str | None, now: Time) -> None:
        if old == new:
            return
        t_rel = (now - self._t0).nanoseconds * 1e-9
        if old is not None and new is not None:
            print(f'[SWITCH t={t_rel:.3f}s] {old} -> {new}', flush=True)
            self._switch_count += 1
        elif old is not None and new is None:
            print(f'[SWITCH t={t_rel:.3f}s] {old} -> (none)', flush=True)
            self._switch_count += 1
        self._current_selected_id = new
        self._committed_since = now

    def _maybe_hold_log(self, now: Time, msg: str) -> None:
        # Avoid spamming HOLD every control tick.
        if self._last_hold_log is None:
            self._last_hold_log = now
            print(msg, flush=True)
            return
        age = (now - self._last_hold_log).nanoseconds * 1e-9
        if age >= 0.5:
            self._last_hold_log = now
            print(msg, flush=True)

    def _update_committed_selection(
        self,
        rows: list[tuple[str, bool, float | None]],
        now: Time,
        tx: float,
        ty: float,
        tz: float,
    ) -> str | None:
        tti_by = {iid: (fe, tti) for iid, fe, tti in rows}
        feasible_cands = [(tti, iid) for iid, (fe, tti) in tti_by.items() if fe and tti is not None]
        best_id: str | None = None
        if self._dome_sel_mode == 'nearest':
            best_id = self._nearest_interceptor_id(tx, ty, tz)
            self._best_id_last = best_id
        else:
            if feasible_cands:
                best_id = min(feasible_cands, key=lambda x: (x[0], self._ids.index(x[1])))[1]
            self._best_id_last = best_id

        cur = self._current_selected_id

        # Lock selection after first feasible pick so only one interceptor will ever be commanded/launched.
        if self._lock_after_first and self._locked_selected_id is not None:
            self._current_selected_id = self._locked_selected_id
            return self._current_selected_id

        if cur is None:
            # If any interceptor is feasible, never commit to (none).
            self._apply_commit_change(None, best_id, now)
            if self._lock_after_first and self._current_selected_id is not None:
                self._locked_selected_id = self._current_selected_id
            return self._current_selected_id

        c_feas, c_tti = tti_by.get(cur, (False, None))
        any_feasible = best_id is not None

        # If the committed interceptor is currently infeasible, apply a grace period before dropping it.
        # Do not clear the lost timer on a single transient feasible tick; require reacquire_confirm_s.
        if not c_feas or c_tti is None:
            self._reacquire_since = None
            if self._lost_since is None:
                self._lost_since = now
            lost_age = (now - self._lost_since).nanoseconds * 1e-9

            if any_feasible:
                # Prefer to keep current during transient infeasible periods.
                if lost_age < self._lost_timeout_s:
                    self._maybe_hold_log(
                        now,
                        f'[HOLD] keeping {cur} despite temporary infeasible (lost_age={lost_age:.2f}s)',
                    )
                    return cur
                # After timeout, switch to best feasible (never to none if any feasible).
                self._apply_commit_change(cur, best_id, now)
                self._lost_since = None
                return self._current_selected_id

            # No feasible interceptors at all: allow (none) only after timeout.
            if lost_age < self._lost_timeout_s:
                self._maybe_hold_log(
                    now,
                    f'[HOLD] keeping {cur} despite all-infeasible (lost_age={lost_age:.2f}s)',
                )
                return cur
            self._apply_commit_change(cur, None, now)
            self._lost_since = None
            return self._current_selected_id

        # Committed is feasible again -> reset lost timer only after reacquire_confirm_s.
        if self._lost_since is not None and self._reacquire_confirm_s > 0.0:
            if self._reacquire_since is None:
                self._reacquire_since = now
            reacq_age = (now - self._reacquire_since).nanoseconds * 1e-9
            if reacq_age >= self._reacquire_confirm_s:
                self._lost_since = None
                self._reacquire_since = None
        else:
            self._lost_since = None
            self._reacquire_since = None

        new_id = cur
        if best_id is not None and best_id != cur:
            b_feas, b_tti = tti_by[best_id]
            dwell = self._time_since_commit_s(now)
            if (
                b_feas
                and b_tti is not None
                and c_tti is not None
                and b_tti + self._tti_margin < c_tti
                and dwell >= self._switch_window_s
            ):
                new_id = best_id

        self._apply_commit_change(cur, new_id, now)
        if self._lock_after_first and self._current_selected_id is not None and self._locked_selected_id is None:
            self._locked_selected_id = self._current_selected_id
        return self._current_selected_id

    def _multi_ram_velocity_cmd(
        self,
        tx: float,
        ty: float,
        tz: float,
        v_tx: float,
        v_ty: float,
        v_tz: float,
        selected: str,
        strike_ok: bool,
        ix: float,
        iy: float,
        iz: float,
        dist: float,
    ) -> Vector3:
        self._record_miss_distance(dist)
        v_ix, v_iy, v_iz = self._estimate_interceptor_vel(selected, ix, iy, iz)
        zero = Vector3()
        gx, gy, gz = tx, ty, tz
        if strike_ok and self._dome_enabled and self._aim_mid_shell:
            gx, gy, gz = self._point_on_mid_shell(tx, ty, tz)
        if dist < self._r_stop:
            return zero
        sol = _compute_intercept(tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, self._closing)
        ux = uy = uz = 0.0
        use_predict = False
        if sol is not None:
            t_hit, _phx, _phy, _phz, ux, uy, uz = sol
            if self._t_hit_min <= t_hit <= self._t_hit_max and _norm(ux, uy, uz) > 1e-6:
                use_predict = True
        if not use_predict:
            losx, losy, losz = _unit(gx - ix, gy - iy, gz - iz)
            bl = self._pursuit_lead_blend
            spv = abs(v_tx) + abs(v_ty) + abs(v_tz)
            if bl > 1e-6 and spv > 0.02:
                lx = tx + v_tx * self._naive_lead
                ly = ty + v_ty * self._naive_lead
                lz = tz + v_tz * self._naive_lead
                lox, loy, loz = _unit(lx - ix, ly - iy, lz - iz)
                if _norm(lox, loy, loz) > 1e-9:
                    ux = (1.0 - bl) * losx + bl * lox
                    uy = (1.0 - bl) * losy + bl * loy
                    uz = (1.0 - bl) * losz + bl * loz
                    un = _norm(ux, uy, uz)
                    if un > 1e-9:
                        ux, uy, uz = ux / un, uy / un, uz / un
                    else:
                        ux, uy, uz = losx, losy, losz
                else:
                    ux, uy, uz = losx, losy, losz
            else:
                ux, uy, uz = losx, losy, losz
            if _norm(ux, uy, uz) < 1e-9:
                ux, uy, uz = losx, losy, losz
        if self._use_pn and self._pn_blend > 1e-9 and dist > 1e-6:
            px, py, pz = (gx, gy, gz) if (strike_ok and self._dome_enabled and self._aim_mid_shell) else (tx, ty, tz)
            rx, ry, rz = px - ix, py - iy, pz - iz
            v_rx = v_tx - v_ix
            v_ry = v_ty - v_iy
            v_rz = v_tz - v_iz
            sx, sy, sz, _vc_log, pn_ok = _pn_steering_vector(
                rx, ry, rz, v_rx, v_ry, v_rz, self._pn_n, self._pn_min_vc,
            )
            if pn_ok:
                bx = ux + self._pn_blend * sx
                by = uy + self._pn_blend * sy
                bz = uz + self._pn_blend * sz
                un = _norm(bx, by, bz)
                if un > 1e-9:
                    ux, uy, uz = bx / un, by / un, bz / un
        vx = ux * self._closing
        vy = uy * self._closing
        vz = uz * self._closing
        vx, vy, vz = self._clamp_speed(vx, vy, vz)
        cmd = Vector3()
        cmd.x, cmd.y, cmd.z = vx, vy, vz
        return cmd

    def _log_active_targets_remaining(self) -> None:
        remaining = [lab for lab in self._multi_labels if not self._hits_multi.get(lab, False)]
        if not remaining:
            print('[ACTIVE TARGETS] remaining: (none)', flush=True)
        else:
            print(f'[ACTIVE TARGETS] remaining: {", ".join(remaining)}', flush=True)

    def _multi_try_hit(
        self,
        tx: float,
        ty: float,
        tz: float,
        tlabel: str,
        iid: str,
        layer_for_log: str,
        *,
        strike_ok: bool,
    ) -> bool:
        if self._hits_multi.get(tlabel):
            return False
        p_sel = self._inter_pos.get(iid)
        if p_sel is None:
            return False
        ix, iy, iz = float(p_sel.x), float(p_sel.y), float(p_sel.z)
        dist_to_target = _norm(tx - ix, ty - iy, tz - iz)
        self._record_miss_distance(dist_to_target)
        air_ok = self._hit_min_tz is None or tz >= self._hit_min_tz
        if dist_to_target < self._hit_thresh and air_ok and strike_ok:
            self._hits_multi[tlabel] = True
            extra = f' layer={layer_for_log}' if layer_for_log else ''
            mm = f'{self._min_miss_distance:.4f} m' if self._min_miss_distance < float("inf") else 'n/a'
            print(
                f'[HIT] {tlabel} by {iid}{extra}  min_miss={mm}  hit_threshold = {self._hit_thresh:.4f} m',
                flush=True,
            )
            self._publish_stop_signal(tlabel)
            self._log_active_targets_remaining()
            if self._pause_gz_on_hit:
                self._pause_gazebo_world()
            return True
        return False

    def _maybe_print_threat_and_assignment(
        self,
        threat_ranking: list[tuple[str, float, str]],
        assign: dict[str, str],
    ) -> None:
        now = self.get_clock().now()
        if self._last_assignment_print is not None:
            if (now - self._last_assignment_print).nanoseconds * 1e-9 < self._assignment_print_period:
                return
        self._last_assignment_print = now
        lines: list[str] = []
        if threat_ranking:
            lines.append('=== Threat Ranking ===')
            lines.append(
                f'[THREAT] weights dist={self._w_threat_dist} vz={self._w_threat_vz} tti={self._w_threat_tti} | '
                f'dive: vz<-{self._threat_dive_thr:g} => +{self._threat_dive_boost} | '
                f'crit: d<{self._threat_crit_r:g}m => +{self._threat_crit_boost} | '
                f'stability={self._assign_stability} switch_margin_tti={self._assign_switch_margin}s',
            )
            for lab, sc, tags in threat_ranking:
                lines.append(f'{lab}: score={sc:.2f} [{tags}]')
        lines.append('=== Assignment ===')
        for lab in self._multi_labels:
            intr = assign.get(lab)
            if intr is None:
                lines.append(f'{lab} -> none')
            else:
                lines.append(f'{lab} -> {intr}')
        print('\n'.join(lines), flush=True)

    def _publish_assignment_strings(self, assign: dict[str, str]) -> None:
        inv: dict[str, str] = {}
        for lab, iid in assign.items():
            inv[iid] = lab
        for iid in self._ids:
            self._pub_assigned[iid].publish(String(data=inv.get(iid, '')))

    def _on_control_multi(self) -> None:
        zero = Vector3()
        self._pub_selected.publish(String(data=''))
        if all(self._hits_multi.get(l, False) for l in self._multi_labels):
            self._publish_assignment_strings({})
            self._publish_all({i: zero for i in self._ids})
            return
        if not any(self._targets.get(l) is not None for l in self._multi_labels):
            self._publish_assignment_strings({})
            self._publish_all({i: zero for i in self._ids})
            return

        v_by: dict[str, tuple[float, float, float]] = {}
        for lab in self._multi_labels:
            if self._hits_multi.get(lab):
                continue
            pt = self._targets.get(lab)
            if pt is None:
                continue
            tx, ty, tz = float(pt.x), float(pt.y), float(pt.z)
            v_by[lab] = self._estimate_target_vel_multi(lab, tx, ty, tz)

        layer_pub = 'engage'
        for lab in self._multi_labels:
            if self._hits_multi.get(lab):
                continue
            pt = self._targets.get(lab)
            if pt is None:
                continue
            d0 = self._dist_threat(float(pt.x), float(pt.y), float(pt.z))
            layer_pub = self._layer_from_dist(d0)
            break
        self._pub_layer.publish(String(data=layer_pub))

        active_t: list[str] = []
        for lab in self._multi_labels:
            if self._hits_multi.get(lab):
                continue
            pt = self._targets.get(lab)
            if pt is None:
                continue
            tx, ty, tz = float(pt.x), float(pt.y), float(pt.z)
            if self._dome_enabled and self._dist_threat(tx, ty, tz) > self._r_outer:
                continue
            active_t.append(lab)

        active_i = list(self._ids)
        assign: dict[str, str] = {}
        n_t = len(active_t)
        n_i = len(active_i)
        threat_ranking: list[tuple[str, float, str]] = []
        if n_t > 0:
            cost: list[list[float | None]] = (
                [[None] * n_i for _ in range(n_t)] if n_i > 0 else [[] for _ in range(n_t)]
            )
            if n_i > 0:
                for ti, tlabel in enumerate(active_t):
                    pt = self._targets[tlabel]
                    assert pt is not None
                    tx, ty, tz = float(pt.x), float(pt.y), float(pt.z)
                    vx, vy, vz = v_by[tlabel]
                    for ii, iid in enumerate(active_i):
                        p = self._inter_pos.get(iid)
                        if p is None:
                            continue
                        ix, iy, iz = float(p.x), float(p.y), float(p.z)
                        ok, tti = _tti_feasible(
                            tx, ty, tz, vx, vy, vz, ix, iy, iz,
                            self._closing, self._t_hit_min, self._t_hit_max,
                        )
                        if ok and tti is not None:
                            cost[ti][ii] = tti

            threat_items: list[tuple[float, str, str]] = []
            for ti, tlabel in enumerate(active_t):
                pt = self._targets[tlabel]
                assert pt is not None
                tx, ty, tz = float(pt.x), float(pt.y), float(pt.z)
                _, _, vz = v_by[tlabel]
                d_origin = self._dist_threat(tx, ty, tz)
                row = cost[ti] if n_i > 0 else []
                finite_tti = [x for x in row if x is not None]
                min_tti = min(finite_tti) if finite_tti else None
                sc = self._threat_score(d_origin, vz, min_tti)
                tags = self._threat_reason_tags(d_origin, vz)
                threat_items.append((sc, tlabel, tags))
            threat_items.sort(key=lambda s: (-s[0], self._multi_labels.index(s[1])))
            threat_ranking = [(lab, sc, tags) for sc, lab, tags in threat_items]
            threat_ordered_labels = [lab for sc, lab, tags in threat_items]

            if n_i > 0:
                remaining = set(active_i)
                for _sc, tlabel, _tags in threat_items:
                    ti = active_t.index(tlabel)
                    best_iid: str | None = None
                    best_tti: float | None = None
                    for ii, iid in enumerate(active_i):
                        if iid not in remaining:
                            continue
                        tti = cost[ti][ii]
                        if tti is None:
                            continue
                        if best_tti is None or tti < best_tti - 1e-12:
                            best_tti = float(tti)
                            best_iid = iid
                        elif best_tti is not None and abs(float(tti) - best_tti) <= 1e-12 and best_iid is not None:
                            if self._ids.index(iid) < self._ids.index(best_iid):
                                best_iid = iid
                    if best_iid is not None:
                        assign[tlabel] = best_iid
                        remaining.discard(best_iid)

                assign = self._stabilize_multi_assignment(
                    active_t, active_i, cost, assign, threat_ordered_labels,
                )

        self._maybe_print_threat_and_assignment(threat_ranking, assign)
        self._publish_assignment_strings(assign)

        out_map = {i: zero for i in self._ids}
        for tlabel, iid in assign.items():
            if self._hits_multi.get(tlabel):
                continue
            pt = self._targets.get(tlabel)
            if pt is None:
                continue
            tx, ty, tz = float(pt.x), float(pt.y), float(pt.z)
            vx, vy, vz = v_by[tlabel]
            d_th = self._dist_threat(tx, ty, tz)
            layer = self._layer_from_dist(d_th)

            if self._dome_enabled:
                if d_th > self._r_outer:
                    continue
                if self._in_strike_zone(d_th):
                    if self._multi_try_hit(tx, ty, tz, tlabel, iid, layer, strike_ok=True):
                        continue
                    p_sel = self._inter_pos.get(iid)
                    if p_sel is None:
                        continue
                    ix, iy, iz = float(p_sel.x), float(p_sel.y), float(p_sel.z)
                    dist = _norm(tx - ix, ty - iy, tz - iz)
                    out_map[iid] = self._multi_ram_velocity_cmd(
                        tx, ty, tz, vx, vy, vz, iid, True, ix, iy, iz, dist,
                    )
                elif d_th > self._r_mid:
                    continue
                elif d_th > self._r_inner:
                    continue
                else:
                    continue
            else:
                if self._multi_try_hit(tx, ty, tz, tlabel, iid, layer, strike_ok=True):
                    continue
                p_sel = self._inter_pos.get(iid)
                if p_sel is None:
                    continue
                ix, iy, iz = float(p_sel.x), float(p_sel.y), float(p_sel.z)
                dist = _norm(tx - ix, ty - iy, tz - iz)
                out_map[iid] = self._multi_ram_velocity_cmd(
                    tx, ty, tz, vx, vy, vz, iid, True, ix, iy, iz, dist,
                )

        self._publish_all(out_map)
        self._maybe_emit_miss_distance_log_multi()

    def _on_control(self) -> None:
        if self._multi_enabled:
            self._on_control_multi()
            return
        zero = Vector3()
        if self._target is None:
            self._publish_all({i: zero for i in self._ids})
            return

        # After impact: do not run selection or guidance; keep target stopped and interceptors frozen.
        if self._hit:
            self._publish_stop_signal(None)
            self._publish_all({i: zero for i in self._ids})
            return

        tx, ty, tz = float(self._target.x), float(self._target.y), float(self._target.z)
        v_tx, v_ty, v_tz = self._estimate_target_vel(tx, ty, tz)

        d_th = self._dist_threat(tx, ty, tz)
        layer = self._layer_from_dist(d_th)
        self._pub_layer.publish(String(data=layer))
        if layer != self._last_layer:
            print(f'[LAYER] {layer}', flush=True)
        self._last_layer = layer

        if self._dome_enabled:
            if d_th > self._r_outer:
                if self._reset_lock_outside:
                    self._clear_assignments()
                self._publish_all({i: zero for i in self._ids})
                self._pub_selected.publish(String(data=''))
                return

            if self._in_strike_zone(d_th):
                nid = self._nearest_interceptor_id(tx, ty, tz)
                if nid is not None:
                    self._locked_selected_id = nid
                    self._current_selected_id = nid
                rows = self._build_tti_rows(tx, ty, tz, v_tx, v_ty, v_tz)
                selected = self._locked_selected_id or self._nearest_interceptor_id(tx, ty, tz)
                self._pub_selected.publish(String(data=selected if selected is not None else ''))
                self._maybe_selection_log(rows, selected, tx, ty, tz, layer)
                self._maybe_print_selection_algorithm_verbose(
                    'strike_shell',
                    tx, ty, tz, v_tx, v_ty, v_tz, rows, selected, layer, d_th,
                )
                if selected is None:
                    self._publish_all({i: zero for i in self._ids})
                    return
                self._run_ram_guidance(
                    tx, ty, tz, v_tx, v_ty, v_tz, selected, strike_ok=True, layer_for_log=layer,
                )
                return

            # ในโดมแต่ไม่อยู่แถบชนรอบ r_mid
            if d_th > self._r_mid:
                self._clear_assignments()
                self._publish_all({i: zero for i in self._ids})
                self._pub_selected.publish(String(data=''))
                return
            if d_th > self._r_inner:
                selected = self._locked_selected_id or self._nearest_interceptor_id(tx, ty, tz)
                self._pub_selected.publish(String(data=selected if selected is not None else ''))
                self._publish_all({i: zero for i in self._ids})
                return
            selected = self._locked_selected_id or self._nearest_interceptor_id(tx, ty, tz)
            self._pub_selected.publish(String(data=selected if selected is not None else ''))
            self._publish_all({i: zero for i in self._ids})
            return

        # dome ปิด: ยังไล่ได้ทั้งปริมาตร (ชั้นเดียวใน logic)
        rows = self._build_tti_rows(tx, ty, tz, v_tx, v_ty, v_tz)
        now = self.get_clock().now()
        selected = self._update_committed_selection(rows, now, tx, ty, tz)
        sel_msg = String()
        sel_msg.data = selected if selected is not None else ''
        self._pub_selected.publish(sel_msg)
        self._maybe_selection_log(rows, selected, tx, ty, tz, layer)
        self._maybe_print_selection_algorithm_verbose(
            'dome_off_hysteresis',
            tx, ty, tz, v_tx, v_ty, v_tz, rows, selected, layer, d_th,
        )

        if selected is None:
            self._publish_all({i: zero for i in self._ids})
            return

        self._run_ram_guidance(tx, ty, tz, v_tx, v_ty, v_tz, selected, strike_ok=True, layer_for_log=layer)

    def _maybe_print_selection_algorithm_verbose(
        self,
        tag: str,
        tx: float,
        ty: float,
        tz: float,
        v_tx: float,
        v_ty: float,
        v_tz: float,
        rows: list[tuple[str, bool, float | None]],
        selected: str | None,
        layer: str,
        d_th: float,
    ) -> None:
        if not self._sel_algo_verbose:
            return
        now = self.get_clock().now()
        if self._last_algo_log is not None:
            if (now - self._last_algo_log).nanoseconds * 1e-9 < self._sel_algo_period:
                return
        self._last_algo_log = now

        tol_s = (
            'tol_intercept = max(0.12, 5e-4 * max(|r(t)|, s_i*t, 1))  '
            '— accept candidate if | |r(t)| - s_i*t | <= tol_intercept'
        )
        lines: list[str] = [
            '',
            f'>>>>>>>>>> SELECTION ALGORITHM [{tag}] <<<<<<<<<<',
            f'Target position P_T = ({tx:.6f}, {ty:.6f}, {tz:.6f})  (smoothed) velocity v_T = ({v_tx:.6f}, {v_ty:.6f}, {v_tz:.6f})',
            f'  EMA: v <- alpha * (P_T-P_prev)/dt + (1-alpha)*v_prev   alpha = {self._tgt_vel_smooth_a}',
            f'Closing speed s_i = {self._closing:.6f} m/s  |  TTI in [{self._t_hit_min}, {self._t_hit_max}] s  |  {tol_s}',
            f'Layer (dome) = {layer}   d_threat = |P_T - C| = {d_th:.6f} m   C = ({self._dome_cx},{self._dome_cy},{self._dome_cz})',
            f'r_outer={self._r_outer}  r_mid={self._r_mid}  r_inner={self._r_inner}  strike_hw={self._strike_shell_hw}',
            '',
        ]

        for iid in self._ids:
            p = self._inter_pos.get(iid)
            lines.append(f'==== Interceptor {iid} ====')
            if p is None:
                lines.append('  (no position yet)')
                lines.append('')
                continue
            ix, iy, iz = float(p.x), float(p.y), float(p.z)
            dist_3d = _norm(tx - ix, ty - iy, tz - iz)
            lines.append(f'  P_I = ({ix:.6f}, {iy:.6f}, {iz:.6f})')
            lines.append(f'  |P_T - P_I| = {dist_3d:.6f} m')
            tr: list[str] = []
            t_hit = _solve_intercept_time(
                tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, self._closing, trace=tr,
            )
            lines.extend(tr)
            if t_hit is None:
                lines.append('  TTI (feasibility row): infeasible (no valid t)')
            else:
                ok_w = self._t_hit_min <= t_hit <= self._t_hit_max
                lines.append(
                    f'  TTI row check: t_hit={t_hit:.8f} in [{self._t_hit_min}, {self._t_hit_max}] -> feasible={ok_w}',
                )
            row_match = next((r for r in rows if r[0] == iid), None)
            if row_match:
                lines.append(f'  (cached row: feasible={row_match[1]}, tti={row_match[2]})')
            lines.append('')

        lines.append('--- Summary rows (same as _build_tti_rows / _tti_feasible) ---')
        for iid, fe, tti in rows:
            lines.append(f'  {iid}: feasible={fe}, TTI={tti}')

        lines.append(f'Selected id (this tick) = {selected!r}')
        lines.append('')

        if tag == 'strike_shell':
            lines.append('Rule (inside strike shell |d - r_mid| <= hw): shooter = nearest 3D to target.')
            if self._strike_shell_hw > 1e-6:
                lines.append(f'  |d_th - r_mid| = |{d_th:.6f} - {self._r_mid}| = {abs(d_th - self._r_mid):.6f} <= {self._strike_shell_hw}')
            dlist: list[tuple[float, str]] = []
            for iid in self._ids:
                p = self._inter_pos.get(iid)
                if p is None:
                    continue
                d = _norm(tx - p.x, ty - p.y, tz - p.z)
                dlist.append((d, iid))
            dlist.sort(key=lambda x: x[0])
            for d, iid in dlist:
                lines.append(f'  |P_T-{iid}| = {d:.6f} m')
            if dlist:
                lines.append(f'  => argmin distance = {dlist[0][1]} (nominal shooter)')
            lines.append(f'  lock: locked_selected_id = {self._locked_selected_id!r}  current_selected_id = {self._current_selected_id!r}')

        elif tag == 'dome_off_hysteresis':
            lines.append('Rules (dome off): committed interceptor via hysteresis + TTI preference.')
            lines.append(
                f'  dome_selection_mode = {self._dome_sel_mode!r}  '
                f'(nearest -> argmin |P_T-P_I|; tti -> argmin TTI among feasible)',
            )
            lines.append(
                f'  margin delta_t = {self._tti_margin}s  |  switch_window_s = {self._switch_window_s}s  '
                f'| lost_timeout = {self._lost_timeout_s}s',
            )
            lines.append(f'  lock_after_first = {self._lock_after_first}  locked_selected_id = {self._locked_selected_id!r}')
            dwell = self._time_since_commit_s(now)
            lines.append(
                f'  committed id = {self._current_selected_id!r}  dwell_since_commit = {dwell:.4f}s '
                f'(inf if none)',
            )
            tti_by = {iid: (fe, tti) for iid, fe, tti in rows}
            feas = [(tti, iid) for iid, (fe, tti) in tti_by.items() if fe and tti is not None]
            lines.append(f'  feasible set (tti, id): {feas}')
            if self._dome_sel_mode == 'nearest':
                best_for_rule = self._nearest_interceptor_id(tx, ty, tz)
                lines.append(f'  best_id (nearest 3D) = {best_for_rule!r}')
            else:
                best_for_rule = min(feas, key=lambda x: (x[0], self._ids.index(x[1])))[1] if feas else None
                lines.append(f'  best_id (min TTI) = {best_for_rule!r}')
            cur = self._current_selected_id
            if cur is not None and best_for_rule is not None and feas:
                c_tti = tti_by.get(cur, (False, None))[1]
                b_tti = tti_by.get(best_for_rule, (False, None))[1]
                if c_tti is not None and b_tti is not None and best_for_rule != cur:
                    lines.append(
                        f'  Reassignment inequality: tti(best)+margin < tti(commit)  ->  '
                        f'{b_tti:.4f}+{self._tti_margin} < {c_tti:.4f}  =>  {b_tti + self._tti_margin < c_tti}',
                    )
                    lines.append(
                        f'  AND dwell {dwell:.4f}s >= switch_window {self._switch_window_s}s  =>  {dwell >= self._switch_window_s}',
                    )
        lines.append('>>>>>>>>>> END SELECTION DUMP <<<<<<<<<<')
        print('\n'.join(lines), flush=True)

    def _maybe_selection_log(
        self,
        rows: list[tuple[str, bool, float | None]],
        selected: str | None,
        tx: float,
        ty: float,
        tz: float,
        layer: str,
    ) -> None:
        now = self.get_clock().now()
        if (now - self._last_sel_log).nanoseconds * 1e-9 < self._sel_log_period:
            return
        self._last_sel_log = now
        lines = ['=== Interceptor Selection ===']
        for iid, fe, tti in rows:
            if tti is None:
                t_s = 'n/a'
            else:
                t_s = f'{tti:.3f}'
            lines.append(f'{iid}: feasible={fe}, tti={t_s}')
        lines.append(f'selected: {selected if selected is not None else "(none)"}')
        lines.append(f'best_tti: {self._best_id_last if self._best_id_last is not None else "(none)"}')
        lines.append(f'switch_count: {self._switch_count}')
        lines.append(f'layer: {layer}')
        # Use ROS logger so the table reliably appears in `ros2 launch` output.
        self.get_logger().info('\n'.join(lines))

    def _intercept_alignment(
        self,
        tx: float,
        ty: float,
        tz: float,
        ix: float,
        iy: float,
        iz: float,
        vx: float,
        vy: float,
        vz: float,
    ) -> float | None:
        """
        r = P_T - P_I, v_cmd = commanded velocity.
        alignment = dot(unit(r), unit(v_cmd)) — ~1 means cmd parallel to LOS (pure chase);
        lower values mean intercept geometry (lead / PN) off the instantaneous LOS.
        """
        rx = tx - ix
        ry = ty - iy
        rz = tz - iz
        urx, ury, urz = _unit(rx, ry, rz)
        uvx, uvy, uvz = _unit(vx, vy, vz)
        if _norm(urx, ury, urz) < 1e-12 or _norm(uvx, uvy, uvz) < 1e-12:
            return None
        return _dot(urx, ury, urz, uvx, uvy, uvz)

    def _maybe_detail_log(
        self,
        tx: float,
        ty: float,
        tz: float,
        ix: float,
        iy: float,
        iz: float,
        dist: float,
        t_hit: float | None,
        phx: float,
        phy: float,
        phz: float,
        vx: float,
        vy: float,
        vz: float,
        mode: str,
        pn_active: bool,
        vc: float,
        selected: str,
    ) -> None:
        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds * 1e-9 < self._log_period:
            return
        self._last_log = now
        if t_hit is not None and math.isfinite(phx):
            phs = f'({phx:.2f},{phy:.2f},{phz:.2f})'
            ts = f'{t_hit:.2f}'
        else:
            phs = '(n/a)'
            ts = 'n/a'
        pn_s = 'on' if pn_active else 'off'
        if self._intercept_debug:
            al = self._intercept_alignment(tx, ty, tz, ix, iy, iz, vx, vy, vz)
            align_s = f'{al:.4f}' if al is not None else 'n/a'
            print(
                '[Intercept Debug]\n'
                f'mode={mode}\n'
                f'distance={dist:.4f}\n'
                f't_hit={ts}\n'
                f'target_pos=({tx:.4f},{ty:.4f},{tz:.4f})\n'
                f'interceptor_pos=({ix:.4f},{iy:.4f},{iz:.4f})\n'
                f'cmd_vel=({vx:.4f},{vy:.4f},{vz:.4f})\n'
                f'alignment={align_s}',
                flush=True,
            )
        else:
            print(
                f'[interception] sel={selected} mode={mode} pn={pn_s} dist={dist:.2f} m | t_hit={ts} s p_hit={phs} | '
                f'Vc={vc:.2f} m/s | target=({tx:.2f},{ty:.2f},{tz:.2f}) inter=({ix:.2f},{iy:.2f},{iz:.2f}) | '
                f'cmd_vel=({vx:.2f},{vy:.2f},{vz:.2f})',
                flush=True,
            )
        self._print_miss_distance_lines()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = InterceptionLogicNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
