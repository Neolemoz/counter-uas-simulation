"""
Multi-interceptor: hysteresis TTI reassignment + predictive intercept + PN on the committed unit only.

**Three-layer dome:** ตามนโยบาย ``engagement_layer`` — ค่า ``select`` จำกัด HIT ให้เกิดเมื่อ ``d_threat``
อยู่ใน **select+engage** (``d ≤ dome_middle_m``) เท่านั้น; ค่า ``detect`` อนุญาต HIT ตั้งแต่ **detect annulus**
(``dome_middle_m < d ≤ dome_outer_m``) เป็นต้นไป — เหมาะสำหรับ **ram แถบไกล** ร่วมกับ feasibility.

เมื่อ ``engagement_layer`` ว่าง: กลับไปใช้แถบ ``|d - r_mid| ≤ strike_shell_half_width_m`` เท่านั้น.
Topic ``/danger_zone/layer``.

**Feasibility-based engagement (opt-in):** When ``feasibility_based_engagement`` is true, interceptors fly
only if ``is_intercept_feasible`` passes (intercept time window + minimum closing speed ≤ interceptor cap).
Radar/track/classification gates apply first; domes remain **policy** for HIT legality, not the sole trigger.

**Scenario (counter-UAS):** interceptors start near ground; guidance is **3-D** with ``vz`` to climb.
"""

from __future__ import annotations

import math
import random
import shutil
import time
from pathlib import Path
import subprocess
from collections import deque
from typing import TYPE_CHECKING, NamedTuple

import rclpy
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
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


def compute_required_speed(
    p_ix: float,
    p_iy: float,
    p_iz: float,
    p_hx: float,
    p_hy: float,
    p_hz: float,
    t_go: float,
) -> float:
    """
    Minimum constant speed along a straight segment P_I → P_hit to cover the distance in t_go s:

        v_req = |P_hit - P_I| / t_go
    """
    if t_go <= 0.0 or not math.isfinite(t_go):
        return float('inf')
    dist = _norm(p_hx - p_ix, p_hy - p_iy, p_hz - p_iz)
    if not math.isfinite(dist):
        return float('inf')
    return dist / t_go


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


def _closest_dist_segments_3d(
    p1: tuple[float, float, float],
    p2: tuple[float, float, float],
    p3: tuple[float, float, float],
    p4: tuple[float, float, float],
) -> float:
    """
    Minimum distance between closed segments ``p1→p2`` and ``p3→p4``.

    Based on Ericson, *Real-Time Collision Detection* (segment–segment closest points).
    Used to register kinetic HIT when the closest pass happens *between* ROS ticks (tunneling).
    """
    ux, uy, uz = p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]
    vx, vy, vz = p4[0] - p3[0], p4[1] - p3[1], p4[2] - p3[2]
    wx, wy, wz = p1[0] - p3[0], p1[1] - p3[1], p1[2] - p3[2]
    small = 1e-12
    a = ux * ux + uy * uy + uz * uz
    b = ux * vx + uy * vy + uz * vz
    c = vx * vx + vy * vy + vz * vz
    d = ux * wx + uy * wy + uz * wz
    e = vx * wx + vy * wy + vz * wz
    dd = a * c - b * b
    s_d = dd
    s_n = 0.0
    t_d = dd
    t_n = 0.0
    if dd < small:
        s_n = 0.0
        s_d = 1.0
        t_n = e
        t_d = c if abs(c) > small else 1.0
    else:
        s_n = (b * e) - (c * d)
        t_n = (a * e) - (b * d)
        if s_n < 0.0:
            s_n = 0.0
            t_n = e
            t_d = c if abs(c) > small else 1.0
        elif s_n > s_d:
            s_n = s_d
            t_n = e + b
            t_d = c if abs(c) > small else 1.0
    if t_n < 0.0:
        t_n = 0.0
        if -d < 0.0:
            s_n = 0.0
        elif -d > a:
            s_n = s_d
        else:
            s_n = -d
            s_d = a if abs(a) > small else 1.0
    elif abs(t_d) > small and t_n > t_d:
        t_n = t_d
        if (-d + b) < 0.0:
            s_n = 0.0
        elif (-d + b) > a:
            s_n = s_d
        else:
            s_n = -d + b
            s_d = a if abs(a) > small else 1.0
    sc = 0.0 if abs(s_n) < small else s_n / s_d
    tc = 0.0 if abs(t_n) < small else t_n / t_d
    dpx = wx + sc * ux - tc * vx
    dpy = wy + sc * uy - tc * vy
    dpz = wz + sc * uz - tc * vz
    return float(_norm(dpx, dpy, dpz))


def _rotate_dir_toward(
    ax: float, ay: float, az: float,
    bx: float, by: float, bz: float,
    max_angle: float,
) -> tuple[float, float, float]:
    """
    Rotate unit vector **a** toward unit vector **b** by at most *max_angle* radians.

    Uses spherical linear interpolation (SLERP) in the a–b plane so the output
    is always a unit vector and the rotation rate is constant.

    Edge cases:
      - a ≈ b  (angle ≈ 0): returns b directly.
      - a ≈ −b (angle ≈ π): returns b directly (degenerate plane).
      - max_angle ≥ angle: returns b directly (no clamping needed).
    """
    dot = max(-1.0, min(1.0, _dot(ax, ay, az, bx, by, bz)))
    angle = math.acos(dot)
    if angle <= max_angle or angle < 1e-9:
        return (bx, by, bz)
    t = max_angle / angle          # fraction of rotation to apply
    sin_a = math.sin(angle)
    if sin_a < 1e-9:               # a and b are antiparallel — degenerate
        return (bx, by, bz)
    fa = math.sin((1.0 - t) * angle) / sin_a
    fb = math.sin(t * angle) / sin_a
    rx = fa * ax + fb * bx
    ry = fa * ay + fb * by
    rz = fa * az + fb * bz
    rn = _norm(rx, ry, rz)
    if rn < 1e-9:
        return (ax, ay, az)
    return (rx / rn, ry / rn, rz / rn)


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
            trace.append(
                '    no valid intercept root: no candidate passed geometric tolerance (|r(t)| ≈ s_i*t); returning None',
            )
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


def _dashed_line_list_points(
    ax: float,
    ay: float,
    az: float,
    bx: float,
    by: float,
    bz: float,
    dash_len: float,
    gap_len: float,
) -> list[Point]:
    """Points for Marker LINE_LIST: segment pairs along A→B (approx dashed)."""
    out: list[Point] = []
    dx, dy, dz = bx - ax, by - ay, bz - az
    L = _norm(dx, dy, dz)
    if L < 1e-6 or dash_len <= 0.0:
        return out
    ux, uy, uz = dx / L, dy / L, dz / L
    t = 0.0
    draw_dash = True
    while t < L - 1e-9:
        if draw_dash:
            t1 = min(t + dash_len, L)
            out.append(Point(x=ax + ux * t, y=ay + uy * t, z=az + uz * t))
            out.append(Point(x=ax + ux * t1, y=ay + uy * t1, z=az + uz * t1))
            t = t1
        else:
            t = min(t + max(gap_len, 1e-6), L)
        draw_dash = not draw_dash
    return out


def _rgba_from_param(val: object, default: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    if isinstance(val, list) and len(val) >= 4:
        return (float(val[0]), float(val[1]), float(val[2]), float(val[3]))
    return default


def _trail_deque(max_points: int) -> deque[tuple[float, float, float]]:
    """Trail buffer: max_points <= 0 keeps up to 200k samples (full-history mode)."""
    cap = 200_000 if max_points <= 0 else max(200, int(max_points))
    return deque(maxlen=cap)


def _trail_push(
    d: deque[tuple[float, float, float]],
    x: float,
    y: float,
    z: float,
    min_step: float,
) -> None:
    if not d:
        d.append((x, y, z))
        return
    lx, ly, lz = d[-1]
    if _norm(x - lx, y - ly, z - lz) >= min_step:
        d.append((x, y, z))


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


def _t_in_feasible_window(t: float | None, t_min: float, t_max: float) -> bool:
    if t is None or not math.isfinite(t):
        return False
    return t_min <= t <= t_max


def _intercept_speed_has_solution_in_window(
    tx: float,
    ty: float,
    tz: float,
    v_tx: float,
    v_ty: float,
    v_tz: float,
    ix: float,
    iy: float,
    iz: float,
    s_i: float,
    t_min: float,
    t_max: float,
) -> bool:
    t = _solve_intercept_time(tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, s_i)
    return _t_in_feasible_window(t, t_min, t_max)


def minimum_intercept_closing_speed(
    tx: float,
    ty: float,
    tz: float,
    v_tx: float,
    v_ty: float,
    v_tz: float,
    ix: float,
    iy: float,
    iz: float,
    t_min: float,
    t_max: float,
    s_max: float,
    bisect_iters: int = 28,
) -> tuple[float | None, float | None]:
    """
    Smallest constant closing speed s in (0, s_max] with intercept time in [t_min, t_max].
    Deterministic bisection; returns (s_min, t_at_s_min) or (None, None).
    """
    if s_max <= 0.0 or not math.isfinite(s_max):
        return None, None
    if not _intercept_speed_has_solution_in_window(
        tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, s_max, t_min, t_max,
    ):
        return None, None
    lo = 1e-4
    hi = float(s_max)
    if _intercept_speed_has_solution_in_window(
        tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, lo, t_min, t_max,
    ):
        t0 = _solve_intercept_time(tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, lo)
        return lo, float(t0) if t0 is not None else None
    for _ in range(max(8, bisect_iters)):
        mid = 0.5 * (lo + hi)
        if _intercept_speed_has_solution_in_window(
            tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, mid, t_min, t_max,
        ):
            hi = mid
        else:
            lo = mid
    t_final = _solve_intercept_time(tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, hi)
    if not _t_in_feasible_window(t_final, t_min, t_max):
        return None, None
    return float(hi), float(t_final) if t_final is not None else None


def is_intercept_feasible(
    tx: float,
    ty: float,
    tz: float,
    v_tx: float,
    v_ty: float,
    v_tz: float,
    ix: float,
    iy: float,
    iz: float,
    t_min: float,
    t_max: float,
    interceptor_max_speed: float,
    bisect_iters: int = 28,
) -> tuple[bool, float | None, float | None]:
    """
    True iff a constant closing speed <= interceptor_max achieves intercept in [t_min, t_max].
    Returns (feasible, t_intercept, required_min_speed).
    """
    s_min, t_int = minimum_intercept_closing_speed(
        tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz,
        t_min, t_max, interceptor_max_speed, bisect_iters,
    )
    if s_min is None or t_int is None:
        return False, None, None
    if s_min > interceptor_max_speed + 1e-3:
        return False, None, None
    return True, t_int, s_min


class FeasibilityVizState(NamedTuple):
    """Run-time feasibility snapshot for RViz + logs (uses existing ``is_intercept_feasible`` / solver)."""

    status: str          # 'feasible' | 'infeasible' | 'no_solution'
    reason: str          # '' | 'NO SOLUTION' | 'TIME LIMIT' | 'TOO SLOW'
    feasible: bool
    t_go: float | None
    phx: float | None
    phy: float | None
    phz: float | None
    v_req: float | None


def finalize_feasibility_viz_state(feas: FeasibilityVizState, v_i_max: float) -> FeasibilityVizState:
    """
    Post-process feasibility for NaNs, non-positive t_go, and required-speed cap — without changing the solver.

    Aligns ``feasible`` with geometry we are willing to show (no misleading P_hit when invalid).
    """
    vm = max(float(v_i_max), 1e-6)

    def _bad_hit_point() -> bool:
        if feas.phx is None or feas.phy is None or feas.phz is None:
            return False
        return not all(math.isfinite(float(x)) for x in (feas.phx, feas.phy, feas.phz))

    if _bad_hit_point() or (feas.v_req is not None and not math.isfinite(feas.v_req)):
        if feas.status != 'no_solution':
            print('[FEAS_GUARD] infeasible: NaN/inf in feasibility geometry (P_hit or v_req)', flush=True)
        return FeasibilityVizState('no_solution', 'NO SOLUTION', False, None, None, None, None, None)

    if feas.status == 'no_solution':
        return feas

    tg = feas.t_go
    if tg is None or not math.isfinite(tg) or tg <= 0.0:
        return FeasibilityVizState('no_solution', 'NO SOLUTION', False, None, None, None, None, None)

    vr = feas.v_req
    if vr is None or not math.isfinite(vr):
        return FeasibilityVizState('no_solution', 'NO SOLUTION', False, None, None, None, None, None)

    if vr > vm + 1e-3:
        return FeasibilityVizState(
            'infeasible', 'TOO SLOW', False, float(tg), feas.phx, feas.phy, feas.phz, float(vr),
        )

    return feas


def classify_intercept_feasibility_for_viz(
    tx: float,
    ty: float,
    tz: float,
    v_tx: float,
    v_ty: float,
    v_tz: float,
    ix: float,
    iy: float,
    iz: float,
    *,
    v_i_max: float,
    t_min: float,
    t_max: float,
) -> FeasibilityVizState:
    """
    Feasibility and intercept geometry at the interceptor speed cap ``v_i_max``.

    Reuses ``is_intercept_feasible`` (bisection + window) and ``_solve_intercept_time``; does not change them.
    """
    for val in (tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz):
        if not math.isfinite(val):
            print('[FEAS_GUARD] non-finite position/velocity; treating as no intercept solution', flush=True)
            return FeasibilityVizState('no_solution', 'NO SOLUTION', False, None, None, None, None, None)

    ok, t_f, s_min = is_intercept_feasible(
        tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, t_min, t_max, v_i_max,
    )
    t_cap = _solve_intercept_time(tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, v_i_max)

    out: FeasibilityVizState
    if ok and t_f is not None and math.isfinite(t_f) and s_min is not None:
        phx = tx + v_tx * t_f
        phy = ty + v_ty * t_f
        phz = tz + v_tz * t_f
        v_req = compute_required_speed(ix, iy, iz, phx, phy, phz, t_f)
        out = FeasibilityVizState('feasible', '', True, float(t_f), phx, phy, phz, v_req)

    elif t_cap is None or not math.isfinite(t_cap) or t_cap <= 0.0:
        out = FeasibilityVizState('no_solution', 'NO SOLUTION', False, None, None, None, None, None)

    else:
        phx = tx + v_tx * t_cap
        phy = ty + v_ty * t_cap
        phz = tz + v_tz * t_cap
        v_req = compute_required_speed(ix, iy, iz, phx, phy, phz, t_cap)

        if not _t_in_feasible_window(t_cap, t_min, t_max):
            out = FeasibilityVizState(
                'infeasible', 'TIME LIMIT', False, float(t_cap), phx, phy, phz, v_req,
            )
        else:
            out = FeasibilityVizState(
                'infeasible', 'TOO SLOW', False, float(t_cap), phx, phy, phz, v_req,
            )

    return finalize_feasibility_viz_state(out, v_i_max)


def _heatmap_rgba_for_score(score: float, alpha: float) -> tuple[float, float, float, float]:
    """Green (high), yellow (mid), red (low); alpha shared."""
    a = min(1.0, max(0.0, float(alpha)))
    if score >= 0.8:
        return (0.15, 0.85, 0.22, a)
    if score >= 0.5:
        return (0.95, 0.82, 0.12, a)
    return (0.92, 0.18, 0.14, a)


def _heatmap_svg_escape(text: str) -> str:
    return (
        text.replace('&', '&amp;')
        .replace('<', '&lt;')
        .replace('>', '&gt;')
        .replace('"', '&quot;')
    )


def _prune_intercept_heatmap_prob_export_dir(root: Path) -> None:
    """
    Remove prior ``intercept_heatmap_prob_*.{csv,svg}`` in *root* so each new process run
    only accumulates fresh ``*_latest.*`` (no timestamped pile-up). Called once at node init.
    """
    try:
        root.mkdir(parents=True, exist_ok=True)
    except OSError:
        return
    keep = {
        'intercept_heatmap_prob_latest.csv',
        'intercept_heatmap_prob_latest.svg',
    }
    try:
        for p in root.iterdir():
            if not p.is_file():
                continue
            if p.name in keep:
                continue
            if p.name.startswith('intercept_heatmap_prob_') and p.name.endswith(('.csv', '.svg')):
                p.unlink(missing_ok=True)
    except OSError:
        pass


def export_intercept_heatmap_prob_to_disk(
    root: Path,
    lbl_cells: list[tuple[float, float, float, float]],
    *,
    frame_id: str,
    ref_iid: str,
    mc_n: int,
    heatmap_model: str,
    stamp_wall: str,
    stamp_files: bool,
) -> None:
    """
    Write MC probability heatmap as CSV + SVG (top-down XY, colors = P(hit|noise model)).

    ``intercept_heatmap_prob_latest.*`` is overwritten each heatmap tick.

    If ``stamp_files`` is true, also overwrites ``intercept_heatmap_prob_stamped.*`` (fixed names;
    no per-timestamp files — avoids huge directories / long runs).
    """
    root.mkdir(parents=True, exist_ok=True)
    csv_body = ['x_m,y_m,z_m,p_hit_noise_model\n']
    for tx, ty, tz, p in lbl_cells:
        csv_body.append(f'{tx:.3f},{ty:.3f},{tz:.3f},{min(1.0, max(0.0, float(p))):.6f}\n')
    text_csv = ''.join(csv_body)
    latest_csv = root / 'intercept_heatmap_prob_latest.csv'
    latest_svg = root / 'intercept_heatmap_prob_latest.svg'
    latest_csv.write_text(text_csv, encoding='utf-8')
    if stamp_files:
        (root / 'intercept_heatmap_prob_stamped.csv').write_text(text_csv, encoding='utf-8')

    xs = [c[0] for c in lbl_cells]
    ys = [c[1] for c in lbl_cells]
    span = max(abs(max(xs) - min(xs)), abs(max(ys) - min(ys)), 200.0)
    pad = span * 0.06 + 80.0
    xmin, xmax = min(xs) - pad, max(xs) + pad
    ymin, ymax = min(ys) - pad, max(ys) + pad
    xr = max(xmax - xmin, 1.0)
    yr = max(ymax - ymin, 1.0)
    w, h = 900.0, 900.0
    margin = 40.0
    plot_w, plot_h = w - 2 * margin, h - 2 * margin
    ncell = max(1, len(lbl_cells))
    r_px = max(3.5, min(24.0, 420.0 / math.sqrt(float(ncell))))

    parts: list[str] = [
        '<?xml version="1.0" encoding="UTF-8"?>\n',
        '<svg xmlns="http://www.w3.org/2000/svg" width="900" height="900" ',
        'viewBox="0 0 900 900">\n',
        '  <rect width="100%" height="100%" fill="#1a1a1e"/>\n',
        '  <text x="24" y="34" fill="#e8e8ec" font-size="17" font-family="Ubuntu,DejaVu Sans,sans-serif">',
        _heatmap_svg_escape(
            f'P(hit|noise model)  cells={ncell}  MC n={mc_n}  model={heatmap_model}  ref={ref_iid}',
        ),
        '</text>\n',
        '  <text x="24" y="58" fill="#9aa0a6" font-size="13" font-family="Ubuntu,DejaVu Sans,sans-serif">',
        _heatmap_svg_escape(f'frame={frame_id}  UTC={stamp_wall}  XY top-down (m)'),
        '</text>\n',
    ]
    for tx, ty, _tz, p in lbl_cells:
        sc = min(1.0, max(0.0, float(p)))
        rr, gg, bb, _ = _heatmap_rgba_for_score(sc, 1.0)
        fill = f'#{int(rr * 255):02x}{int(gg * 255):02x}{int(bb * 255):02x}'
        u = (tx - xmin) / xr
        v = (ty - ymin) / yr
        cx = margin + u * plot_w
        cy = margin + (1.0 - v) * plot_h
        parts.append(
            f'  <circle cx="{cx:.2f}" cy="{cy:.2f}" r="{r_px:.2f}" '
            f'fill="{fill}" stroke="#0d0d10" stroke-width="0.6" opacity="0.94"/>\n',
        )
    parts.append('</svg>\n')
    svg_txt = ''.join(parts)
    latest_svg.write_text(svg_txt, encoding='utf-8')
    if stamp_files:
        (root / 'intercept_heatmap_prob_stamped.svg').write_text(svg_txt, encoding='utf-8')


def _apply_rollout_velocity_step(
    pvx: float,
    pvy: float,
    pvz: float,
    vx: float,
    vy: float,
    vz: float,
    *,
    max_turn_rate_rad_s: float,
    max_accel_m_s2: float,
    dt: float,
) -> tuple[float, float, float]:
    """
    One-step velocity limiter matching ``InterceptionLogicNode._accel_limit_velocity`` order:
    turn-rate cap on direction, then acceleration cap on delta-v.
    """
    nx, ny, nz = vx, vy, vz
    if max_turn_rate_rad_s > 1e-9:
        p_spd = _norm(pvx, pvy, pvz)
        d_spd = _norm(vx, vy, vz)
        if p_spd > 1e-6 and d_spd > 1e-6:
            max_angle = max_turn_rate_rad_s * dt
            pdx, pdy, pdz = pvx / p_spd, pvy / p_spd, pvz / p_spd
            ddx, ddy, ddz = vx / d_spd, vy / d_spd, vz / d_spd
            rdx, rdy, rdz = _rotate_dir_toward(pdx, pdy, pdz, ddx, ddy, ddz, max_angle)
            nx, ny, nz = rdx * d_spd, rdy * d_spd, rdz * d_spd
    dvx, dvy, dvz = nx - pvx, ny - pvy, nz - pvz
    dv = _norm(dvx, dvy, dvz)
    max_dv = max_accel_m_s2 * dt
    if max_dv > 1e-12 and dv > max_dv and dv > 1e-9:
        s = max_dv / dv
        dvx, dvy, dvz = dvx * s, dvy * s, dvz * s
    return pvx + dvx, pvy + dvy, pvz + dvz


def _monte_carlo_kinematic_hit_rollout(
    tx: float,
    ty: float,
    tz: float,
    v_tx: float,
    v_ty: float,
    v_tz: float,
    ix: float,
    iy: float,
    iz: float,
    *,
    v_i_max: float,
    hit_thresh_m: float,
    t_horizon: float,
    dt: float,
    max_turn_rate_rad_s: float = 0.0,
    max_accel_m_s2: float = 0.0,
) -> bool:
    """
    Closed-loop rollout: each step aim with ``_compute_intercept`` (same aim law as MC rollout).

    * Default ``max_turn_rate_rad_s`` / ``max_accel_m_s2`` ≤ 0: legacy instant heading at ``v_i_max``.
    * With positive limits: integrate interceptor **velocity** from rest; matches turn-then-accel
      ordering in the live node's ``_accel_limit_velocity``.
    """
    px_t, py_t, pz_t = tx, ty, tz
    px_i, py_i, pz_i = ix, iy, iz
    dt_e = max(1e-3, min(0.5, float(dt)))
    vm = max(float(v_i_max), 1e-6)
    steps = max(8, int(math.ceil(max(0.5, t_horizon) / dt_e)) + 4)
    ht = max(0.05, float(hit_thresh_m))
    use_dyn = max_turn_rate_rad_s > 1e-9 or max_accel_m_s2 > 1e-9
    ivx, ivy, ivz = 0.0, 0.0, 0.0

    for _ in range(steps):
        if _norm(px_t - px_i, py_t - py_i, pz_t - pz_i) <= ht:
            return True
        sol = _compute_intercept(px_t, py_t, pz_t, v_tx, v_ty, v_tz, px_i, py_i, pz_i, vm)
        if sol is None:
            ux, uy, uz = _unit(px_t - px_i, py_t - py_i, pz_t - pz_i)
        else:
            _tg, _phx, _phy, _phz, ux, uy, uz = sol
            if _norm(ux, uy, uz) < 1e-9:
                ux, uy, uz = _unit(px_t - px_i, py_t - py_i, pz_t - pz_i)
        if use_dyn:
            dvx_des, dvy_des, dvz_des = ux * vm, uy * vm, uz * vm
            ivx, ivy, ivz = _apply_rollout_velocity_step(
                ivx,
                ivy,
                ivz,
                dvx_des,
                dvy_des,
                dvz_des,
                max_turn_rate_rad_s=max_turn_rate_rad_s,
                max_accel_m_s2=max_accel_m_s2,
                dt=dt_e,
            )
            px_i += ivx * dt_e
            py_i += ivy * dt_e
            pz_i += ivz * dt_e
        else:
            px_i += ux * vm * dt_e
            py_i += uy * vm * dt_e
            pz_i += uz * vm * dt_e
        px_t += v_tx * dt_e
        py_t += v_ty * dt_e
        pz_t += v_tz * dt_e
    return _norm(px_t - px_i, py_t - py_i, pz_t - pz_i) <= ht


def simulate_intercept_once(
    tx: float,
    ty: float,
    tz: float,
    v_tx: float,
    v_ty: float,
    v_tz: float,
    ix: float,
    iy: float,
    iz: float,
    *,
    v_i_max: float,
    t_min: float,
    t_max: float,
    hit_thresh_m: float,
    pos_sigma_m: float,
    vel_sigma_m_s: float,
    interceptor_pos_sigma_m: float,
    delay_mean_s: float,
    delay_jitter_s: float,
    rng: random.Random,
    use_kinematic_rollout: bool,
    rollout_dt: float,
    rollout_max_turn_rate_rad_s: float = 0.0,
    rollout_max_accel_m_s2: float = 0.0,
) -> bool:
    """
    One stochastic draw: noisy target track, interceptor pose, delay; then feasibility or short rollout “hit”.

    Default (rollout off): success ≡ ``is_intercept_feasible`` on the perturbed state (fast).

    Rollout on: short horizon ``_monte_carlo_kinematic_hit_rollout`` using ``_compute_intercept`` each step.
    With positive ``rollout_max_turn_rate_rad_s`` / ``rollout_max_accel_m_s2``, integrates velocity from rest
    (same limiter ordering as live ``cmd_vel`` smoothing); zeros preserve legacy instant-heading rollout.
    """
    dly = max(0.0, float(delay_mean_s) + rng.gauss(0.0, float(delay_jitter_s)))
    _vs = float(vel_sigma_m_s)
    vtx = v_tx + rng.gauss(0.0, _vs)
    vty = v_ty + rng.gauss(0.0, _vs)
    vtz = v_tz + rng.gauss(0.0, _vs)
    sigp = max(0.0, float(pos_sigma_m))
    tpx = tx + rng.gauss(0.0, sigp) - vtx * dly
    tpy = ty + rng.gauss(0.0, sigp) - vty * dly
    tpz = tz + rng.gauss(0.0, sigp) - vtz * dly
    sigi = max(0.0, float(interceptor_pos_sigma_m))
    iix = ix + rng.gauss(0.0, sigi)
    iiy = iy + rng.gauss(0.0, sigi)
    iiz = iz + rng.gauss(0.0, sigi)
    if not all(math.isfinite(v) for v in (tpx, tpy, tpz, vtx, vty, vtz, iix, iiy, iiz)):
        return False

    if use_kinematic_rollout:
        return _monte_carlo_kinematic_hit_rollout(
            tpx, tpy, tpz, vtx, vty, vtz, iix, iiy, iiz,
            v_i_max=v_i_max,
            hit_thresh_m=hit_thresh_m,
            t_horizon=t_max,
            dt=rollout_dt,
            max_turn_rate_rad_s=float(rollout_max_turn_rate_rad_s),
            max_accel_m_s2=float(rollout_max_accel_m_s2),
        )

    ok, _, _ = is_intercept_feasible(
        tpx, tpy, tpz, vtx, vty, vtz, iix, iiy, iiz, t_min, t_max, v_i_max,
    )
    return bool(ok)


def estimate_hit_probability_light(
    tx: float,
    ty: float,
    tz: float,
    v_tx: float,
    v_ty: float,
    v_tz: float,
    ix: float,
    iy: float,
    iz: float,
    *,
    v_i_max: float,
    t_min: float,
    t_max: float,
    hit_thresh_m: float,
    n_samples: int,
    pos_sigma_m: float,
    vel_sigma_m_s: float,
    delay_mean_s: float,
    delay_jitter_s: float,
    rng: random.Random,
    ivx: float | None = None,
    ivy: float | None = None,
    ivz: float | None = None,
) -> float:
    """
    Phase-1 lightweight Monte Carlo **P(success | noise + constant-speed intercept model)** — not a
    calibrated real-world hit probability.

    Per sample: noisy track + delay, ``_solve_intercept_time`` for ``t_go``. Interceptor position at
    intercept uses **constant velocity over t_go**: either the last published cmd (``iv*``) when
    available (clamped to ``v_i_max`` magnitude), else closing at ``v_i_max`` toward ``p_hit``.
    Success when ``‖p_hit − p_i(t_go)‖ < hit_threshold_m`` and required speed at the geometry is
    consistent with ``v_i_max``.
    """
    n = max(0, min(30, int(n_samples)))
    if n == 0:
        return 0.0
    vm = max(float(v_i_max), 1e-6)
    ht = max(0.05, float(hit_thresh_m))
    sigp = max(0.0, float(pos_sigma_m))
    sigv = max(0.0, float(vel_sigma_m_s))
    sigj = max(0.0, float(delay_jitter_s))
    dmean = max(0.0, float(delay_mean_s))
    ok_n = 0
    use_cmd = (
        ivx is not None and ivy is not None and ivz is not None
        and all(math.isfinite(v) for v in (ivx, ivy, ivz))
    )
    cmd_mag = _norm(float(ivx), float(ivy), float(ivz)) if use_cmd else 0.0
    use_cmd = use_cmd and cmd_mag > 1e-9
    for _ in range(n):
        vtx = v_tx + rng.gauss(0.0, sigv)
        vty = v_ty + rng.gauss(0.0, sigv)
        vtz = v_tz + rng.gauss(0.0, sigv)
        dly = max(0.0, dmean + rng.gauss(0.0, sigj))
        tpx = tx + rng.gauss(0.0, sigp) - vtx * dly
        tpy = ty + rng.gauss(0.0, sigp) - vty * dly
        tpz = tz + rng.gauss(0.0, sigp) - vtz * dly
        if not all(math.isfinite(v) for v in (tpx, tpy, tpz, vtx, vty, vtz)):
            continue
        t_go = _solve_intercept_time(tpx, tpy, tpz, vtx, vty, vtz, ix, iy, iz, vm, None)
        if t_go is None or (not math.isfinite(t_go)) or t_go <= 0.0:
            continue
        if not _t_in_feasible_window(t_go, t_min, t_max):
            continue
        phx = tpx + vtx * t_go
        phy = tpy + vty * t_go
        phz = tpz + vtz * t_go
        if not all(math.isfinite(v) for v in (phx, phy, phz)):
            continue
        v_req = compute_required_speed(ix, iy, iz, phx, phy, phz, t_go)
        if not math.isfinite(v_req) or v_req > vm + 0.05:
            continue
        if use_cmd:
            s = min(1.0, vm / cmd_mag)
            fvx = float(ivx) * s
            fvy = float(ivy) * s
            fvz = float(ivz) * s
            ifix = ix + fvx * t_go
            ifiy = iy + fvy * t_go
            ifiz = iz + fvz * t_go
        else:
            dx, dy, dz = phx - ix, phy - iy, phz - iz
            if _norm(dx, dy, dz) < 1e-9:
                continue
            ux, uy, uz = _unit(dx, dy, dz)
            ifix = ix + ux * vm * t_go
            ifiy = iy + uy * vm * t_go
            ifiz = iz + uz * vm * t_go
        dist = _norm(phx - ifix, phy - ifiy, phz - ifiz)
        if dist < ht:
            ok_n += 1
    return ok_n / float(n)


def estimate_hit_probability(
    tx: float,
    ty: float,
    tz: float,
    v_tx: float,
    v_ty: float,
    v_tz: float,
    ix: float,
    iy: float,
    iz: float,
    *,
    v_i_max: float,
    t_min: float,
    t_max: float,
    hit_thresh_m: float,
    n_samples: int,
    pos_sigma_m: float,
    vel_sigma_m_s: float,
    interceptor_pos_sigma_m: float,
    delay_mean_s: float,
    delay_jitter_s: float,
    rng: random.Random,
    use_kinematic_rollout: bool,
    rollout_dt: float,
    rollout_max_turn_rate_rad_s: float = 0.0,
    rollout_max_accel_m_s2: float = 0.0,
) -> float:
    n = max(0, int(n_samples))
    if n == 0:
        return 0.0
    success = 0
    for _ in range(n):
        hit = simulate_intercept_once(
            tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz,
            v_i_max=v_i_max,
            t_min=t_min,
            t_max=t_max,
            hit_thresh_m=hit_thresh_m,
            pos_sigma_m=pos_sigma_m,
            vel_sigma_m_s=vel_sigma_m_s,
            interceptor_pos_sigma_m=interceptor_pos_sigma_m,
            delay_mean_s=delay_mean_s,
            delay_jitter_s=delay_jitter_s,
            rng=rng,
            use_kinematic_rollout=use_kinematic_rollout,
            rollout_dt=rollout_dt,
            rollout_max_turn_rate_rad_s=rollout_max_turn_rate_rad_s,
            rollout_max_accel_m_s2=rollout_max_accel_m_s2,
        )
        if hit:
            success += 1
    return success / float(n)


def _build_dome_disk_xy_offsets(r_outer: float, step_m: float) -> list[tuple[float, float]]:
    R = max(float(r_outer), 1.0)
    ds = max(50.0, float(step_m))
    r2 = R * R
    out: list[tuple[float, float]] = []
    n = int(math.ceil(R / ds))
    for i in range(-n, n + 1):
        gx = i * ds
        for j in range(-n, n + 1):
            gy = j * ds
            if gx * gx + gy * gy <= r2 + 1e-3:
                out.append((gx, gy))
    return out


def intercept_heatmap_prob_threat_altitude_m(
    dome_cx: float,
    dome_cy: float,
    dome_cz: float,
    tx: float,
    ty: float,
    *,
    mode: str,
    z_offset_m: float,
    z_inner_m: float,
    z_outer_m: float,
    r_ref_m: float,
) -> float:
    """
    Threat *body* altitude ``tz`` for a heatmap cell at horizontal ``(tx, ty)``.

    * ``flat``: legacy plate — ``dome_cz + z_offset_m`` (independent of range).
    * ``los_ramp`` (aliases: ``los_ramp_inbound``, ``los_ramp_far_high``): altitude **low** at the
      asset (r=0) and **high** at horizontal range ``r_ref_m``. Envelope slopes **down** toward the
      asset when viewed from the side (inbound from higher cruise at long range).
    * ``los_ramp_center_high`` (aliases: ``los_ramp_far_low``): the opposite — **high** above the
      asset and **low** at ``r_ref_m`` (rim). Use when you want the peak at the dome center in XY.
    """
    m = (mode or 'flat').strip().lower()
    base = float(dome_cz) + float(z_offset_m)
    if m in ('flat', 'legacy', 'none', 'plate'):
        return base
    rh = math.hypot(float(tx) - float(dome_cx), float(ty) - float(dome_cy))
    R = max(float(r_ref_m), 1.0)
    t = min(1.0, rh / R)
    if m in ('los_ramp_center_high', 'los_ramp_far_low', 'los_ramp_invert'):
        # High at asset, low at outer r (swap ends vs standard inbound ramp).
        z_body = float(z_outer_m) + t * (float(z_inner_m) - float(z_outer_m))
    elif m in ('los_ramp', 'los_ramp_inbound', 'los_ramp_far_high'):
        z_body = float(z_inner_m) + t * (float(z_outer_m) - float(z_inner_m))
    else:
        z_body = float(z_inner_m) + t * (float(z_outer_m) - float(z_inner_m))
    return base + z_body


def intercept_heatmap_prob_cell_velocity(
    dome_cx: float,
    dome_cy: float,
    dome_cz: float,
    tx: float,
    ty: float,
    tz: float,
    *,
    use_cell_los: bool,
    los_spd_m_s: float,
    v_tx_param: float,
    v_ty_param: float,
    v_tz_param: float,
    v_smooth: tuple[float, float, float],
) -> tuple[float, float, float]:
    """
    Target velocity for one probability-heatmap disk cell: LOS toward the dome center when enabled,
    with degeneracy handling at/near the asset (see ``_publish_intercept_heatmap_prob``).
    """
    if not use_cell_los:
        return (v_tx_param, v_ty_param, v_tz_param)
    ddx = dome_cx - tx
    ddy = dome_cy - ty
    ddz = dome_cz - tz
    nn = math.sqrt(ddx * ddx + ddy * ddy + ddz * ddz)
    _snap_eps = 0.45
    if nn < _snap_eps:
        pv = math.hypot(v_tx_param, v_ty_param, v_tz_param)
        if pv > 1e-3:
            return (v_tx_param, v_ty_param, v_tz_param)
        sx, sy, sz = v_smooth
        sv = math.hypot(sx, sy, sz)
        if sv > 1e-3:
            return (sx, sy, sz)
        ndx = dome_cx - (tx + _snap_eps)
        ndy = dome_cy - ty
        ndz = dome_cz - tz
        nnn = math.sqrt(ndx * ndx + ndy * ndy + ndz * ndz)
        if nnn < 1e-9:
            ndx, ndy, ndz = -1.0, 0.0, 0.0
            nnn = 1.0
        inv = float(los_spd_m_s) / nnn
        return (ndx * inv, ndy * inv, ndz * inv)
    inv = float(los_spd_m_s) / nn
    return (ddx * inv, ddy * inv, ddz * inv)


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
        # Smooth velocity ramp: limits how fast commanded speed can change per second.
        self.declare_parameter('max_acceleration_m_s2', 3.0)
        # Turn-rate limit: maximum direction change per second (radians/s).
        # Prevents sharp heading reversals and oscillation.
        self.declare_parameter('max_turn_rate_rad_s', 1.5)
        # Adaptive speed (optional): raise commanded speed up to max_speed when far away (helps vertical catch-up).
        self.declare_parameter('adaptive_speed_enabled', False)
        self.declare_parameter('adaptive_speed_gain', 0.25)
        # Dome engagement policy: if set, HIT is only checked in this layer (e.g. "select").
        # Empty => allow HIT wherever current logic runs guidance.
        self.declare_parameter('engagement_layer', '')
        # Layer at which guidance (flying toward target) starts. "detect" = start early so interceptors
        # can climb in time. Empty => same as engagement_layer.
        self.declare_parameter('guidance_start_layer', '')
        # If True: HIT registers ONLY when target is exactly in engagement_layer (not deeper layers).
        # Useful to start guidance while still in the outer detect band (between r_mid and r_outer);
        # radii come from launch (e.g. km-class site defense).
        # penetrates the protected volume. False (default) = HIT allowed in engagement_layer + all
        # inner layers (original behaviour).
        self.declare_parameter('hit_outer_layer_only', False)
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
        # Single-target target position: subscription resolved from intercept_measurement_source.
        self.declare_parameter('target_topic', '/drone/position')
        # Source selector:
        #   ground_truth -> Point on target_topic
        #   fused        -> Point on fused_detections_topic
        #   tracks       -> Point on tracks_topic        (legacy; KF velocity dropped)
        #   tracks_state -> Odometry on tracks_state_topic (uses KF position + velocity + cov)
        self.declare_parameter('intercept_measurement_source', 'ground_truth')
        self.declare_parameter('fused_detections_topic', '/fused_detections')
        self.declare_parameter('tracks_topic', '/tracks')
        # Track-state stream from tracking_node (nav_msgs/Odometry: pose + twist + covariance).
        # When ``intercept_measurement_source == tracks_state`` guidance uses the **filter
        # velocity** instead of differencing positions in _estimate_target_vel — this is the
        # key fix for noisy / delayed intercept geometry under the tracks pipeline.
        self.declare_parameter('tracks_state_topic', '/tracks/state')
        self.declare_parameter('selected_id_topic', '/interceptor/selected_id')
        self.declare_parameter('lock_selected_after_first', True)
        self.declare_parameter('hit_threshold_m', 1.0)
        # Debug: pure predictive guidance toward solver P_hit + separate HIT radius (does not replace hit_threshold_m for MC/heatmap).
        self.declare_parameter('intercept_debug_force_predictive_guidance', False)
        self.declare_parameter('hit_threshold_m_debug', 2.5)
        self.declare_parameter('intercept_debug_speed_margin_m_s', 0.25)
        self.declare_parameter('intercept_debug_hit_log_period_s', 0.2)
        self.declare_parameter('intercept_debug_no_hit_p_threshold', 0.55)
        self.declare_parameter('intercept_debug_no_hit_grace_s', 2.0)
        # ถ้า >= 0: นับ HIT เฉพาะเมื่อเป้ายังอยู่เหนือระดับนี้ (เมตรใน world) เช่น โจมตีในอากาศในชั้น engage
        self.declare_parameter('hit_min_target_z_m', -1.0)
        # Safety guard: HIT requires the selected interceptor to be physically airborne.
        # Without this, a stationary ground interceptor that happens to be close to the
        # target's trajectory could register a false HIT. 1.5 m >> typical start z=0.
        self.declare_parameter('hit_min_interceptor_z_m', 1.5)
        # Safety guard: HIT also requires the selected interceptor to have traveled
        # at least this distance from its launch/start position — proves it actually flew.
        self.declare_parameter('hit_min_interceptor_travel_m', 2.0)
        self.declare_parameter('stop_topic', '/target/stop')
        # VOLATILE /target/stop can be missed if target_controller arms its subscription late
        # (``ignore_stop_true_first_s``).  Re-publish bursts for this window so the explosion
        # sequence still runs once the subscriber exists.
        self.declare_parameter('stop_signal_repeat_duration_s', 15.0)
        self.declare_parameter('stop_signal_repeat_period_s', 0.35)
        # Phase 2 → Phase 3 gating (opt-in): require radar-range detection + tracking delay elapsed.
        # When disabled (default), behaviour matches the pre-gating implementation.
        self.declare_parameter('sensing_gate_enabled', False)
        self.declare_parameter('radar_position_x', 0.0)
        self.declare_parameter('radar_position_y', 0.0)
        self.declare_parameter('radar_position_z', 0.0)
        self.declare_parameter('radar_range_m', 6500.0)
        self.declare_parameter('tracking_delay_s', 0.0)
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
        self.declare_parameter('dome_outer_m', 6000.0)
        # Hysteresis on |P-C| vs r_outer: avoids chattering when target crosses the outer
        # shell (selected_id toggles '' <-> id every tick → bogus /target/stop + STANDBY/LAUNCH spam).
        self.declare_parameter('dome_outer_hysteresis_m', 72.0)
        self.declare_parameter('dome_middle_m', 3000.0)
        self.declare_parameter('dome_inner_m', 1200.0)
        self.declare_parameter('reset_lock_when_outside_dome', True)
        # When dome disabled: 'nearest' or 'tti' for committed selection.
        self.declare_parameter('dome_selection_mode', 'nearest')
        self.declare_parameter('publish_dome_rviz_marker', True)
        self.declare_parameter('dome_marker_frame_id', 'map')
        # Metrics + RViz debug markers.
        # metrics_log_period_s: how often to print the metrics summary line (seconds).
        # publish_intercept_markers: publish SPHERE + LINE_STRIP on /interception/markers.
        self.declare_parameter('metrics_log_period_s', 1.0)
        # publish_intercept_markers: enable /interception/markers (RViz / presentation viz).
        self.declare_parameter('publish_intercept_markers', True)
        # C-UAS presentation: path trails + optional predicted intercept (demo scale, not physics).
        self.declare_parameter('cuas_trails_enabled', True)
        self.declare_parameter('cuas_trail_max_points', 12000)
        self.declare_parameter('cuas_trail_min_step_m', 2.0)
        self.declare_parameter('cuas_trail_line_width_m', 0.45)
        self.declare_parameter('cuas_trail_hostile_rgba', [0.95, 0.12, 0.08, 0.85])
        self.declare_parameter('cuas_trail_interceptor_rgba', [0.12, 0.35, 0.95, 0.88])
        # One-shot HIT flash for RViz (independent of publish_intercept_markers / predict viz).
        self.declare_parameter('publish_hit_markers', True)
        self.declare_parameter('hit_marker_topic', '/interception/hit_markers')
        self.declare_parameter('hit_marker_duration_s', 8.0)
        self.declare_parameter('hit_marker_text_scale_m', 220.0)
        self.declare_parameter('hit_marker_text_z_offset_m', 180.0)
        self.declare_parameter('hit_marker_pulse_period_s', 0.12)
        self.declare_parameter('hit_marker_pulse_min_diameter_m', 120.0)
        self.declare_parameter('hit_marker_pulse_max_diameter_m', 520.0)
        # Extra RViz-only “bloom” sphere (km-scale visibility; Gazebo uses hit_explosion SDF).
        self.declare_parameter('hit_marker_outer_diameter_m', 720.0)
        self.declare_parameter('hit_marker_outer_alpha_max', 0.26)
        self.declare_parameter('cuas_intercept_predict_viz_enabled', True)
        self.declare_parameter('cuas_intercept_marker_diameter_m', 2.8)
        self.declare_parameter('cuas_intercept_line_width_m', 0.15)
        self.declare_parameter('cuas_intercept_line_dashed', True)
        self.declare_parameter('cuas_intercept_dash_length_m', 150.0)
        self.declare_parameter('cuas_intercept_gap_length_m', 100.0)
        self.declare_parameter('cuas_intercept_line_rgba', [0.15, 0.45, 1.0, 0.88])
        self.declare_parameter('cuas_intercept_marker_rgba', [1.0, 0.92, 0.15, 0.95])
        # RViz TEXT_VIEW_FACING ที่จุดทำนาย: แสดง t_go, s_i ของโมเดล |r0+v_T t|=s_i t, |v_cmd|
        self.declare_parameter('cuas_intercept_predict_text_enabled', True)
        self.declare_parameter('cuas_intercept_predict_text_z_offset_m', 120.0)
        self.declare_parameter('cuas_intercept_predict_text_scale_m', 90.0)
        self.declare_parameter('cuas_intercept_predict_text_rgba', [0.95, 0.95, 0.2, 1.0])
        # Physics-forward RViz layer: green P_hit, yellow bearing, red/blue trails (short history), explicit v_req.
        self.declare_parameter('intercept_math_viz_enabled', True)
        self.declare_parameter('intercept_math_hide_legacy_predict', True)
        self.declare_parameter('intercept_math_trail_max_points', 200)
        self.declare_parameter('intercept_math_phit_scale_m', 1.75)
        self.declare_parameter('intercept_math_bearing_line_width_m', 0.15)
        self.declare_parameter('intercept_math_trail_line_width_m', 0.18)
        self.declare_parameter('intercept_math_phit_rgba', [0.15, 0.85, 0.2, 0.95])
        self.declare_parameter('intercept_math_bearing_rgba', [1.0, 0.92, 0.1, 0.92])
        self.declare_parameter('intercept_math_trail_target_rgba', [0.92, 0.12, 0.1, 0.9])
        self.declare_parameter('intercept_math_trail_inter_rgba', [0.12, 0.35, 0.95, 0.9])
        self.declare_parameter('intercept_math_frame_id', 'map')
        self.declare_parameter('intercept_math_text_enabled', True)
        self.declare_parameter('intercept_math_text_z_offset_m', 35.0)
        self.declare_parameter('intercept_math_text_scale_m', 2.0)
        self.declare_parameter('intercept_math_text_rgba', [1.0, 1.0, 1.0, 0.95])
        self.declare_parameter('intercept_math_debug_log', True)
        self.declare_parameter('intercept_feasibility_debug', True)
        # Shared frame / reference for MC probability heatmap only (legacy feasibility heatmap removed).
        self.declare_parameter('intercept_heatmap_frame_id', '')
        self.declare_parameter('intercept_heatmap_z_offset_m', 0.0)
        # '' or 'first' → first declared interceptor id; 'selected' → committed unit when available.
        self.declare_parameter('intercept_heatmap_reference_interceptor', '')
        self.declare_parameter('intercept_heatmap_target_vel_m_s', [0.0, 0.0, 0.0])
        # Monte Carlo P(success) heatmap (noise + delay + optional rollout).
        self.declare_parameter('intercept_heatmap_prob_enabled', False)
        self.declare_parameter('intercept_heatmap_prob_topic', '/interception/heatmap_prob')
        self.declare_parameter('intercept_heatmap_prob_update_period_s', 1.0)
        self.declare_parameter('intercept_heatmap_prob_grid_step_m', 300.0)
        self.declare_parameter('intercept_heatmap_prob_sphere_diameter_m', 150.0)
        self.declare_parameter('intercept_heatmap_prob_alpha', 0.4)
        self.declare_parameter('intercept_heatmap_prob_mc_n', 25)
        self.declare_parameter('intercept_heatmap_prob_pos_sigma_m', 22.0)
        self.declare_parameter('intercept_heatmap_prob_vel_sigma_m_s', 1.35)
        self.declare_parameter('intercept_heatmap_prob_interceptor_pos_sigma_m', 3.0)
        self.declare_parameter('intercept_heatmap_prob_delay_jitter_s', 0.28)
        self.declare_parameter('intercept_heatmap_prob_use_node_tracking_delay', True)
        self.declare_parameter('intercept_heatmap_prob_use_cmd_vel', False)
        self.declare_parameter('intercept_heatmap_prob_use_kinematic_rollout', True)
        self.declare_parameter('intercept_heatmap_prob_rollout_dt_s', 0.05)
        self.declare_parameter('intercept_heatmap_prob_rollout_mc_cap', 12)
        # Per-cell LOS toward dome center (matches ``target_controller_node`` attack_los_to_origin).
        self.declare_parameter('intercept_heatmap_prob_use_cell_los_velocity', True)
        self.declare_parameter('intercept_heatmap_prob_target_los_speed_m_s', 30.0)
        # Threat altitude for heatmap MC: flat = legacy; los_ramp = z rises with horizontal range toward outer dome.
        self.declare_parameter('intercept_heatmap_prob_threat_z_mode', 'los_ramp')
        self.declare_parameter('intercept_heatmap_prob_threat_z_inner_m', 0.0)
        self.declare_parameter('intercept_heatmap_prob_threat_z_outer_m', 300.0)
        self.declare_parameter('intercept_heatmap_prob_threat_z_r_ref_m', 0.0)
        self.declare_parameter('intercept_heatmap_prob_cache_ttl_s', 0.0)
        self.declare_parameter('intercept_heatmap_prob_debug_samples', 0)
        self.declare_parameter('intercept_heatmap_prob_rng_seed', -1)
        self.declare_parameter('intercept_heatmap_prob_show_percent_labels', True)
        self.declare_parameter('intercept_heatmap_prob_label_scale_m', 72.0)
        self.declare_parameter('intercept_heatmap_prob_label_z_offset_m', 95.0)
        self.declare_parameter('intercept_heatmap_prob_export_dir', '')
        self.declare_parameter('intercept_heatmap_prob_export_stamp_files', False)
        self.declare_parameter('intercept_mc_trail_color_by_prob', True)
        self.declare_parameter('intercept_mc_engagement_enabled', False)
        self.declare_parameter('intercept_mc_min_prob_strike', 0.5)
        self.declare_parameter('intercept_mc_runtime_n', 20)
        self.declare_parameter('intercept_mc_use_light_hit_model', True)
        self.declare_parameter('intercept_mc_light_pos_sigma_m', 2.0)
        self.declare_parameter('intercept_mc_light_vel_sigma_m_s', 0.5)
        self.declare_parameter('intercept_mc_light_delay_jitter_s', 0.2)
        self.declare_parameter('intercept_mc_light_samples', 25)
        self.declare_parameter('intercept_mc_hysteresis_enabled', True)
        self.declare_parameter('intercept_mc_hysteresis_high', 0.55)
        self.declare_parameter('intercept_mc_hysteresis_low', 0.35)
        self.declare_parameter('intercept_heatmap_prob_time_budget_s', 0.15)
        self.declare_parameter('intercept_mc_phase1_consistency_warn', True)
        self.declare_parameter('intercept_mc_phase1_debug_log', False)
        self.declare_parameter('intercept_mc_high_p_no_hit_s', 12.0)
        self.declare_parameter('world_name', 'counter_uas_target')
        self.declare_parameter('pause_gz_on_hit', True)
        self.declare_parameter('reset_on_sim_clock_rewind', True)
        # ครึ่งความหนาเปลือกชน (m) รอบ ``dome_middle_m`` — ขอบระหว่าง detect กับ select
        self.declare_parameter('strike_shell_half_width_m', 75.0)
        # Feasibility-based engagement: require intercept physics + radar/track/class gates; zones set HIT policy only.
        self.declare_parameter('feasibility_based_engagement', False)
        self.declare_parameter('classification_gating_enabled', True)
        self.declare_parameter('classification_confidence_threshold', 0.55)
        # Placeholder: 1.0 = assumed hostile drone until fused classifier exists.
        self.declare_parameter('classification_placeholder_confidence', 1.0)
        self.declare_parameter('feasibility_log_period_s', 1.0)
        # 0 => use max_speed_m_s as interceptor hard cap for feasibility.
        self.declare_parameter('interceptor_max_speed_m_s', 0.0)
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
        # EMA smoothing for the predicted intercept point (0 = off, 1 = raw).
        # Lower values track slower but suppress jitter more aggressively.
        self.declare_parameter('intercept_smoothing_alpha', 0.3)
        # Mode-switching hysteresis: require N consecutive qualifying frames before
        # committing to a mode change so a single noisy frame cannot flip guidance.
        # predict_enter_frames: valid-solution streak required to enter predict mode.
        # predict_exit_frames:  invalid-solution streak required to return to pursuit.
        self.declare_parameter('predict_enter_frames', 3)
        self.declare_parameter('predict_exit_frames', 3)
        # Time-to-go aware speed control.
        # Desired speed:  v = k1*dist + k2*dist / max(t_go, tgo_min)
        #   k1 — proportional-to-distance term: sets baseline speed when no t_go
        #   k2 — urgency term: scales with required closing rate (dist/t_go)
        # Pursuit fallback (no t_go): v = k1*dist
        # Result is always clamped to [speed_vmin, max_speed_m_s].
        self.declare_parameter('speed_k1', 0.15)
        self.declare_parameter('speed_k2', 0.5)
        self.declare_parameter('speed_vmin', 1.0)        # hard floor on commanded speed
        self.declare_parameter('speed_tgo_min', 0.2)     # t_go floor (prevents spike near intercept)
        # ── Phase 2 (guidance dynamics alignment) ──────────────────────────────────────
        # Delay compensation: extrapolate target by ``measurement_delay_s`` (CV) before the
        # intercept quadratic. Set this to the *measured* sensor + transport latency you want
        # the guidance loop to absorb. 0 disables (legacy behaviour).
        self.declare_parameter('measurement_delay_s', 0.0)
        # When True, after the intercept solve and direction selection, replace the heuristic
        # speed law with ``s = |P_hit - P_I| / t_go`` capped to ``v_max``. This makes the
        # commanded speed the same constant ``s`` the solver assumed, which is the dominant
        # source of bias once accel/turn limits and predictive replanning are stacked.
        self.declare_parameter('align_speed_to_solver', True)
        # Low-pass on raw t_go to avoid whipping P_hit when the quadratic root jumps cycle to
        # cycle under noisy v_T. 1.0 = raw (off); 0.3–0.6 is a sane operating range.
        self.declare_parameter('t_go_filter_alpha', 0.5)

        self._closing = max(float(self.get_parameter('closing_speed_m_s').value), 0.1)
        self._vmax = max(float(self.get_parameter('max_speed_m_s').value), self._closing)
        _imax_parm = float(self.get_parameter('interceptor_max_speed_m_s').value)
        self._interceptor_max_speed = _imax_parm if _imax_parm > 0.0 else self._vmax
        self._max_accel = max(float(self.get_parameter('max_acceleration_m_s2').value), 0.1)
        self._max_turn_rate = max(float(self.get_parameter('max_turn_rate_rad_s').value), 0.0)
        self._intercept_alpha = min(max(float(self.get_parameter('intercept_smoothing_alpha').value), 0.0), 1.0)
        self._predict_enter_frames = max(int(self.get_parameter('predict_enter_frames').value), 1)
        self._predict_exit_frames = max(int(self.get_parameter('predict_exit_frames').value), 1)
        self._speed_k1 = max(float(self.get_parameter('speed_k1').value), 0.0)
        self._speed_k2 = max(float(self.get_parameter('speed_k2').value), 0.0)
        self._speed_vmin = max(float(self.get_parameter('speed_vmin').value), 0.0)
        self._speed_tgo_min = max(float(self.get_parameter('speed_tgo_min').value), 1e-3)
        # Phase 2: guidance dynamics alignment knobs.
        self._meas_delay_s = max(0.0, float(self.get_parameter('measurement_delay_s').value))
        self._align_speed_to_solver = bool(self.get_parameter('align_speed_to_solver').value)
        self._t_go_alpha = min(1.0, max(0.0, float(self.get_parameter('t_go_filter_alpha').value)))
        # Last filtered t_go per interceptor — used by Phase 2 t_go low-pass / dynamics alignment.
        self._t_go_filtered: dict[str, float | None] = {}
        self._adaptive_speed = bool(self.get_parameter('adaptive_speed_enabled').value)
        self._adaptive_gain = max(float(self.get_parameter('adaptive_speed_gain').value), 0.0)
        self._engagement_layer = str(self.get_parameter('engagement_layer').value).strip().lower()
        _gsl = str(self.get_parameter('guidance_start_layer').value).strip().lower()
        self._guidance_start_layer = _gsl if _gsl else self._engagement_layer
        self._hit_outer_layer_only = bool(self.get_parameter('hit_outer_layer_only').value)
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
        self._feasibility_based = bool(self.get_parameter('feasibility_based_engagement').value)
        self._class_gating = bool(self.get_parameter('classification_gating_enabled').value)
        self._class_threshold = float(self.get_parameter('classification_confidence_threshold').value)
        self._class_placeholder = float(self.get_parameter('classification_placeholder_confidence').value)
        self._feas_log_period = max(float(self.get_parameter('feasibility_log_period_s').value), 0.05)
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
        self._dome_outer_hyst = max(float(self.get_parameter('dome_outer_hysteresis_m').value), 0.0)
        self._reset_lock_outside = bool(self.get_parameter('reset_lock_when_outside_dome').value)
        self._dome_sel_mode = str(self.get_parameter('dome_selection_mode').value).strip().lower()
        self._pub_dome_marker = bool(self.get_parameter('publish_dome_rviz_marker').value)
        self._dome_frame = str(self.get_parameter('dome_marker_frame_id').value).strip() or 'map'
        self._metrics_period = max(float(self.get_parameter('metrics_log_period_s').value), 0.1)
        self._publish_hit_markers = bool(self.get_parameter('publish_hit_markers').value)
        self._hit_marker_topic = str(self.get_parameter('hit_marker_topic').value).strip() or '/interception/hit_markers'
        self._hit_marker_duration = max(0.5, float(self.get_parameter('hit_marker_duration_s').value))
        self._hit_marker_text_scale = max(10.0, float(self.get_parameter('hit_marker_text_scale_m').value))
        self._hit_marker_text_dz = float(self.get_parameter('hit_marker_text_z_offset_m').value)
        self._hit_marker_pulse_period = max(0.05, float(self.get_parameter('hit_marker_pulse_period_s').value))
        self._hit_pulse_d_min = max(1.0, float(self.get_parameter('hit_marker_pulse_min_diameter_m').value))
        self._hit_pulse_d_max = max(
            self._hit_pulse_d_min,
            float(self.get_parameter('hit_marker_pulse_max_diameter_m').value),
        )
        self._hit_outer_d = max(float(self.get_parameter('hit_marker_outer_diameter_m').value), self._hit_pulse_d_min)
        self._hit_outer_alpha_max = min(1.0, max(0.0, float(self.get_parameter('hit_marker_outer_alpha_max').value)))
        self._pub_intercept_markers = bool(self.get_parameter('publish_intercept_markers').value)
        self._cuas_trails_enabled = bool(self.get_parameter('cuas_trails_enabled').value) and self._pub_intercept_markers
        self._cuas_predict_viz_enabled = bool(self.get_parameter('cuas_intercept_predict_viz_enabled').value) and self._pub_intercept_markers
        self._cuas_trail_max_points = int(self.get_parameter('cuas_trail_max_points').value)
        self._cuas_trail_min_step = max(float(self.get_parameter('cuas_trail_min_step_m').value), 0.05)
        self._cuas_trail_line_w = max(float(self.get_parameter('cuas_trail_line_width_m').value), 0.01)
        _h = _rgba_from_param(self.get_parameter('cuas_trail_hostile_rgba').value, (0.95, 0.12, 0.08, 0.85))
        self._cuas_trail_hostile_rgba = ColorRGBA(r=_h[0], g=_h[1], b=_h[2], a=_h[3])
        _i = _rgba_from_param(self.get_parameter('cuas_trail_interceptor_rgba').value, (0.12, 0.35, 0.95, 0.88))
        self._cuas_trail_inter_rgba = ColorRGBA(r=_i[0], g=_i[1], b=_i[2], a=_i[3])
        self._cuas_pred_marker_d = max(float(self.get_parameter('cuas_intercept_marker_diameter_m').value), 0.2)
        self._cuas_pred_line_w = max(float(self.get_parameter('cuas_intercept_line_width_m').value), 0.01)
        self._cuas_pred_dashed = bool(self.get_parameter('cuas_intercept_line_dashed').value)
        self._cuas_pred_dash = max(float(self.get_parameter('cuas_intercept_dash_length_m').value), 1.0)
        self._cuas_pred_gap = max(float(self.get_parameter('cuas_intercept_gap_length_m').value), 1.0)
        pr_line = _rgba_from_param(self.get_parameter('cuas_intercept_line_rgba').value, (0.15, 0.45, 1.0, 0.88))
        pr_mark = _rgba_from_param(self.get_parameter('cuas_intercept_marker_rgba').value, (1.0, 0.92, 0.15, 0.95))
        self._cuas_pred_text = bool(self.get_parameter('cuas_intercept_predict_text_enabled').value)
        self._cuas_pred_text_dz = float(self.get_parameter('cuas_intercept_predict_text_z_offset_m').value)
        self._cuas_pred_text_scale = max(float(self.get_parameter('cuas_intercept_predict_text_scale_m').value), 1.0)
        self._cuas_pred_text_col = _rgba_from_param(
            self.get_parameter('cuas_intercept_predict_text_rgba').value, (0.95, 0.95, 0.2, 1.0),
        )
        self._cuas_pred_line_col = ColorRGBA(r=pr_line[0], g=pr_line[1], b=pr_line[2], a=pr_line[3])
        self._cuas_pred_sphere_col = ColorRGBA(r=pr_mark[0], g=pr_mark[1], b=pr_mark[2], a=pr_mark[3])
        self._math_viz = (
            bool(self.get_parameter('intercept_math_viz_enabled').value)
            and self._pub_intercept_markers
        )
        self._math_hide_legacy = bool(self.get_parameter('intercept_math_hide_legacy_predict').value)
        self._math_trail_cap = max(10, min(10_000, int(self.get_parameter('intercept_math_trail_max_points').value)))
        self._math_phit_s = max(0.25, float(self.get_parameter('intercept_math_phit_scale_m').value))
        self._math_bear_w = max(0.1, min(0.2, float(self.get_parameter('intercept_math_bearing_line_width_m').value)))
        self._math_trail_w = max(0.08, float(self.get_parameter('intercept_math_trail_line_width_m').value))
        _mg = _rgba_from_param(self.get_parameter('intercept_math_phit_rgba').value, (0.15, 0.85, 0.2, 0.95))
        self._math_phit_col = ColorRGBA(r=_mg[0], g=_mg[1], b=_mg[2], a=_mg[3])
        _my = _rgba_from_param(self.get_parameter('intercept_math_bearing_rgba').value, (1.0, 0.92, 0.1, 0.92))
        self._math_bear_col = ColorRGBA(r=_my[0], g=_my[1], b=_my[2], a=_my[3])
        _mr = _rgba_from_param(self.get_parameter('intercept_math_trail_target_rgba').value, (0.92, 0.12, 0.1, 0.9))
        self._math_trail_tgt_col = ColorRGBA(r=_mr[0], g=_mr[1], b=_mr[2], a=_mr[3])
        _mb = _rgba_from_param(self.get_parameter('intercept_math_trail_inter_rgba').value, (0.12, 0.35, 0.95, 0.9))
        self._math_trail_int_col = ColorRGBA(r=_mb[0], g=_mb[1], b=_mb[2], a=_mb[3])
        self._math_frame = str(self.get_parameter('intercept_math_frame_id').value).strip() or 'map'
        self._math_txt_on = bool(self.get_parameter('intercept_math_text_enabled').value)
        self._math_txt_dz = float(self.get_parameter('intercept_math_text_z_offset_m').value)
        self._math_txt_scale = max(0.5, float(self.get_parameter('intercept_math_text_scale_m').value))
        _mtx = _rgba_from_param(self.get_parameter('intercept_math_text_rgba').value, (1.0, 1.0, 1.0, 0.95))
        self._math_txt_col = ColorRGBA(r=_mtx[0], g=_mtx[1], b=_mtx[2], a=_mtx[3])
        self._math_dbg = bool(self.get_parameter('intercept_math_debug_log').value)
        self._feas_debug = bool(self.get_parameter('intercept_feasibility_debug').value)
        _hf = str(self.get_parameter('intercept_heatmap_frame_id').value).strip()
        self._heatmap_frame = _hf or self._dome_frame
        self._heatmap_z_off = float(self.get_parameter('intercept_heatmap_z_offset_m').value)
        self._heatmap_ref_mode = str(self.get_parameter('intercept_heatmap_reference_interceptor').value).strip().lower()
        _htv = list(self.get_parameter('intercept_heatmap_target_vel_m_s').value)
        self._heatmap_v_tx = float(_htv[0]) if len(_htv) > 0 else 0.0
        self._heatmap_v_ty = float(_htv[1]) if len(_htv) > 1 else 0.0
        self._heatmap_v_tz = float(_htv[2]) if len(_htv) > 2 else 0.0
        # Per (target_key, interceptor_id): feasible flag latched when assignment to that interceptor begins.
        # Single-hostile mode uses target_key ''.
        self._feasible_at_engagement_start_by_pair: dict[tuple[str, str], bool] = {}
        self._feas_eng_latch_assign: dict[str, str] = {}
        self._last_feas_debug_log: Time | None = None
        self._last_feas_warn_log: Time | None = None
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
        self._sense_gate_enabled = bool(self.get_parameter('sensing_gate_enabled').value)
        self._radar_px = float(self.get_parameter('radar_position_x').value)
        self._radar_py = float(self.get_parameter('radar_position_y').value)
        self._radar_pz = float(self.get_parameter('radar_position_z').value)
        self._radar_range_m = max(float(self.get_parameter('radar_range_m').value), 0.0)
        self._tracking_delay_s = max(float(self.get_parameter('tracking_delay_s').value), 0.0)
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
        # Per-target outer-dome hysteresis (multi mode only; single-target uses _dome_outside_latched).
        self._multi_dome_outside_latched: dict[str, bool] = {}
        self._multi_dome_hyst_init: dict[str, bool] = {}
        if self._multi_enabled:
            for lab in self._multi_labels:
                self._multi_dome_outside_latched[lab] = False
                self._multi_dome_hyst_init[lab] = False

        tgt_topic_raw = str(self.get_parameter('target_topic').value).strip()
        _meas_src = str(self.get_parameter('intercept_measurement_source').value).strip().lower()
        _fused_t = str(self.get_parameter('fused_detections_topic').value).strip()
        _tracks_t = str(self.get_parameter('tracks_topic').value).strip()
        _tracks_state_t = str(self.get_parameter('tracks_state_topic').value).strip()
        # ``_meas_use_filter_vel``: when True, _on_target_state has populated
        # ``_target_filter_velocity`` and _estimate_target_vel returns it directly instead of
        # differencing positions (which is the root of jitter when /tracks runs at 10 Hz).
        self._meas_use_filter_vel = False
        # Latest velocity reported by the upstream estimator (nav_msgs/Odometry.twist.linear).
        self._target_filter_velocity: tuple[float, float, float] | None = None
        if _meas_src in ('fused', 'fused_detections'):
            tgt_topic = _fused_t or '/fused_detections'
        elif _meas_src in ('tracks', 'track'):
            tgt_topic = _tracks_t or '/tracks'
        elif _meas_src in ('tracks_state', 'track_state', 'odometry'):
            tgt_topic = _tracks_state_t or '/tracks/state'
            self._meas_use_filter_vel = True
        elif _meas_src in ('ground_truth', 'direct', 'gt', ''):
            tgt_topic = tgt_topic_raw or '/drone/position'
        else:
            self.get_logger().warning(
                f'unknown intercept_measurement_source={_meas_src!r}; using target_topic',
            )
            tgt_topic = tgt_topic_raw or '/drone/position'
        self.get_logger().info(
            f'intercept target subscription: source={_meas_src or "ground_truth"} topic={tgt_topic}'
            f'{" (filter velocity)" if self._meas_use_filter_vel else ""}',
        )
        # Cache for branching the subscription type below. tgt_topic is the resolved name;
        # tgt_is_state controls whether we subscribe Odometry or Point.
        self._tgt_is_state = self._meas_use_filter_vel
        sel_topic = str(self.get_parameter('selected_id_topic').value).strip()
        stop_topic = str(self.get_parameter('stop_topic').value).strip()
        self._stop_topic = stop_topic or '/target/stop'
        self._stop_repeat_duration_s = max(0.0, float(self.get_parameter('stop_signal_repeat_duration_s').value))
        self._stop_repeat_period_s = max(0.05, float(self.get_parameter('stop_signal_repeat_period_s').value))
        rate = max(float(self.get_parameter('rate_hz').value), 1.0)
        self._lock_after_first = bool(self.get_parameter('lock_selected_after_first').value)
        self._hit_thresh = max(float(self.get_parameter('hit_threshold_m').value), 0.05)
        self._hit_thresh_dbg = max(float(self.get_parameter('hit_threshold_m_debug').value), 0.05)
        self._dbg_predictive = bool(self.get_parameter('intercept_debug_force_predictive_guidance').value)
        self._dbg_spd_margin = max(0.0, float(self.get_parameter('intercept_debug_speed_margin_m_s').value))
        self._dbg_hit_log_period = max(0.0, float(self.get_parameter('intercept_debug_hit_log_period_s').value))
        self._dbg_no_hit_p_thr = min(1.0, max(0.0, float(self.get_parameter('intercept_debug_no_hit_p_threshold').value)))
        self._dbg_no_hit_grace = max(0.0, float(self.get_parameter('intercept_debug_no_hit_grace_s').value))
        self._last_hit_debug_print_m: float = 0.0
        # (watch_key -> monotonic start) when feasible ∧ P high ∧ have t_go; used for t_go+grace warning.
        self._dbg_no_hit_watch_start_m: dict[str, float] = {}
        self._dbg_no_hit_watch_tgo: dict[str, float] = {}
        self._dbg_no_hit_warn_last_m: dict[str, float] = {}
        self._last_hit_gate_block_print_m: float = 0.0
        self._multi_hit_safety_logged: set[tuple[str, str]] = set()
        # MC probability heatmap (reads after hit_thresh / timing params exist).
        self._heatmap_prob_on = bool(self.get_parameter('intercept_heatmap_prob_enabled').value)
        _hm_ex = str(self.get_parameter('intercept_heatmap_prob_export_dir').value).strip()
        self._heatmap_prob_export_dir: Path | None = (
            Path(_hm_ex).expanduser() if _hm_ex else None
        )
        self._heatmap_prob_export_stamp_files = bool(
            self.get_parameter('intercept_heatmap_prob_export_stamp_files').value,
        )
        self._heatmap_prob_export_warned = False
        if self._heatmap_prob_export_dir is not None:
            self._heatmap_prob_on = True
            _prune_intercept_heatmap_prob_export_dir(self._heatmap_prob_export_dir)
        self._heatmap_prob_topic = (
            str(self.get_parameter('intercept_heatmap_prob_topic').value).strip() or '/interception/heatmap_prob'
        )
        _hppb = float(self.get_parameter('intercept_heatmap_prob_update_period_s').value)
        self._heatmap_prob_period = min(2.0, max(0.5, _hppb))
        _hdpr = float(self.get_parameter('intercept_heatmap_prob_grid_step_m').value)
        # Coarser grid reduces cell count × MC cost for real-time demos (floor 300 m).
        self._heatmap_prob_step = min(400.0, max(300.0, _hdpr))
        _hspr = float(self.get_parameter('intercept_heatmap_prob_sphere_diameter_m').value)
        self._heatmap_prob_sphere_d = min(250.0, max(100.0, _hspr))
        self._heatmap_prob_alpha = min(0.55, max(0.25, float(self.get_parameter('intercept_heatmap_prob_alpha').value)))
        self._heatmap_prob_mc_n = max(8, min(30, int(self.get_parameter('intercept_heatmap_prob_mc_n').value)))
        self._heatmap_prob_pos_sig = max(0.0, float(self.get_parameter('intercept_heatmap_prob_pos_sigma_m').value))
        self._heatmap_prob_vel_sig = max(0.0, float(self.get_parameter('intercept_heatmap_prob_vel_sigma_m_s').value))
        self._heatmap_prob_is_sig = max(0.0, float(
            self.get_parameter('intercept_heatmap_prob_interceptor_pos_sigma_m').value))
        self._heatmap_prob_dly_jit = max(0.0, float(self.get_parameter('intercept_heatmap_prob_delay_jitter_s').value))
        self._heatmap_prob_use_td = bool(self.get_parameter('intercept_heatmap_prob_use_node_tracking_delay').value)
        self._heatmap_prob_use_cmd_vel = bool(
            self.get_parameter('intercept_heatmap_prob_use_cmd_vel').value,
        )
        self._heatmap_prob_rollout = bool(self.get_parameter('intercept_heatmap_prob_use_kinematic_rollout').value)
        self._heatmap_prob_rollout_dt = max(
            0.02, min(0.5, float(self.get_parameter('intercept_heatmap_prob_rollout_dt_s').value)),
        )
        self._heatmap_prob_rollout_mc_cap = max(
            4, min(30, int(self.get_parameter('intercept_heatmap_prob_rollout_mc_cap').value)),
        )
        self._heatmap_prob_cell_los = bool(
            self.get_parameter('intercept_heatmap_prob_use_cell_los_velocity').value,
        )
        self._heatmap_prob_los_spd = max(
            0.05, float(self.get_parameter('intercept_heatmap_prob_target_los_speed_m_s').value),
        )
        self._heatmap_prob_threat_z_mode = str(
            self.get_parameter('intercept_heatmap_prob_threat_z_mode').value,
        )
        self._heatmap_prob_threat_z_inner_m = float(
            self.get_parameter('intercept_heatmap_prob_threat_z_inner_m').value,
        )
        self._heatmap_prob_threat_z_outer_m = float(
            self.get_parameter('intercept_heatmap_prob_threat_z_outer_m').value,
        )
        self._heatmap_prob_threat_z_r_ref_m = float(
            self.get_parameter('intercept_heatmap_prob_threat_z_r_ref_m').value,
        )
        self._heatmap_prob_cache_ttl = max(0.0, float(self.get_parameter('intercept_heatmap_prob_cache_ttl_s').value))
        self._heatmap_prob_dbg_n = max(0, min(50, int(self.get_parameter('intercept_heatmap_prob_debug_samples').value)))
        _seed = int(self.get_parameter('intercept_heatmap_prob_rng_seed').value)
        self._heatmap_prob_rng = random.Random()
        if _seed >= 0:
            self._heatmap_prob_rng.seed(_seed)
        self._heatmap_prob_xy_offsets: list[tuple[float, float]] = []
        # (cell_idx, iq_x, iq_y) -> (prob, monotonic_timestamp); optional TTL refresh.
        self._heatmap_prob_cache: dict[tuple[int, int, int], tuple[float, float]] = {}
        self._heatmap_prob_cache_q = max(40.0, self._heatmap_prob_step * 0.5)
        if self._heatmap_prob_on:
            self._heatmap_prob_xy_offsets = _build_dome_disk_xy_offsets(self._r_outer, self._heatmap_prob_step)
        self._heatmap_prob_show_pct = bool(
            self.get_parameter('intercept_heatmap_prob_show_percent_labels').value)
        self._heatmap_prob_lbl_scale = max(
            8.0, float(self.get_parameter('intercept_heatmap_prob_label_scale_m').value))
        self._heatmap_prob_lbl_dz = float(self.get_parameter('intercept_heatmap_prob_label_z_offset_m').value)
        self._intercept_mc_trail_by_prob = bool(self.get_parameter('intercept_mc_trail_color_by_prob').value)
        self._intercept_mc_engagement = bool(self.get_parameter('intercept_mc_engagement_enabled').value)
        self._intercept_mc_min_p = min(
            1.0, max(0.0, float(self.get_parameter('intercept_mc_min_prob_strike').value)))
        self._intercept_mc_runtime_n = max(
            8, min(48, int(self.get_parameter('intercept_mc_runtime_n').value)))
        self._intercept_mc_use_light = bool(self.get_parameter('intercept_mc_use_light_hit_model').value)
        self._intercept_mc_light_pos_sig = max(
            0.0, float(self.get_parameter('intercept_mc_light_pos_sigma_m').value))
        self._intercept_mc_light_vel_sig = max(
            0.0, float(self.get_parameter('intercept_mc_light_vel_sigma_m_s').value))
        self._intercept_mc_light_dly_jit = max(
            0.0, float(self.get_parameter('intercept_mc_light_delay_jitter_s').value))
        self._intercept_mc_light_samples = max(1, min(30, int(self.get_parameter('intercept_mc_light_samples').value)))
        self._intercept_mc_hyst_on = bool(self.get_parameter('intercept_mc_hysteresis_enabled').value)
        self._intercept_mc_hyst_hi = min(1.0, max(0.0, float(self.get_parameter('intercept_mc_hysteresis_high').value)))
        self._intercept_mc_hyst_lo = min(1.0, max(0.0, float(self.get_parameter('intercept_mc_hysteresis_low').value)))
        if self._intercept_mc_hyst_lo >= self._intercept_mc_hyst_hi:
            self._intercept_mc_hyst_lo = max(0.0, self._intercept_mc_hyst_hi - 0.1)
        self._heatmap_prob_budget = max(0.05, float(self.get_parameter('intercept_heatmap_prob_time_budget_s').value))
        self._heatmap_prob_skip_next: bool = False
        self._mc_engage_state: dict[str, bool] = {}
        self._last_guidance_cmd: dict[str, tuple[float, float, float]] = {i: (0.0, 0.0, 0.0) for i in self._ids}
        self._phase1_warn = bool(self.get_parameter('intercept_mc_phase1_consistency_warn').value)
        self._phase1_dbg = bool(self.get_parameter('intercept_mc_phase1_debug_log').value)
        self._mc_high_p_no_hit_s = max(3.0, float(self.get_parameter('intercept_mc_high_p_no_hit_s').value))
        self._mc_high_p_start_m: dict[str, float] = {}
        self._last_phase1_high_p_warn: dict[str, float] = {}
        self._last_phase1_feas_warn_m: float = 0.0
        self._last_phase1_vreq_warn_m: float = 0.0
        self._last_phase1_engage_warn_m: float = 0.0
        self._mc_p_last: dict[str, float] = {}
        _min_tz = float(self.get_parameter('hit_min_target_z_m').value)
        self._hit_min_tz: float | None = _min_tz if _min_tz > -0.5 else None
        self._hit_min_iz = float(self.get_parameter('hit_min_interceptor_z_m').value)
        self._hit_min_travel = max(float(self.get_parameter('hit_min_interceptor_travel_m').value), 0.0)
        self._inter_start_pos: dict[str, tuple[float, float, float]] = {}

        self._pubs: dict[str, Publisher] = {}
        for iid in self._ids:
            cmd_topic = f'/{iid}/cmd_velocity'
            self._pubs[iid] = self.create_publisher(Vector3, cmd_topic, 10)
        self._pub_selected = self.create_publisher(String, sel_topic, 10)
        # VOLATILE: /target/stop is a one-shot command. TRANSIENT_LOCAL would latch True
        # and late-joining target_controller could "explode" on startup with a stale HIT
        # from a previous run (no [HIT] line in the current log).
        _stop_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
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
        elif self._tgt_is_state:
            self.create_subscription(Odometry, tgt_topic, self._on_target_state, 10)
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
        # Per-target first time the target is considered "in radar range" for tracking-delay gating.
        self._target_detect_time: Time | None = None
        self._multi_target_detect_time: dict[str, Time | None] = {}
        self._multi_prev_assign: dict[str, str] = {}
        self._multi_prev_target: dict[str, tuple[float, float, float] | None] = {}
        self._multi_prev_target_time: dict[str, Time | None] = {}
        self._multi_v_smooth: dict[str, tuple[float, float, float]] = {}
        self._last_assignment_print: Time | None = None
        if self._multi_enabled:
            for lab in self._multi_labels:
                self._targets[lab] = None
                self._hits_multi[lab] = False
                self._multi_target_detect_time[lab] = None
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

        # Interceptor assignment lock: prevents reselection for a fixed duration.
        self._assigned_interceptor_id: str | None = None
        self._assignment_time: Time | None = None
        self._assignment_lock_duration: float = 1.5
        self._last_assign_lock_log: Time | None = None

        # Acceleration limit: smooth velocity commands to prevent fly-stop-fly jerks.
        # _control_dt is updated every cycle; _control_dt_default is the nominal period.
        self._control_dt_default: float = 1.0 / rate
        self._control_dt: float = self._control_dt_default
        self._last_control_time: Time | None = None
        self._prev_velocity: dict[str, tuple[float, float, float]] = {
            iid: (0.0, 0.0, 0.0) for iid in self._ids
        }

        # Intercept-point EMA filter: per-interceptor smoothed predicted hit location.
        # None = uninitialized (first valid solve seeds the filter without blending).
        self._intercept_point_filtered: dict[str, tuple[float, float, float] | None] = {
            iid: None for iid in self._ids
        }

        # Mode-switching hysteresis state (per interceptor).
        # _guidance_mode: committed active guidance mode ('pursuit' or 'predict').
        # _valid_streak:  consecutive frames that produced a valid predict solution.
        # _invalid_streak: consecutive frames without a valid predict solution.
        # Both streaks are reset to 0 whenever the opposite signal is observed.
        self._guidance_mode: dict[str, str] = {iid: 'pursuit' for iid in self._ids}
        self._valid_streak: dict[str, int] = {iid: 0 for iid in self._ids}
        self._invalid_streak: dict[str, int] = {iid: 0 for iid in self._ids}

        self._hit = False
        # Previous-tick target positions for segment-vs-segment HIT (velocity estimator overwrites _prev_target).
        self._hit_snap_target_prev: tuple[float, float, float] | None = None
        self._hit_snap_target_prev_multi: dict[str, tuple[float, float, float]] = {}
        self._logged_guard_block = False
        self._dome_outside_latched = False
        self._dome_hyst_initialized = False
        self._last_layer = ''
        self._world_name_gz = str(self.get_parameter('world_name').value).strip() or 'counter_uas_target'
        self._pause_gz_on_hit = bool(self.get_parameter('pause_gz_on_hit').value)
        self._gz_pause_sent = False
        self._last_algo_log: Time | None = None
        self._last_feas_log: Time | None = None
        self._last_class_warn: Time | None = None
        # Closest 3D range |P_T - P_I| seen this run (engagement paths only); reset on sim rewind.
        self._min_miss_distance = float("inf")
        # Last plausible |P_T-P_I| per interceptor on control ticks — rejects one-tick "teleports"
        # toward the target when Gazebo/pose pipeline glitches (bogus HIT / HIT_BLOCKED_SAFETY).
        self._last_hit_range: dict[str, float | None] = {i: None for i in self._ids}
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

        self._pub_heatmap_prob: Publisher | None = None
        if self._heatmap_prob_on:
            self._pub_heatmap_prob = self.create_publisher(Marker, self._heatmap_prob_topic, qos_marker)
            self.create_timer(self._heatmap_prob_period, self._publish_intercept_heatmap_prob)

        # ── RViz intercept markers ────────────────────────────────────────────
        # SPHERE at predicted hit point + LINE_STRIP interceptor→hit point.
        self._pub_intercept_viz: Publisher | None = (
            self.create_publisher(Marker, '/interception/markers', qos_marker)
            if self._pub_intercept_markers else None
        )
        self._pub_hit_viz: Publisher | None = None
        self._hit_viz_timer = None
        self._hit_viz_t0_m = 0.0
        self._hit_viz_mid: tuple[float, float, float] | None = None
        self._hit_viz_subtitle = ''
        if self._publish_hit_markers:
            self._pub_hit_viz = self.create_publisher(Marker, self._hit_marker_topic, qos_marker)
        self._stop_repeat_timer = None
        self._stop_repeat_deadline: Time | None = None
        self._stop_repeat_pending_label: str | None = None
        _tm = self._cuas_trail_max_points
        self._trail_target = _trail_deque(_tm)
        self._trail_inters: dict[str, deque[tuple[float, float, float]]] = {
            i: _trail_deque(_tm) for i in self._ids
        }
        self._trail_multi_hostile: dict[str, deque[tuple[float, float, float]]] = {}
        if self._multi_enabled:
            for lab in self._multi_labels:
                self._trail_multi_hostile[lab] = _trail_deque(_tm)

        self._math_trail_target: deque[tuple[float, float, float]] = deque(maxlen=self._math_trail_cap)
        self._math_trail_inters: dict[str, deque[tuple[float, float, float]]] = {
            i: deque(maxlen=self._math_trail_cap) for i in self._ids
        }

        # ── Per-tick metrics snapshot (updated in guidance callbacks) ─────────
        # Stores the latest values for each interceptor so the metrics timer can
        # print a consolidated line without coupling to the guidance hot-path.
        self._metrics: dict[str, dict] = {
            iid: {
                'dist': float('nan'),
                't_go': None,
                'vel_mag': 0.0,
                'mode': 'pursuit',
                'feasible': None,
                'feas_status': 'n/a',
                'feas_reason': '',
            }
            for iid in self._ids
        }
        # Metrics print timer (separate from intercept detail log).
        self.create_timer(self._metrics_period, self._print_metrics)

        period = 1.0 / rate
        self._timer = self.create_timer(period, self._on_control)
        if bool(self.get_parameter('reset_on_sim_clock_rewind').value):
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

    # ── Metrics & visualization helpers ──────────────────────────────────────

    def _print_metrics(self) -> None:
        """Print a one-line metrics summary for every active interceptor."""
        lines = ['[METRICS]']
        for iid in self._ids:
            m = self._metrics[iid]
            dist = m['dist']
            t_go = m['t_go']
            vel  = m['vel_mag']
            mode = m['mode']
            miss = self._min_miss_distance

            dist_s = f'{dist:.3f} m'   if math.isfinite(dist) else 'n/a'
            tgo_s  = f'{t_go:.3f} s'   if t_go is not None else 'n/a'
            miss_s = f'{miss:.3f} m'   if miss < float("inf") else 'n/a'

            fs = str(m.get('feas_status', 'n/a'))
            fr = str(m.get('feas_reason', ''))
            fb = m.get('feasible')
            fbs = 'n/a' if fb is None else str(bool(fb))
            feas_col = f' feas={fbs} [{fs}]'
            if fr:
                feas_col += f' ({fr})'
            lines.append(
                f'  id={iid:20s} | dist={dist_s:9s} | t_go={tgo_s:9s} | '
                f'vel={vel:.3f} m/s | mode={mode:8s} | min_miss={miss_s}{feas_col}'
            )
        print('\n'.join(lines), flush=True)

    def _cancel_hit_viz_timer(self) -> None:
        if self._hit_viz_timer is not None:
            self.destroy_timer(self._hit_viz_timer)
            self._hit_viz_timer = None

    def _clear_hit_markers(self) -> None:
        if self._pub_hit_viz is None:
            return
        stamp = self.get_clock().now().to_msg()
        clr = Marker()
        clr.header.frame_id = self._dome_frame
        clr.header.stamp = stamp
        clr.ns = 'hit_flash'
        clr.action = Marker.DELETEALL
        self._pub_hit_viz.publish(clr)

    def _publish_hit_markers_frame(self, elapsed: float) -> None:
        if self._pub_hit_viz is None or self._hit_viz_mid is None:
            return
        mx, my, mz = self._hit_viz_mid
        stamp = self.get_clock().now().to_msg()
        frame = self._dome_frame
        tnorm = min(1.0, max(0.0, elapsed / self._hit_marker_duration))
        pulse = math.sin(tnorm * math.pi)
        diam = self._hit_pulse_d_min + (self._hit_pulse_d_max - self._hit_pulse_d_min) * pulse

        text = Marker()
        text.header.frame_id = frame
        text.header.stamp = stamp
        text.ns = 'hit_flash'
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = mx
        text.pose.position.y = my
        text.pose.position.z = mz + self._hit_marker_text_dz
        text.pose.orientation.w = 1.0
        text.scale.z = self._hit_marker_text_scale
        body = 'HIT'
        if self._hit_viz_subtitle:
            body = f'HIT\n{self._hit_viz_subtitle}'
        text.text = body
        text.color.r = 1.0
        text.color.g = 0.95
        text.color.b = 0.15
        text.color.a = min(1.0, 0.35 + 0.65 * pulse)

        sph = Marker()
        sph.header.frame_id = frame
        sph.header.stamp = stamp
        sph.ns = 'hit_flash'
        sph.id = 1
        sph.type = Marker.SPHERE
        sph.action = Marker.ADD
        sph.pose.position.x = mx
        sph.pose.position.y = my
        sph.pose.position.z = mz
        sph.pose.orientation.w = 1.0
        sph.scale.x = diam
        sph.scale.y = diam
        sph.scale.z = diam
        sph.color.r = 1.0
        sph.color.g = 0.25
        sph.color.b = 0.05
        sph.color.a = 0.28 + 0.35 * pulse

        outer = Marker()
        outer.header.frame_id = frame
        outer.header.stamp = stamp
        outer.ns = 'hit_flash'
        outer.id = 2
        outer.type = Marker.SPHERE
        outer.action = Marker.ADD
        outer.pose.position.x = mx
        outer.pose.position.y = my
        outer.pose.position.z = mz
        outer.pose.orientation.w = 1.0
        od = float(self._hit_outer_d)
        outer.scale.x = outer.scale.y = outer.scale.z = od
        outer.color.r = 1.0
        outer.color.g = 0.55
        outer.color.b = 0.08
        outer.color.a = self._hit_outer_alpha_max * (0.55 + 0.45 * pulse)

        self._pub_hit_viz.publish(text)
        self._pub_hit_viz.publish(sph)
        self._pub_hit_viz.publish(outer)

    def _on_hit_viz_tick(self) -> None:
        if self._pub_hit_viz is None or self._hit_viz_mid is None:
            self._cancel_hit_viz_timer()
            return
        elapsed = time.monotonic() - self._hit_viz_t0_m
        if elapsed >= self._hit_marker_duration:
            self._clear_hit_markers()
            self._cancel_hit_viz_timer()
            self._hit_viz_mid = None
            return
        self._publish_hit_markers_frame(elapsed)

    def _start_hit_visual(
        self,
        tx: float,
        ty: float,
        tz: float,
        ix: float,
        iy: float,
        iz: float,
        subtitle: str,
    ) -> None:
        if self._pub_hit_viz is None:
            return
        self._cancel_hit_viz_timer()
        self._hit_viz_mid = ((tx + ix) * 0.5, (ty + iy) * 0.5, (tz + iz) * 0.5)
        self._hit_viz_subtitle = subtitle.strip()
        self._hit_viz_t0_m = time.monotonic()
        self._publish_hit_markers_frame(0.0)
        self._hit_viz_timer = self.create_timer(self._hit_marker_pulse_period, self._on_hit_viz_tick)

    def _clear_intercept_heatmap_prob_markers(self) -> None:
        if self._pub_heatmap_prob is None:
            return
        stamp = self.get_clock().now().to_msg()
        for ns in ('intercept_heatmap_prob', 'intercept_heatmap_prob_lbl'):
            clr = Marker()
            clr.header.frame_id = self._heatmap_frame
            clr.header.stamp = stamp
            clr.ns = ns
            clr.action = Marker.DELETEALL
            self._pub_heatmap_prob.publish(clr)

    def _heatmap_reference_iid(self) -> str:
        mode = self._heatmap_ref_mode
        if mode in ('', 'first'):
            return self._ids[0]
        if mode == 'selected':
            cur = self._current_selected_id
            if cur is not None and cur in self._ids:
                return cur
            return self._ids[0]
        if mode in self._ids:
            return mode
        return self._ids[0]

    def _publish_intercept_heatmap_prob(self) -> None:
        """
        Phase-1 probability heatmap (≈1 Hz): dome disk grid, sphere colors by P_hit.

        Model priority (heatmap only):

        1. ``intercept_heatmap_prob_use_kinematic_rollout`` → ``estimate_hit_probability`` with
           ``simulate_intercept_once`` / ``_monte_carlo_kinematic_hit_rollout`` (turn + accel limits).
        2. Else ``intercept_mc_use_light_hit_model`` → ``estimate_hit_probability_light``.
        3. Else MC on ``is_intercept_feasible`` (no rollout).

        Set ``intercept_heatmap_prob_use_cmd_vel:=true`` only for legacy light-mode cmd extrapolation.
        """
        if self._pub_heatmap_prob is None or not self._heatmap_prob_xy_offsets:
            return
        if self._heatmap_prob_skip_next:
            self._heatmap_prob_skip_next = False
            return
        wall0 = time.monotonic()
        iid = self._heatmap_reference_iid()
        p_sel = self._inter_pos.get(iid)
        if p_sel is None:
            self._clear_intercept_heatmap_prob_markers()
            return
        ix, iy, iz = float(p_sel.x), float(p_sel.y), float(p_sel.z)
        cx, cy = self._dome_cx, self._dome_cy
        r_ref_hm = float(self._heatmap_prob_threat_z_r_ref_m)
        if r_ref_hm <= 0.0:
            r_ref_hm = float(self._r_outer)
        v_tx_param, v_ty_param, v_tz_param = self._heatmap_v_tx, self._heatmap_v_ty, self._heatmap_v_tz
        # Optional: extrapolate interceptor motion along last cmd (legacy). Default false: use v_i_max
        # toward each sample's p_hit so grid cells match their own intercept geometry.
        ivx_hm = ivy_hm = ivz_hm = None
        if self._heatmap_prob_use_cmd_vel:
            _lvx, _lvy, _lvz = self._last_guidance_cmd.get(iid, (0.0, 0.0, 0.0))
            _cm = _norm(_lvx, _lvy, _lvz)
            if _cm > 1e-6:
                ivx_hm, ivy_hm, ivz_hm = _lvx, _lvy, _lvz
        delay_mean = float(self._tracking_delay_s) if self._heatmap_prob_use_td else 0.0
        stamp = self.get_clock().now().to_msg()
        now_m = time.monotonic()
        ttl = float(self._heatmap_prob_cache_ttl)
        cq = max(1e-3, float(self._heatmap_prob_cache_q))
        iq_x = int(ix / cq)
        iq_y = int(iy / cq)
        m = Marker()
        m.header.frame_id = self._heatmap_frame
        m.header.stamp = stamp
        m.ns = 'intercept_heatmap_prob'
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = self._heatmap_prob_sphere_d
        m.scale.y = self._heatmap_prob_sphere_d
        m.scale.z = self._heatmap_prob_sphere_d
        pts: list[Point] = []
        cols: list[ColorRGBA] = []
        dbg_lines: list[str] = []
        lbl_cells: list[tuple[float, float, float, float]] = []
        hm_rollout = self._heatmap_prob_rollout
        n_hm_mc = self._heatmap_prob_mc_n
        if hm_rollout:
            n_hm_mc = min(n_hm_mc, self._heatmap_prob_rollout_mc_cap)
        heatmap_model_tag = (
            'kinematic_rollout'
            if hm_rollout
            else ('light' if self._intercept_mc_use_light else 'mc_feasibility')
        )
        for idx, (gx, gy) in enumerate(self._heatmap_prob_xy_offsets):
            tx = cx + gx
            ty = cy + gy
            tz = intercept_heatmap_prob_threat_altitude_m(
                self._dome_cx,
                self._dome_cy,
                self._dome_cz,
                tx,
                ty,
                mode=self._heatmap_prob_threat_z_mode,
                z_offset_m=self._heatmap_z_off,
                z_inner_m=self._heatmap_prob_threat_z_inner_m,
                z_outer_m=self._heatmap_prob_threat_z_outer_m,
                r_ref_m=r_ref_hm,
            )
            v_tx, v_ty, v_tz = intercept_heatmap_prob_cell_velocity(
                self._dome_cx,
                self._dome_cy,
                self._dome_cz,
                tx,
                ty,
                tz,
                use_cell_los=self._heatmap_prob_cell_los,
                los_spd_m_s=self._heatmap_prob_los_spd,
                v_tx_param=v_tx_param,
                v_ty_param=v_ty_param,
                v_tz_param=v_tz_param,
                v_smooth=self._v_tgt_smooth,
            )
            ckey = (idx, iq_x, iq_y)
            prob = 0.0
            need_mc = True
            if ttl > 1e-9:
                ent = self._heatmap_prob_cache.get(ckey)
                if ent is not None:
                    p_cached, t_cached = ent
                    if now_m - t_cached < ttl:
                        prob = p_cached
                        need_mc = False
            if need_mc:
                if hm_rollout:
                    prob = estimate_hit_probability(
                        tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz,
                        v_i_max=self._interceptor_max_speed,
                        t_min=self._t_hit_min,
                        t_max=self._t_hit_max,
                        hit_thresh_m=self._hit_thresh,
                        n_samples=n_hm_mc,
                        pos_sigma_m=self._heatmap_prob_pos_sig,
                        vel_sigma_m_s=self._heatmap_prob_vel_sig,
                        interceptor_pos_sigma_m=self._heatmap_prob_is_sig,
                        delay_mean_s=delay_mean,
                        delay_jitter_s=self._heatmap_prob_dly_jit,
                        rng=self._heatmap_prob_rng,
                        use_kinematic_rollout=True,
                        rollout_dt=self._heatmap_prob_rollout_dt,
                        rollout_max_turn_rate_rad_s=self._max_turn_rate,
                        rollout_max_accel_m_s2=self._max_accel,
                    )
                elif self._intercept_mc_use_light:
                    prob = estimate_hit_probability_light(
                        tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz,
                        v_i_max=self._interceptor_max_speed,
                        t_min=self._t_hit_min,
                        t_max=self._t_hit_max,
                        hit_thresh_m=self._hit_thresh,
                        n_samples=self._heatmap_prob_mc_n,
                        pos_sigma_m=self._intercept_mc_light_pos_sig,
                        vel_sigma_m_s=self._intercept_mc_light_vel_sig,
                        delay_mean_s=delay_mean,
                        delay_jitter_s=self._intercept_mc_light_dly_jit,
                        rng=self._heatmap_prob_rng,
                        ivx=ivx_hm, ivy=ivy_hm, ivz=ivz_hm,
                    )
                else:
                    prob = estimate_hit_probability(
                        tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz,
                        v_i_max=self._interceptor_max_speed,
                        t_min=self._t_hit_min,
                        t_max=self._t_hit_max,
                        hit_thresh_m=self._hit_thresh,
                        n_samples=n_hm_mc,
                        pos_sigma_m=self._heatmap_prob_pos_sig,
                        vel_sigma_m_s=self._heatmap_prob_vel_sig,
                        interceptor_pos_sigma_m=self._heatmap_prob_is_sig,
                        delay_mean_s=delay_mean,
                        delay_jitter_s=self._heatmap_prob_dly_jit,
                        rng=self._heatmap_prob_rng,
                        use_kinematic_rollout=False,
                        rollout_dt=self._heatmap_prob_rollout_dt,
                        rollout_max_turn_rate_rad_s=0.0,
                        rollout_max_accel_m_s2=0.0,
                    )
                if ttl > 1e-9:
                    self._heatmap_prob_cache[ckey] = (prob, now_m)
            r, g, b, a = _heatmap_rgba_for_score(prob, self._heatmap_prob_alpha)
            pts.append(Point(x=tx, y=ty, z=tz))
            cols.append(ColorRGBA(r=r, g=g, b=b, a=a))
            lbl_cells.append((tx, ty, tz, prob))
            if len(dbg_lines) < self._heatmap_prob_dbg_n:
                dbg_lines.append(
                    f'[P_HEATMAP] pos=({tx:.1f},{ty:.1f},{tz:.1f}) '
                    f'P(hit|noise model)={prob:.2f}',
                )
        m.points = pts
        m.colors = cols
        self._pub_heatmap_prob.publish(m)
        if self._heatmap_prob_export_dir is not None and lbl_cells:
            try:
                stamp_wall = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
                export_intercept_heatmap_prob_to_disk(
                    self._heatmap_prob_export_dir,
                    lbl_cells,
                    frame_id=m.header.frame_id,
                    ref_iid=iid,
                    mc_n=n_hm_mc,
                    heatmap_model=heatmap_model_tag,
                    stamp_wall=stamp_wall,
                    stamp_files=self._heatmap_prob_export_stamp_files,
                )
            except OSError as exc:
                if not self._heatmap_prob_export_warned:
                    self._heatmap_prob_export_warned = True
                    self.get_logger().warning(
                        f'intercept_heatmap_prob export failed (suppressing further logs): {exc}',
                    )
        if self._heatmap_prob_show_pct and lbl_cells:
            lbl_clr = Marker()
            lbl_clr.header.frame_id = self._heatmap_frame
            lbl_clr.header.stamp = stamp
            lbl_clr.ns = 'intercept_heatmap_prob_lbl'
            lbl_clr.action = Marker.DELETEALL
            self._pub_heatmap_prob.publish(lbl_clr)
            for idx, (tx_c, ty_c, tz_c, prob_c) in enumerate(lbl_cells):
                tm = Marker()
                tm.header.frame_id = self._heatmap_frame
                tm.header.stamp = stamp
                tm.ns = 'intercept_heatmap_prob_lbl'
                tm.id = idx
                tm.type = Marker.TEXT_VIEW_FACING
                tm.action = Marker.ADD
                tm.pose.position.x = tx_c
                tm.pose.position.y = ty_c
                tm.pose.position.z = tz_c + self._heatmap_prob_lbl_dz
                tm.pose.orientation.w = 1.0
                tm.scale.z = self._heatmap_prob_lbl_scale
                lr, lg, lb, _ = _heatmap_rgba_for_score(prob_c, 1.0)
                tm.color = ColorRGBA(r=lr, g=lg, b=lb, a=0.92)
                tm.text = f'{prob_c * 100.0:.0f}%\nP(hit|noise)'
                tm.lifetime.sec = 0
                self._pub_heatmap_prob.publish(tm)
        if dbg_lines:
            print('\n'.join(dbg_lines), flush=True)
        ncells = len(self._heatmap_prob_xy_offsets)
        if hm_rollout:
            nsamp_eff = n_hm_mc
        elif self._intercept_mc_use_light:
            nsamp_eff = self._heatmap_prob_mc_n
        else:
            nsamp_eff = n_hm_mc
        elapsed_hm = time.monotonic() - wall0
        print(
            f'[HEATMAP_PERF] cells={ncells} samples={nsamp_eff} time={elapsed_hm:.3f}s '
            f'budget={self._heatmap_prob_budget:.3f}s model={heatmap_model_tag}',
            flush=True,
        )
        if elapsed_hm > self._heatmap_prob_budget:
            self._heatmap_prob_skip_next = True

    def _mc_delay_mean_s(self) -> float:
        return float(self._tracking_delay_s) if self._heatmap_prob_use_td else 0.0

    def _update_mc_engage_hysteresis(self, iid: str, p: float) -> None:
        """Schmitt trigger on P_hit to avoid engagement flicker (Phase 1)."""
        if not self._intercept_mc_engagement or not self._intercept_mc_hyst_on:
            return
        if not math.isfinite(p):
            return
        if iid not in self._mc_engage_state:
            self._mc_engage_state[iid] = False
        if p >= self._intercept_mc_hyst_hi:
            self._mc_engage_state[iid] = True
        elif p < self._intercept_mc_hyst_lo:
            self._mc_engage_state[iid] = False

    def _prune_mc_engage_keys(self) -> None:
        for k in list(self._mc_engage_state.keys()):
            if k not in self._ids:
                del self._mc_engage_state[k]
        for k in list(self._mc_high_p_start_m.keys()):
            if k not in self._ids:
                del self._mc_high_p_start_m[k]
        for k in list(self._last_phase1_high_p_warn.keys()):
            if k not in self._ids:
                del self._last_phase1_high_p_warn[k]

    def _mc_engage_ok(self, iid: str) -> bool:
        if not self._intercept_mc_engagement:
            return True
        if self._intercept_mc_hyst_on:
            return bool(self._mc_engage_state.get(iid, False))
        return self._mc_p_last.get(iid, 0.0) >= self._intercept_mc_min_p

    def _estimate_mc_hit_probability(
        self,
        tx: float,
        ty: float,
        tz: float,
        v_tx: float,
        v_ty: float,
        v_tz: float,
        ix: float,
        iy: float,
        iz: float,
        *,
        n_samples: int | None = None,
        iid: str | None = None,
    ) -> float:
        if n_samples is None:
            n_use = self._intercept_mc_light_samples if self._intercept_mc_use_light else self._intercept_mc_runtime_n
        else:
            n_use = min(30, int(n_samples))
        if self._intercept_mc_use_light:
            lvx, lvy, lvz = (0.0, 0.0, 0.0)
            if iid is not None:
                lvx, lvy, lvz = self._last_guidance_cmd.get(iid, (0.0, 0.0, 0.0))
            cm = _norm(lvx, lvy, lvz)
            ivxx = lvx if cm > 1e-3 else None
            ivyy = lvy if cm > 1e-3 else None
            ivzz = lvz if cm > 1e-3 else None
            prob = estimate_hit_probability_light(
                tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz,
                v_i_max=self._interceptor_max_speed,
                t_min=self._t_hit_min,
                t_max=self._t_hit_max,
                hit_thresh_m=self._hit_thresh,
                n_samples=n_use,
                pos_sigma_m=self._intercept_mc_light_pos_sig,
                vel_sigma_m_s=self._intercept_mc_light_vel_sig,
                delay_mean_s=self._mc_delay_mean_s(),
                delay_jitter_s=self._intercept_mc_light_dly_jit,
                rng=self._heatmap_prob_rng,
                ivx=ivxx, ivy=ivyy, ivz=ivzz,
            )
            if self._phase1_dbg:
                print(
                    f'[P_DEBUG] P(hit|noise model)={prob:.3f} samples={n_use} '
                    f'pos_sigma={self._intercept_mc_light_pos_sig:.3g} vel_sigma={self._intercept_mc_light_vel_sig:.3g} '
                    f'delay_jitter={self._intercept_mc_light_dly_jit:.3g} iid={iid!r}',
                    flush=True,
                )
            return prob
        use_roll = self._heatmap_prob_rollout
        return estimate_hit_probability(
            tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz,
            v_i_max=self._interceptor_max_speed,
            t_min=self._t_hit_min,
            t_max=self._t_hit_max,
            hit_thresh_m=self._hit_thresh,
            n_samples=max(0, min(80, n_use)),
            pos_sigma_m=self._heatmap_prob_pos_sig,
            vel_sigma_m_s=self._heatmap_prob_vel_sig,
            interceptor_pos_sigma_m=self._heatmap_prob_is_sig,
            delay_mean_s=self._mc_delay_mean_s(),
            delay_jitter_s=self._heatmap_prob_dly_jit,
            rng=self._heatmap_prob_rng,
            use_kinematic_rollout=use_roll,
            rollout_dt=self._heatmap_prob_rollout_dt,
            rollout_max_turn_rate_rad_s=self._max_turn_rate if use_roll else 0.0,
            rollout_max_accel_m_s2=self._max_accel if use_roll else 0.0,
        )

    def _maybe_warn_mc_high_p_no_hit(self, iid: str, p: float) -> None:
        if not self._phase1_warn or not self._intercept_mc_engagement or self._multi_enabled:
            return
        if self._hit:
            self._mc_high_p_start_m.pop(iid, None)
            return
        if (not math.isfinite(p)) or p < 0.7:
            self._mc_high_p_start_m.pop(iid, None)
            return
        nowm = time.monotonic()
        self._mc_high_p_start_m.setdefault(iid, nowm)
        t0 = self._mc_high_p_start_m[iid]
        if nowm - t0 < self._mc_high_p_no_hit_s:
            return
        lw = self._last_phase1_high_p_warn.get(iid, 0.0)
        if nowm - lw < 5.0:
            return
        print(
            f'[P_WARN] iid={iid} P(hit|noise model)={p:.2f} ≥0.7 for ≥{self._mc_high_p_no_hit_s:.0f}s without HIT '
            f'(model optimistic vs dynamics/gates)',
            flush=True,
        )
        self._last_phase1_high_p_warn[iid] = nowm

    def _maybe_phase1_runtime_warnings(
        self,
        selected: str,
        feas: object,
        strike_ok_eff: bool,
        *,
        layer_s: str,
    ) -> None:
        if not self._phase1_warn:
            return
        nowm = time.monotonic()
        vm = float(self._interceptor_max_speed)
        try:
            t_go = getattr(feas, 't_go', None)
            v_req = getattr(feas, 'v_req', None)
            feasible = bool(getattr(feas, 'feasible', False))
        except Exception:
            return
        if (
            t_go is not None
            and math.isfinite(t_go)
            and v_req is not None
            and math.isfinite(v_req)
            and v_req > vm + 0.02
            and nowm - self._last_phase1_vreq_warn_m > 3.0
        ):
            print(
                f'[P_CONSIST] feas: t_go={t_go:.3f}s but v_req={v_req:.2f} m/s > cap={vm:.2f} m/s '
                f'(selected={selected!r} {layer_s})',
                flush=True,
            )
            self._last_phase1_vreq_warn_m = nowm
        eng = self._mc_engage_ok(selected) if self._intercept_mc_engagement else False
        if (
            self._intercept_mc_engagement
            and self._intercept_mc_hyst_on
            and (not feasible)
            and eng
            and nowm - self._last_phase1_engage_warn_m > 2.0
        ):
            print(
                f'[P_CONSIST] feasible=False but MC engage latched True (hysteresis) sel={selected!r} '
                f'P_last={self._mc_p_last.get(selected, float("nan")):.3f} — review gates/model alignment',
                flush=True,
            )
            self._last_phase1_engage_warn_m = nowm

    def _refresh_mc_probs_single(
        self, tx: float, ty: float, tz: float, v_tx: float, v_ty: float, v_tz: float,
    ) -> None:
        if not (self._intercept_mc_trail_by_prob or self._intercept_mc_engagement):
            return
        self._prune_mc_engage_keys()
        for iid in self._ids:
            p = self._inter_pos.get(iid)
            if p is None:
                continue
            self._mc_p_last[iid] = self._estimate_mc_hit_probability(
                tx, ty, tz, v_tx, v_ty, v_tz, float(p.x), float(p.y), float(p.z),
                iid=iid,
            )
            self._update_mc_engage_hysteresis(iid, self._mc_p_last[iid])
            self._maybe_warn_mc_high_p_no_hit(iid, self._mc_p_last[iid])

    def _refresh_mc_probs_multi(
        self,
        assign: dict[str, str],
        v_by: dict[str, tuple[float, float, float]],
    ) -> None:
        if not (self._intercept_mc_trail_by_prob or self._intercept_mc_engagement):
            return
        self._prune_mc_engage_keys()
        for iid in self._ids:
            self._mc_p_last.pop(iid, None)
        for tlabel, iid in assign.items():
            pt = self._targets.get(tlabel)
            p_int = self._inter_pos.get(iid)
            if pt is None or p_int is None:
                continue
            vx, vy, vz = v_by.get(tlabel, (0.0, 0.0, 0.0))
            self._mc_p_last[iid] = self._estimate_mc_hit_probability(
                float(pt.x), float(pt.y), float(pt.z),
                vx, vy, vz,
                float(p_int.x), float(p_int.y), float(p_int.z),
                iid=iid,
            )
            self._update_mc_engage_hysteresis(iid, self._mc_p_last[iid])

    def _publish_intercept_markers(
        self,
        iid: str,
        iid_index: int,
        ix: float,
        iy: float,
        iz: float,
        phx: float,
        phy: float,
        phz: float,
        sol_valid: bool,
        *,
        t_go: float | None = None,
        solver_closing_m_s: float | None = None,
        v_cmd_m_s: float | None = None,
        mc_hit_prob: float | None = None,
    ) -> None:
        """Predicted intercept: sphere + dashed line + optional TEXT (t_go, s_i model, |v_cmd|, P_hit)."""
        if self._pub_intercept_viz is None:
            return
        stamp = self.get_clock().now().to_msg()
        frame = self._dome_frame
        # Spacing avoids id collisions across interceptors (sphere/line/text triples).
        base_id = iid_index * 10
        predict_ns = 'cuas_predict'

        def _del_ids(*mids: int) -> None:
            for mid in mids:
                clr = Marker()
                clr.header.frame_id = frame
                clr.header.stamp = stamp
                clr.ns = predict_ns
                clr.id = mid
                clr.action = Marker.DELETE
                self._pub_intercept_viz.publish(clr)

        if self._math_viz and self._math_hide_legacy:
            _del_ids(base_id, base_id + 1, base_id + 2)
            return

        if not self._cuas_predict_viz_enabled or not sol_valid:
            _del_ids(base_id, base_id + 1, base_id + 2)
            return

        # ── Sphere at predicted intercept point ──────────────────────────────
        sphere = Marker()
        sphere.header.frame_id = frame
        sphere.header.stamp = stamp
        sphere.ns = predict_ns
        sphere.id = base_id
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = phx
        sphere.pose.position.y = phy
        sphere.pose.position.z = phz
        sphere.pose.orientation.w = 1.0
        d = self._cuas_pred_marker_d
        sphere.scale.x = sphere.scale.y = sphere.scale.z = d
        sphere.color = self._cuas_pred_sphere_col
        if mc_hit_prob is not None and math.isfinite(mc_hit_prob):
            sr, sg, sb, sa = _heatmap_rgba_for_score(float(mc_hit_prob), float(self._cuas_pred_sphere_col.a))
            sphere.color = ColorRGBA(r=sr, g=sg, b=sb, a=sa)
        sphere.lifetime.sec = 0
        self._pub_intercept_viz.publish(sphere)

        line = Marker()
        line.header.frame_id = frame
        line.header.stamp = stamp
        line.ns = predict_ns
        line.id = base_id + 1
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = self._cuas_pred_line_w
        line.color = self._cuas_pred_line_col
        if mc_hit_prob is not None and math.isfinite(mc_hit_prob):
            lr, lg, lb, la = _heatmap_rgba_for_score(float(mc_hit_prob), float(self._cuas_pred_line_col.a))
            line.color = ColorRGBA(r=lr, g=lg, b=lb, a=la)
        line.lifetime.sec = 0
        if self._cuas_pred_dashed:
            line.type = Marker.LINE_LIST
            line.points = _dashed_line_list_points(
                ix, iy, iz, phx, phy, phz, self._cuas_pred_dash, self._cuas_pred_gap,
            )
            if not line.points:
                line.action = Marker.DELETE
        else:
            line.type = Marker.LINE_STRIP
            line.points = [Point(x=ix, y=iy, z=iz), Point(x=phx, y=phy, z=phz)]
        self._pub_intercept_viz.publish(line)

        show_txt = (
            self._cuas_pred_text
            and t_go is not None
            and math.isfinite(t_go)
            and math.isfinite(phx)
            and math.isfinite(phy)
            and math.isfinite(phz)
        )
        if show_txt:
            tc = self._cuas_pred_text_col
            si_s = (
                f'{solver_closing_m_s:.2f}'
                if solver_closing_m_s is not None and math.isfinite(solver_closing_m_s)
                else 'n/a'
            )
            vc_s = f'{v_cmd_m_s:.2f}' if v_cmd_m_s is not None and math.isfinite(v_cmd_m_s) else 'n/a'
            mc_s = (
                f'MC P≈{mc_hit_prob * 100.0:.0f}%\n'
                if mc_hit_prob is not None and math.isfinite(mc_hit_prob)
                else ''
            )
            tm = Marker()
            tm.header.frame_id = frame
            tm.header.stamp = stamp
            tm.ns = predict_ns
            tm.id = base_id + 2
            tm.type = Marker.TEXT_VIEW_FACING
            tm.action = Marker.ADD
            tm.pose.position.x = phx
            tm.pose.position.y = phy
            tm.pose.position.z = phz + self._cuas_pred_text_dz
            tm.pose.orientation.w = 1.0
            tm.scale.z = self._cuas_pred_text_scale
            tm.color = ColorRGBA(r=tc[0], g=tc[1], b=tc[2], a=tc[3])
            tm.lifetime.sec = 0
            tm.text = (
                f'{iid}\n'
                f'{mc_s}'
                f't_go={t_go:.2f} s  (model |r0+v_T t| = s_i t)\n'
                f's_i={si_s} m/s\n'
                f'|v_cmd|={vc_s} m/s\n'
                f'P_hit=({phx:.0f},{phy:.0f},{phz:.0f})'
            )
            self._pub_intercept_viz.publish(tm)
        else:
            _del_ids(base_id + 2)

    def _publish_intercept_math_markers(
        self,
        iid: str,
        iid_index: int,
        ix: float,
        iy: float,
        iz: float,
        tx: float,
        ty: float,
        tz: float,
        feas: FeasibilityVizState,
        *,
        v_cmd: float | None,
    ) -> None:
        """
        Feasibility-colored layer: green = feasible at cap, red = infeasible, grey = no quadratic solution.

        Uses ``intercept_math_frame_id`` (default ``map``).
        """
        if self._pub_intercept_viz is None or not self._math_viz:
            return
        stamp = self.get_clock().now().to_msg()
        frame = self._math_frame
        ns = 'intercept_math'
        base = 1000 + iid_index * 10

        col_feas_s = ColorRGBA(r=0.15, g=0.88, b=0.22, a=0.95)
        col_feas_l = col_feas_s
        col_feas_t = ColorRGBA(r=0.96, g=1.0, b=0.96, a=0.98)

        col_bad_s = ColorRGBA(r=0.92, g=0.18, b=0.15, a=0.95)
        col_bad_l = col_bad_s
        col_bad_t = ColorRGBA(r=1.0, g=0.28, b=0.22, a=0.95)

        col_grey_s = ColorRGBA(r=0.5, g=0.52, b=0.55, a=0.9)
        col_grey_t = ColorRGBA(r=0.78, g=0.8, b=0.82, a=0.95)

        def _del_ms(*mids: int) -> None:
            for mid in mids:
                dm = Marker()
                dm.header.frame_id = frame
                dm.header.stamp = stamp
                dm.ns = ns
                dm.id = mid
                dm.action = Marker.DELETE
                self._pub_intercept_viz.publish(dm)

        def _vcmd_s() -> str:
            if v_cmd is not None and math.isfinite(v_cmd):
                return f'{v_cmd:.1f}'
            return 'n/a'

        st = feas.status
        if st == 'no_solution':
            dm = Marker()
            dm.header.frame_id = frame
            dm.header.stamp = stamp
            dm.ns = ns
            dm.id = base + 1
            dm.action = Marker.DELETE
            self._pub_intercept_viz.publish(dm)
            sph0 = Marker()
            sph0.header.frame_id = frame
            sph0.header.stamp = stamp
            sph0.ns = ns
            sph0.id = base
            sph0.type = Marker.SPHERE
            sph0.action = Marker.ADD
            sph0.pose.position.x = tx
            sph0.pose.position.y = ty
            sph0.pose.position.z = tz
            sph0.pose.orientation.w = 1.0
            ds = self._math_phit_s * 0.65
            sph0.scale.x = sph0.scale.y = sph0.scale.z = ds
            sph0.color = col_grey_s
            sph0.lifetime.sec = 0
            self._pub_intercept_viz.publish(sph0)
            if self._math_txt_on:
                tm = Marker()
                tm.header.frame_id = frame
                tm.header.stamp = stamp
                tm.ns = ns
                tm.id = base + 2
                tm.type = Marker.TEXT_VIEW_FACING
                tm.action = Marker.ADD
                tm.pose.position.x = tx
                tm.pose.position.y = ty
                tm.pose.position.z = tz + self._math_txt_dz
                tm.pose.orientation.w = 1.0
                tm.scale.z = self._math_txt_scale
                tm.color = col_grey_t
                tm.lifetime.sec = 0
                tm.text = (
                    'STATUS: NO SOLUTION\n'
                    f't_go: n/a\n'
                    'v_req: n/a\n'
                    f'v_cmd: {_vcmd_s()} m/s'
                )
                self._pub_intercept_viz.publish(tm)
            return

        if feas.phx is None or feas.phy is None or feas.phz is None:
            grey = FeasibilityVizState('no_solution', 'NO SOLUTION', False, None, None, None, None, None)
            self._publish_intercept_math_markers(iid, iid_index, ix, iy, iz, tx, ty, tz, grey, v_cmd=v_cmd)
            return
        phx, phy, phz = float(feas.phx), float(feas.phy), float(feas.phz)

        if feas.feasible:
            cs, cl, ct = col_feas_s, col_feas_l, col_feas_t
        else:
            cs, cl, ct = col_bad_s, col_bad_l, col_bad_t

        sph = Marker()
        sph.header.frame_id = frame
        sph.header.stamp = stamp
        sph.ns = ns
        sph.id = base
        sph.type = Marker.SPHERE
        sph.action = Marker.ADD
        sph.pose.position.x = phx
        sph.pose.position.y = phy
        sph.pose.position.z = phz
        sph.pose.orientation.w = 1.0
        sdim = self._math_phit_s
        sph.scale.x = sph.scale.y = sph.scale.z = sdim
        sph.color = cs
        sph.lifetime.sec = 0
        self._pub_intercept_viz.publish(sph)

        br = Marker()
        br.header.frame_id = frame
        br.header.stamp = stamp
        br.ns = ns
        br.id = base + 1
        br.type = Marker.LINE_STRIP
        br.action = Marker.ADD
        br.pose.orientation.w = 1.0
        br.scale.x = self._math_bear_w
        br.color = cl
        br.points = [Point(x=ix, y=iy, z=iz), Point(x=phx, y=phy, z=phz)]
        br.lifetime.sec = 0
        self._pub_intercept_viz.publish(br)

        if self._math_txt_on:
            tgs = (
                f'{feas.t_go:.1f}'
                if feas.t_go is not None and math.isfinite(feas.t_go)
                else 'n/a'
            )
            if feas.v_req is not None and math.isfinite(feas.v_req):
                vreq_s = f'{feas.v_req:.1f}'
            else:
                vreq_s = 'n/a'
            sw = 'FEASIBLE' if feas.feasible else 'INFEASIBLE'
            extra = f'\nreason: {feas.reason}' if feas.reason else ''
            tm = Marker()
            tm.header.frame_id = frame
            tm.header.stamp = stamp
            tm.ns = ns
            tm.id = base + 2
            tm.type = Marker.TEXT_VIEW_FACING
            tm.action = Marker.ADD
            tm.pose.position.x = phx
            tm.pose.position.y = phy
            tm.pose.position.z = phz + self._math_txt_dz
            tm.pose.orientation.w = 1.0
            tm.scale.z = self._math_txt_scale
            tm.color = ct
            tm.lifetime.sec = 0
            tm.text = (
                f'STATUS: {sw}{extra}\n'
                f't_go: {tgs} s\n'
                f'v_req: {vreq_s} m/s\n'
                f'v_cmd: {_vcmd_s()} m/s'
            )
            self._pub_intercept_viz.publish(tm)
        else:
            _del_ms(base + 2)

    def _publish_math_trail_markers(self) -> None:
        """Red target trail + blue interceptor trails (``intercept_math`` namespace)."""
        if self._pub_intercept_viz is None or not self._math_viz:
            return
        pub = self._pub_intercept_viz
        stamp = self.get_clock().now().to_msg()
        frame = self._math_frame
        ns = 'intercept_math'

        def _strip(dq: deque[tuple[float, float, float]], mid: int, color: ColorRGBA) -> None:
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = stamp
            m.ns = ns
            m.id = mid
            if len(dq) < 2:
                m.action = Marker.DELETE
                pub.publish(m)
                return
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = self._math_trail_w
            m.color = color
            m.points = [Point(x=a, y=b, z=c) for a, b, c in dq]
            m.lifetime.sec = 0
            pub.publish(m)

        _strip(self._math_trail_target, 10, self._math_trail_tgt_col)
        for idx, iid in enumerate(self._ids):
            _strip(self._math_trail_inters[iid], 20 + idx, self._math_trail_int_col)

    def _maybe_feasibility_debug_log(
        self,
        *,
        feas: FeasibilityVizState,
        v_cmd: float | None,
        distance: float,
        layer: str,
        iid: str,
    ) -> None:
        if not (self._feas_debug and self._math_dbg):
            return
        now = self.get_clock().now()
        if self._last_feas_debug_log is not None:
            if (now - self._last_feas_debug_log).nanoseconds * 1e-9 < self._log_period:
                return
        self._last_feas_debug_log = now
        tgs = (
            f'{feas.t_go:.4f}'
            if feas.t_go is not None and math.isfinite(feas.t_go)
            else 'n/a'
        )
        vrs = f'{feas.v_req:.4f}' if feas.v_req is not None and math.isfinite(feas.v_req) else 'n/a'
        vcs = f'{v_cmd:.4f}' if v_cmd is not None and math.isfinite(v_cmd) else 'n/a'
        lyr = layer if layer else 'n/a'
        print(
            f'[INTERCEPT_DEBUG] id={iid} feasible={feas.feasible} status={feas.status!r} reason={feas.reason!r}\n'
            f'  t_go={tgs}  v_required={vrs} m/s  v_cmd={vcs} m/s  distance={distance:.4f} m  layer={lyr}',
            flush=True,
        )

    def _maybe_feasibility_warnings(
        self,
        *,
        feas: FeasibilityVizState,
        guidance_sol_ok: bool,
        guidance_t_go: float | None,
        v_cmd: float | None,
        strike_engaged: bool,
        iid: str,
    ) -> None:
        now = self.get_clock().now()
        if self._last_feas_warn_log is not None:
            if (now - self._last_feas_warn_log).nanoseconds * 1e-9 < self._log_period:
                return
        msgs: list[str] = []
        cap = float(self._interceptor_max_speed)
        if v_cmd is not None and math.isfinite(v_cmd) and v_cmd > cap + 0.05:
            msgs.append(
                f'[FEAS_WARN] {iid}: v_cmd={v_cmd:.3f} m/s > interceptor_cap={cap:.3f} m/s',
            )
        if guidance_sol_ok and guidance_t_go is not None and (not feas.feasible):
            msgs.append(
                f'[FEAS_WARN] {iid}: guidance has t_go={guidance_t_go:.3f}s but cap-feasibility=False '
                f'({feas.status!r} {feas.reason!r})',
            )
        if strike_engaged and (v_cmd is not None and v_cmd > 0.08) and (not feas.feasible):
            msgs.append(
                f'[FEAS_WARN] {iid}: non-zero cmd while infeasible ({feas.reason!r}) strike_engaged=True',
            )
        if msgs:
            self._last_feas_warn_log = now
            print('\n'.join(msgs), flush=True)

    def _record_math_trails(self) -> None:
        """Short history (default 200 pts) for red target / blue interceptor paths — physics viz layer."""
        if self._multi_enabled:
            for lab in self._multi_labels:
                if self._hits_multi.get(lab, False):
                    continue
                pt = self._targets.get(lab)
                if pt is not None:
                    self._math_trail_target.append((float(pt.x), float(pt.y), float(pt.z)))
                    break
        elif not self._hit and self._target is not None:
            self._math_trail_target.append(
                (float(self._target.x), float(self._target.y), float(self._target.z)),
            )
        for iid in self._ids:
            p = self._inter_pos.get(iid)
            if p is not None:
                self._math_trail_inters[iid].append((float(p.x), float(p.y), float(p.z)))

    def _cuas_record_trails(self) -> None:
        if self._pub_intercept_viz is None:
            return
        if self._cuas_trails_enabled:
            if self._multi_enabled:
                for lab, pt in self._targets.items():
                    if self._hits_multi.get(lab, False):
                        continue
                    dq = self._trail_multi_hostile.get(lab)
                    if pt is not None and dq is not None:
                        _trail_push(dq, float(pt.x), float(pt.y), float(pt.z), self._cuas_trail_min_step)
            else:
                if not self._hit and self._target is not None:
                    _trail_push(
                        self._trail_target,
                        float(self._target.x), float(self._target.y), float(self._target.z),
                        self._cuas_trail_min_step,
                    )
            for iid in self._ids:
                p = self._inter_pos.get(iid)
                if p is not None:
                    _trail_push(
                        self._trail_inters[iid], float(p.x), float(p.y), float(p.z),
                        self._cuas_trail_min_step,
                    )
        if self._math_viz:
            self._record_math_trails()

    def _cuas_publish_trail_markers(self) -> None:
        if self._pub_intercept_viz is None:
            return
        if self._cuas_trails_enabled:
            pub = self._pub_intercept_viz
            stamp = self.get_clock().now().to_msg()
            frame = self._dome_frame
            ns = 'cuas_trail'

            def _publish_strip(dq: deque[tuple[float, float, float]], mid: int, color: ColorRGBA) -> None:
                m = Marker()
                m.header.frame_id = frame
                m.header.stamp = stamp
                m.ns = ns
                m.id = mid
                if len(dq) < 2:
                    m.action = Marker.DELETE
                    pub.publish(m)
                    return
                m.type = Marker.LINE_STRIP
                m.action = Marker.ADD
                m.pose.orientation.w = 1.0
                m.scale.x = self._cuas_trail_line_w
                m.color = color
                m.points = [Point(x=a, y=b, z=c) for a, b, c in dq]
                m.lifetime.sec = 0
                pub.publish(m)

            if self._multi_enabled:
                for li, lab in enumerate(self._multi_labels):
                    _publish_strip(self._trail_multi_hostile[lab], 300 + li, self._cuas_trail_hostile_rgba)
            else:
                _publish_strip(self._trail_target, 1, self._cuas_trail_hostile_rgba)
            for idx, iid in enumerate(self._ids):
                if self._intercept_mc_trail_by_prob:
                    pr = self._mc_p_last.get(iid)
                    if pr is not None and math.isfinite(pr):
                        tr, tg, tb, ta = _heatmap_rgba_for_score(float(pr), float(self._cuas_trail_inter_rgba.a))
                        icol = ColorRGBA(r=tr, g=tg, b=tb, a=ta)
                    else:
                        icol = self._cuas_trail_inter_rgba
                else:
                    icol = self._cuas_trail_inter_rgba
                _publish_strip(self._trail_inters[iid], 100 + idx, icol)
        if self._math_viz:
            self._publish_math_trail_markers()

    def _cuas_clear_viz_markers(self) -> None:
        if self._pub_intercept_viz is None:
            return
        stamp = self.get_clock().now().to_msg()
        for ns, frame in (
            ('cuas_trail', self._dome_frame),
            ('cuas_predict', self._dome_frame),
            ('intercept_math', self._math_frame),
        ):
            clr = Marker()
            clr.header.frame_id = frame
            clr.header.stamp = stamp
            clr.ns = ns
            clr.action = Marker.DELETEALL
            self._pub_intercept_viz.publish(clr)

    def _command_speed(self, dist: float, t_go: float | None = None) -> float:
        """
        Time-to-go aware speed controller.

        Formula (predict mode — t_go known):
            v = k1 * dist + k2 * dist / max(t_go, tgo_min)

        Formula (pursuit mode — t_go = None):
            v = k1 * dist   (distance-proportional fallback)

        Both branches are clamped to [speed_vmin, max_speed_m_s].

        Design intent:
          • k1*dist   — baseline speed; naturally ramps up far away and
                        slows down as the interceptor closes in.
          • k2*dist/t_go — urgency term; equals the average closing rate
                        required to reach the intercept point on time.
                        Drives higher speed when t_go is short.
          • speed_vmin  — hard floor so the interceptor never crawls.
          • speed_tgo_min — guards against near-zero t_go producing an
                        infinite desired speed.
        """
        if not math.isfinite(dist) or dist <= 0.0:
            return float(self._speed_vmin)
        if t_go is not None and math.isfinite(t_go) and t_go > 0.0:
            t_eff = max(float(t_go), self._speed_tgo_min)
            v = self._speed_k1 * dist + self._speed_k2 * dist / t_eff
        else:
            v = self._speed_k1 * dist
        return float(min(self._vmax, max(self._speed_vmin, v)))

    def _on_gz_sim_reset(self) -> None:
        """กด Reset ใน Gazebo -> เวลาจำลองย้อน — เคลียร์ HIT/pause/ล็อกให้รันรอบใหม่ได้."""
        self.get_logger().info('Sim reset (/clock rewind): clearing interception state (hit, pause flag, locks).')
        self._hit = False
        self._gz_pause_sent = False
        self._dome_hyst_initialized = False
        self._clear_assignments()
        self._locked_selected_id = None
        self._current_selected_id = None
        self._committed_since = None
        self._lost_since = None
        self._reacquire_since = None
        self._switch_count = 0
        self._best_id_last = None
        self._prev_velocity = {iid: (0.0, 0.0, 0.0) for iid in self._ids}
        self._last_control_time = None
        self._control_dt = self._control_dt_default
        for iid in self._ids:
            self._intercept_point_filtered[iid] = None
            self._guidance_mode[iid] = 'pursuit'
            self._valid_streak[iid] = 0
            self._invalid_streak[iid] = 0
        self._last_layer = ''
        self._last_feas_log = None
        self._last_class_warn = None
        self._v_tgt_smooth = (0.0, 0.0, 0.0)
        self._prev_target = None
        self._prev_target_time = None
        self._hit_snap_target_prev = None
        self._hit_snap_target_prev_multi.clear()
        self._target_detect_time = None
        for iid in self._ids:
            self._prev_inter_pos[iid] = None
            self._prev_inter_time[iid] = None
        if self._multi_enabled:
            for lab in self._multi_labels:
                self._hits_multi[lab] = False
                self._targets[lab] = None
                self._multi_target_detect_time[lab] = None
                self._multi_prev_target[lab] = None
                self._multi_prev_target_time[lab] = None
                self._multi_v_smooth[lab] = (0.0, 0.0, 0.0)
            self._hit_snap_target_prev_multi.clear()
            self._last_assignment_print = None
            self._multi_prev_assign.clear()
            for lab in self._multi_labels:
                self._multi_dome_outside_latched[lab] = False
                self._multi_dome_hyst_init[lab] = False
        self._min_miss_distance = float("inf")
        self._last_hit_range = {i: None for i in self._ids}
        self._logged_guard_block = False
        self._trail_target.clear()
        for d in self._trail_inters.values():
            d.clear()
        for d in self._trail_multi_hostile.values():
            d.clear()
        self._math_trail_target.clear()
        for d in self._math_trail_inters.values():
            d.clear()
        self._feasible_at_engagement_start_by_pair.clear()
        self._feas_eng_latch_assign.clear()
        self._last_feas_debug_log = None
        self._last_feas_warn_log = None
        self._cuas_clear_viz_markers()
        self._cancel_stop_signal_repeat_timer()
        self._heatmap_prob_cache.clear()
        self._clear_intercept_heatmap_prob_markers()
        self._mc_p_last.clear()
        self._mc_engage_state.clear()
        self._heatmap_prob_skip_next = False
        for _iid in self._ids:
            self._last_guidance_cmd[_iid] = (0.0, 0.0, 0.0)
        self._mc_high_p_start_m.clear()
        self._last_phase1_high_p_warn.clear()
        self._last_phase1_vreq_warn_m = 0.0
        self._last_phase1_engage_warn_m = 0.0
        self._dbg_no_hit_watch_start_m.clear()
        self._dbg_no_hit_watch_tgo.clear()
        self._dbg_no_hit_warn_last_m.clear()
        self._last_hit_debug_print_m = 0.0
        self._last_hit_gate_block_print_m = 0.0
        self._multi_hit_safety_logged.clear()
        if self._pub_intercept_viz is not None:
            leg = Marker()
            leg.header.frame_id = self._dome_frame
            leg.header.stamp = self.get_clock().now().to_msg()
            leg.ns = 'intercept_viz'
            leg.action = Marker.DELETEALL
            self._pub_intercept_viz.publish(leg)

    def _cancel_stop_signal_repeat_timer(self) -> None:
        if self._stop_repeat_timer is not None:
            self.destroy_timer(self._stop_repeat_timer)
            self._stop_repeat_timer = None
        self._stop_repeat_deadline = None
        self._stop_repeat_pending_label = None

    def _schedule_stop_signal_repeats(self, target_label: str | None) -> None:
        """Burst ``/target/stop`` VOLATILE publishes until ``target_controller`` arms (and beyond)."""
        self._cancel_stop_signal_repeat_timer()
        if self._stop_repeat_duration_s <= 1e-6:
            return
        self._stop_repeat_pending_label = target_label
        deadline = self.get_clock().now() + Duration(seconds=self._stop_repeat_duration_s)
        self._stop_repeat_deadline = deadline
        self._stop_repeat_timer = self.create_timer(
            self._stop_repeat_period_s,
            self._on_stop_signal_repeat_tick,
        )

    def _on_stop_signal_repeat_tick(self) -> None:
        if self._stop_repeat_deadline is None:
            self._cancel_stop_signal_repeat_timer()
            return
        now = self.get_clock().now()
        if now.nanoseconds >= self._stop_repeat_deadline.nanoseconds:
            self._cancel_stop_signal_repeat_timer()
            return
        self._publish_stop_signal(self._stop_repeat_pending_label, log_hit=False)

    def _publish_stop_signal(self, target_label: str | None, *, log_hit: bool = True) -> None:
        topic_s = repr(self._stop_topic)
        if log_hit:
            self.get_logger().info(
                f'HIT: publishing {topic_s} True ×3 '
                f'(multi_label={target_label!r}; repeat_until_armed=yes).',
            )
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

    def _classification_allows_engagement(self) -> bool:
        """Placeholder hostile/drone gate; fused classifier can replace ``classification_placeholder_confidence``."""
        if not self._class_gating:
            return True
        ok = self._class_placeholder >= self._class_threshold
        if not ok:
            now = self.get_clock().now()
            if self._last_class_warn is None or (now - self._last_class_warn).nanoseconds * 1e-9 >= 2.0:
                print(
                    f'[CLASSIFICATION] blocked engagement: placeholder_conf={self._class_placeholder:.3f} '
                    f'< threshold={self._class_threshold:.3f}',
                    flush=True,
                )
                self._last_class_warn = now
        return ok

    def _intercept_feasibility_triple(
        self,
        tx: float,
        ty: float,
        tz: float,
        v_tx: float,
        v_ty: float,
        v_tz: float,
        ix: float,
        iy: float,
        iz: float,
    ) -> tuple[bool, float | None, float | None]:
        """
        (feasible, t_intercept_at_max_speed, required_min_speed).

        required_min_speed comes from bisection (slowest closing speed that still fits the time window).
        t_intercept_at_max_speed is used for ranking / comparison (aligns with TTI at capability cap).
        """
        s_min, _t_at_min_s = minimum_intercept_closing_speed(
            tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz,
            self._t_hit_min, self._t_hit_max, self._interceptor_max_speed,
        )
        if s_min is None or _t_at_min_s is None:
            return False, None, None
        if s_min > self._interceptor_max_speed + 1e-3:
            return False, None, None
        t_cap = _solve_intercept_time(
            tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, self._interceptor_max_speed,
        )
        if not _t_in_feasible_window(t_cap, self._t_hit_min, self._t_hit_max):
            return False, None, None
        return True, float(t_cap), float(s_min)

    def _pick_best_feasible_interceptor_single(
        self,
        tx: float,
        ty: float,
        tz: float,
        v_tx: float,
        v_ty: float,
        v_tz: float,
    ) -> tuple[str | None, bool, float | None, float | None]:
        """Best interceptor by smallest feasible intercept time."""
        best_i: str | None = None
        best_t: float | None = None
        best_s: float | None = None
        any_feas = False
        for iid in self._ids:
            p = self._inter_pos.get(iid)
            if p is None:
                continue
            ix, iy, iz = float(p.x), float(p.y), float(p.z)
            ok, t_i, s_i = self._intercept_feasibility_triple(tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz)
            if ok and t_i is not None:
                any_feas = True
                if best_t is None or t_i < best_t - 1e-9:
                    best_t = t_i
                    best_i = iid
                    best_s = s_i
                elif best_t is not None and abs(t_i - best_t) <= 1e-9 and best_i is not None:
                    if self._ids.index(iid) < self._ids.index(best_i):
                        best_i = iid
                        best_s = s_i
        return best_i, any_feas, best_t, best_s

    def _feasibility_tti_for_assignment(
        self,
        tx: float,
        ty: float,
        tz: float,
        vx: float,
        vy: float,
        vz: float,
        ix: float,
        iy: float,
        iz: float,
    ) -> float | None:
        """TTI at interceptor cap for bipartite cost matrix (None if infeasible)."""
        s_min, _t0 = minimum_intercept_closing_speed(
            tx, ty, tz, vx, vy, vz, ix, iy, iz,
            self._t_hit_min, self._t_hit_max, self._interceptor_max_speed,
        )
        if s_min is None or s_min > self._interceptor_max_speed + 1e-3:
            return None
        t_cap = _solve_intercept_time(
            tx, ty, tz, vx, vy, vz, ix, iy, iz, self._interceptor_max_speed,
        )
        if not _t_in_feasible_window(t_cap, self._t_hit_min, self._t_hit_max):
            return None
        return float(t_cap)

    def _maybe_log_feasibility_single(
        self,
        *,
        feasible: bool,
        t_int: float | None,
        s_req: float | None,
        selected: str | None,
        d_th: float,
    ) -> None:
        now = self.get_clock().now()
        if self._last_feas_log is not None:
            if (now - self._last_feas_log).nanoseconds * 1e-9 < self._feas_log_period:
                return
        self._last_feas_log = now
        sreq = f'{s_req:.3f}' if s_req is not None and math.isfinite(s_req) else 'n/a'
        tint = f'{t_int:.3f}' if t_int is not None and math.isfinite(t_int) else 'n/a'
        print(
            f'[FEASIBILITY] feasible={feasible}  t_intercept@cap={tint}s  required_min_speed={sreq} m/s  '
            f'interceptor_cap={self._interceptor_max_speed:.3f} m/s  selected={selected!r}  d_threat={d_th:.3f} m',
            flush=True,
        )

    def _on_target(self, msg: Point) -> None:
        self._target = msg

    def _on_target_state(self, msg: Odometry) -> None:
        """Track-state callback: cache position **and** filter velocity from upstream Kalman.

        Single-target consumer for ``intercept_measurement_source: tracks_state``.  Without this
        path, guidance has to finite-difference noisy 10 Hz Points which destroys the predictive
        intercept solution under realistic measurement rates.
        """
        p = Point()
        p.x = float(msg.pose.pose.position.x)
        p.y = float(msg.pose.pose.position.y)
        p.z = float(msg.pose.pose.position.z)
        self._target = p
        self._target_filter_velocity = (
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
            float(msg.twist.twist.linear.z),
        )

    def _on_target_multi(self, label: str, msg: Point) -> None:
        self._targets[label] = msg

    def _dist_target_to_radar_m(self, tx: float, ty: float, tz: float) -> float:
        dx = tx - self._radar_px
        dy = ty - self._radar_py
        dz = tz - self._radar_pz
        return float(math.sqrt(dx * dx + dy * dy + dz * dz))

    def is_detected(self, tx: float, ty: float, tz: float) -> bool:
        """True if target is inside radar range (||p - p_radar|| <= radar_range)."""
        if not self._sense_gate_enabled:
            return True
        if not math.isfinite(tx) or not math.isfinite(ty) or not math.isfinite(tz):
            return False
        return self._dist_target_to_radar_m(tx, ty, tz) <= self._radar_range_m

    def is_tracking_ready(self, label: str | None, now: Time) -> bool:
        """
        True once sim time >= detection_time + tracking_delay.

        ``label`` is the multi-target label, or None for single-target mode.
        """
        if not self._sense_gate_enabled:
            return True
        det_t = self._multi_target_detect_time.get(label, self._target_detect_time) if label is not None else self._target_detect_time
        if det_t is None:
            return False
        dt_ns = (now - det_t).nanoseconds
        if dt_ns < 0:
            # Sim time stepped backward without a full node reset; re-arm delay from this epoch.
            if label is None:
                self._target_detect_time = now
            else:
                self._multi_target_detect_time[label] = now
            return False
        ready_s = dt_ns * 1e-9
        return bool(ready_s >= self._tracking_delay_s)

    def _maybe_arm_detection_time(self, label: str | None, tx: float, ty: float, tz: float) -> None:
        """Arm detection epoch on transition into radar range (or first in-range observation)."""
        if not self._sense_gate_enabled:
            return
        if not self.is_detected(tx, ty, tz):
            if label is None:
                self._target_detect_time = None
            else:
                self._multi_target_detect_time[label] = None
            return
        if label is None:
            if self._target_detect_time is None:
                self._target_detect_time = self.get_clock().now()
            return
        if self._multi_target_detect_time.get(label) is None:
            self._multi_target_detect_time[label] = self.get_clock().now()

    def _on_inter(self, iid: str, msg: Point) -> None:
        self._inter_pos[iid] = msg
        if iid not in self._inter_start_pos:
            self._inter_start_pos[iid] = (float(msg.x), float(msg.y), float(msg.z))

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

    def _strike_ok_for_dome_policy(self, d_th: float, layer: str) -> bool:
        """Whether geometry allows counting a kinetic HIT under layered dome policy."""
        if not self._dome_enabled:
            return True
        if self._engagement_layer:
            _layer_order = {'outside': 0, 'detect': 1, 'select': 2, 'engage': 3}
            cur_order = _layer_order.get(layer, -1)
            hit_order = _layer_order.get(self._engagement_layer, 2)
            if self._hit_outer_layer_only:
                return cur_order == hit_order
            return cur_order >= hit_order
        return self._in_strike_zone(d_th)

    def _dome_outside_effective(self, d_th: float) -> bool:
        """True if threat is treated as outside the outer dome for engagement (hysteresis optional)."""
        if not self._dome_enabled:
            return False
        if self._dome_outer_hyst <= 0.0:
            return d_th > self._r_outer
        if not self._dome_hyst_initialized:
            self._dome_outside_latched = d_th > self._r_outer
            self._dome_hyst_initialized = True
            return self._dome_outside_latched
        h = self._dome_outer_hyst
        if self._dome_outside_latched:
            if d_th < self._r_outer - h:
                self._dome_outside_latched = False
        else:
            if d_th > self._r_outer + h:
                self._dome_outside_latched = True
        return self._dome_outside_latched

    def _dome_outside_effective_multi(self, label: str, d_th: float) -> bool:
        """Same hysteresis as _dome_outside_effective but independent state per multi-target label."""
        if not self._dome_enabled:
            return False
        if self._dome_outer_hyst <= 0.0:
            return d_th > self._r_outer
        if not self._multi_dome_hyst_init.get(label, False):
            self._multi_dome_outside_latched[label] = d_th > self._r_outer
            self._multi_dome_hyst_init[label] = True
            return self._multi_dome_outside_latched[label]
        h = self._dome_outer_hyst
        latched = self._multi_dome_outside_latched.get(label, False)
        if latched:
            if d_th < self._r_outer - h:
                latched = False
        else:
            if d_th > self._r_outer + h:
                latched = True
        self._multi_dome_outside_latched[label] = latched
        return latched

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
        self._assigned_interceptor_id = None
        self._assignment_time = None
        self._last_assign_lock_log = None
        self._mc_engage_state.clear()
        # Reset intercept filter and mode-hysteresis state so a new assignment
        # starts from scratch with no stale prediction or committed mode.
        for iid in self._ids:
            self._intercept_point_filtered[iid] = None
            self._guidance_mode[iid] = 'pursuit'
            self._valid_streak[iid] = 0
            self._invalid_streak[iid] = 0
        self._last_hit_range = {i: None for i in self._ids}
        self._feasible_at_engagement_start_by_pair.clear()
        self._feas_eng_latch_assign.clear()

    def _hit_range_plausible_for_tick(self, iid: str, dist_now: float) -> bool:
        """True if dist_now is consistent with closing speed vs previous tick (reject pose spikes)."""
        if not math.isfinite(dist_now) or dist_now < 0.0:
            return False
        prev = self._last_hit_range.get(iid)
        if prev is not None and math.isfinite(prev):
            dt_e = max(self._control_dt, 0.05)
            # Fast hostile + interceptor can close ≫ vmax·dt per tick; old cap (~15 m) false-blocked real HITs.
            v_budget = self._vmax + 45.0
            max_drop = max(40.0, v_budget * dt_e * 1.15)
            if (prev - dist_now) > max_drop and dist_now < self._hit_gate_distance_m() * 2.0:
                return False
        self._last_hit_range[iid] = dist_now
        return True

    def _hit_gate_distance_m(self) -> float:
        """HIT registration distance: wider tolerance in predictive debug mode."""
        return self._hit_thresh_dbg if self._dbg_predictive else self._hit_thresh

    def _maybe_log_hit_gate_block(
        self,
        *,
        tag: str,
        dist_to_target: float,
        eff_hit: float,
        air_ok: bool,
        strike_ok: bool,
        strike_ok_eff: bool,
        feas_ok: bool,
        mc_ok: bool,
        launched_ok: bool,
        layer: str,
    ) -> None:
        if dist_to_target >= eff_hit:
            return
        nowm = time.monotonic()
        if nowm - self._last_hit_gate_block_print_m < 0.4:
            return
        self._last_hit_gate_block_print_m = nowm
        reasons: list[str] = []
        if not air_ok:
            reasons.append('air_ok(target z)')
        if not feas_ok:
            reasons.append('feasible==False')
        if not strike_ok:
            reasons.append('strike_ok(layer/dome)')
        if feas_ok and strike_ok and not mc_ok:
            reasons.append('mc_ok(False) probability/hysteresis gate')
        if not launched_ok:
            reasons.append('launched_ok(safety)')
        if not strike_ok_eff:
            reasons.append('strike_ok_eff composite')
        print(
            f'[HIT_BLOCKED_GATE] {tag} dist={dist_to_target:.2f}m < eff_hit={eff_hit:.2f}m '
            f'strike_ok={strike_ok} feas={feas_ok} mc_ok={mc_ok} strike_ok_eff={strike_ok_eff} '
            f'launched_ok={launched_ok} layer={layer!r} -> {", ".join(reasons) or "unknown"}',
            flush=True,
        )

    def _maybe_print_hit_debug_line(
        self,
        *,
        tag: str,
        dist_to_target: float,
        dist_to_p_hit: float,
        v_req: float,
        v_cmd: float,
        feasible: bool,
        p_mc: float,
        strike_ok: bool,
        strike_ok_eff: bool,
        mc_ok: bool,
        launched_ok: bool,
        layer: str,
        t_go: float | None,
        predictive_dbg: bool,
    ) -> None:
        if not self._dbg_predictive:
            return
        nowm = time.monotonic()
        if self._dbg_hit_log_period > 1e-9 and (nowm - self._last_hit_debug_print_m) < self._dbg_hit_log_period:
            return
        self._last_hit_debug_print_m = nowm
        tgs = f'{t_go:.3f}' if t_go is not None and math.isfinite(t_go) else 'n/a'
        vrs = f'{v_req:.3f}' if math.isfinite(v_req) else 'n/a'
        print(
            f'[HIT_DEBUG] {tag} dist_to_target={dist_to_target:.3f} dist_to_p_hit={dist_to_p_hit:.3f} '
            f'v_required={vrs} v_cmd={v_cmd:.3f} feasible={feasible} P(hit|noise)={p_mc:.3f} '
            f'strike_ok={strike_ok} strike_ok_eff={strike_ok_eff} mc_ok={mc_ok} launched_ok={launched_ok} '
            f'layer={layer!r} t_go={tgs} eff_hit_m={self._hit_gate_distance_m():.3f} pred_dbg={predictive_dbg}',
            flush=True,
        )

    def _maybe_debug_predictive_no_hit_watch(
        self,
        *,
        watch_key: str,
        sim_hit: bool,
        feasible: bool,
        p_mc: float,
        t_go: float | None,
        v_req: float,
        v_cmd: float,
        strike_ok_eff: bool,
        launched_ok: bool,
        dist_to_target: float,
        dist_to_p_hit: float,
        eff_hit: float,
        layer: str,
    ) -> None:
        if not self._dbg_predictive or sim_hit:
            self._dbg_no_hit_watch_start_m.pop(watch_key, None)
            self._dbg_no_hit_watch_tgo.pop(watch_key, None)
            return
        if (
            not feasible
            or (not math.isfinite(p_mc)) or p_mc < self._dbg_no_hit_p_thr
            or t_go is None
            or (not math.isfinite(t_go))
            or t_go <= 0.0
        ):
            self._dbg_no_hit_watch_start_m.pop(watch_key, None)
            self._dbg_no_hit_watch_tgo.pop(watch_key, None)
            return
        nowm = time.monotonic()
        if watch_key not in self._dbg_no_hit_watch_start_m:
            self._dbg_no_hit_watch_start_m[watch_key] = nowm
            self._dbg_no_hit_watch_tgo[watch_key] = float(t_go)
        t0 = self._dbg_no_hit_watch_start_m[watch_key]
        tgo0 = max(self._dbg_no_hit_watch_tgo[watch_key], float(t_go))
        limit_s = tgo0 + self._dbg_no_hit_grace
        if nowm - t0 < limit_s:
            return
        lw = self._dbg_no_hit_warn_last_m.get(watch_key, 0.0)
        if nowm - lw < 4.0:
            return
        cands: list[str] = []
        if math.isfinite(v_req) and v_cmd + 0.05 < v_req:
            cands.append('command speed below required (accel cap / clamp?)')
        if not strike_ok_eff:
            cands.append('gate blocked HIT (strike_ok_eff False)')
        if not launched_ok:
            cands.append('launched_ok blocked HIT')
        if dist_to_p_hit > eff_hit * 1.5:
            cands.append('prediction stale or target maneuver (large dist_to_p_hit)')
        if dist_to_target > eff_hit:
            cands.append('hit threshold too small vs closing geometry (dist_to_target > eff_hit)')
        if not cands:
            cands.append('unknown — check range_plausible, air_ok, multi-assignment')
        print(
            f'[HIT_DEBUG_WARN] key={watch_key!r} feasible=True P={p_mc:.3f} ≥ {self._dbg_no_hit_p_thr} '
            f'elapsed={nowm - t0:.1f}s > t_go+grace={limit_s:.1f}s no HIT — candidates: {"; ".join(cands)} '
            f'layer={layer!r}',
            flush=True,
        )
        self._dbg_no_hit_warn_last_m[watch_key] = nowm
        self._dbg_no_hit_watch_start_m.pop(watch_key, None)
        self._dbg_no_hit_watch_tgo.pop(watch_key, None)

    def _latch_feasible_at_engagement_start(self, target_key: str, selected: str, feasible: bool) -> None:
        """When *selected* becomes the active interceptor for *target_key*, latch feasibility once."""
        prev_iid = self._feas_eng_latch_assign.get(target_key)
        if prev_iid != selected:
            self._feasible_at_engagement_start_by_pair[(target_key, selected)] = feasible
            self._feas_eng_latch_assign[target_key] = selected

    def _ensure_latch_on_first_hit(self, target_key: str, interceptor_id: str, feasible_now: bool) -> None:
        """If RAM never ran for this pair, still record feasibility before HIT / RESULT."""
        key = (target_key, interceptor_id)
        if key not in self._feasible_at_engagement_start_by_pair:
            self._feasible_at_engagement_start_by_pair[key] = feasible_now

    def _prune_feas_pair_maps_multi(self, assign: dict[str, str]) -> None:
        """Avoid unbounded growth: keep only current (target, interceptor) assignment keys for multi labels."""
        keys_keep = {(lab, iid) for lab, iid in assign.items()}
        self._feasible_at_engagement_start_by_pair = {
            k: v
            for k, v in self._feasible_at_engagement_start_by_pair.items()
            if k[0] not in self._multi_labels or k in keys_keep
        }
        for tk in list(self._feas_eng_latch_assign.keys()):
            if tk in self._multi_labels and tk not in assign:
                del self._feas_eng_latch_assign[tk]

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
        *,
        intercept_closing_m_s: float | None = None,
    ) -> list[tuple[str, bool, float | None]]:
        rows: list[tuple[str, bool, float | None]] = []
        s_use = self._vmax if intercept_closing_m_s is None else intercept_closing_m_s
        for iid in self._ids:
            p = self._inter_pos.get(iid)
            if p is None:
                rows.append((iid, False, None))
                continue
            ix, iy, iz = float(p.x), float(p.y), float(p.z)
            # Default: vmax (top guidance clamp). Feasibility mode passes interceptor cap so rows match ranking.
            ok, tti = _tti_feasible(
                tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz,
                s_use, self._t_hit_min, self._t_hit_max,
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
        target_prev: tuple[float, float, float] | None = None,
    ) -> None:
        zero = Vector3()
        p_sel = self._inter_pos.get(selected)
        if p_sel is None:
            print(f'[GUIDANCE_WARN] no position for selected={selected!r}, all_known={list(k for k,v in self._inter_pos.items() if v is not None)}', flush=True)
            self._publish_all({i: zero for i in self._ids}, immediate=True)
            return

        # Phase 2: extrapolate target by measurement_delay_s using current v_T estimate.
        # Without this the solver consumes a stale p_T and aims at where the target *was*,
        # which is the dominant miss-distance bias under realistic sensor / transport delay.
        if self._meas_delay_s > 0.0:
            tx = tx + v_tx * self._meas_delay_s
            ty = ty + v_ty * self._meas_delay_s
            tz = tz + v_tz * self._meas_delay_s

        ix, iy, iz = float(p_sel.x), float(p_sel.y), float(p_sel.z)
        self._refresh_mc_probs_single(tx, ty, tz, v_tx, v_ty, v_tz)
        feas = classify_intercept_feasibility_for_viz(
            tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz,
            v_i_max=self._interceptor_max_speed,
            t_min=self._t_hit_min,
            t_max=self._t_hit_max,
        )
        self._latch_feasible_at_engagement_start('', selected, feas.feasible)
        self._ensure_latch_on_first_hit('', selected, feas.feasible)
        mc_ok = self._mc_engage_ok(selected)
        strike_ok_eff = strike_ok and feas.feasible and mc_ok
        self._maybe_phase1_runtime_warnings(
            selected, feas, strike_ok_eff, layer_s=layer_for_log or 'single',
        )
        eff_hit = self._hit_gate_distance_m()

        dist_to_target = _norm(tx - ix, ty - iy, tz - iz)
        d_seg = float('inf')
        ipr = self._prev_inter_pos.get(selected)
        if (
            target_prev is not None
            and ipr is not None
            and math.isfinite(dist_to_target)
        ):
            d_seg = _closest_dist_segments_3d(ipr, (ix, iy, iz), target_prev, (tx, ty, tz))
        dist_for_hit = float(min(dist_to_target, d_seg)) if math.isfinite(d_seg) else dist_to_target

        range_plausible = self._hit_range_plausible_for_tick(selected, dist_to_target)
        if (
            not range_plausible
            and target_prev is not None
            and ipr is not None
            and math.isfinite(d_seg)
            and d_seg <= eff_hit * 1.25
        ):
            # Swept volume intersects kill sphere even though point samples failed spike gate / tunneling.
            range_plausible = True
        if range_plausible:
            self._record_miss_distance(dist_for_hit)

        air_ok = self._hit_min_tz is None or tz >= self._hit_min_tz
        # Safety guards: selected interceptor must be airborne and have actually travelled
        # from its ground start position — prevents a stationary ground interceptor from
        # triggering a false HIT just because the target's descent trajectory grazes it.
        start = self._inter_start_pos.get(selected)
        travel = _norm(ix - start[0], iy - start[1], iz - start[2]) if start is not None else 0.0
        launched_ok = (iz >= self._hit_min_iz) and (travel >= self._hit_min_travel)
        if (
            range_plausible
            and (not self._hit)
            and dist_for_hit < eff_hit
            and air_ok
            and strike_ok_eff
            and launched_ok
        ):
            self._hit = True
            extra = f' layer={layer_for_log}' if layer_for_log else ''
            mm = f'{self._min_miss_distance:.4f} m' if self._min_miss_distance < float("inf") else 'n/a'
            print(
                f'[HIT] {selected}{extra}  min_miss={mm}  hit_threshold = {eff_hit:.4f} m '
                f'(nominal={self._hit_thresh:.4f} debug_mode={self._dbg_predictive}) '
                f'interceptor_pos=({ix:.2f},{iy:.2f},{iz:.2f}) travel={travel:.2f}m '
                f'target_pos=({tx:.2f},{ty:.2f},{tz:.2f}) dist_pt={dist_to_target:.3f}m dist_swept={d_seg:.3f}m',
                flush=True,
            )
            elapsed = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
            f0 = self._feasible_at_engagement_start_by_pair.get(('', selected))
            f0s = 'n/a' if f0 is None else ('True' if f0 else 'False')
            f1s = 'True' if feas.feasible else 'False'
            print(
                f'[RESULT]\n'
                f'success=True\n'
                f'miss_distance={mm}\n'
                f'intercept_time={elapsed:.3f}s\n'
                f'feasible_at_start={f0s}\n'
                f'feasible_at_hit={f1s}',
                flush=True,
            )
            hit_sub = selected + (f' ({layer_for_log})' if layer_for_log else '')
            self._start_hit_visual(tx, ty, tz, ix, iy, iz, hit_sub)
            self._publish_stop_signal(None)
            self._schedule_stop_signal_repeats(None)
            # immediate=True: bypass accel limit so interceptor freezes at impact instantly.
            self._publish_all({i: zero for i in self._ids}, immediate=True)
            # Delay pause so target_controller has time to receive stop signal,
            # remove target model, and spawn explosion visual before world freezes.
            self.create_timer(3.5, self._pause_gazebo_world)
            return
        if (
            range_plausible
            and (not self._hit)
            and dist_for_hit < eff_hit
            and air_ok
            and strike_ok_eff
            and (not launched_ok)
        ):
            if not self._logged_guard_block:
                print(
                    f'[HIT_BLOCKED_SAFETY] {selected} dist={dist_for_hit:.2f}m < eff_hit={eff_hit:.2f} but interceptor '
                    f'not launched: iz={iz:.2f}m (need>={self._hit_min_iz:.2f}, ok={iz >= self._hit_min_iz}) '
                    f'travel={travel:.2f}m (need>={self._hit_min_travel:.2f}, ok={travel >= self._hit_min_travel}) '
                    f'interceptor=({ix:.2f},{iy:.2f},{iz:.2f}) '
                    f'target=({tx:.2f},{ty:.2f},{tz:.2f})',
                    flush=True,
                )
                self._logged_guard_block = True
            # Keep guiding — do not freeze/pause; proximity was a false positive (ground idle).
        elif (
            range_plausible
            and (not self._hit)
            and dist_for_hit < eff_hit
            and air_ok
            and (not strike_ok_eff)
        ):
            self._maybe_log_hit_gate_block(
                tag=selected,
                dist_to_target=dist_for_hit,
                eff_hit=eff_hit,
                air_ok=air_ok,
                strike_ok=strike_ok,
                strike_ok_eff=strike_ok_eff,
                feas_ok=feas.feasible,
                mc_ok=mc_ok,
                launched_ok=launched_ok,
                layer=layer_for_log or 'single',
            )

        v_ix, v_iy, v_iz = self._estimate_interceptor_vel(selected, ix, iy, iz)
        dist = dist_to_target
        out_map = {i: Vector3() for i in self._ids}

        gx, gy, gz = tx, ty, tz
        if strike_ok_eff and self._dome_enabled and self._aim_mid_shell:
            gx, gy, gz = self._point_on_mid_shell(tx, ty, tz)

        if dist < self._r_stop:
            # Hard stop at minimum distance: bypass accel limit so the interceptor
            # does not overshoot the hold radius due to ramp-down lag.
            self._publish_all(out_map, immediate=True)
            self._maybe_detail_log(tx, ty, tz, ix, iy, iz, dist, None, None, None, 0.0, 0.0, 0.0, 'hold', False, 0.0, selected)
            iid_h = self._ids.index(selected)
            self._publish_intercept_math_markers(
                selected, iid_h, ix, iy, iz, tx, ty, tz, feas, v_cmd=0.0,
            )
            return

        # ── Predict direction ────────────────────────────────────────────────
        # Initialise raw-point variables to nan so the debug log is safe even
        # when no fresh solve is available this cycle.
        t_hit: float | None = None
        phx_raw = phy_raw = phz_raw = float('nan')
        phx = phy = phz = 0.0
        ux_pred = uy_pred = uz_pred = 0.0
        sol_valid = False

        # ── Iterative intercept solve (2-pass) ───────────────────────────────
        # Pass 1: rough t_go using distance-only speed (k1*dist, no urgency term).
        # Pass 2: recompute with actual commanded speed sp_actual = k1*d + k2*d/t_go
        #         so the interceptor's real flight speed matches the solver assumption.
        # Without this, the interceptor arrives at the predicted point BEFORE the target
        # (because the urgency term k2*d/t_go makes it fly faster than the solver expected)
        # → overshoot.
        # s_for_model tracks which constant closing speed s_i was used in |r0+v_T t|=s_i t.
        sp_plan = self._command_speed(dist)   # k1*dist — no t_go yet
        s_for_model = sp_plan
        sol = _compute_intercept(tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, sp_plan)
        if sol is not None:
            t_rough = sol[0]
            if self._t_hit_min <= t_rough <= self._t_hit_max:
                sp_actual = self._command_speed(dist, t_go=t_rough)
                if abs(sp_actual - sp_plan) > 0.05:     # re-solve only if speed differs
                    sol2 = _compute_intercept(tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, sp_actual)
                    if sol2 is not None:
                        sol = sol2
                        s_for_model = sp_actual
        if sol is not None:
            t_hit, phx, phy, phz, ux_pred, uy_pred, uz_pred = sol
            if self._t_hit_min <= t_hit <= self._t_hit_max and _norm(ux_pred, uy_pred, uz_pred) > 1e-6:
                sol_valid = True
                # EMA filter on raw intercept point; keep raw for debug log.
                phx_raw, phy_raw, phz_raw = phx, phy, phz
                phx, phy, phz = self._smooth_intercept_point(selected, phx, phy, phz)
                ux_f, uy_f, uz_f = _unit(phx - ix, phy - iy, phz - iz)
                if _norm(ux_f, uy_f, uz_f) > 1e-6:
                    ux_pred, uy_pred, uz_pred = ux_f, uy_f, uz_f

        dbg_cmd_override = False
        v_req_dbg = float('nan')
        ux_d = uy_d = uz_d = 0.0
        sp_dbg = 0.0
        if self._dbg_predictive and sol_valid and t_hit is not None:
            phu_x, phu_y, phu_z = phx_raw, phy_raw, phz_raw
            dx = phu_x - ix
            dy = phu_y - iy
            dz = phu_z - iz
            unr = _norm(dx, dy, dz)
            if unr > 1e-9:
                v_req_dbg = compute_required_speed(ix, iy, iz, phu_x, phu_y, phu_z, t_hit)
                if math.isfinite(v_req_dbg) and v_req_dbg < float('inf'):
                    ux_d, uy_d, uz_d = _unit(dx, dy, dz)
                    sp_dbg = min(
                        self._interceptor_max_speed,
                        max(0.0, v_req_dbg + self._dbg_spd_margin),
                    )
                    dbg_cmd_override = True

        blend = self._update_guidance_mode(selected, sol_valid)
        committed_mode = self._guidance_mode[selected]

        pn_active = False
        vc_log = 0.0
        mode = committed_mode
        if dbg_cmd_override:
            ux, uy, uz = ux_d, uy_d, uz_d
            mode = 'predict_dbg'
        else:
            # ── Pursuit direction (always computed — needed for blending) ─────────
            # ใน strike + aim_mid_shell ใช้จุดบนเปลือก r_mid เป็นทิศ LOS (ขอบชั้น 1–2)
            losx, losy, losz = _unit(gx - ix, gy - iy, gz - iz)
            bl = self._pursuit_lead_blend
            spv = abs(v_tx) + abs(v_ty) + abs(v_tz)
            if bl > 1e-6 and spv > 0.02:
                lx = tx + v_tx * self._naive_lead
                ly = ty + v_ty * self._naive_lead
                lz = tz + v_tz * self._naive_lead
                lox, loy, loz = _unit(lx - ix, ly - iy, lz - iz)
                if _norm(lox, loy, loz) > 1e-9:
                    ux_pur = (1.0 - bl) * losx + bl * lox
                    uy_pur = (1.0 - bl) * losy + bl * loy
                    uz_pur = (1.0 - bl) * losz + bl * loz
                    un = _norm(ux_pur, uy_pur, uz_pur)
                    if un > 1e-9:
                        ux_pur, uy_pur, uz_pur = ux_pur / un, uy_pur / un, uz_pur / un
                    else:
                        ux_pur, uy_pur, uz_pur = losx, losy, losz
                else:
                    ux_pur, uy_pur, uz_pur = losx, losy, losz
            else:
                ux_pur, uy_pur, uz_pur = losx, losy, losz
            if _norm(ux_pur, uy_pur, uz_pur) < 1e-9:
                ux_pur, uy_pur, uz_pur = losx, losy, losz

            # ── Mode hysteresis + direction blending ──────────────────────────────
            # blend=0 → full pursuit; blend=1 → full predict.
            # Transitions are spread over enter/exit_frames to prevent rapid switching.

            # During exit transition (blend > 0 but no fresh solve), fall back to
            # the last known filtered intercept point to maintain a predict direction.
            if not sol_valid and blend > 1e-6:
                flt = self._intercept_point_filtered[selected]
                if flt is not None:
                    ux_f, uy_f, uz_f = _unit(flt[0] - ix, flt[1] - iy, flt[2] - iz)
                    if _norm(ux_f, uy_f, uz_f) > 1e-6:
                        ux_pred, uy_pred, uz_pred = ux_f, uy_f, uz_f
                    else:
                        blend = 0.0  # filter point is at interceptor location — use pursuit
                else:
                    blend = 0.0  # no filter history — use pursuit

            if blend > 1e-6:
                bx = (1.0 - blend) * ux_pur + blend * ux_pred
                by = (1.0 - blend) * uy_pur + blend * uy_pred
                bz = (1.0 - blend) * uz_pur + blend * uz_pred
                un = _norm(bx, by, bz)
                ux, uy, uz = (bx / un, by / un, bz / un) if un > 1e-9 else (ux_pur, uy_pur, uz_pur)
            else:
                ux, uy, uz = ux_pur, uy_pur, uz_pur

            mode = committed_mode          # 'predict' or 'pursuit' — reported in logs
            if not sol_valid:
                t_hit = None
                phx = phy = phz = float('nan')

            if self._use_pn and self._pn_blend > 1e-9 and dist > 1e-6:
                # เมื่อ aim shell: ใช้เวกเตอร์สัมพัธ์ไปจุดบนเปลือกเพื่อให้สอดคล้อง pursuit
                px, py, pz = (gx, gy, gz) if (strike_ok_eff and self._dome_enabled and self._aim_mid_shell) else (tx, ty, tz)
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

        if dbg_cmd_override:
            sp = sp_dbg
        elif (
            self._align_speed_to_solver
            and sol_valid
            and t_hit is not None
            and math.isfinite(t_hit)
            and t_hit > 0.0
            and all(math.isfinite(v) for v in (phx_raw, phy_raw, phz_raw))
        ):
            # Phase 2: replace heuristic distance-based speed with the constant ``s`` that the
            # CV intercept quadratic implicitly assumes: s = |P_hit - P_I| / t_go, capped to
            # the interceptor speed limit. Filter t_go first so a single noisy frame cannot
            # whip the commanded magnitude. ``ux,uy,uz`` already point at (smoothed) P_hit.
            t_go_raw = float(t_hit)
            t_go_filt = self._t_go_filtered.get(selected)
            t_go_eff = t_go_raw if t_go_filt is None else (
                self._t_go_alpha * t_go_raw + (1.0 - self._t_go_alpha) * t_go_filt
            )
            self._t_go_filtered[selected] = t_go_eff
            range_to_phit = _norm(phx_raw - ix, phy_raw - iy, phz_raw - iz)
            t_go_use = max(t_go_eff, self._speed_tgo_min)
            s_solver = max(self._speed_vmin, min(self._vmax, range_to_phit / t_go_use))
            sp = s_solver
        else:
            # Pursuit fallback (no fresh solve) — keep the legacy distance-only law and reset the
            # filter so the next predict cycle starts cleanly without stale t_go.
            self._t_go_filtered[selected] = None
            sp = self._command_speed(dist, t_go=t_hit)
        vx = ux * sp
        vy = uy * sp
        vz = uz * sp
        vx, vy, vz = self._clamp_speed(vx, vy, vz)
        cmd = Vector3()
        cmd.x, cmd.y, cmd.z = vx, vy, vz
        out_map[selected] = cmd
        self._publish_all(out_map)

        solver_s_plot = s_for_model if sol_valid else None
        v_cmd_mag = _norm(vx, vy, vz)

        start_h = self._inter_start_pos.get(selected)
        travel_h = (
            _norm(ix - start_h[0], iy - start_h[1], iz - start_h[2]) if start_h is not None else 0.0
        )
        launched_ok_h = (iz >= self._hit_min_iz) and (travel_h >= self._hit_min_travel)
        dist_ph = (
            _norm(phx_raw - ix, phy_raw - iy, phz_raw - iz)
            if sol_valid and all(math.isfinite(v) for v in (phx_raw, phy_raw, phz_raw))
            else float('nan')
        )
        if dbg_cmd_override and math.isfinite(v_req_dbg):
            v_req_log = v_req_dbg
        elif sol_valid and t_hit is not None and math.isfinite(t_hit) and t_hit > 0.0:
            v_req_log = compute_required_speed(ix, iy, iz, phx_raw, phy_raw, phz_raw, t_hit)
        else:
            v_req_log = float('nan')
        p_mc = self._mc_p_last.get(selected, float('nan'))
        self._maybe_print_hit_debug_line(
            tag=selected,
            dist_to_target=dist,
            dist_to_p_hit=dist_ph,
            v_req=v_req_log,
            v_cmd=v_cmd_mag,
            feasible=feas.feasible,
            p_mc=p_mc if math.isfinite(p_mc) else 0.0,
            strike_ok=strike_ok,
            strike_ok_eff=strike_ok_eff,
            mc_ok=mc_ok,
            launched_ok=launched_ok_h,
            layer=layer_for_log or 'single',
            t_go=t_hit,
            predictive_dbg=dbg_cmd_override,
        )
        self._maybe_debug_predictive_no_hit_watch(
            watch_key=selected,
            sim_hit=self._hit,
            feasible=feas.feasible,
            p_mc=p_mc if math.isfinite(p_mc) else 0.0,
            t_go=t_hit,
            v_req=v_req_log,
            v_cmd=v_cmd_mag,
            strike_ok_eff=strike_ok_eff,
            launched_ok=launched_ok_h,
            dist_to_target=dist,
            dist_to_p_hit=dist_ph,
            eff_hit=eff_hit,
            layer=layer_for_log or 'single',
        )

        # ── Metrics snapshot (read by _print_metrics timer) ───────────────────
        self._metrics[selected] = {
            'dist':    dist,
            't_go':    t_hit,
            'vel_mag': v_cmd_mag,
            'mode':    mode,
            'feasible': feas.feasible,
            'feas_status': feas.status,
            'feas_reason': feas.reason,
        }

        # ── RViz markers: sphere at intercept point + trajectory line ─────────
        iid_index = self._ids.index(selected)
        _vis_phx = phx_raw if sol_valid else float('nan')
        _raw_mc = (
            self._mc_p_last.get(selected)
            if (self._intercept_mc_trail_by_prob or self._intercept_mc_engagement)
            else None
        )
        _mc_viz = _raw_mc if (_raw_mc is not None and math.isfinite(_raw_mc)) else None
        self._publish_intercept_markers(
            selected, iid_index, ix, iy, iz, _vis_phx,
            phy_raw if sol_valid else float('nan'),
            phz_raw if sol_valid else float('nan'),
            sol_valid,
            t_go=t_hit,
            solver_closing_m_s=solver_s_plot,
            v_cmd_m_s=v_cmd_mag,
            mc_hit_prob=_mc_viz,
        )

        self._publish_intercept_math_markers(
            selected, iid_index, ix, iy, iz, tx, ty, tz, feas, v_cmd=v_cmd_mag,
        )
        self._maybe_feasibility_debug_log(
            feas=feas,
            v_cmd=v_cmd_mag,
            distance=dist,
            layer=layer_for_log,
            iid=selected,
        )
        self._maybe_feasibility_warnings(
            feas=feas,
            guidance_sol_ok=sol_valid,
            guidance_t_go=t_hit,
            v_cmd=v_cmd_mag,
            strike_engaged=strike_ok_eff,
            iid=selected,
        )

        # Log raw intercept point when a fresh solve was available this cycle;
        # nan otherwise (pursuit or exit-transition with no current solution).
        _log_phx = phx_raw if sol_valid else phx
        _log_phy = phy_raw if sol_valid else phy
        _log_phz = phz_raw if sol_valid else phz
        self._maybe_detail_log(
            tx, ty, tz, ix, iy, iz, dist, t_hit, _log_phx, _log_phy, _log_phz,
            vx, vy, vz, mode, pn_active, vc_log, selected,
            solver_closing_m_s=solver_s_plot,
        )

    def _estimate_target_vel(self, tx: float, ty: float, tz: float) -> tuple[float, float, float]:
        # Prefer the upstream filter velocity (Odometry.twist) when available.  Differencing
        # positions at the control rate amplifies measurement noise / quantisation that the
        # Kalman filter has already rejected; using the filtered velocity keeps the predictive
        # intercept consistent with the same state the tracker reports.
        if self._meas_use_filter_vel and self._target_filter_velocity is not None:
            sx, sy, sz = self._target_filter_velocity
            # Light EMA on top of the filter output for continuity if the topic stalls; this is
            # additive smoothing only and does not re-introduce finite differences.
            a = self._tgt_vel_smooth_a
            self._v_tgt_smooth = (
                a * sx + (1.0 - a) * self._v_tgt_smooth[0],
                a * sy + (1.0 - a) * self._v_tgt_smooth[1],
                a * sz + (1.0 - a) * self._v_tgt_smooth[2],
            )
            return self._v_tgt_smooth
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

    def _update_guidance_mode(self, iid: str, sol_valid: bool) -> float:
        """
        Hysteresis state machine for pursuit ↔ predict mode transitions.

        Requires *predict_enter_frames* consecutive valid-solution frames to
        enter predict mode and *predict_exit_frames* consecutive invalid frames
        to return to pursuit.  This prevents a single noisy frame from flipping
        guidance.

        Returns *blend_alpha* in [0.0, 1.0]:
          0.0 = full pursuit direction
          1.0 = full predict direction

        The blend ramps linearly across the transition window so the output
        direction rotates smoothly rather than snapping between modes.

        Blend derivation:
          Entering predict (committed='pursuit', valid_streak growing):
            blend = valid_streak / enter_frames   (0 → 1 as we count up)
          Exiting predict (committed='predict', invalid_streak growing):
            blend = 1 - invalid_streak / exit_frames  (1 → 0 as we count up)
        """
        # Update streaks: one resets whenever the other increments.
        if sol_valid:
            self._valid_streak[iid] = min(self._valid_streak[iid] + 1, self._predict_enter_frames)
            self._invalid_streak[iid] = 0
        else:
            self._invalid_streak[iid] = min(self._invalid_streak[iid] + 1, self._predict_exit_frames)
            self._valid_streak[iid] = 0

        # State transitions (only after threshold is met).
        prev = self._guidance_mode[iid]
        if prev == 'pursuit' and self._valid_streak[iid] >= self._predict_enter_frames:
            self._guidance_mode[iid] = 'predict'
            print(
                f'[MODE] {iid!r}: pursuit → predict '
                f'({self._predict_enter_frames} consecutive valid frames)',
                flush=True,
            )
        elif prev == 'predict' and self._invalid_streak[iid] >= self._predict_exit_frames:
            self._guidance_mode[iid] = 'pursuit'
            print(
                f'[MODE] {iid!r}: predict → pursuit '
                f'({self._predict_exit_frames} consecutive invalid frames)',
                flush=True,
            )

        # Compute blend factor for smooth direction interpolation.
        committed = self._guidance_mode[iid]
        if committed == 'predict':
            # Entering: valid_streak ramps from 0→enter_frames → blend 0→1.
            # Exiting:  invalid_streak ramps from 0→exit_frames → blend 1→0.
            blend = 1.0 - self._invalid_streak[iid] / max(self._predict_exit_frames, 1)
        else:  # 'pursuit'
            # Counting toward predict: valid_streak ramps 0→enter_frames → blend 0→1.
            # Stable in pursuit (valid_streak == 0): blend stays 0.
            blend = self._valid_streak[iid] / max(self._predict_enter_frames, 1)

        return max(0.0, min(1.0, blend))

    def _smooth_intercept_point(
        self, iid: str, phx: float, phy: float, phz: float
    ) -> tuple[float, float, float]:
        """
        Exponential Moving Average (EMA) filter on the predicted intercept point.

        Reduces frame-to-frame jitter caused by noise in target-velocity estimates
        and mode switching between predict / pursuit.  The filtered point is used
        for computing the guidance direction; the raw value is preserved for logs.

        On the first valid solve the filter is seeded with the raw value so there
        is no lag on engagement start.  alpha=1.0 disables smoothing (raw pass-through).
        """
        prev = self._intercept_point_filtered.get(iid)
        if prev is None:
            self._intercept_point_filtered[iid] = (phx, phy, phz)
            return (phx, phy, phz)
        a = self._intercept_alpha
        fx = a * phx + (1.0 - a) * prev[0]
        fy = a * phy + (1.0 - a) * prev[1]
        fz = a * phz + (1.0 - a) * prev[2]
        self._intercept_point_filtered[iid] = (fx, fy, fz)
        return (fx, fy, fz)

    def _accel_limit_velocity(
        self, iid: str, vx: float, vy: float, vz: float
    ) -> tuple[float, float, float]:
        """
        Two-stage velocity command smoother applied every control cycle.

        Stage 1 — Turn-rate limit  (applied first, on the RAW desired direction)
          Compare prev-velocity direction vs desired-velocity direction.
          Rotate prev direction toward desired by at most max_turn_rate * dt rad.
          Keep the desired speed magnitude so guidance controls speed normally.
          Prevents sharp heading reversals and oscillation at ALL flight speeds.
          When _max_turn_rate == 0.0 this stage is skipped entirely.

        Stage 2 — Acceleration limit  (applied second, on the turn-limited result)
          Clamp |dv| ≤ max_accel * dt so the velocity vector cannot jump
          discontinuously between cycles.  Prevents fly-stop-fly jerks and
          limits speed ramp-up rate.

        Order matters: Turn first so the angular budget is relative to the raw
        guidance direction (not an already-clipped intermediate), then Accel
        to enforce the delta-v ceiling.  Both share _prev_velocity which is
        written exactly once (at the end, after both stages).

        Edge-case guards:
          - p_spd < 1e-6 (starting from rest): skip Stage 1, ramp freely.
          - d_spd < 1e-6 (stopping):          skip Stage 1, decelerate freely.
        """
        pvx, pvy, pvz = self._prev_velocity[iid]

        # ── Stage 1: turn-rate limit on raw desired direction ─────────────────
        # nx/ny/nz starts as the raw desired; Stage 1 may rotate its direction
        # while keeping the desired speed magnitude.
        nx, ny, nz = vx, vy, vz
        if self._max_turn_rate > 1e-9:
            p_spd = _norm(pvx, pvy, pvz)
            d_spd = _norm(vx, vy, vz)
            if p_spd > 1e-6 and d_spd > 1e-6:
                max_angle = self._max_turn_rate * self._control_dt
                pdx, pdy, pdz = pvx / p_spd, pvy / p_spd, pvz / p_spd
                ddx, ddy, ddz = vx / d_spd, vy / d_spd, vz / d_spd
                rdx, rdy, rdz = _rotate_dir_toward(
                    pdx, pdy, pdz, ddx, ddy, ddz, max_angle,
                )
                # Apply the turn-limited direction with the desired speed so
                # guidance retains full speed authority.
                nx, ny, nz = rdx * d_spd, rdy * d_spd, rdz * d_spd

        # ── Stage 2: acceleration limit on the turn-limited velocity ──────────
        dvx, dvy, dvz = nx - pvx, ny - pvy, nz - pvz
        dv = _norm(dvx, dvy, dvz)
        max_dv = self._max_accel * self._control_dt
        if dv > max_dv and dv > 1e-9:
            s = max_dv / dv
            dvx, dvy, dvz = dvx * s, dvy * s, dvz * s
        nx, ny, nz = pvx + dvx, pvy + dvy, pvz + dvz

        self._prev_velocity[iid] = (nx, ny, nz)
        return (nx, ny, nz)

    def _publish_all(self, velocities: dict[str, Vector3], *, immediate: bool = False) -> None:
        for iid in self._ids:
            v = velocities.get(iid, Vector3())
            if immediate:
                # Hard stop (HIT / emergency): bypass accel limit and reset state so
                # the next guidance ramp starts cleanly from zero.
                self._prev_velocity[iid] = (0.0, 0.0, 0.0)
                self._last_guidance_cmd[iid] = (0.0, 0.0, 0.0)
                self._pubs[iid].publish(v)
            else:
                lx, ly, lz = self._accel_limit_velocity(iid, v.x, v.y, v.z)
                # Monte Carlo hit model uses **published** velocity (post accel limit).
                self._last_guidance_cmd[iid] = (lx, ly, lz)
                limited = Vector3()
                limited.x, limited.y, limited.z = lx, ly, lz
                self._pubs[iid].publish(limited)

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

        # Update assignment lock state whenever the committed interceptor changes.
        if new != self._assigned_interceptor_id:
            prev = self._assigned_interceptor_id
            self._assigned_interceptor_id = new
            self._assignment_time = now
            self._last_assign_lock_log = None
            print(
                f'[ASSIGN] interceptor assignment changed: {prev!r} -> {new!r} '
                f'(lock for {self._assignment_lock_duration:.1f}s) | '
                f'current assigned: {self._assigned_interceptor_id!r}',
                flush=True,
            )

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
        # --- Assignment lock: skip reselection while the lock is still active ---
        if self._assigned_interceptor_id is not None and self._assignment_time is not None:
            lock_age = (now - self._assignment_time).nanoseconds * 1e-9
            if lock_age < self._assignment_lock_duration:
                # Rate-limit this log to once per 0.5 s so it doesn't spam at 20 Hz.
                if self._last_assign_lock_log is None or \
                        (now - self._last_assign_lock_log).nanoseconds * 1e-9 >= 0.5:
                    self._last_assign_lock_log = now
                    print(
                        f'[ASSIGN_LOCK] holding {self._assigned_interceptor_id!r} '
                        f'(lock_age={lock_age:.3f}s / {self._assignment_lock_duration:.1f}s)',
                        flush=True,
                    )
                return self._assigned_interceptor_id

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
        *,
        layer_for_log: str = '',
        target_label: str = '',
    ) -> Vector3:
        self._record_miss_distance(dist)
        feas_m = classify_intercept_feasibility_for_viz(
            tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz,
            v_i_max=self._interceptor_max_speed,
            t_min=self._t_hit_min,
            t_max=self._t_hit_max,
        )
        self._latch_feasible_at_engagement_start(target_label, selected, feas_m.feasible)
        self._ensure_latch_on_first_hit(target_label, selected, feas_m.feasible)
        mc_ok_m = self._mc_engage_ok(selected)
        strike_ok_eff = strike_ok and feas_m.feasible and mc_ok_m
        layer_s = f'{target_label}:{layer_for_log}'.strip(':') or 'multi'
        self._maybe_phase1_runtime_warnings(selected, feas_m, strike_ok_eff, layer_s=layer_s)
        v_ix, v_iy, v_iz = self._estimate_interceptor_vel(selected, ix, iy, iz)
        zero = Vector3()
        gx, gy, gz = tx, ty, tz
        if strike_ok_eff and self._dome_enabled and self._aim_mid_shell:
            gx, gy, gz = self._point_on_mid_shell(tx, ty, tz)
        if dist < self._r_stop:
            # Hard stop: reset accel state so next engagement ramps from zero.
            self._prev_velocity[selected] = (0.0, 0.0, 0.0)
            iid_h = self._ids.index(selected)
            self._publish_intercept_math_markers(
                selected, iid_h, ix, iy, iz, tx, ty, tz, feas_m, v_cmd=0.0,
            )
            return zero
        # ── Predict direction (2-pass iterative solve) ───────────────────────
        ux_pred = uy_pred = uz_pred = 0.0
        sol_valid_m = False
        _t_hit_m: float | None = None
        _phx = _phy = _phz = float('nan')   # always defined for marker / log safety
        _phx_raw_m = _phy_raw_m = _phz_raw_m = float('nan')
        # Pass 1: rough estimate with distance-only planning speed.
        _sp_plan_m = self._command_speed(dist)
        s_for_model_m = _sp_plan_m
        sol = _compute_intercept(tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, _sp_plan_m)
        if sol is not None:
            _t_rough_m = sol[0]
            if self._t_hit_min <= _t_rough_m <= self._t_hit_max:
                _sp_actual_m = self._command_speed(dist, t_go=_t_rough_m)
                if abs(_sp_actual_m - _sp_plan_m) > 0.05:
                    _sol2_m = _compute_intercept(tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, _sp_actual_m)
                    if _sol2_m is not None:
                        sol = _sol2_m
                        s_for_model_m = _sp_actual_m
        if sol is not None:
            _t_hit_m, _phx, _phy, _phz, ux_pred, uy_pred, uz_pred = sol
            if self._t_hit_min <= _t_hit_m <= self._t_hit_max and _norm(ux_pred, uy_pred, uz_pred) > 1e-6:
                sol_valid_m = True
                _phx_raw_m, _phy_raw_m, _phz_raw_m = _phx, _phy, _phz
                # EMA filter on raw intercept point.
                _phx_f, _phy_f, _phz_f = self._smooth_intercept_point(selected, _phx, _phy, _phz)
                ux_f, uy_f, uz_f = _unit(_phx_f - ix, _phy_f - iy, _phz_f - iz)
                if _norm(ux_f, uy_f, uz_f) > 1e-6:
                    ux_pred, uy_pred, uz_pred = ux_f, uy_f, uz_f

        dbg_cmd_override_m = False
        v_req_dbg_m = float('nan')
        ux_dm = uy_dm = uz_dm = 0.0
        sp_dbg_m = 0.0
        if self._dbg_predictive and sol_valid_m and _t_hit_m is not None:
            dxm = _phx_raw_m - ix
            dym = _phy_raw_m - iy
            dzm = _phz_raw_m - iz
            unrm = _norm(dxm, dym, dzm)
            if unrm > 1e-9:
                v_req_dbg_m = compute_required_speed(ix, iy, iz, _phx_raw_m, _phy_raw_m, _phz_raw_m, _t_hit_m)
                if math.isfinite(v_req_dbg_m) and v_req_dbg_m < float('inf'):
                    ux_dm, uy_dm, uz_dm = _unit(dxm, dym, dzm)
                    sp_dbg_m = min(
                        self._interceptor_max_speed,
                        max(0.0, v_req_dbg_m + self._dbg_spd_margin),
                    )
                    dbg_cmd_override_m = True

        blend_m = self._update_guidance_mode(selected, sol_valid_m)
        committed_mode_m = self._guidance_mode[selected]

        if dbg_cmd_override_m:
            ux, uy, uz = ux_dm, uy_dm, uz_dm
            mode_m = 'predict_dbg'
        else:
            # ── Pursuit direction (always computed) ──────────────────────────────
            losx, losy, losz = _unit(gx - ix, gy - iy, gz - iz)
            bl = self._pursuit_lead_blend
            spv = abs(v_tx) + abs(v_ty) + abs(v_tz)
            if bl > 1e-6 and spv > 0.02:
                lx = tx + v_tx * self._naive_lead
                ly = ty + v_ty * self._naive_lead
                lz = tz + v_tz * self._naive_lead
                lox, loy, loz = _unit(lx - ix, ly - iy, lz - iz)
                if _norm(lox, loy, loz) > 1e-9:
                    ux_pur = (1.0 - bl) * losx + bl * lox
                    uy_pur = (1.0 - bl) * losy + bl * loy
                    uz_pur = (1.0 - bl) * losz + bl * loz
                    un = _norm(ux_pur, uy_pur, uz_pur)
                    if un > 1e-9:
                        ux_pur, uy_pur, uz_pur = ux_pur / un, uy_pur / un, uz_pur / un
                    else:
                        ux_pur, uy_pur, uz_pur = losx, losy, losz
                else:
                    ux_pur, uy_pur, uz_pur = losx, losy, losz
            else:
                ux_pur, uy_pur, uz_pur = losx, losy, losz
            if _norm(ux_pur, uy_pur, uz_pur) < 1e-9:
                ux_pur, uy_pur, uz_pur = losx, losy, losz

            # ── Mode hysteresis + direction blending ──────────────────────────────
            if not sol_valid_m and blend_m > 1e-6:
                flt = self._intercept_point_filtered[selected]
                if flt is not None:
                    ux_f, uy_f, uz_f = _unit(flt[0] - ix, flt[1] - iy, flt[2] - iz)
                    if _norm(ux_f, uy_f, uz_f) > 1e-6:
                        ux_pred, uy_pred, uz_pred = ux_f, uy_f, uz_f
                    else:
                        blend_m = 0.0
                else:
                    blend_m = 0.0

            if blend_m > 1e-6:
                bx = (1.0 - blend_m) * ux_pur + blend_m * ux_pred
                by = (1.0 - blend_m) * uy_pur + blend_m * uy_pred
                bz = (1.0 - blend_m) * uz_pur + blend_m * uz_pred
                un = _norm(bx, by, bz)
                ux, uy, uz = (bx / un, by / un, bz / un) if un > 1e-9 else (ux_pur, uy_pur, uz_pur)
            else:
                ux, uy, uz = ux_pur, uy_pur, uz_pur
            mode_m = committed_mode_m
            if not sol_valid_m:
                _t_hit_m = None

            if self._use_pn and self._pn_blend > 1e-9 and dist > 1e-6:
                px, py, pz = (gx, gy, gz) if (strike_ok_eff and self._dome_enabled and self._aim_mid_shell) else (tx, ty, tz)
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
        if dbg_cmd_override_m:
            sp = sp_dbg_m
        else:
            # Pass t_go only when a fresh predict solution is available; None → pursuit fallback.
            sp = self._command_speed(dist, t_go=_t_hit_m if sol_valid_m else None)
        vx = ux * sp
        vy = uy * sp
        vz = uz * sp
        vx, vy, vz = self._clamp_speed(vx, vy, vz)
        v_cmd_m = _norm(vx, vy, vz)
        # Accel limit is applied by the caller (_publish_all), not here, so that
        # _prev_velocity is written exactly once per cycle per interceptor.

        eff_hit_m = self._hit_gate_distance_m()
        start_mh = self._inter_start_pos.get(selected)
        travel_mh = (
            _norm(ix - start_mh[0], iy - start_mh[1], iz - start_mh[2]) if start_mh is not None else 0.0
        )
        launched_ok_mh = (iz >= self._hit_min_iz) and (travel_mh >= self._hit_min_travel)
        dist_ph_m = (
            _norm(_phx_raw_m - ix, _phy_raw_m - iy, _phz_raw_m - iz)
            if sol_valid_m and all(math.isfinite(v) for v in (_phx_raw_m, _phy_raw_m, _phz_raw_m))
            else float('nan')
        )
        if dbg_cmd_override_m and math.isfinite(v_req_dbg_m):
            v_req_log_m = v_req_dbg_m
        elif sol_valid_m and _t_hit_m is not None and math.isfinite(_t_hit_m) and _t_hit_m > 0.0:
            v_req_log_m = compute_required_speed(ix, iy, iz, _phx_raw_m, _phy_raw_m, _phz_raw_m, _t_hit_m)
        else:
            v_req_log_m = float('nan')
        p_mcm = self._mc_p_last.get(selected, float('nan'))
        wkey_m = f'{target_label}|{selected}'
        self._maybe_print_hit_debug_line(
            tag=wkey_m,
            dist_to_target=dist,
            dist_to_p_hit=dist_ph_m,
            v_req=v_req_log_m,
            v_cmd=v_cmd_m,
            feasible=feas_m.feasible,
            p_mc=p_mcm if math.isfinite(p_mcm) else 0.0,
            strike_ok=strike_ok,
            strike_ok_eff=strike_ok_eff,
            mc_ok=mc_ok_m,
            launched_ok=launched_ok_mh,
            layer=layer_s,
            t_go=_t_hit_m,
            predictive_dbg=dbg_cmd_override_m,
        )
        self._maybe_debug_predictive_no_hit_watch(
            watch_key=wkey_m,
            sim_hit=self._hits_multi.get(target_label, False),
            feasible=feas_m.feasible,
            p_mc=p_mcm if math.isfinite(p_mcm) else 0.0,
            t_go=_t_hit_m,
            v_req=v_req_log_m,
            v_cmd=v_cmd_m,
            strike_ok_eff=strike_ok_eff,
            launched_ok=launched_ok_mh,
            dist_to_target=dist,
            dist_to_p_hit=dist_ph_m,
            eff_hit=eff_hit_m,
            layer=layer_s,
        )

        # ── Metrics snapshot ──────────────────────────────────────────────────
        self._metrics[selected] = {
            'dist':    dist,
            't_go':    _t_hit_m if sol_valid_m else None,
            'vel_mag': v_cmd_m,
            'mode':    mode_m,
            'feasible': feas_m.feasible,
            'feas_status': feas_m.status,
            'feas_reason': feas_m.reason,
        }

        # ── RViz markers ──────────────────────────────────────────────────────
        iid_index_m = self._ids.index(selected)
        _vis_phx_m = _phx_raw_m if sol_valid_m else float('nan')
        _vis_phy_m = _phy_raw_m if sol_valid_m else float('nan')
        _vis_phz_m = _phz_raw_m if sol_valid_m else float('nan')
        _solver_m = s_for_model_m if sol_valid_m else None
        _raw_mcm = (
            self._mc_p_last.get(selected)
            if (self._intercept_mc_trail_by_prob or self._intercept_mc_engagement)
            else None
        )
        _mc_viz_m = _raw_mcm if (_raw_mcm is not None and math.isfinite(_raw_mcm)) else None
        self._publish_intercept_markers(
            selected, iid_index_m, ix, iy, iz,
            _vis_phx_m, _vis_phy_m, _vis_phz_m,
            sol_valid_m,
            t_go=_t_hit_m if sol_valid_m else None,
            solver_closing_m_s=_solver_m,
            v_cmd_m_s=v_cmd_m,
            mc_hit_prob=_mc_viz_m,
        )
        self._publish_intercept_math_markers(
            selected, iid_index_m, ix, iy, iz, tx, ty, tz, feas_m, v_cmd=v_cmd_m,
        )
        self._maybe_feasibility_debug_log(
            feas=feas_m,
            v_cmd=v_cmd_m,
            distance=dist,
            layer=layer_for_log,
            iid=selected,
        )
        self._maybe_feasibility_warnings(
            feas=feas_m,
            guidance_sol_ok=sol_valid_m,
            guidance_t_go=_t_hit_m if sol_valid_m else None,
            v_cmd=v_cmd_m,
            strike_engaged=strike_ok_eff,
            iid=selected,
        )

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
        v_tx: float,
        v_ty: float,
        v_tz: float,
        tlabel: str,
        iid: str,
        layer_for_log: str,
        *,
        strike_ok: bool,
        target_prev: tuple[float, float, float] | None = None,
    ) -> bool:
        if self._hits_multi.get(tlabel):
            return False
        p_sel = self._inter_pos.get(iid)
        if p_sel is None:
            return False
        ix, iy, iz = float(p_sel.x), float(p_sel.y), float(p_sel.z)
        feas_hit = classify_intercept_feasibility_for_viz(
            tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz,
            v_i_max=self._interceptor_max_speed,
            t_min=self._t_hit_min,
            t_max=self._t_hit_max,
        )
        self._ensure_latch_on_first_hit(tlabel, iid, feas_hit.feasible)
        mc_ok_h = self._mc_engage_ok(iid)
        strike_ok_eff = strike_ok and feas_hit.feasible and mc_ok_h
        eff_hit_mu = self._hit_gate_distance_m()
        dist_to_target = _norm(tx - ix, ty - iy, tz - iz)
        d_seg = float('inf')
        ipr = self._prev_inter_pos.get(iid)
        if target_prev is not None and ipr is not None and math.isfinite(dist_to_target):
            d_seg = _closest_dist_segments_3d(ipr, (ix, iy, iz), target_prev, (tx, ty, tz))
        dist_for_hit = float(min(dist_to_target, d_seg)) if math.isfinite(d_seg) else dist_to_target
        range_plausible = self._hit_range_plausible_for_tick(iid, dist_to_target)
        if (
            not range_plausible
            and target_prev is not None
            and ipr is not None
            and math.isfinite(d_seg)
            and d_seg <= eff_hit_mu * 1.25
        ):
            range_plausible = True
        if not range_plausible:
            return False
        self._record_miss_distance(dist_for_hit)
        air_ok = self._hit_min_tz is None or tz >= self._hit_min_tz
        start = self._inter_start_pos.get(iid)
        travel = _norm(ix - start[0], iy - start[1], iz - start[2]) if start is not None else 0.0
        launched_ok = (iz >= self._hit_min_iz) and (travel >= self._hit_min_travel)
        if dist_for_hit < eff_hit_mu and air_ok and strike_ok_eff and launched_ok:
            self._hits_multi[tlabel] = True
            extra = f' layer={layer_for_log}' if layer_for_log else ''
            mm = f'{self._min_miss_distance:.4f} m' if self._min_miss_distance < float("inf") else 'n/a'
            print(
                f'[HIT] {tlabel} by {iid}{extra}  min_miss={mm}  hit_threshold = {eff_hit_mu:.4f} m '
                f'(nominal={self._hit_thresh:.4f} debug_mode={self._dbg_predictive}) '
                f'interceptor_pos=({ix:.2f},{iy:.2f},{iz:.2f}) travel={travel:.2f}m '
                f'target_pos=({tx:.2f},{ty:.2f},{tz:.2f}) dist_pt={dist_to_target:.3f}m dist_swept={d_seg:.3f}m',
                flush=True,
            )
            elapsed = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
            f0 = self._feasible_at_engagement_start_by_pair.get((tlabel, iid))
            f0s = 'n/a' if f0 is None else ('True' if f0 else 'False')
            f1s = 'True' if feas_hit.feasible else 'False'
            print(
                f'[RESULT]\n'
                f'success=True\n'
                f'miss_distance={mm}\n'
                f'intercept_time={elapsed:.3f}s\n'
                f'feasible_at_start={f0s}\n'
                f'feasible_at_hit={f1s}',
                flush=True,
            )
            hit_sub = f'{tlabel} ← {iid}' + (f' ({layer_for_log})' if layer_for_log else '')
            self._start_hit_visual(tx, ty, tz, ix, iy, iz, hit_sub)
            self._publish_stop_signal(tlabel)
            self._schedule_stop_signal_repeats(tlabel)
            self._log_active_targets_remaining()
            if self._pause_gz_on_hit:
                self._pause_gazebo_world()
            return True
        if (
            dist_for_hit < eff_hit_mu
            and air_ok
            and strike_ok_eff
            and (not launched_ok)
        ):
            kpair = (tlabel, iid)
            if kpair not in self._multi_hit_safety_logged:
                self._multi_hit_safety_logged.add(kpair)
                print(
                    f'[HIT_BLOCKED_SAFETY] multi {tlabel}/{iid} dist={dist_for_hit:.2f}m < eff_hit={eff_hit_mu:.2f} '
                    f'but not launched: iz={iz:.2f}m travel={travel:.2f}m',
                    flush=True,
                )
        elif dist_for_hit < eff_hit_mu and air_ok and (not strike_ok_eff):
            self._maybe_log_hit_gate_block(
                tag=f'{tlabel}:{iid}',
                dist_to_target=dist_for_hit,
                eff_hit=eff_hit_mu,
                air_ok=air_ok,
                strike_ok=strike_ok,
                strike_ok_eff=strike_ok_eff,
                feas_ok=feas_hit.feasible,
                mc_ok=mc_ok_h,
                launched_ok=launched_ok,
                layer=layer_for_log or 'multi',
            )
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
            self._publish_all({i: zero for i in self._ids}, immediate=True)
            return
        if not any(self._targets.get(l) is not None for l in self._multi_labels):
            self._publish_assignment_strings({})
            self._publish_all({i: zero for i in self._ids})
            return

        if self._feasibility_based and not self._classification_allows_engagement():
            self._publish_assignment_strings({})
            self._publish_all({i: zero for i in self._ids})
            return

        v_by: dict[str, tuple[float, float, float]] = {}
        tgt_prev_by: dict[str, tuple[float, float, float] | None] = {}
        for lab in self._multi_labels:
            if self._hits_multi.get(lab):
                continue
            pt = self._targets.get(lab)
            if pt is None:
                continue
            tx, ty, tz = float(pt.x), float(pt.y), float(pt.z)
            tgt_prev_by[lab] = self._hit_snap_target_prev_multi.get(lab)
            self._hit_snap_target_prev_multi[lab] = (tx, ty, tz)
            v_by[lab] = self._estimate_target_vel_multi(lab, tx, ty, tz)

        # Deepest-intrusion target (min d_threat) drives layer display.
        layer_pub = 'engage'
        d_layer: float | None = None
        for lab in self._multi_labels:
            if self._hits_multi.get(lab):
                continue
            pt = self._targets.get(lab)
            if pt is None:
                continue
            d0 = self._dist_threat(float(pt.x), float(pt.y), float(pt.z))
            if d_layer is None or d0 < d_layer:
                d_layer = d0
                layer_pub = self._layer_from_dist(d0)
        self._pub_layer.publish(String(data=layer_pub))

        active_t: list[str] = []
        now_gate = self.get_clock().now()
        for lab in self._multi_labels:
            if self._hits_multi.get(lab):
                continue
            pt = self._targets.get(lab)
            if pt is None:
                continue
            tx, ty, tz = float(pt.x), float(pt.y), float(pt.z)
            self._maybe_arm_detection_time(lab, tx, ty, tz)
            if not self.is_detected(tx, ty, tz):
                continue
            if not self.is_tracking_ready(lab, now_gate):
                continue
            d_threat = self._dist_threat(tx, ty, tz)
            if self._dome_enabled and self._dome_outside_effective_multi(lab, d_threat):
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
                        ok = False
                        tti = None
                        if self._feasibility_based:
                            tti = self._feasibility_tti_for_assignment(
                                tx, ty, tz, vx, vy, vz, ix, iy, iz,
                            )
                            ok = tti is not None
                        else:
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
        self._refresh_mc_probs_multi(assign, v_by)

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
            strike_ok_m = self._strike_ok_for_dome_policy(d_th, layer)

            if self._dome_enabled:
                if self._dome_outside_effective_multi(tlabel, d_th):
                    continue
                may_ram = self._in_strike_zone(d_th) or self._feasibility_based
                if may_ram:
                    if self._multi_try_hit(
                        tx, ty, tz, vx, vy, vz, tlabel, iid, layer,
                        strike_ok=strike_ok_m,
                        target_prev=tgt_prev_by.get(tlabel),
                    ):
                        continue
                    p_sel = self._inter_pos.get(iid)
                    if p_sel is None:
                        continue
                    ix, iy, iz = float(p_sel.x), float(p_sel.y), float(p_sel.z)
                    dist = _norm(tx - ix, ty - iy, tz - iz)
                    out_map[iid] = self._multi_ram_velocity_cmd(
                        tx, ty, tz, vx, vy, vz, iid, strike_ok_m, ix, iy, iz, dist,
                        layer_for_log=layer,
                        target_label=tlabel,
                    )
                elif d_th > self._r_mid:
                    continue
                elif d_th > self._r_inner:
                    continue
                else:
                    continue
            else:
                if self._multi_try_hit(
                    tx, ty, tz, vx, vy, vz, tlabel, iid, layer,
                    strike_ok=True,
                    target_prev=tgt_prev_by.get(tlabel),
                ):
                    continue
                p_sel = self._inter_pos.get(iid)
                if p_sel is None:
                    continue
                ix, iy, iz = float(p_sel.x), float(p_sel.y), float(p_sel.z)
                dist = _norm(tx - ix, ty - iy, tz - iz)
                out_map[iid] = self._multi_ram_velocity_cmd(
                    tx, ty, tz, vx, vy, vz, iid, strike_ok_m, ix, iy, iz, dist,
                    layer_for_log=layer,
                    target_label=tlabel,
                )

        self._publish_all(out_map)
        self._maybe_emit_miss_distance_log_multi()
        self._prune_feas_pair_maps_multi(assign)

    def _on_control(self) -> None:
        # Compute actual dt from wall/sim clock so accel limit is rate-independent.
        _now_ctrl = self.get_clock().now()
        if self._last_control_time is not None:
            _dt = (_now_ctrl - self._last_control_time).nanoseconds * 1e-9
            if 1e-3 < _dt < 1.0:   # sanity clamp: ignore stale or sub-ms ticks
                self._control_dt = _dt
        self._last_control_time = _now_ctrl

        self._cuas_record_trails()
        try:
            if self._multi_enabled:
                self._on_control_multi()
            else:
                zero = Vector3()
                if self._target is None:
                    self._publish_all({i: zero for i in self._ids})
                    return
                
                # After impact: do not run selection or guidance; keep target stopped and interceptors frozen.
                # Do not re-publish /target/stop every tick — that spams DDS and can confuse tooling; HIT path
                # already published 3× True with reliable QoS.
                if self._hit:
                    self._publish_all({i: zero for i in self._ids}, immediate=True)
                    # Deselect all so launched interceptors return to ground (idle/standby).
                    self._pub_selected.publish(String(data=''))
                    return
                
                tx, ty, tz = float(self._target.x), float(self._target.y), float(self._target.z)
                tgt_prev_hit = self._hit_snap_target_prev
                self._hit_snap_target_prev = (tx, ty, tz)
                v_tx, v_ty, v_tz = self._estimate_target_vel(tx, ty, tz)
                
                self._maybe_arm_detection_time(None, tx, ty, tz)
                if not self.is_detected(tx, ty, tz) or not self.is_tracking_ready(None, _now_ctrl):
                    self._publish_all({i: zero for i in self._ids})
                    self._pub_selected.publish(String(data=''))
                    return
                
                d_th = self._dist_threat(tx, ty, tz)
                layer = self._layer_from_dist(d_th)
                self._pub_layer.publish(String(data=layer))
                if layer != self._last_layer:
                    print(f'[LAYER] {layer}', flush=True)
                self._last_layer = layer
                
                if self._dome_enabled:
                    if self._dome_outside_effective(d_th):
                        if self._reset_lock_outside:
                            self._clear_assignments()
                        self._publish_all({i: zero for i in self._ids})
                        self._pub_selected.publish(String(data=''))
                        return
                
                    # strike_ok = layered HIT policy only (not engagement trigger).
                    strike_ok = self._strike_ok_for_dome_policy(d_th, layer)
                
                    if self._feasibility_based:
                        if not self._classification_allows_engagement():
                            self._publish_all({i: zero for i in self._ids})
                            self._pub_selected.publish(String(data=''))
                            return
                        rows = self._build_tti_rows(
                            tx, ty, tz, v_tx, v_ty, v_tz,
                            intercept_closing_m_s=self._interceptor_max_speed,
                        )
                        if self._locked_selected_id is not None:
                            p_l = self._inter_pos.get(self._locked_selected_id)
                            if p_l is None:
                                self._locked_selected_id = None
                                self._current_selected_id = None
                            else:
                                lx, ly, lz = float(p_l.x), float(p_l.y), float(p_l.z)
                                ok_l, _, _ = self._intercept_feasibility_triple(
                                    tx, ty, tz, v_tx, v_ty, v_tz, lx, ly, lz,
                                )
                                if not ok_l:
                                    self._locked_selected_id = None
                                    self._current_selected_id = None
                        if self._locked_selected_id is None:
                            best_i, any_feas, best_t, best_s = self._pick_best_feasible_interceptor_single(
                                tx, ty, tz, v_tx, v_ty, v_tz,
                            )
                            self._maybe_log_feasibility_single(
                                feasible=any_feas,
                                t_int=best_t,
                                s_req=best_s,
                                selected=best_i,
                                d_th=d_th,
                            )
                            if not any_feas or best_i is None:
                                self._clear_assignments()
                                self._publish_all({i: zero for i in self._ids})
                                self._pub_selected.publish(String(data=''))
                                return
                            self._locked_selected_id = best_i
                            self._current_selected_id = best_i
                        else:
                            p_sel = self._inter_pos.get(self._locked_selected_id)
                            if p_sel is not None:
                                ix, iy, iz = float(p_sel.x), float(p_sel.y), float(p_sel.z)
                                ok_c, t_c, s_c = self._intercept_feasibility_triple(
                                    tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz,
                                )
                                self._maybe_log_feasibility_single(
                                    feasible=ok_c,
                                    t_int=t_c,
                                    s_req=s_c,
                                    selected=self._locked_selected_id,
                                    d_th=d_th,
                                )
                        selected = self._locked_selected_id
                        self._pub_selected.publish(String(data=selected if selected is not None else ''))
                        self._maybe_selection_log(rows, selected, tx, ty, tz, layer)
                        self._maybe_print_selection_algorithm_verbose(
                            'feasibility_based' if not self._engagement_layer else f'feas_layer_{self._engagement_layer}',
                            tx, ty, tz, v_tx, v_ty, v_tz, rows, selected, layer, d_th,
                            trace_closing_m_s=self._interceptor_max_speed,
                        )
                        if selected is None:
                            self._publish_all({i: zero for i in self._ids})
                            return
                        self._run_ram_guidance(
                            tx, ty, tz, v_tx, v_ty, v_tz, selected,
                            strike_ok=strike_ok, layer_for_log=layer, target_prev=tgt_prev_hit,
                        )
                        return
                
                    # Legacy zone-triggered guidance (guidance_start_layer gates flying).
                    _layer_order = {'outside': 0, 'detect': 1, 'select': 2, 'engage': 3}
                    cur_order = _layer_order.get(layer, -1)
                    if self._engagement_layer:
                        start_order = _layer_order.get(self._guidance_start_layer, 2)
                        if cur_order < start_order:
                            if d_th > self._r_mid:
                                self._clear_assignments()
                            self._publish_all({i: zero for i in self._ids})
                            self._pub_selected.publish(String(data=''))
                            return
                    elif not self._in_strike_zone(d_th):
                        if d_th > self._r_mid:
                            self._clear_assignments()
                        self._publish_all({i: zero for i in self._ids})
                        self._pub_selected.publish(String(data=''))
                        return
                
                    rows = self._build_tti_rows(tx, ty, tz, v_tx, v_ty, v_tz)
                    if self._locked_selected_id is None:
                        feasible = [
                            (tti, iid) for iid, ok, tti in rows
                            if ok and tti is not None
                        ]
                        if feasible:
                            best_iid = min(feasible, key=lambda x: (x[0], self._ids.index(x[1])))[1]
                            self._locked_selected_id = best_iid
                            self._current_selected_id = best_iid
                        else:
                            nid = self._nearest_interceptor_id(tx, ty, tz)
                            if nid is not None:
                                self._locked_selected_id = nid
                                self._current_selected_id = nid
                
                    selected = self._locked_selected_id
                    self._pub_selected.publish(String(data=selected if selected is not None else ''))
                    self._maybe_selection_log(rows, selected, tx, ty, tz, layer)
                    self._maybe_print_selection_algorithm_verbose(
                        'strike_shell' if not self._engagement_layer else f'layer_{self._engagement_layer}',
                        tx, ty, tz, v_tx, v_ty, v_tz, rows, selected, layer, d_th,
                    )
                    if selected is None:
                        self._publish_all({i: zero for i in self._ids})
                        return
                    self._run_ram_guidance(
                        tx, ty, tz, v_tx, v_ty, v_tz, selected,
                        strike_ok=strike_ok, layer_for_log=layer, target_prev=tgt_prev_hit,
                    )
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
                
                self._run_ram_guidance(
                    tx, ty, tz, v_tx, v_ty, v_tz, selected,
                    strike_ok=True, layer_for_log=layer, target_prev=tgt_prev_hit,
                )
                
        finally:
            self._cuas_publish_trail_markers()

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
        *,
        trace_closing_m_s: float | None = None,
    ) -> None:
        if not self._sel_algo_verbose:
            return
        now = self.get_clock().now()
        if self._last_algo_log is not None:
            if (now - self._last_algo_log).nanoseconds * 1e-9 < self._sel_algo_period:
                return
        self._last_algo_log = now

        s_trace = self._closing if trace_closing_m_s is None else float(trace_closing_m_s)
        tol_s = (
            'tol_intercept = max(0.12, 5e-4 * max(|r(t)|, s_i*t, 1))  '
            '— accept candidate if | |r(t)| - s_i*t | <= tol_intercept'
        )
        lines: list[str] = [
            '',
            f'>>>>>>>>>> SELECTION ALGORITHM [{tag}] <<<<<<<<<<',
            f'Target position P_T = ({tx:.6f}, {ty:.6f}, {tz:.6f})  (smoothed) velocity v_T = ({v_tx:.6f}, {v_ty:.6f}, {v_tz:.6f})',
            f'  EMA: v <- alpha * (P_T-P_prev)/dt + (1-alpha)*v_prev   alpha = {self._tgt_vel_smooth_a}',
            f'Closing speed s_i = {s_trace:.6f} m/s  |  TTI in [{self._t_hit_min}, {self._t_hit_max}] s  |  {tol_s}',
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
                tx, ty, tz, v_tx, v_ty, v_tz, ix, iy, iz, s_trace, trace=tr,
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
        *,
        solver_closing_m_s: float | None = None,
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
        # ── Speed breakdown (matches _command_speed formula) ─────────────────
        if math.isfinite(dist) and dist > 0.0:
            if t_hit is not None and math.isfinite(t_hit) and t_hit > 0.0:
                _tgo_eff = max(float(t_hit), self._speed_tgo_min)
                _v_desired = self._speed_k1 * dist + self._speed_k2 * dist / _tgo_eff
                _tgo_s = f'{t_hit:.3f}'
            else:
                _v_desired = self._speed_k1 * dist
                _tgo_s = 'n/a'
            _v_cmd = _norm(vx, vy, vz)
            _v_desired_clamped = min(self._vmax, max(self._speed_vmin, _v_desired))
            speed_s = (
                f'dist={dist:.3f} t_go={_tgo_s} '
                f'v_desired={_v_desired:.3f} v_clamped={_v_desired_clamped:.3f} v_cmd={_v_cmd:.3f}'
            )
        else:
            speed_s = f'dist={dist:.3f} (invalid)'

        if solver_closing_m_s is not None and math.isfinite(solver_closing_m_s):
            si_line = (
                f'intercept_model: s_i={solver_closing_m_s:.4f} m/s  '
                f'P_hit=P_T+v_T*t_go  (quadratic |r0+v_T t|^2=(s_i t)^2)'
            )
        else:
            si_line = 'intercept_model: s_i=n/a (no geometry solve this tick)'

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
                f'alignment={align_s}\n'
                f'{si_line}\n'
                f'[SPEED] {speed_s}',
                flush=True,
            )
        else:
            print(
                f'[interception] sel={selected} mode={mode} pn={pn_s} dist={dist:.2f} m | t_hit={ts} s p_hit={phs} | '
                f'Vc={vc:.2f} m/s | target=({tx:.2f},{ty:.2f},{tz:.2f}) inter=({ix:.2f},{iy:.2f},{iz:.2f}) | '
                f'cmd_vel=({vx:.2f},{vy:.2f},{vz:.2f})\n'
                f'{si_line}\n'
                f'[SPEED] {speed_s}',
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
