"""Pure-Python guidance primitives extracted from ``interception_logic_node`` for reuse and tests.

This module deliberately depends only on the stdlib so it can be unit-tested without ROS, and
re-imported by other tools (e.g. offline heatmap exporter, scenario harness).  Functions here
are the **mathematical core** of constant-velocity intercept: they take floats, return floats,
and have no side effects.

Phase 2 adds two pieces that the live node needs to behave correctly under realistic
measurement / actuator conditions:

* :func:`compensate_target_for_delay` — extrapolates target state to the control instant when
  the measurement was produced ``tau`` seconds ago.  Without this, the predictive intercept
  is solving for a target ghost.
* :func:`align_speed_after_saturation` — after the velocity command is clipped by accel and
  speed limits, re-projects it along the same direction so the magnitude actually used by the
  plant matches the closing speed assumed by :func:`solve_intercept_time`.  This closes the
  loop between the kinematic solver and the bounded dynamics.

The geometric solver is re-exported here so external callers can use it without importing
the giant node module; the node still owns its own copy for backwards compatibility.
"""

from __future__ import annotations

import math


def norm(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def unit(dx: float, dy: float, dz: float, eps: float = 1e-9) -> tuple[float, float, float]:
    n = norm(dx, dy, dz)
    if n < eps:
        return (0.0, 0.0, 0.0)
    return (dx / n, dy / n, dz / n)


def solve_intercept_time(
    p_tx: float, p_ty: float, p_tz: float,
    v_tx: float, v_ty: float, v_tz: float,
    p_ix: float, p_iy: float, p_iz: float,
    s_i: float,
) -> float | None:
    """Smallest ``t > 0`` with ``|r0 + v_T t| = s_i t`` (constant-speed CV intercept).

    Same form as ``interception_logic_node._solve_intercept_time`` but trimmed of trace logs;
    returns None when no positive root passes the geometric tolerance check.
    """
    r0x = p_tx - p_ix
    r0y = p_ty - p_iy
    r0z = p_tz - p_iz
    vv = v_tx * v_tx + v_ty * v_ty + v_tz * v_tz
    rv = r0x * v_tx + r0y * v_ty + r0z * v_tz
    rr = r0x * r0x + r0y * r0y + r0z * r0z
    a = vv - s_i * s_i
    b = 2.0 * rv
    c = rr
    eps = 1e-12
    candidates: list[float] = []
    if abs(a) < eps:
        if abs(b) < eps:
            return None
        t_lin = -c / b
        if t_lin > 0.0:
            candidates.append(t_lin)
    else:
        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            return None
        sqrt_d = math.sqrt(disc)
        for t in ((-b - sqrt_d) / (2.0 * a), (-b + sqrt_d) / (2.0 * a)):
            if t > 0.0:
                candidates.append(float(t))
    if not candidates:
        return None
    valid: list[float] = []
    for t in candidates:
        hx = p_tx + v_tx * t - p_ix
        hy = p_ty + v_ty * t - p_iy
        hz = p_tz + v_tz * t - p_iz
        lhs = norm(hx, hy, hz)
        rhs = s_i * t
        tol = max(0.12, 5e-4 * max(lhs, rhs, 1.0))
        if abs(lhs - rhs) <= tol:
            valid.append(t)
    if not valid:
        return None
    return min(valid)


def compensate_target_for_delay(
    p_tx: float, p_ty: float, p_tz: float,
    v_tx: float, v_ty: float, v_tz: float,
    tau_s: float,
) -> tuple[float, float, float]:
    """Constant-velocity extrapolation by ``tau_s`` seconds.

    Use ``tau_s = (t_now - measurement_stamp) + sensor_delay`` so the solver consumes a target
    state at the **control** instant rather than at the (older) measurement instant.  Negative
    or non-finite ``tau_s`` is treated as zero — guidance never extrapolates *backwards*.
    """
    if not math.isfinite(tau_s) or tau_s <= 0.0:
        return (p_tx, p_ty, p_tz)
    return (p_tx + v_tx * tau_s, p_ty + v_ty * tau_s, p_tz + v_tz * tau_s)


def compute_intercept(
    p_tx: float, p_ty: float, p_tz: float,
    v_tx: float, v_ty: float, v_tz: float,
    p_ix: float, p_iy: float, p_iz: float,
    s_i: float,
) -> tuple[float, float, float, float, float, float, float] | None:
    """Returns ``(t, p_hit_xyz, u_xyz)`` or None — same contract as the node-local helper."""
    t = solve_intercept_time(p_tx, p_ty, p_tz, v_tx, v_ty, v_tz, p_ix, p_iy, p_iz, s_i)
    if t is None or not math.isfinite(t):
        return None
    phx = p_tx + v_tx * t
    phy = p_ty + v_ty * t
    phz = p_tz + v_tz * t
    ux, uy, uz = unit(phx - p_ix, phy - p_iy, phz - p_iz)
    if norm(ux, uy, uz) < 1e-9:
        return None
    return (t, phx, phy, phz, ux, uy, uz)


def align_speed_after_saturation(
    ux: float, uy: float, uz: float,
    p_ix: float, p_iy: float, p_iz: float,
    p_hit_x: float, p_hit_y: float, p_hit_z: float,
    t_go: float,
    v_max: float,
    t_go_min: float = 1e-2,
) -> tuple[float, float, float, float]:
    """Choose ``v_cmd = u * s`` with ``s = clamp(|p_hit - p_I| / t_go, 0, v_max)``.

    The CV intercept solver assumes the interceptor flies at the closing speed ``s`` it was
    given.  When the live node first plans with one heuristic speed (``k1*dist``) and only
    later re-derives speed from ``t_go``, the geometry and the executed speed disagree by
    enough margin to bias the miss distance.  Computing the **required** speed from
    ``range_to_p_hit / t_go`` makes the command consistent with the time-of-flight that the
    same-cycle solver returned.

    Returns ``(vx, vy, vz, s)`` where ``s`` is the realised commanded speed magnitude.
    """
    if not math.isfinite(t_go) or t_go <= 0.0:
        s = max(0.0, min(v_max, 0.0))
        return (0.0, 0.0, 0.0, 0.0)
    range_rem = norm(p_hit_x - p_ix, p_hit_y - p_iy, p_hit_z - p_iz)
    s_req = range_rem / max(t_go, t_go_min)
    s = max(0.0, min(v_max, s_req))
    return (ux * s, uy * s, uz * s, s)


def filter_t_go(prev: float | None, raw: float, alpha: float) -> float:
    """Low-pass on the predicted time-to-go to avoid jerk when the BVP root jumps cycle to cycle.

    Without filtering, the desired intercept point ``p_hit = p_T + v_T * t_go`` whips around
    every replan — that is the dominant source of guidance instability under noisy ``v_T``
    even when the geometry is consistent.
    """
    raw = max(0.0, float(raw))
    if prev is None or not math.isfinite(prev):
        return raw
    a = max(0.0, min(1.0, float(alpha)))
    return a * raw + (1.0 - a) * float(prev)


__all__ = [
    'norm',
    'unit',
    'solve_intercept_time',
    'compute_intercept',
    'compensate_target_for_delay',
    'align_speed_after_saturation',
    'filter_t_go',
]
