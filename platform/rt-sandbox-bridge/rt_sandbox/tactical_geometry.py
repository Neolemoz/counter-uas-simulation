"""CV intercept geometry for RT sandbox tactical controller (PLAT-RT-TAC2).

Vendored from gazebo_target_sim.guidance_lib (stdlib only) for bridge isolation.
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
) -> float | None:
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


def compute_intercept(
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
