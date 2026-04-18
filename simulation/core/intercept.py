from __future__ import annotations

import numpy as np

from core.utils import normalize


def solve_intercept_time(
    p_t0: np.ndarray,
    v_t: np.ndarray,
    p_i0: np.ndarray,
    s_i: float,
) -> float | None:
    r0 = p_t0 - p_i0
    vv = float(np.dot(v_t, v_t))
    rv = float(np.dot(r0, v_t))
    rr = float(np.dot(r0, r0))

    a = vv - s_i**2
    b = 2.0 * rv
    c = rr

    eps = 1e-12
    candidates: list[float] = []

    if abs(a) < eps:
        if abs(b) < eps:
            return None
        t = -c / b
        if t > 0:
            candidates.append(t)
    else:
        disc = b * b - 4.0 * a * c
        if disc < 0:
            return None
        sqrt_d = np.sqrt(disc)
        for t in ((-b - sqrt_d) / (2.0 * a), (-b + sqrt_d) / (2.0 * a)):
            if t > 0:
                candidates.append(float(t))

    if not candidates:
        return None

    valid: list[float] = []
    for t in candidates:
        d = (p_t0 + v_t * t) - p_i0
        lhs = float(np.linalg.norm(d))
        rhs = s_i * t
        if abs(lhs - rhs) <= 1e-6 * max(1.0, lhs, rhs):
            valid.append(t)

    if not valid:
        return None
    return min(valid)


def compute_intercept(
    p_t0: np.ndarray,
    v_t: np.ndarray,
    p_i0: np.ndarray,
    s_i: float,
) -> tuple[float, np.ndarray, np.ndarray] | None:
    t = solve_intercept_time(p_t0, v_t, p_i0, s_i)
    if t is None:
        return None

    p_hit = p_t0 + v_t * t
    delta = p_hit - p_i0
    u = normalize(delta)
    return t, p_hit, u
