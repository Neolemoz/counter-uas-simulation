"""Interceptor kinematics: turn-rate limit, magnitude clamp (numpy only)."""

from __future__ import annotations

import numpy as np


def limit_magnitude(vec: np.ndarray, max_val: float) -> np.ndarray:
    n = float(np.linalg.norm(vec))
    if n < 1e-15 or n <= max_val:
        return vec.astype(float).copy()
    return vec * (max_val / n)


def _normalize(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-15:
        return np.array([1.0, 0.0, 0.0])
    return v / n


def _rodrigues(v: np.ndarray, k: np.ndarray, theta: float) -> np.ndarray:
    k = _normalize(k)
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    return v * cos_t + np.cross(k, v) * sin_t + k * np.dot(k, v) * (1.0 - cos_t)


def _rotate_toward(current_dir: np.ndarray, desired_dir: np.ndarray, max_angle: float) -> np.ndarray:
    c = _normalize(current_dir)
    d = _normalize(desired_dir)
    cos_t = float(np.clip(np.dot(c, d), -1.0, 1.0))
    angle = float(np.arccos(cos_t))
    alpha = min(angle, max_angle)
    if alpha < 1e-15:
        return c

    cross_cd = np.cross(c, d)
    nk = float(np.linalg.norm(cross_cd))

    if nk < 1e-12:
        if cos_t > -1.0 + 1e-12:
            return c
        orth = np.array([1.0, 0.0, 0.0])
        if abs(c[0]) > 0.9:
            orth = np.array([0.0, 1.0, 0.0])
        k = _normalize(np.cross(c, orth))
        out = _rodrigues(c, k, alpha)
        return _normalize(out)

    k = cross_cd / nk
    out = _rodrigues(c, k, alpha)
    return _normalize(out)


def update_velocity(
    v: np.ndarray,
    desired_dir: np.ndarray,
    speed: float,
    max_turn_rate: float,
    dt: float,
) -> np.ndarray:
    nv = float(np.linalg.norm(v))
    if nv < 1e-15:
        current_dir = _normalize(desired_dir)
    else:
        current_dir = v / nv

    d = _normalize(desired_dir)
    max_angle = max_turn_rate * dt
    new_dir = _rotate_toward(current_dir, d, max_angle)
    return new_dir * speed


def update_velocity_from_pn_lateral(
    v: np.ndarray,
    a_cmd: np.ndarray,
    speed: float,
    max_turn_rate: float,
    dt: float,
) -> np.ndarray:
    """Apply PN acceleration as lateral steering, then match ``update_velocity`` speed / turn cap.

    **PN acceleration:** ``a_cmd`` is treated as a commanded *lateral* acceleration in the sense
    that only the component perpendicular to the current velocity is integrated. The component
    of ``a_cmd`` parallel to ``v`` would change speed along the flight path; we drop it so that
    speed control stays in this single function: first lateral Euler step, then fixed ``speed``.

    **Speed normalization:** After ``v_euler = v + a_lat * dt``, the *direction* we want before
    the actuator limit is ``v_euler / ||v_euler||``. The output velocity always has magnitude
    ``speed`` (same contract as ``update_velocity``).

    **Turn-rate limit:** We do not jump instantly to that direction. ``update_velocity`` rotates
    the current heading toward ``v_euler`` by at most ``max_turn_rate * dt``, then scales to
    ``speed`` — identical turn-rate semantics as naive / predictive.
    """
    nv = float(np.linalg.norm(v))
    if nv < 1e-15:
        na = float(np.linalg.norm(a_cmd))
        if na < 1e-15:
            return np.array([1.0, 0.0, 0.0], dtype=float) * float(speed)
        return _normalize(a_cmd) * float(speed)

    v_hat = v / nv
    along = float(np.dot(a_cmd, v_hat))
    a_lat = a_cmd - along * v_hat
    v_euler = v + a_lat * float(dt)
    ne = float(np.linalg.norm(v_euler))
    if ne < 1e-15:
        na = float(np.linalg.norm(a_lat))
        if na < 1e-15:
            return v_hat * float(speed)
        desired = a_lat
    else:
        desired = v_euler
    return update_velocity(v, desired, speed, max_turn_rate, dt)
