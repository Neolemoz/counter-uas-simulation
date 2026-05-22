"""Proportional navigation acceleration (3D) — stable pure-PN form."""

from __future__ import annotations

import numpy as np


def compute_pn_acceleration(
    p_t: np.ndarray,
    v_t: np.ndarray,
    p_i: np.ndarray,
    v_i: np.ndarray,
    N: float,
) -> np.ndarray:
    r = p_t - p_i
    v_r = v_t - v_i
    r_norm = float(np.linalg.norm(r))
    if r_norm < 1e-6:
        return np.zeros(3)

    r_hat = r / r_norm
    los_rate = np.cross(r, v_r) / (r_norm**2)
    V_c = -float(np.dot(r_hat, v_r))

    a = N * V_c * np.cross(los_rate, r_hat)

    max_acc = 40.0
    na = float(np.linalg.norm(a))
    if na > max_acc:
        a = a / na * max_acc
    return a
