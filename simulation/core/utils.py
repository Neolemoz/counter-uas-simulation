from __future__ import annotations

import numpy as np


def normalize(vector: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(vector))
    if n < 1e-15:
        raise ValueError("Cannot normalize zero-length vector")
    return vector / n


def generate_trajectories(
    p_t0: np.ndarray,
    v_t: np.ndarray,
    p_i0: np.ndarray,
    s_i: float,
    t_hit: float,
    num_samples: int = 400,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    t = np.linspace(0.0, float(t_hit), num_samples)
    target = p_t0 + np.outer(t, v_t)
    p_hit = p_t0 + v_t * t_hit
    delta = p_hit - p_i0
    u = normalize(delta)
    interceptor = p_i0 + np.outer(t, u * s_i)
    return t, target, interceptor
