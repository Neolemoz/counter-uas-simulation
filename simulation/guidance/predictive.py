from __future__ import annotations

import numpy as np

from core.intercept import compute_intercept


def compute_predictive_direction(
    p_t: np.ndarray,
    v_t: np.ndarray,
    p_i: np.ndarray,
    s_i: float,
) -> np.ndarray | None:
    result = compute_intercept(p_t, v_t, p_i, s_i)
    if result is None:
        return None
    _, _, u = result
    return u
