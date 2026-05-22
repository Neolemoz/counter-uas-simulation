from __future__ import annotations

import numpy as np

from core.utils import normalize


def compute_naive_direction(
    p_target: np.ndarray, p_interceptor: np.ndarray
) -> np.ndarray:
    return normalize(p_target - p_interceptor)
