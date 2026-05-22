#!/usr/bin/env python3
"""Single-shot predictive intercept: closed-form time, straight-line interceptor path."""

from __future__ import annotations

import numpy as np

from core.intercept import compute_intercept
from core.utils import generate_trajectories
from viz.plot3d import plot_intercept_single


def main() -> None:
    p_t0 = np.array([0.0, 0.0, 20.0])
    v_t = np.array([8.0, 3.0, -1.0])
    p_i0 = np.array([-40.0, -20.0, 0.0])
    s_i = 18.0

    result = compute_intercept(p_t0, v_t, p_i0, s_i)
    if result is None:
        print("No feasible interception")
        return

    t_hit, p_hit, u = result
    print(f"Intercept time:   {t_hit:.6f} s")
    print(f"Intercept point:  {p_hit}")
    print(f"Direction vector: {u}")

    _, target_traj, int_traj = generate_trajectories(
        p_t0, v_t, p_i0, s_i, t_hit
    )
    plot_intercept_single(p_t0, p_i0, target_traj, int_traj, p_hit)


if __name__ == "__main__":
    main()
