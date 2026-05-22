#!/usr/bin/env python3
"""
Deterministic replay of target_controller integration (same math as _velocity + p += v*dt).
Sync numbers with: src/gazebo_target_sim/launch/gazebo_target.launch.py

Run:
  python3 scripts/simulate_target_debug_output.py
"""

from __future__ import annotations

import math

# --- sync with gazebo_target.launch.py (TGT_*) ---
START_X, START_Y, START_Z = -10.0, 8.0, 36.0
APPROACH_SPEED = 3.0
DIVE_SPEED = -0.45
RATE_HZ = 10.0
LOG_PERIOD_S = 1.0


def velocity(px: float, py: float) -> tuple[float, float, float]:
    dx, dy = -px, -py
    norm = math.hypot(dx, dy)
    eps = 1e-6
    v_h = float(APPROACH_SPEED)
    v_h_abs = abs(v_h)
    if norm < eps:
        vx, vy = 0.0, 0.0
    else:
        inv = 1.0 / norm
        vx = dx * inv * v_h
        vy = dy * inv * v_h
    vz = float(DIVE_SPEED)
    h = math.hypot(vx, vy)
    if h > 1e-12 and v_h_abs > 1e-12 and h > v_h_abs + 1e-9:
        s = v_h_abs / h
        vx, vy = vx * s, vy * s
    return (vx, vy, vz)


def main() -> None:
    dt = 1.0 / RATE_HZ
    px, py, pz = START_X, START_Y, START_Z
    t = 0.0
    next_log = 0.0
    first = True

    print("=== Simulated [Target Debug] (launch params) ===\n")
    logged_near_origin = 0
    while t < 15.0:
        vx, vy, vz = velocity(px, py)
        px += vx * dt
        py += vy * dt
        pz += vz * dt
        if pz < 0.0:
            pz = 0.0
        t += dt

        should_log = first or (t >= next_log)
        dist_xy = math.hypot(px, py)
        if should_log:
            print("[Target Debug]")
            print(f"pos=({px:.3f},{py:.3f},{pz:.3f})")
            print(f"dist_to_center={dist_xy:.3f}")
            print(f"vel=({vx:.3f},{vy:.3f},{vz:.3f})")
            print()
            first = False
            next_log += LOG_PERIOD_S
            if dist_xy < 0.2:
                logged_near_origin += 1
                if logged_near_origin >= 3:
                    break


if __name__ == "__main__":
    main()
