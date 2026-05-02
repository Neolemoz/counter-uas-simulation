"""Regression test: acceleration-limit ramp time at km-scale.

Why this exists
---------------
``InterceptionLogicNode._accel_limit_velocity`` clamps ``|dv|`` to ``max_accel * dt`` per
control cycle.  At the lab-toy default (3 m/s² @ 20 Hz) the per-cycle delta-v is 0.15 m/s,
so reaching the km-scale interceptor ``v_max`` (≈ 78 m/s) takes ~26 s — long enough that
the operator perceives the interceptor as "not engaging" while the attacker (38 m/s) closes.

This test mirrors the live-node maths exactly and asserts:
1. The lab-toy default would still produce a > 20 s ramp at km scale (proves the bug).
2. The km-scale config (30 m/s²) cuts the ramp under 3 s.
"""

from __future__ import annotations

import math


def _norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def _accel_limited_step(
    prev: tuple[float, float, float],
    desired: tuple[float, float, float],
    max_accel: float,
    dt: float,
) -> tuple[float, float, float]:
    """Replicates ``InterceptionLogicNode._accel_limit_velocity`` Stage 2.

    Stage 1 (turn-rate) is irrelevant for a head-on ramp from rest, so we skip it.
    """
    pvx, pvy, pvz = prev
    nvx, nvy, nvz = desired
    dvx, dvy, dvz = nvx - pvx, nvy - pvy, nvz - pvz
    dv = _norm3(dvx, dvy, dvz)
    max_dv = max_accel * dt
    if dv > max_dv and dv > 1e-9:
        s = max_dv / dv
        dvx, dvy, dvz = dvx * s, dvy * s, dvz * s
    return (pvx + dvx, pvy + dvy, pvz + dvz)


def _cycles_to_reach_speed(
    target_speed: float,
    max_accel: float,
    dt: float,
    *,
    eps: float = 1e-2,
) -> int:
    cur = (0.0, 0.0, 0.0)
    desired = (target_speed, 0.0, 0.0)
    for k in range(1, 100_000):
        cur = _accel_limited_step(cur, desired, max_accel, dt)
        if _norm3(*cur) >= target_speed - eps:
            return k
    raise AssertionError("did not converge within 100k cycles")


def test_lab_default_accel_too_slow_at_km_scale() -> None:
    """Demonstrates the bug: 3 m/s² @ 20 Hz takes > 20 s to reach 78 m/s."""
    cycles = _cycles_to_reach_speed(target_speed=78.0, max_accel=3.0, dt=0.05)
    seconds = cycles * 0.05
    assert seconds > 20.0, f"expected > 20 s ramp, got {seconds:.2f}"


def test_km_scale_accel_completes_under_three_seconds() -> None:
    """The km-scale config (30 m/s²) cuts the ramp under 3 s — interceptor visibly engages."""
    cycles = _cycles_to_reach_speed(target_speed=78.0, max_accel=30.0, dt=0.05)
    seconds = cycles * 0.05
    assert seconds < 3.0, f"expected < 3 s ramp at 30 m/s², got {seconds:.2f}"


def test_first_cycle_dv_is_max_accel_times_dt() -> None:
    """Smoke check: per-cycle delta-v equals max_accel * dt for a from-rest head-on ramp."""
    cur = (0.0, 0.0, 0.0)
    desired = (78.0, 0.0, 0.0)
    cur = _accel_limited_step(cur, desired, max_accel=30.0, dt=0.05)
    assert abs(_norm3(*cur) - 30.0 * 0.05) < 1e-6
