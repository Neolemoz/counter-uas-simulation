"""Unit tests for the extracted guidance primitives (Phase 2).

These exercise the geometric solver, delay compensation, and post-saturation speed alignment
without spinning up ROS — they are the mathematical core that the live node calls every cycle.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_guidance_lib():  # noqa: ANN201
    path = _REPO_ROOT / 'src' / 'gazebo_target_sim' / 'gazebo_target_sim' / 'guidance_lib.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('guidance_lib', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def test_intercept_head_on_constant_velocity() -> None:
    lib = _load_guidance_lib()
    # Target at +x, 100 m away, closing at 10 m/s along -x; interceptor at origin, 20 m/s.
    t = lib.solve_intercept_time(100.0, 0.0, 0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 20.0)
    # Closing speed = 30 m/s on a head-on engagement → t = 100/30.
    assert t is not None
    assert t == pytest.approx(100.0 / 30.0, rel=1e-6)


def test_intercept_no_solution_when_target_too_fast() -> None:
    lib = _load_guidance_lib()
    # Target faster than interceptor and moving directly away: no intercept.
    t = lib.solve_intercept_time(0.0, 0.0, 0.0, 30.0, 0.0, 0.0, -100.0, 0.0, 0.0, 10.0)
    assert t is None


def test_compute_intercept_returns_phit_and_unit() -> None:
    lib = _load_guidance_lib()
    sol = lib.compute_intercept(100.0, 0.0, 0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 20.0)
    assert sol is not None
    t, phx, phy, phz, ux, uy, uz = sol
    # P_hit = P_T + v_T * t  → on the +X axis, between 0 and 100.
    assert phy == pytest.approx(0.0, abs=1e-9)
    assert phz == pytest.approx(0.0, abs=1e-9)
    assert phx == pytest.approx(100.0 - 10.0 * t, rel=1e-9)
    assert ux == pytest.approx(1.0, rel=1e-9)
    assert uy == pytest.approx(0.0, abs=1e-9)
    assert uz == pytest.approx(0.0, abs=1e-9)


def test_compensate_target_for_delay_extrapolates_cv() -> None:
    lib = _load_guidance_lib()
    p = lib.compensate_target_for_delay(0.0, 0.0, 100.0, 5.0, 0.0, -1.0, 0.2)
    assert p == pytest.approx((1.0, 0.0, 99.8), rel=1e-9)


def test_compensate_target_for_delay_clamps_negative_tau() -> None:
    lib = _load_guidance_lib()
    p = lib.compensate_target_for_delay(1.0, 2.0, 3.0, 5.0, 5.0, 5.0, -0.5)
    assert p == (1.0, 2.0, 3.0)


def test_align_speed_after_saturation_matches_solver_assumption() -> None:
    lib = _load_guidance_lib()
    # P_hit at distance 60 m, t_go = 4 s, v_max = 50 m/s.
    # Required speed = 60/4 = 15 m/s, well under v_max → s = 15.
    vx, vy, vz, s = lib.align_speed_after_saturation(
        ux=1.0, uy=0.0, uz=0.0,
        p_ix=0.0, p_iy=0.0, p_iz=0.0,
        p_hit_x=60.0, p_hit_y=0.0, p_hit_z=0.0,
        t_go=4.0, v_max=50.0,
    )
    assert s == pytest.approx(15.0, rel=1e-9)
    assert (vx, vy, vz) == pytest.approx((15.0, 0.0, 0.0), rel=1e-9)


def test_align_speed_after_saturation_clips_to_v_max() -> None:
    lib = _load_guidance_lib()
    # Required > v_max → clipped to v_max.
    vx, vy, vz, s = lib.align_speed_after_saturation(
        ux=0.0, uy=1.0, uz=0.0,
        p_ix=0.0, p_iy=0.0, p_iz=0.0,
        p_hit_x=0.0, p_hit_y=200.0, p_hit_z=0.0,
        t_go=1.0, v_max=50.0,
    )
    assert s == pytest.approx(50.0, rel=1e-9)
    assert vy == pytest.approx(50.0, rel=1e-9)


def test_filter_t_go_is_low_pass() -> None:
    lib = _load_guidance_lib()
    # First sample seeds the filter.
    assert lib.filter_t_go(None, 4.0, 0.5) == pytest.approx(4.0)
    # 50% blend toward a new value.
    assert lib.filter_t_go(4.0, 6.0, 0.5) == pytest.approx(5.0)
    # alpha = 0 → keeps previous value (full smoothing).
    assert lib.filter_t_go(4.0, 6.0, 0.0) == pytest.approx(4.0)
