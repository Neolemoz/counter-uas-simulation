"""Regression tests for the candidate-confirmation gate.

Motivation
----------
Before the predictive-gate fix, ``CANDIDATE_MERGE_M = 1.0`` was a hard module-level constant
matched against the *raw last detection*.  For a 38 m/s target sampled at 10 Hz the per-frame
displacement is ~3.8 m, so consecutive detections were always > 1 m apart and **no candidate
ever confirmed**.  The downstream effect was that ``/tracks/state`` stayed empty and the
interceptor never engaged when the target entered the outer dome.

These tests reproduce that scenario at the dataclass level — no ROS, no Gazebo — so the fix
is locked in by CI.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_tracking_module():  # noqa: ANN201
    path = _REPO_ROOT / 'src' / 'tracking' / 'tracking' / 'tracking_node.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('tracking_node_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    # The module defines an @dataclass with deque field types; the dataclass machinery looks
    # the host module up in sys.modules during class creation, so register before exec.
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _make_candidate(mod, x: float, y: float, z: float):  # noqa: ANN001, ANN201
    cand = mod.Candidate(x=x, y=y, z=z, hit_count=1)
    cand.pos_history.append((x, y, z))
    return cand


def _make_point(mod, x: float, y: float, z: float):  # noqa: ANN001, ANN201
    p = mod.Point()
    p.x, p.y, p.z = x, y, z
    return p


def test_predicted_candidate_xyz_extrapolates_with_history() -> None:
    mod = _load_tracking_module()
    cand = mod.Candidate(x=10.0, y=0.0, z=0.0, hit_count=2)
    cand.pos_history.append((6.2, 0.0, 0.0))
    cand.pos_history.append((10.0, 0.0, 0.0))

    pxyz = mod._predicted_candidate_xyz(cand)
    expected_v = (10.0 - 6.2) / mod.CYCLE_PERIOD_S
    assert abs(pxyz[0] - (10.0 + expected_v * mod.CYCLE_PERIOD_S)) < 1e-9
    assert abs(pxyz[1]) < 1e-9
    assert abs(pxyz[2]) < 1e-9


def test_predicted_candidate_xyz_no_history_falls_back_to_last_position() -> None:
    mod = _load_tracking_module()
    cand = mod.Candidate(x=5.0, y=-2.0, z=1.0, hit_count=1)
    cand.pos_history.append((5.0, -2.0, 1.0))

    pxyz = mod._predicted_candidate_xyz(cand)
    assert pxyz == (5.0, -2.0, 1.0)


def test_legacy_gate_misses_fast_target_at_10hz() -> None:
    """At 38 m/s the legacy 1 m gate fails — this is the original bug."""
    mod = _load_tracking_module()
    # Frame N-1 detection at x = 6.2; candidate position is updated to that.
    cand = _make_candidate(mod, 6.2, 0.0, 0.0)
    cand.pos_history.append((10.0, 0.0, 0.0))  # frame N: candidate has 2 history points

    # New detection at frame N+1 (after another 0.1 s @ 38 m/s).
    new_det = _make_point(mod, 13.8, 0.0, 0.0)

    legacy_d = mod._distance_point_to_candidate(new_det, cand)
    assert legacy_d > 1.0, 'sanity: legacy distance must exceed 1 m gate'


def test_predictive_gate_admits_fast_target_at_10hz() -> None:
    """With predictive matching the residual collapses to ~ measurement noise (≈ 0)."""
    mod = _load_tracking_module()
    # Build a candidate that has seen two frames at the target's CV speed.
    cand = mod.Candidate(x=10.0, y=0.0, z=0.0, hit_count=2)
    cand.pos_history.append((6.2, 0.0, 0.0))
    cand.pos_history.append((10.0, 0.0, 0.0))

    # New detection one cycle later: 10 + 38 * 0.1 = 13.8 m (perfect CV, no noise).
    new_det = _make_point(mod, 13.8, 0.0, 0.0)
    pxyz = mod._predicted_candidate_xyz(cand)
    pred_d = mod._distance_point_to_xyz(new_det, pxyz)
    assert pred_d < 0.1, f'predictive gate residual should be ~0, got {pred_d}'


def test_predictive_gate_tolerates_realistic_noise() -> None:
    """At ~0.5 m fused-position noise, predictive gate still matches at 8 m operational gate."""
    mod = _load_tracking_module()
    cand = mod.Candidate(x=10.0, y=0.0, z=0.0, hit_count=2)
    cand.pos_history.append((6.2, 0.0, 0.0))
    cand.pos_history.append((10.0, 0.0, 0.0))

    # Detection drift simulating measurement noise on top of perfect CV.
    new_det = _make_point(mod, 13.8 + 0.4, 0.3, -0.2)
    pxyz = mod._predicted_candidate_xyz(cand)
    pred_d = mod._distance_point_to_xyz(new_det, pxyz)
    assert pred_d < 8.0, f'predictive gate must still admit noisy match, got {pred_d}'
    assert pred_d < 1.0, f'predictive residual should be ~ noise std, got {pred_d}'
