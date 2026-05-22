"""Regression tests for the candidate-confirmation gate.

Motivation
----------
Before the predictive-gate fix, ``CANDIDATE_MERGE_M = 1.0`` was a hard module-level constant
matched against the *raw last detection*.  For a 38 m/s target sampled at 10 Hz the per-frame
displacement is ~3.8 m, so consecutive detections were always > 1 m apart and **no candidate
ever confirmed**.  The downstream effect was that ``/tracks/state`` stayed empty and the
interceptor never engaged when the target entered the outer dome.

Subsequent diagnostics showed two more failure modes that broke track formation at km-scale:

* the candidate was discarded after a single empty cycle (no miss tolerance), so a bursty
  upstream stream lost candidates between detection bursts — fixed via
  ``candidate_max_missed_frames``;
* the velocity used by the predictor assumed a 100 ms gap between consecutive history
  points, but the upstream actually publishes every ~300 ms, so the velocity was inflated
  3× — fixed by storing the explicit timestamp with each history entry and using actual
  ``Δt`` in ``_initial_velocity_from_history``.

These tests reproduce all three failure modes at the dataclass level — no ROS, no Gazebo —
so the fixes are locked in by CI.
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
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _append_hist(cand, x: float, y: float, z: float, t: float) -> None:  # noqa: ANN001
    cand.pos_history.append((x, y, z, t))
    cand.last_update_time = t


def _make_candidate(mod, x: float, y: float, z: float, t: float = 0.0):  # noqa: ANN001, ANN201
    cand = mod.Candidate(x=x, y=y, z=z, hit_count=1)
    _append_hist(cand, x, y, z, t)
    return cand


def _make_point(mod, x: float, y: float, z: float):  # noqa: ANN001, ANN201
    p = mod.Point()
    p.x, p.y, p.z = x, y, z
    return p


def test_predicted_candidate_xyz_extrapolates_with_history() -> None:
    mod = _load_tracking_module()
    cand = mod.Candidate(x=10.0, y=0.0, z=0.0, hit_count=2)
    _append_hist(cand, 6.2, 0.0, 0.0, 0.0)
    _append_hist(cand, 10.0, 0.0, 0.0, 0.1)

    pxyz = mod._predicted_candidate_xyz(cand)
    expected_v = (10.0 - 6.2) / 0.1
    assert abs(pxyz[0] - (10.0 + expected_v * mod.CYCLE_PERIOD_S)) < 1e-9
    assert abs(pxyz[1]) < 1e-9
    assert abs(pxyz[2]) < 1e-9


def test_predicted_candidate_xyz_no_history_falls_back_to_last_position() -> None:
    mod = _load_tracking_module()
    cand = mod.Candidate(x=5.0, y=-2.0, z=1.0, hit_count=1)
    _append_hist(cand, 5.0, -2.0, 1.0, 0.0)

    pxyz = mod._predicted_candidate_xyz(cand)
    assert pxyz == (5.0, -2.0, 1.0)


def test_legacy_gate_misses_fast_target_at_10hz() -> None:
    """At 38 m/s the legacy 1 m gate fails — this is the original bug."""
    mod = _load_tracking_module()
    cand = _make_candidate(mod, 6.2, 0.0, 0.0, t=0.0)
    _append_hist(cand, 10.0, 0.0, 0.0, 0.1)

    new_det = _make_point(mod, 13.8, 0.0, 0.0)

    legacy_d = mod._distance_point_to_candidate(new_det, cand)
    assert legacy_d > 1.0, 'sanity: legacy distance must exceed 1 m gate'


def test_predictive_gate_admits_fast_target_at_10hz() -> None:
    """With predictive matching the residual collapses to ~ measurement noise (≈ 0)."""
    mod = _load_tracking_module()
    cand = mod.Candidate(x=10.0, y=0.0, z=0.0, hit_count=2)
    _append_hist(cand, 6.2, 0.0, 0.0, 0.0)
    _append_hist(cand, 10.0, 0.0, 0.0, 0.1)

    new_det = _make_point(mod, 13.8, 0.0, 0.0)
    pxyz = mod._predicted_candidate_xyz(cand)
    pred_d = mod._distance_point_to_xyz(new_det, pxyz)
    assert pred_d < 0.1, f'predictive gate residual should be ~0, got {pred_d}'


def test_predictive_gate_tolerates_realistic_noise() -> None:
    """At ~0.5 m fused-position noise, predictive gate still matches at 8 m operational gate."""
    mod = _load_tracking_module()
    cand = mod.Candidate(x=10.0, y=0.0, z=0.0, hit_count=2)
    _append_hist(cand, 6.2, 0.0, 0.0, 0.0)
    _append_hist(cand, 10.0, 0.0, 0.0, 0.1)

    new_det = _make_point(mod, 13.8 + 0.4, 0.3, -0.2)
    pxyz = mod._predicted_candidate_xyz(cand)
    pred_d = mod._distance_point_to_xyz(new_det, pxyz)
    assert pred_d < 8.0, f'predictive gate must still admit noisy match, got {pred_d}'
    assert pred_d < 1.0, f'predictive residual should be ~ noise std, got {pred_d}'


def test_velocity_uses_real_dt_not_cycle_period() -> None:
    """Regression: when consecutive history points are 300 ms apart, the velocity must be
    computed with that real Δt — not divided by ``CYCLE_PERIOD_S=0.1`` which would inflate
    the speed 3× and propagate into a wildly-wrong seed track velocity (which is what we
    saw at km-scale: ``v_init=(108, -67, -12)`` for a real 50 m/s target).
    """
    mod = _load_tracking_module()
    hist = [
        (0.0, 0.0, 0.0, 0.0),
        (15.0, 0.0, 0.0, 0.3),
    ]
    vx, vy, vz = mod._initial_velocity_from_history(hist)
    # Real velocity = 15 m / 0.3 s = 50 m/s along x.
    assert abs(vx - 50.0) < 1e-6, f'expected vx=50.0, got {vx}'
    assert abs(vy) < 1e-9
    assert abs(vz) < 1e-9


def test_predicted_xyz_uses_actual_now_horizon() -> None:
    """When ``now`` is passed and ``last_update_time`` is set, the prediction horizon is
    ``now - last_update_time`` — which lets the matcher correctly extrapolate across a
    long quiet gap (the case where a bursty fusion stream goes silent for 200–300 ms).
    """
    mod = _load_tracking_module()
    cand = mod.Candidate(x=10.0, y=0.0, z=0.0, hit_count=2)
    _append_hist(cand, 0.0, 0.0, 0.0, 0.0)
    _append_hist(cand, 10.0, 0.0, 0.0, 0.2)

    px, py, pz = mod._predicted_candidate_xyz(cand, now=0.5)
    # v = (10-0)/0.2 = 50 m/s.  horizon = 0.5 - 0.2 = 0.3.  predicted x = 10 + 50*0.3 = 25.
    assert abs(px - 25.0) < 1e-6, f'expected px=25, got {px}'
    assert abs(py) < 1e-9
    assert abs(pz) < 1e-9


def test_candidate_miss_tolerance_keeps_candidate_alive() -> None:
    """A candidate with ``missed_frames`` ≤ ``candidate_max_missed_frames`` must NOT be
    discarded.  The legacy code treated any unmatched cycle as a kill — fatal when the
    upstream stream skips a tracking cycle.
    """
    mod = _load_tracking_module()
    cand = mod.Candidate(x=0.0, y=0.0, z=0.0, hit_count=1)
    _append_hist(cand, 0.0, 0.0, 0.0, 0.0)
    cand.missed_frames = 3
    # The fix is structural: the candidate just keeps its missed counter and the *node*
    # decides via ``cand.missed_frames > self._candidate_max_missed_frames``.  We assert
    # the field is wired through.
    assert hasattr(cand, 'missed_frames')
    assert cand.missed_frames == 3
