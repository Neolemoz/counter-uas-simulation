"""Heatmap credibility contracts for sparse Gazebo agreement checks."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_validate_module():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'validate_heatmap_vs_gazebo.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('validate_heatmap_vs_gazebo', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def test_validation_cell_selection_is_seeded_and_tiered() -> None:
    mod = _load_validate_module()
    fixture = _REPO_ROOT / 'scripts' / 'evaluation' / 'fixtures' / 'sample_heatmap_for_validate.csv'

    cells_a = mod.select_validation_cells(str(fixture), rng_seed=7)
    cells_b = mod.select_validation_cells(str(fixture), rng_seed=7)

    assert cells_a == cells_b
    assert len(cells_a) == 9
    assert [c['tier'] for c in cells_a].count('high') == 3
    assert [c['tier'] for c in cells_a].count('mid') == 3
    assert [c['tier'] for c in cells_a].count('low') == 3
    assert all(0.0 <= c['p_hit'] <= 1.0 for c in cells_a)


def test_validation_replay_launch_args_pin_cell_and_headless_mode() -> None:
    mod = _load_validate_module()

    args = mod._launch_args_for_cell(1.2345678, -2.0, 30.5, headless=True)

    assert 'target_start_x_m:=1.234568' in args
    assert 'target_start_y_m:=-2.000000' in args
    assert 'target_start_z_m:=30.500000' in args
    assert 'use_gazebo_gui:=false' in args


def test_validation_verdict_thresholds_are_surrogate_agreement_bands() -> None:
    mod = _load_validate_module()

    assert mod._verdict(0.15, 0.25) == 'GOOD'
    assert mod._verdict(0.30, 0.45) == 'PARTIAL'
    assert mod._verdict(0.31, 0.45) == 'POOR'
    assert mod._verdict(float('nan'), 0.0) == 'POOR'


def test_validation_metrics_compare_absolute_probability_error() -> None:
    mod = _load_validate_module()
    rows, mean_e, max_e = mod.compute_validation_metrics(
        [
            {'cell': (0.0, 0.0, 0.0), 'tier': 'high', 'p_heatmap': 0.75, 'success_count': 3, 'total_runs': 4},
            {'cell': (1.0, 0.0, 0.0), 'tier': 'low', 'p_heatmap': 0.10, 'success_count': 0, 'total_runs': 4},
        ],
    )

    assert rows[0]['p_gazebo'] == pytest.approx(0.75)
    assert rows[0]['error'] == pytest.approx(0.0)
    assert rows[1]['p_gazebo'] == pytest.approx(0.0)
    assert rows[1]['error'] == pytest.approx(0.10)
    assert mean_e == pytest.approx(0.05)
    assert max_e == pytest.approx(0.10)
