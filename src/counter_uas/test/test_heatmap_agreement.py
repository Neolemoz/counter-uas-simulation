"""Layer C heatmap-vs-Gazebo agreement metric regressions."""

from __future__ import annotations

import importlib.util
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_validate():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'validate_heatmap_vs_gazebo.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('validate_heatmap_vs_gazebo', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def test_compute_validation_metrics_adds_uncertainty() -> None:
    validate = _load_validate()
    rows, mean_e, max_e = validate.compute_validation_metrics(
        [
            {
                'cell': (1.0, 2.0, 3.0),
                'tier': 'mid',
                'p_heatmap': 0.5,
                'success_count': 3,
                'total_runs': 4,
                'seed_list': [10, 11, 12, 13],
                'log_paths': ['a.log'],
            },
        ],
    )
    assert mean_e == max_e == 0.25
    assert rows[0]['p_gazebo'] == 0.75
    assert rows[0]['p_gazebo_ci95']['method'] == 'wilson'
    assert rows[0]['signed_error'] == 0.25


def test_compute_agreement_summary_reports_pilot_caution() -> None:
    validate = _load_validate()
    rows, _, _ = validate.compute_validation_metrics(
        [
            {'cell': (0, 0, 0), 'tier': 'low', 'p_heatmap': 0.2, 'success_count': 1, 'total_runs': 2},
            {'cell': (1, 0, 0), 'tier': 'high', 'p_heatmap': 0.8, 'success_count': 2, 'total_runs': 2},
        ],
    )
    summary = validate.compute_agreement_summary(rows, bootstrap_seed=5)
    assert summary['n_cells'] == 2
    assert summary['cohort_tier'] == 'smoke'
    assert summary['mean_abs_error_ci95']['method'] == 'bootstrap_mean_abs_calibration_error'
    assert 'N>=40' in summary['caution']
