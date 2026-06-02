"""Dry-run and mocked subprocess tests for statistical validation MC sweep."""

from __future__ import annotations

import csv
import importlib.util
import json
import subprocess
import sys
from pathlib import Path
from unittest.mock import patch

_REPO_ROOT = Path(__file__).resolve().parents[3]
_SWEEP = _REPO_ROOT / 'scripts' / 'evaluation' / 'run_statistical_validation_sweep.py'


def _load_sweep():  # noqa: ANN201
    spec = importlib.util.spec_from_file_location('run_statistical_validation_sweep', _SWEEP)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def test_dry_run_writes_summary_without_subprocess(tmp_path: Path) -> None:
    out_csv = tmp_path / 'statistical_validation_summary.csv'
    r = subprocess.run(
        [
            sys.executable,
            str(_SWEEP),
            '--dry-run',
            '--out-csv',
            str(out_csv),
            '--out-dir',
            str(tmp_path / 'mc'),
        ],
        cwd=_REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert r.returncode == 0, r.stderr
    assert out_csv.is_file()
    rows = list(csv.DictReader(out_csv.open(encoding='utf-8', newline='')))
    assert len(rows) == 7
    assert rows[0]['label'] == 'predictive_baseline'
    assert rows[0]['seed_base'] == '6201'
    assert rows[0]['success_rate_pct'] == '0.0'


def test_mock_subprocess_sweep_aggregates_monte_carlo_json(tmp_path: Path) -> None:
    sweep = _load_sweep()
    mc_dir = tmp_path / 'mc'
    mc_dir.mkdir()

    def _fake_run(cmd, cwd=None, text=True, capture_output=True):  # noqa: ANN001, ARG001
        label_idx = cmd.index('--label')
        label = cmd[label_idx + 1]
        summary = {
            'label': label,
            'n_runs': 2,
            'n_success': 1,
            'success_rate': 0.5,
            'miss_distance_m': {'mean': 1.5, 'p95': 3.0},
            'intercept_time_s': {'mean': 12.0},
        }
        (mc_dir / f'{label}.json').write_text(json.dumps(summary), encoding='utf-8')
        (mc_dir / f'{label}.csv').write_text('run_id,success,miss_distance_m,intercept_time_s,layer_at_hit,log_path\n', encoding='utf-8')
        return subprocess.CompletedProcess(cmd, 0, stdout='', stderr='')

    out_csv = tmp_path / 'summary.csv'
    argv = [
        str(_SWEEP),
        '--profiles-csv',
        str(_REPO_ROOT / 'scripts/evaluation/fixtures/statistical_validation_profiles.csv'),
        '--out-dir',
        str(mc_dir),
        '--out-csv',
        str(out_csv),
        '--n',
        '2',
        '--seed-base',
        '6201',
    ]
    with patch.object(sweep, 'subprocess') as mock_sub:
        mock_sub.run.side_effect = _fake_run
        with patch.object(sys, 'argv', argv):
            assert sweep.main() == 0

    rows = list(csv.DictReader(out_csv.open(encoding='utf-8', newline='')))
    assert len(rows) == 7
    predictive = next(r for r in rows if r['label'] == 'predictive_intercept')
    assert predictive['success_rate_pct'] == '50.0'
    assert predictive['miss_mean_m'] == '1.500'
    assert predictive['miss_p95_m'] == '3.000'
    assert 'statistical_validation_predictive_intercept' in predictive['mc_label']


def test_override_n_uses_pilot_count_in_mc_label(tmp_path: Path) -> None:
    sweep = _load_sweep()
    mc_dir = tmp_path / 'mc'
    mc_dir.mkdir()
    calls: list[list[str]] = []

    def _fake_run(cmd, cwd=None, text=True, capture_output=True):  # noqa: ANN001, ARG001
        calls.append(cmd)
        label = cmd[cmd.index('--label') + 1]
        summary = {'n_runs': 15, 'success_rate': 0.0, 'miss_distance_m': {}, 'intercept_time_s': {}}
        (mc_dir / f'{label}.json').write_text(json.dumps(summary), encoding='utf-8')
        (mc_dir / f'{label}.csv').write_text('run_id,success,miss_distance_m,intercept_time_s,layer_at_hit,log_path\n', encoding='utf-8')
        return subprocess.CompletedProcess(cmd, 0, stdout='', stderr='')

    out_csv = tmp_path / 'summary.csv'
    argv = [
        str(_SWEEP),
        '--override-n',
        '15',
        '--out-dir',
        str(mc_dir),
        '--out-csv',
        str(out_csv),
    ]
    with patch.object(sweep, 'subprocess') as mock_sub:
        mock_sub.run.side_effect = _fake_run
        with patch.object(sys, 'argv', argv):
            assert sweep.main() == 0

    assert any('predictive_baseline_n15_s6201' in ' '.join(c) for c in calls)
