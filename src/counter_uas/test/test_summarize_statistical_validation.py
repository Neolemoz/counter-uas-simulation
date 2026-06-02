"""Tests for summarize_statistical_validation pilot report builder."""

from __future__ import annotations

import csv
import importlib.util
import json
from pathlib import Path
from unittest.mock import patch

_REPO_ROOT = Path(__file__).resolve().parents[3]
_SUMMARIZE = _REPO_ROOT / 'scripts' / 'evaluation' / 'summarize_statistical_validation.py'


def _load():  # noqa: ANN201
    spec = importlib.util.spec_from_file_location('summarize_statistical_validation', _SUMMARIZE)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def _write_mc_csv(path: Path, *, seed_base: int, success: bool) -> None:
    log = path.parent / f'run_{seed_base}.log'
    meta = log.with_suffix('.meta.json')
    log.write_text('[HIT] min_miss=0.5 m\n', encoding='utf-8')
    meta.write_text(json.dumps({'notes': f'seed={seed_base}'}), encoding='utf-8')
    with path.open('w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(
            f,
            fieldnames=['run_id', 'success', 'miss_distance_m', 'intercept_time_s', 'layer_at_hit', 'log_path'],
        )
        w.writeheader()
        w.writerow(
            {
                'run_id': log.stem,
                'success': str(success).lower(),
                'miss_distance_m': '1.0',
                'intercept_time_s': '10.0',
                'layer_at_hit': '',
                'log_path': str(log),
            }
        )


def test_build_report_pairs_and_failure_hist(tmp_path: Path) -> None:
    mod = _load()
    summary_csv = tmp_path / 'summary.csv'
    base_csv = tmp_path / 'base.csv'
    cand_csv = tmp_path / 'cand.csv'
    _write_mc_csv(base_csv, seed_base=6201, success=True)
    _write_mc_csv(cand_csv, seed_base=6201, success=False)

    with summary_csv.open('w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(
            f,
            fieldnames=[
                'label',
                'scenario',
                'n',
                'seed_base',
                'success_rate_pct',
                'miss_mean_m',
                'miss_p95_m',
                'intercept_time_mean_s',
                'per_run_csv',
                'summary_json',
            ],
        )
        w.writeheader()
        w.writerows(
            [
                {
                    'label': 'predictive_baseline',
                    'scenario': 'single',
                    'n': '1',
                    'seed_base': '6201',
                    'success_rate_pct': '100.0',
                    'miss_mean_m': '1.0',
                    'miss_p95_m': '1.0',
                    'intercept_time_mean_s': '10.0',
                    'per_run_csv': str(base_csv),
                    'summary_json': '',
                },
                {
                    'label': 'predictive_intercept',
                    'scenario': 'single',
                    'n': '1',
                    'seed_base': '6201',
                    'success_rate_pct': '0.0',
                    'miss_mean_m': '2.0',
                    'miss_p95_m': '2.0',
                    'intercept_time_mean_s': '11.0',
                    'per_run_csv': str(cand_csv),
                    'summary_json': '',
                },
            ]
        )

    def _fake_failure_hist(csv_path: Path, out_json: Path) -> dict:  # noqa: ANN001
        payload = {'failure_hist': {'F0_none': 1}}
        out_json.parent.mkdir(parents=True, exist_ok=True)
        out_json.write_text(json.dumps(payload), encoding='utf-8')
        return payload

    with patch.object(mod, '_failure_hist', side_effect=_fake_failure_hist):
        report = mod.build_report(
            pilot_summary_csv=summary_csv,
            focused_summary_csv=None,
            out_dir=tmp_path / 'out',
            miss_delta=0.02,
            tint_delta=0.05,
        )

    assert report['artifact_type'] == 'statistical_validation_pilot_report'
    assert report['pilot_n15']['status'] == 'ok'
    assert len(report['pilot_n15']['paired_comparisons']) == 2
    pred = report['pilot_n15']['paired_comparisons'][0]
    assert pred['status'] == 'ok'
    assert pred['bucket_counts'].get('G1_base_ok_cand_fail') == 1
    assert 'sensor_realism_on' in report['triage']
