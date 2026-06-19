"""Regression tests for Layer C statistical validation reports."""

from __future__ import annotations

import csv
import importlib.util
import json
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_layer_c():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'evaluation' / 'statistical_validation.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('statistical_validation', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open('w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def test_aggregate_report_includes_intervals_and_seed_diagnostics() -> None:
    layer_c = _load_layer_c()
    rows = [
        {'success': 'true', 'miss_distance_m': '1.0', 'intercept_time_s': '5.0', 'seed': '10', 'cohort': 'c'},
        {'success': 'false', 'miss_distance_m': '4.0', 'intercept_time_s': '8.0', 'seed': '11', 'cohort': 'c'},
    ]
    report = layer_c.aggregate_report(rows, label='unit', bootstrap_seed=5)
    assert report['label'] == 'unit'
    assert report['cohort_tier'] == 'smoke'
    assert report['success_rate']['ci95']['method'] == 'wilson'
    assert report['seeds']['n_with_seed'] == 2
    assert report['miss_distance_m']['p95_ci95']['seed'] == 100


def test_paired_report_uses_matched_seed_rows() -> None:
    layer_c = _load_layer_c()
    baseline = [
        {'success': 'true', 'miss_distance_m': '3.0', 'intercept_time_s': '9.0', 'seed': '1'},
        {'success': 'false', 'miss_distance_m': '6.0', 'intercept_time_s': '12.0', 'seed': '2'},
        {'success': 'true', 'miss_distance_m': '2.0', 'intercept_time_s': '7.0', 'seed': '99'},
    ]
    candidate = [
        {'success': 'true', 'miss_distance_m': '2.0', 'intercept_time_s': '8.0', 'seed': '1'},
        {'success': 'true', 'miss_distance_m': '4.0', 'intercept_time_s': '10.0', 'seed': '2'},
    ]
    report, rows = layer_c.paired_report(baseline, candidate, bootstrap_seed=7)
    assert report['matched_seed_count'] == 2
    assert report['baseline_only_seeds'] == [99]
    assert report['paired_success']['candidate_only_success'] == 1
    assert report['miss_distance_m']['p50_delta_ci95']['estimate'] < 0.0
    assert [r['seed'] for r in rows] == [1, 2]


def test_paired_report_rejects_duplicate_seeds() -> None:
    layer_c = _load_layer_c()
    baseline = [
        {'success': 'true', 'miss_distance_m': '3.0', 'intercept_time_s': '9.0', 'seed': '1'},
        {'success': 'false', 'miss_distance_m': '6.0', 'intercept_time_s': '12.0', 'seed': '1'},
    ]
    candidate = [{'success': 'true', 'miss_distance_m': '2.0', 'intercept_time_s': '8.0', 'seed': '1'}]
    with pytest.raises(ValueError, match='duplicate seed'):
        layer_c.paired_report(baseline, candidate)


def test_failure_class_uses_capture_rc_and_skips_hits(tmp_path: Path) -> None:
    layer_c = _load_layer_c()
    timeout_log = tmp_path / 'timeout.log'
    timeout_log.write_text('no hit before timeout\n', encoding='utf-8')
    hit_log = tmp_path / 'hit.log'
    hit_log.write_text('[HIT] interceptor_0 min_miss=0.1 m\n', encoding='utf-8')
    assert layer_c._failure_class({'log_path': str(timeout_log), 'capture_rc': '124'}) == 'F1_timeout'
    assert layer_c._failure_class({'log_path': str(hit_log), 'capture_rc': '124'}) == ''


def test_validate_manifest_detects_mixed_cohorts(tmp_path: Path) -> None:
    layer_c = _load_layer_c()
    log_path = tmp_path / 'run.log'
    log_path.write_text('', encoding='utf-8')
    log_path.with_suffix('.meta.json').write_text(json.dumps({'cohort': 'expected'}), encoding='utf-8')
    rows = [
        {
            'seed': '1',
            'cohort': 'other',
            'git_dirty': 'False',
            'log_path': str(log_path),
        },
    ]
    result = layer_c.validate_manifest({'n': 1, 'cohort': 'expected', 'require_clean_git': True}, rows)
    assert result['ok'] is False
    assert any('cohorts seen' in p for p in result['problems'])
    assert result['seed_count'] == 1


def test_validate_manifest_rejects_missing_git_dirty(tmp_path: Path) -> None:
    layer_c = _load_layer_c()
    log_path = tmp_path / 'run.log'
    log_path.write_text('', encoding='utf-8')
    log_path.with_suffix('.meta.json').write_text(json.dumps({'cohort': 'expected'}), encoding='utf-8')
    rows = [
        {
            'seed': '1',
            'cohort': 'expected',
            'git_dirty': '',
            'log_path': str(log_path),
        },
    ]
    result = layer_c.validate_manifest({'n': 1, 'cohort': 'expected', 'require_clean_git': True}, rows)
    assert result['ok'] is False
    assert any('missing git_dirty' in p for p in result['problems'])
