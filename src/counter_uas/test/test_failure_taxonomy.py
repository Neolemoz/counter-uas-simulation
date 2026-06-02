"""Layer C failure taxonomy evidence regressions."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_classify():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'evaluation' / 'classify_run.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('classify_run', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def _load_summarize_failure_classes():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'evaluation' / 'summarize_failure_classes.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('summarize_failure_classes', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def test_classify_run_failure_evidence_tracks_timeout(tmp_path: Path) -> None:
    classify = _load_classify()
    log = tmp_path / 'timeout.log'
    log.write_text('=== TIMEOUT ===\n', encoding='utf-8')
    evidence = classify.classify_run_failure_evidence(log, capture_rc=124)
    assert evidence['failure_class'] == 'F1_timeout'
    assert evidence['timeout_seen'] is True


def test_capture_rc_124_marks_miss_timeout_without_log_marker(tmp_path: Path) -> None:
    classify = _load_classify()
    log = tmp_path / 'timeout_rc_only.log'
    log.write_text('[min_miss] = 17.0 m\n', encoding='utf-8')
    evidence = classify.classify_run_failure_evidence(log, capture_rc=124)
    assert evidence['failure_class'] == 'F1_timeout'
    assert evidence['timeout_seen'] is True


def test_hit_log_has_no_failure_class_even_if_capture_timed_out(tmp_path: Path) -> None:
    classify = _load_classify()
    log = tmp_path / 'hit_then_timeout.log'
    log.write_text('[HIT] interceptor_0 min_miss=0.2 m\n=== TIMEOUT ===\n', encoding='utf-8')
    evidence = classify.classify_run_failure_evidence(log, capture_rc=124)
    assert evidence['failure_class'] == ''
    assert evidence['hit_seen'] is True


def test_failure_histogram_skips_successes_and_counts_timeout_rc(tmp_path: Path) -> None:
    summarize = _load_summarize_failure_classes()
    hit_log = tmp_path / 'hit.log'
    hit_log.write_text('[HIT] interceptor_0 min_miss=0.2 m\n=== TIMEOUT ===\n', encoding='utf-8')
    miss_log = tmp_path / 'miss.log'
    miss_log.write_text('[min_miss] = 17.0 m\n', encoding='utf-8')
    csv_path = tmp_path / 'mc.csv'
    out_json = tmp_path / 'hist.json'
    csv_path.write_text(
        'run_id,success,capture_rc,log_path\n'
        f'hit,true,124,{hit_log}\n'
        f'miss,false,124,{miss_log}\n',
        encoding='utf-8',
    )

    old_argv = sys.argv
    try:
        sys.argv = ['summarize_failure_classes.py', str(csv_path), '--out-json', str(out_json)]
        assert summarize.main() == 0
    finally:
        sys.argv = old_argv

    payload = json.loads(out_json.read_text(encoding='utf-8'))
    assert payload['n_classified'] == 1
    assert payload['failure_hist'] == {'F1_timeout': 1}


def test_classify_run_failure_evidence_tracks_instability(tmp_path: Path) -> None:
    classify = _load_classify()
    log = tmp_path / 'unstable.log'
    log.write_text(
        '[ENG_METRIC] delta_t_go_raw=0.1\n'
        '[ENG_METRIC] delta_t_go_raw=9.5\n',
        encoding='utf-8',
    )
    evidence = classify.classify_run_failure_evidence(log)
    assert evidence['failure_class'] == 'F3_track_instability'
    assert evidence['has_eng_metric'] is True
    assert evidence['max_abs_delta_t_go'] == 9.5
