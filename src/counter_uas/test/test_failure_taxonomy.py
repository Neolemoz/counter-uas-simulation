"""Layer C failure taxonomy evidence regressions."""

from __future__ import annotations

import importlib.util
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


def test_classify_run_failure_evidence_skips_successful_hit_timeout(tmp_path: Path) -> None:
    classify = _load_classify()
    log = tmp_path / 'hit_timeout.log'
    log.write_text('[HIT] interceptor_0 min_miss=0.25 m\n=== TIMEOUT ===\n', encoding='utf-8')
    evidence = classify.classify_run_failure_evidence(log, capture_rc=124)
    assert evidence['failure_class'] == ''
    assert evidence['hit_seen'] is True
    assert evidence['timeout_seen'] is True


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


def test_summarize_failure_classes_reads_capture_rc_from_meta(tmp_path: Path) -> None:
    summarize = _load_summarize_failure_classes()
    log = tmp_path / 'timeout.log'
    log.write_text('no hit\n', encoding='utf-8')
    log.with_suffix('.meta.json').write_text('{"capture_rc": 124}\n', encoding='utf-8')
    row = {'log_path': str(log), 'capture_rc': ''}
    assert summarize._capture_rc_from_row_or_meta(row, log) == 124
