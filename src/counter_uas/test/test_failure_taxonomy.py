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


def _load_summarizer():  # noqa: ANN201
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


def test_classify_run_failure_evidence_excludes_success_hits(tmp_path: Path) -> None:
    classify = _load_classify()
    log = tmp_path / 'hit.log'
    log.write_text('[HIT] interceptor_0 min_miss=0.1 m\n=== TIMEOUT ===\n', encoding='utf-8')
    evidence = classify.classify_run_failure_evidence(log, capture_rc=124)
    assert evidence['hit_seen'] is True
    assert evidence['timeout_seen'] is True
    assert evidence['failure_class'] is None
    assert evidence['parser_warnings'] == []


def test_summarize_failure_classes_skips_success_hits(tmp_path: Path) -> None:
    summarize = _load_summarizer()
    hit_log = tmp_path / 'hit.log'
    timeout_log = tmp_path / 'timeout.log'
    hit_log.write_text('[HIT] interceptor_0 min_miss=0.1 m\n', encoding='utf-8')
    timeout_log.write_text('=== TIMEOUT ===\n', encoding='utf-8')
    csv_path = tmp_path / 'runs.csv'
    csv_path.write_text(
        'log_path\n'
        f'{hit_log}\n'
        f'{timeout_log}\n',
        encoding='utf-8',
    )

    payload = summarize.summarize_failure_classes(csv_path)

    assert payload['n_classified'] == 1
    assert payload['n_success'] == 1
    assert payload['failure_hist'] == {'F1_timeout': 1}
