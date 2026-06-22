"""Layer C failure taxonomy evidence regressions."""

from __future__ import annotations

import importlib.util
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


def test_classify_run_failure_ignores_timeout_after_hit(tmp_path: Path) -> None:
    classify = _load_classify()
    log = tmp_path / 'hit_then_timeout.log'
    log.write_text(
        '[interception_logic_node-1] [HIT] interceptor_0 layer=engage min_miss=0.45 m\n'
        '=== TIMEOUT ===\n',
        encoding='utf-8',
    )
    evidence = classify.classify_run_failure_evidence(log, capture_rc=124)
    assert evidence['hit_seen'] is True
    assert evidence['failure_class'] == ''
    assert classify.classify_run_failure(log, capture_rc=124) == ''


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


def test_summarize_failure_classes_skips_successes_and_uses_capture_rc(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    summarize = _load_summarize_failure_classes()
    hit = tmp_path / 'hit.log'
    timeout = tmp_path / 'timeout.log'
    hit.write_text('[HIT] interceptor_0 min_miss=0.25 m\n=== TIMEOUT ===\n', encoding='utf-8')
    timeout.write_text('[min_miss] = 9.0 m\n', encoding='utf-8')
    csv_path = tmp_path / 'mc.csv'
    csv_path.write_text(
        'success,log_path,capture_rc\n'
        f'true,{hit},124\n'
        f'false,{timeout},124\n',
        encoding='utf-8',
    )

    monkeypatch.setattr(sys, 'argv', ['summarize_failure_classes.py', str(csv_path)])
    assert summarize.main() == 0
    out = capsys.readouterr().out
    assert '"F1_timeout": 1' in out
    assert '"F5_unknown"' not in out
    assert '"success_rows_skipped": 1' in out
