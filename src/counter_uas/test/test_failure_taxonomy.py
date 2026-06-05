"""Layer C failure taxonomy evidence regressions."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_classify():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'evaluation' / 'classify_run.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('classify_run', path)
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


def test_classify_run_failure_evidence_tracks_python_bool_feasible_geom(tmp_path: Path) -> None:
    classify = _load_classify()
    log = tmp_path / 'feasible_no_hit.log'
    log.write_text(
        '[ENG_METRIC] id=target_0 range_m=1000.0000 v_closing=100.0000 '
        't_go_raw=10.0000 t_go_filt=10.0000 delta_t_go_raw=0.0000 '
        'feasible_geom=True rollout_gate_ok=True speed_cmd=50.0000 '
        'vx=1.0000 vy=0.0000 vz=0.0000\n',
        encoding='utf-8',
    )
    evidence = classify.classify_run_failure_evidence(log)
    assert evidence['feasible_geom_seen'] is True
    assert evidence['failure_class'] == 'F2_geom_not_dyn'
    assert evidence['parser_warnings'] == []


def test_classify_run_failure_evidence_does_not_bucket_successful_hit_timeout(tmp_path: Path) -> None:
    classify = _load_classify()
    log = tmp_path / 'hit_timeout.log'
    log.write_text(
        '[HIT] interceptor_0 layer=engage  min_miss=0.45 m  hit_threshold = 1.0 m\n'
        '=== TIMEOUT ===\n',
        encoding='utf-8',
    )
    evidence = classify.classify_run_failure_evidence(log, capture_rc=124)
    assert evidence['hit_seen'] is True
    assert evidence['timeout_seen'] is True
    assert evidence['failure_class'] == ''
    assert evidence['parser_warnings'] == []


def test_summarize_failure_classes_skips_successes(tmp_path: Path) -> None:
    hit_log = tmp_path / 'hit.log'
    hit_log.write_text(
        '[HIT] interceptor_0 layer=engage  min_miss=0.45 m  hit_threshold = 1.0 m\n',
        encoding='utf-8',
    )
    miss_log = tmp_path / 'miss.log'
    miss_log.write_text(
        '[ENG_METRIC] delta_t_go_raw=0.0000 feasible_geom=True rollout_gate_ok=True\n',
        encoding='utf-8',
    )
    csv_path = tmp_path / 'mc.csv'
    csv_path.write_text(f'log_path\n{hit_log}\n{miss_log}\n', encoding='utf-8')
    out_json = tmp_path / 'failure_hist.json'

    result = subprocess.run(
        [
            sys.executable,
            str(_REPO_ROOT / 'scripts' / 'evaluation' / 'summarize_failure_classes.py'),
            str(csv_path),
            '--out-json',
            str(out_json),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    payload = json.loads(out_json.read_text(encoding='utf-8'))
    assert payload['n_classified'] == 1
    assert payload['failure_hist'] == {'F2_geom_not_dyn': 1}
    assert len(payload['evidence_rows']) == 1
