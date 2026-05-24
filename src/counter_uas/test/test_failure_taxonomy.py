"""Layer C failure taxonomy evidence regressions."""

from __future__ import annotations

import importlib.util
import json
import subprocess
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


def _load_run_capture():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'run_capture.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('run_capture_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _load_statistical_validation():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'evaluation' / 'statistical_validation.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('statistical_validation_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
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


def test_run_capture_marks_coreutils_timeout_and_persists_rc(tmp_path: Path, monkeypatch) -> None:  # noqa: ANN001
    run_capture = _load_run_capture()
    install_setup = tmp_path / 'install' / 'setup.bash'
    install_setup.parent.mkdir(parents=True)
    install_setup.write_text('# test setup\n', encoding='utf-8')
    monkeypatch.setattr(run_capture, 'WORKSPACE', tmp_path)
    monkeypatch.setattr(run_capture, 'RUNS_DIR', tmp_path / 'runs' / 'logs')
    monkeypatch.setattr(run_capture, 'INSTALL_SETUP', install_setup)

    def fake_run(cmd, *args, **kwargs):  # noqa: ANN001, ANN202
        if isinstance(cmd, list) and cmd and cmd[0] == 'git':
            return subprocess.CompletedProcess(cmd, 1, stdout='', stderr='')
        stdout = kwargs.get('stdout')
        if stdout is not None:
            stdout.write('[FEASIBILITY] feasible_geom=True\n')
            return subprocess.CompletedProcess(cmd, 124, stdout='', stderr='')
        return subprocess.CompletedProcess(cmd, 0, stdout='', stderr='')

    monkeypatch.setattr(run_capture.subprocess, 'run', fake_run)

    log_path, meta_path, meta, rc = run_capture.run_capture(
        scenario='single',
        timeout_s=0.1,
        notes=None,
        launch_args=None,
    )

    assert rc == 124
    assert meta.capture_rc == 124
    assert '=== TIMEOUT ===' in log_path.read_text(encoding='utf-8')
    meta_json = json.loads(meta_path.read_text(encoding='utf-8'))
    assert meta_json['capture_rc'] == 124


def test_failure_class_uses_capture_rc_from_meta(tmp_path: Path) -> None:
    statistical_validation = _load_statistical_validation()
    log = tmp_path / 'timed_out_without_marker.log'
    log.write_text('[FEASIBILITY] feasible_geom=True\n', encoding='utf-8')
    meta = tmp_path / 'timed_out_without_marker.meta.json'
    meta.write_text(json.dumps({'capture_rc': 124}) + '\n', encoding='utf-8')

    assert statistical_validation._failure_class({'log_path': str(log), 'meta_path': str(meta)}) == 'F1_timeout'
