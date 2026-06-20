"""Regression tests for log-capture provenance."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path


_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_run_capture():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'run_capture.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('run_capture', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


class _Completed:
    def __init__(self, returncode: int = 0) -> None:
        self.returncode = returncode
        self.stdout = ''
        self.stderr = ''


def test_coreutils_timeout_writes_marker_and_capture_rc(tmp_path: Path, monkeypatch) -> None:  # noqa: ANN001
    run_capture = _load_run_capture()
    install_setup = tmp_path / 'install' / 'setup.bash'
    install_setup.parent.mkdir()
    install_setup.write_text('# test setup\n', encoding='utf-8')
    monkeypatch.setattr(run_capture, 'INSTALL_SETUP', install_setup)
    monkeypatch.setattr(run_capture, 'RUNS_DIR', tmp_path / 'logs')
    monkeypatch.setattr(run_capture, 'WORKSPACE', tmp_path)
    monkeypatch.setattr(run_capture, '_git_commit', lambda: 'abc123')
    monkeypatch.setattr(run_capture, '_git_dirty', lambda: False)

    calls = {'n': 0}

    def fake_run(cmd, **kwargs):  # noqa: ANN001, ANN202
        calls['n'] += 1
        out = kwargs.get('stdout')
        if out is not None:
            out.write('launch output\n')
            return _Completed(124)
        return _Completed(0)

    monkeypatch.setattr(run_capture.subprocess, 'run', fake_run)

    log_path, meta_path, _meta, rc = run_capture.run_capture(
        scenario='single',
        timeout_s=1.0,
        notes='unit',
        launch_args='use_noisy_measurement:=true',
        cohort='c',
    )

    assert rc == 124
    assert '=== TIMEOUT ===' in log_path.read_text(encoding='utf-8')
    meta = json.loads(meta_path.read_text(encoding='utf-8'))
    assert meta['capture_rc'] == 124
    assert meta['git_dirty'] is False
    assert calls['n'] == 2
