"""Regression tests for capture metadata written by scripts/run_capture.py."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_run_capture():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'run_capture.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('run_capture', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_coreutils_timeout_return_writes_marker_and_capture_rc(tmp_path: Path, monkeypatch) -> None:
    run_capture = _load_run_capture()
    install_setup = tmp_path / 'install' / 'setup.bash'
    install_setup.parent.mkdir()
    install_setup.write_text('# test setup\n', encoding='utf-8')

    monkeypatch.setattr(run_capture, 'WORKSPACE', tmp_path)
    monkeypatch.setattr(run_capture, 'RUNS_DIR', tmp_path / 'runs' / 'logs')
    monkeypatch.setattr(run_capture, 'INSTALL_SETUP', install_setup)

    def fake_run(cmd, **kwargs):  # noqa: ANN001, ANN202
        if cmd and cmd[0] == 'git':
            stdout = 'abc123\n' if cmd[1:] == ['rev-parse', 'HEAD'] else ''
            return SimpleNamespace(returncode=0, stdout=stdout)
        if 'stdout' in kwargs:
            return SimpleNamespace(returncode=124)
        return SimpleNamespace(returncode=0, stdout='')

    monkeypatch.setattr(run_capture.subprocess, 'run', fake_run)

    log_path, meta_path, _meta, rc = run_capture.run_capture(
        scenario='single',
        timeout_s=1.0,
        notes=None,
        launch_args='use_gazebo_gui:=false',
    )

    assert rc == 124
    assert '=== TIMEOUT ===' in log_path.read_text(encoding='utf-8')
    meta = json.loads(meta_path.read_text(encoding='utf-8'))
    assert meta['capture_rc'] == 124
