from __future__ import annotations

import importlib.util
import json
import sys
from types import SimpleNamespace
from pathlib import Path


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


def test_coreutils_timeout_rc_is_persisted_and_marked(tmp_path, monkeypatch) -> None:
    rc_mod = _load_run_capture()
    setup = tmp_path / 'install' / 'setup.bash'
    setup.parent.mkdir()
    setup.write_text('', encoding='utf-8')

    monkeypatch.setattr(rc_mod, 'WORKSPACE', tmp_path)
    monkeypatch.setattr(rc_mod, 'RUNS_DIR', tmp_path / 'runs' / 'logs')
    monkeypatch.setattr(rc_mod, 'INSTALL_SETUP', setup)
    monkeypatch.setattr(rc_mod, '_git_commit', lambda: 'abc123')
    monkeypatch.setattr(rc_mod, '_git_dirty', lambda: False)

    calls = []

    def fake_run(*args, **kwargs):  # noqa: ANN002, ANN003, ANN202
        calls.append((args, kwargs))
        stdout = kwargs.get('stdout')
        if stdout is not None:
            stdout.write('[sim] still running\n')
            return SimpleNamespace(returncode=124)
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(rc_mod.subprocess, 'run', fake_run)

    log_path, meta_path, _meta, rc = rc_mod.run_capture(
        scenario='single',
        timeout_s=1.0,
        notes=None,
        launch_args='use_gazebo_gui:=false',
    )

    assert rc == 124
    assert '=== TIMEOUT ===' in log_path.read_text(encoding='utf-8')
    meta = json.loads(meta_path.read_text(encoding='utf-8'))
    assert meta['capture_rc'] == 124
    assert len(calls) == 2
