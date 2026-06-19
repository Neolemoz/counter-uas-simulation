"""Regression tests for scripts/run_capture.py metadata sidecars."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path
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


def test_coreutils_timeout_rc_is_marked_in_log_and_meta(tmp_path: Path, monkeypatch) -> None:
    rc_mod = _load_run_capture()
    setup = tmp_path / 'install' / 'setup.bash'
    setup.parent.mkdir()
    setup.write_text('# test setup\n', encoding='utf-8')
    monkeypatch.setattr(rc_mod, 'INSTALL_SETUP', setup)
    monkeypatch.setattr(rc_mod, 'RUNS_DIR', tmp_path / 'logs')

    def fake_run(cmd, *args, **kwargs):  # noqa: ANN001, ANN202
        del args
        if cmd[:2] == ['git', 'rev-parse']:
            return SimpleNamespace(returncode=0, stdout='abc123\n', stderr='')
        if cmd[:2] == ['git', 'status']:
            return SimpleNamespace(returncode=0, stdout='', stderr='')
        stdout = kwargs.get('stdout')
        if stdout is not None:
            stdout.write('sim reached wrapper timeout\n')
            return SimpleNamespace(returncode=124, stdout='', stderr='')
        return SimpleNamespace(returncode=0, stdout='', stderr='')

    monkeypatch.setattr(rc_mod.subprocess, 'run', fake_run)

    log_path, meta_path, _meta, capture_rc = rc_mod.run_capture(
        scenario='single',
        timeout_s=1.0,
        notes='unit',
        launch_args='use_gazebo_gui:=false',
        cohort='unit_cohort',
    )

    assert capture_rc == 124
    assert '=== TIMEOUT ===' in log_path.read_text(encoding='utf-8')
    meta = json.loads(meta_path.read_text(encoding='utf-8'))
    assert meta['capture_rc'] == 124
    assert meta['git_commit'] == 'abc123'
    assert meta['git_dirty'] is False
