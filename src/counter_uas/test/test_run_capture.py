"""Regression tests for capture metadata written by scripts/run_capture.py."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_run_capture():  # noqa: ANN201
    path = _REPO_ROOT / "scripts" / "run_capture.py"
    assert path.is_file(), f"missing {path}"
    spec = importlib.util.spec_from_file_location("run_capture", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_coreutils_timeout_rc_is_marked_and_persisted(tmp_path: Path, monkeypatch) -> None:  # noqa: ANN001
    mod = _load_run_capture()
    setup = tmp_path / "install" / "setup.bash"
    setup.parent.mkdir()
    setup.write_text("", encoding="utf-8")
    runs_dir = tmp_path / "runs" / "logs"

    calls: list[object] = []

    def fake_run(cmd, **kwargs):  # noqa: ANN001, ANN202
        calls.append(cmd)
        return SimpleNamespace(returncode=124 if len(calls) == 2 else 0, stdout="", stderr="")

    monkeypatch.setattr(mod, "WORKSPACE", tmp_path)
    monkeypatch.setattr(mod, "INSTALL_SETUP", setup)
    monkeypatch.setattr(mod, "RUNS_DIR", runs_dir)
    monkeypatch.setattr(mod.subprocess, "run", fake_run)
    monkeypatch.setattr(mod, "_git_commit", lambda: "abc123")
    monkeypatch.setattr(mod, "_git_dirty", lambda: False)

    log_path, meta_path, _meta, rc = mod.run_capture(
        scenario="single",
        timeout_s=1.0,
        notes=None,
        launch_args=None,
    )

    assert rc == 124
    assert "=== TIMEOUT ===" in log_path.read_text(encoding="utf-8")
    payload = json.loads(meta_path.read_text(encoding="utf-8"))
    assert payload["capture_rc"] == 124
