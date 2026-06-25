"""Regression tests for machine-readable capture provenance."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_run_capture():  # noqa: ANN201
    path = _REPO_ROOT / "scripts" / "run_capture.py"
    assert path.is_file(), f"missing {path}"
    spec = importlib.util.spec_from_file_location("run_capture", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def test_coreutils_timeout_writes_marker_and_capture_rc(tmp_path, monkeypatch) -> None:  # noqa: ANN001
    rc_mod = _load_run_capture()
    setup = tmp_path / "install" / "setup.bash"
    setup.parent.mkdir(parents=True)
    setup.write_text("# test setup\n", encoding="utf-8")

    monkeypatch.setattr(rc_mod, "WORKSPACE", tmp_path)
    monkeypatch.setattr(rc_mod, "INSTALL_SETUP", setup)
    monkeypatch.setattr(rc_mod, "RUNS_DIR", tmp_path / "runs" / "logs")

    class _Result:
        def __init__(self, returncode: int, stdout: str = "") -> None:
            self.returncode = returncode
            self.stdout = stdout
            self.stderr = ""

    def fake_run(cmd, *args, **kwargs):  # noqa: ANN001, ANN202
        if cmd and cmd[0] == "git":
            if "rev-parse" in cmd:
                return _Result(0, "abc123\n")
            return _Result(0, "")
        stream = kwargs.get("stdout")
        if stream is not None:
            stream.write("sim output before timeout\n")
            return _Result(124)
        return _Result(0)

    monkeypatch.setattr(rc_mod.subprocess, "run", fake_run)

    log_path, meta_path, _meta, capture_rc = rc_mod.run_capture(
        scenario="single",
        timeout_s=1.0,
        notes=None,
        launch_args="use_gazebo_gui:=false",
    )

    assert capture_rc == 124
    assert "=== TIMEOUT ===" in log_path.read_text(encoding="utf-8")
    payload = json.loads(meta_path.read_text(encoding="utf-8"))
    assert payload["capture_rc"] == 124
