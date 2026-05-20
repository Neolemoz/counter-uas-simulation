from __future__ import annotations

import importlib.util
import subprocess
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


def test_coreutils_timeout_return_code_writes_timeout_marker(tmp_path, monkeypatch) -> None:
    mod = _load_run_capture()
    install_setup = tmp_path / "install" / "setup.bash"
    install_setup.parent.mkdir(parents=True)
    install_setup.write_text("# fake setup\n", encoding="utf-8")
    monkeypatch.setattr(mod, "INSTALL_SETUP", install_setup)
    monkeypatch.setattr(mod, "RUNS_DIR", tmp_path / "logs")
    monkeypatch.setattr(mod, "_git_commit", lambda: "abc123")
    monkeypatch.setattr(mod, "_git_dirty", lambda: False)

    def fake_run(args, **kwargs):  # noqa: ANN001, ANN202
        if kwargs.get("stdout") is not None:
            kwargs["stdout"].write("launch output\n")
            return subprocess.CompletedProcess(args, 124)
        return subprocess.CompletedProcess(args, 0, stdout="")

    monkeypatch.setattr(mod.subprocess, "run", fake_run)

    log_path, _meta_path, _meta, rc = mod.run_capture(
        scenario="single",
        timeout_s=1.0,
        notes=None,
        launch_args=None,
    )

    assert rc == 124
    assert "=== TIMEOUT ===" in log_path.read_text(encoding="utf-8")
