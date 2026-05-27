"""Tests for PLAT-RT-M3 P0 rt_session_inspect maintainer CLI."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
_SCRIPT = _REPO / "scripts" / "rt" / "rt_session_inspect.py"


def _write_audit(repo_root: Path, session_id: str, entries: list[dict]) -> Path:
    audit_dir = repo_root / "runs" / "rt_sandbox" / "audit"
    audit_dir.mkdir(parents=True, exist_ok=True)
    path = audit_dir / f"{session_id}.json"
    payload = {
        "schema": "rt_session_audit_log_v1",
        "session_id": session_id,
        "entries": entries,
    }
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def test_audit_tail_read_only(tmp_path: Path) -> None:
    sid = "inspect-test-session"
    path = _write_audit(
        tmp_path,
        sid,
        [
            {
                "command_type": "start_session",
                "result": "OK",
                "timestamp_utc": "2026-05-27T00:00:00+00:00",
            },
            {
                "command_type": "subscribe_telemetry",
                "result": "OK",
                "timestamp_utc": "2026-05-27T00:00:01+00:00",
            },
        ],
    )
    before = path.read_text(encoding="utf-8")

    proc = subprocess.run(
        [
            sys.executable,
            str(_SCRIPT),
            "--repo-root",
            str(tmp_path),
            "audit",
            sid,
            "--tail",
            "1",
            "--json",
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=str(_REPO),
    )
    assert proc.returncode == 0, proc.stderr
    out = json.loads(proc.stdout)
    assert out["count"] == 1
    assert out["entries"][-1]["command_type"] == "subscribe_telemetry"
    assert path.read_text(encoding="utf-8") == before


def test_audit_missing_file_exit_1(tmp_path: Path) -> None:
    proc = subprocess.run(
        [
            sys.executable,
            str(_SCRIPT),
            "--repo-root",
            str(tmp_path),
            "audit",
            "missing-session",
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=str(_REPO),
    )
    assert proc.returncode == 1
    assert "Audit not found" in proc.stderr


def test_summary_bridge_disconnected() -> None:
    proc = subprocess.run(
        [
            sys.executable,
            str(_SCRIPT),
            "--url",
            "http://127.0.0.1:1/v1/command",
            "summary",
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=str(_REPO),
    )
    assert proc.returncode == 1
    assert "BRIDGE_DISCONNECTED" in proc.stderr
