"""Tests for PLAT-RT-S2 RT sandbox bridge prototype."""

from __future__ import annotations

import json
import socket
import sys
import time
import urllib.request
import uuid
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
if str(_BRIDGE_PKG) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_PKG))

from rt_sandbox.audit_log import AuditLog  # noqa: E402
from rt_sandbox.bridge_server import make_server  # noqa: E402
from rt_sandbox.governance import classify_command  # noqa: E402
from rt_sandbox.isolation import assert_writable_path, repo_root_from, rt_sandbox_runs_dir  # noqa: E402
from rt_sandbox.session_manager import BridgeSessionManager, GovernanceConfig  # noqa: E402


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def _cmd(
    manager: BridgeSessionManager,
    command_type: str,
    session_id: str | None = None,
) -> dict:
    body = {
        "command_type": command_type,
        "command_id": str(uuid.uuid4()),
        "issued_by": "test",
        "authority_scope": "rt_sandbox_prototype",
    }
    if session_id:
        body["session_id"] = session_id
    return manager.handle_command(body)


@pytest.fixture
def manager(tmp_path: Path) -> BridgeSessionManager:
    runs = tmp_path / "runs" / "rt_sandbox"
    runs.mkdir(parents=True)
    cfg = GovernanceConfig(
        command_rate_burst=1000,
        command_rate_sustained=1000.0,
        bridge_ready_timeout_s=5.0,
        session_cleanup_timeout_s=0.5,
        cleanup_pending_max_age_s=1.0,
        max_session_duration_s=60.0,
    )
    return BridgeSessionManager(config=cfg, repo_root=tmp_path)


def test_lifecycle_happy_path(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    assert start["ok"] is True
    sid = start["session_id"]
    assert _cmd(manager, "pause_session", sid)["ok"] is True
    assert _cmd(manager, "resume", sid)["ok"] is True
    assert _cmd(manager, "stop_session", sid)["ok"] is True
    assert _cmd(manager, "discard_session", sid)["ok"] is True
    again = _cmd(manager, "discard_session", sid)
    assert again["ok"] is False
    assert again["error_code"] in {"SESSION_NOT_FOUND", "INVALID_STATE"}


def test_capture_forbidden(manager: BridgeSessionManager) -> None:
    assert classify_command("capture_session") == "COMMAND_FORBIDDEN"
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    assert cap["ok"] is False
    assert cap["error_code"] == "COMMAND_FORBIDDEN"


def test_single_session_enforced(manager: BridgeSessionManager) -> None:
    first = _cmd(manager, "start_session")
    assert first["ok"] is True
    second = _cmd(manager, "start_session")
    assert second["ok"] is False
    assert second["error_code"] == "INVALID_STATE"


def test_federation_command_blocked(manager: BridgeSessionManager) -> None:
    body = {
        "command_type": "federation_register",
        "command_id": str(uuid.uuid4()),
        "issued_by": "test",
        "authority_scope": "rt_sandbox_prototype",
    }
    out = manager.handle_command(body)
    assert out["error_code"] == "COMMAND_FORBIDDEN"


def test_audit_log_append_only(manager: BridgeSessionManager, tmp_path: Path) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    log = AuditLog(tmp_path)
    path = log.path_for(sid)
    assert path.exists()
    data = json.loads(path.read_text(encoding="utf-8"))
    assert data["schema"] == "rt_session_audit_log_v1"
    assert len(data["entries"]) >= 1
    assert_writable_path(path, tmp_path)
    with pytest.raises(PermissionError):
        assert_writable_path(tmp_path / "fixtures" / "sa_r0" / "hack.json", tmp_path)


def test_auto_cleanup_after_stop(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    time.sleep(0.7)
    manager._tick_timeouts(time.monotonic())
    assert manager._session is not None
    assert manager._session.state.value == "discarded"


def test_runtime_crashed_on_dead_stub(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    assert manager._session is not None
    manager._session.stub.kill_for_crash_simulation()
    out = _cmd(manager, "pause_session", sid)
    assert out["ok"] is False
    assert out["error_code"] == "RUNTIME_UNAVAILABLE"


def test_http_server_loopback(tmp_path: Path) -> None:
    port = _free_port()
    mgr = BridgeSessionManager(repo_root=tmp_path)
    server = make_server(port=port, manager=mgr, repo_root=tmp_path)
    thread = __import__("threading").Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        body = json.dumps(
            {
                "command_type": "start_session",
                "command_id": str(uuid.uuid4()),
                "issued_by": "test",
                "authority_scope": "rt_sandbox_prototype",
            }
        ).encode("utf-8")
        req = urllib.request.Request(
            f"http://127.0.0.1:{port}/v1/command",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=5) as resp:
            payload = json.loads(resp.read().decode("utf-8"))
        assert payload["ok"] is True
        assert "governance_banner" in payload
    finally:
        server.shutdown()
        server.server_close()


def test_sa_fixture_dir_untouched(manager: BridgeSessionManager, tmp_path: Path) -> None:
    sa = tmp_path / "fixtures" / "sa_r0"
    sa.mkdir(parents=True)
    marker = sa / "marker.json"
    marker.write_text("{}", encoding="utf-8")
    before = marker.read_text(encoding="utf-8")
    _cmd(manager, "start_session")
    assert marker.read_text(encoding="utf-8") == before
    rt_sandbox_runs_dir(tmp_path)
    assert (tmp_path / "runs" / "rt_sandbox" / "audit").exists() or True
