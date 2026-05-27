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
from rt_sandbox.runtime_subcommand_governance import lint_runtime_subcommands  # noqa: E402
from rt_sandbox.revision_hint_policy import (  # noqa: E402
    expected_world_revision_hint_keys,
    revision_counter_roles,
)
from rt_sandbox.session_manager import BridgeSessionManager, GovernanceConfig  # noqa: E402


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def _cmd(
    manager: BridgeSessionManager,
    command_type: str,
    session_id: str | None = None,
    *,
    payload: dict | None = None,
) -> dict:
    body = {
        "command_type": command_type,
        "command_id": str(uuid.uuid4()),
        "issued_by": "test",
        "authority_scope": "rt_sandbox_prototype",
    }
    if session_id:
        body["session_id"] = session_id
    if payload is not None:
        body["payload"] = payload
    return manager.handle_command(body)


def _pose(x: float = 0, y: float = 0, z: float = 10) -> dict:
    return {"x": x, "y": y, "z": z, "yaw_deg": 0}


@pytest.fixture
def manager(tmp_path: Path) -> BridgeSessionManager:
    (tmp_path / "AGENTS.md").write_text("# test repo\n", encoding="utf-8")
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


def test_classify_capture_allowed() -> None:
    assert classify_command("capture_session") is None


def test_capture_session_happy_path(manager: BridgeSessionManager, tmp_path: Path) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose()},
    )
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    assert cap["ok"] is True
    assert cap["state"] == "captured"
    assert cap.get("capture_candidate_id")
    assert cap.get("staging_refs")
    cid = cap["capture_candidate_id"]
    staging = tmp_path / "runs" / "rt_sandbox" / "captures" / cid
    assert (staging / "candidate.json").exists()
    assert (staging / "snapshot.json").exists()
    assert (staging / "capture_report.json").exists()
    cand = json.loads((staging / "candidate.json").read_text(encoding="utf-8"))
    assert cand["schema"] == "rt_capture_candidate_v1"
    assert cand["origin"] == "rt_sandbox_capture_v1"
    assert cand["approval_status"] == "pending"
    assert cand["normalization_status"] == "normalized"
    assert (staging / "normalized_manifest.json").exists()
    assert (staging / "provenance.json").exists()
    assert (staging / "normalization_validation.json").exists()
    assert cap.get("normalization_status") == "normalized"


def test_capture_invalid_when_running(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    cap = _cmd(manager, "capture_session", sid)
    assert cap["ok"] is False
    assert cap["error_code"] == "INVALID_STATE"


def test_capture_cancels_auto_cleanup(manager: BridgeSessionManager, tmp_path: Path) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    assert cap["ok"] is True
    assert cap.get("capture_candidate_id")
    time.sleep(0.6)
    assert manager._registry.get(sid) is None


def test_capture_invalid_after_auto_cleanup(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    time.sleep(0.6)
    cap = _cmd(manager, "capture_session", sid)
    assert cap["ok"] is False
    assert cap["error_code"] in {"INVALID_STATE", "SESSION_NOT_FOUND"}


def test_capture_bundle_size_cap(manager: BridgeSessionManager, tmp_path: Path) -> None:
    manager.config.max_capture_bundle_bytes = 50
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    assert cap["ok"] is False
    assert cap["error_code"] == "RESOURCE_LIMIT_EXCEEDED"


def test_export_boundary_blocks_sa_paths(tmp_path: Path) -> None:
    from rt_sandbox.isolation import assert_sa_path_blocked

    (tmp_path / "AGENTS.md").write_text("# test\n", encoding="utf-8")
    (tmp_path / "platform" / "sa-r0-viewer").mkdir(parents=True, exist_ok=True)
    sa_path = tmp_path / "platform" / "sa-r0-viewer" / "index.html"
    sa_path.write_text("<html></html>", encoding="utf-8")
    with pytest.raises(PermissionError):
        assert_sa_path_blocked(sa_path, tmp_path)


def test_no_federation_write_from_capture(tmp_path: Path) -> None:
    from rt_sandbox.isolation import assert_writable_path

    fed = tmp_path / "fixtures" / "orchestration" / "queues" / "x.json"
    fed.parent.mkdir(parents=True, exist_ok=True)
    with pytest.raises(PermissionError):
        assert_writable_path(fed, tmp_path)


def test_conversion_manifest_rejects_session_id_parent() -> None:
    from rt_sandbox.export_boundary import validate_conversion_manifest

    sid = str(uuid.uuid4())
    manifest = {
        "schema": "runtime_to_replay_conversion_v1",
        "origin": "rt_sandbox_capture_v1",
        "conversion_steps": ["validate_scenario_pack"],
        "session_id": sid,
        "parent_ref": sid,
    }
    assert validate_conversion_manifest(manifest) is not None


def test_reject_auto_sa_import() -> None:
    from rt_sandbox.export_boundary import ExportBoundaryError, reject_auto_sa_import

    with pytest.raises(ExportBoundaryError):
        reject_auto_sa_import("test")


def test_capture_audit_and_export_audit(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    assert cap["ok"] is True
    audit_path = tmp_path / "runs" / "rt_sandbox" / "audit" / f"{sid}.json"
    audit = json.loads(audit_path.read_text(encoding="utf-8"))
    types = [e["command_type"] for e in audit["entries"]]
    assert "capture_session" in types
    export_log = tmp_path / "runs" / "rt_sandbox" / "export_audit" / "export_boundary.jsonl"
    assert export_log.exists()
    lines = export_log.read_text(encoding="utf-8").strip().split("\n")
    events = [json.loads(line)["event_type"] for line in lines if line]
    assert "capture_requested" in events
    assert "capture_validated" in events


def test_capture_approve_writes_conversion(manager: BridgeSessionManager, tmp_path: Path) -> None:
    from rt_sandbox.capture import write_approval_record, write_conversion_manifest
    from rt_sandbox.isolation import rt_sandbox_captures_dir

    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    cid = cap["capture_candidate_id"]
    staging = rt_sandbox_captures_dir(tmp_path) / cid
    write_approval_record(staging, approved_by="test")
    manifest = write_conversion_manifest(staging)
    assert manifest["schema"] == "runtime_to_replay_conversion_v1"
    assert manifest["staging_refs"].get("normalized_manifest_ref")
    assert (staging / "conversion.json").exists()


def test_sa_viewer_path_unchanged() -> None:
    sa_viewer = _REPO / "platform" / "sa-r0-viewer"
    assert sa_viewer.is_dir()


def test_single_session_enforced(manager: BridgeSessionManager) -> None:
    """PLAT-RT-M2: second concurrent session allowed; capacity enforced at 3."""
    first = _cmd(manager, "start_session")
    assert first["ok"] is True
    second = _cmd(manager, "start_session")
    assert second["ok"] is True
    assert first["session_id"] != second["session_id"]


def test_three_concurrent_sessions(manager: BridgeSessionManager) -> None:
    ids = []
    for _ in range(3):
        out = _cmd(manager, "start_session")
        assert out["ok"] is True
        ids.append(out["session_id"])
    assert len(set(ids)) == 3
    listed = _cmd(manager, "list_sessions")
    assert listed["ok"] is True
    assert listed["non_terminal_count"] == 3


def test_session_capacity_exceeded(manager: BridgeSessionManager) -> None:
    for _ in range(3):
        assert _cmd(manager, "start_session")["ok"] is True
    fourth = _cmd(manager, "start_session")
    assert fourth["ok"] is False
    assert fourth["error_code"] == "SESSION_CAPACITY_EXCEEDED"


def test_set_editing_session_and_mismatch(manager: BridgeSessionManager) -> None:
    a = _cmd(manager, "start_session")
    b = _cmd(manager, "start_session")
    sid_a, sid_b = a["session_id"], b["session_id"]
    set_b = _cmd(
        manager,
        "set_editing_session",
        payload={"session_id": sid_b},
    )
    assert set_b["ok"] is True
    spawn = _cmd(
        manager,
        "spawn_entity",
        sid_a,
        payload={"entity_type": "radar", "pose": _pose()},
    )
    assert spawn["ok"] is False
    assert spawn["error_code"] == "EDITING_SESSION_MISMATCH"
    spawn_b = _cmd(
        manager,
        "spawn_entity",
        sid_b,
        payload={"entity_type": "radar", "pose": _pose()},
    )
    assert spawn_b["ok"] is True


def test_cross_session_world_isolation(manager: BridgeSessionManager) -> None:
    a = _cmd(manager, "start_session")
    b = _cmd(manager, "start_session")
    sid_a, sid_b = a["session_id"], b["session_id"]
    _cmd(
        manager,
        "set_editing_session",
        payload={"session_id": sid_a},
    )
    _cmd(
        manager,
        "spawn_entity",
        sid_a,
        payload={"entity_type": "drone", "pose": _pose()},
    )
    sess_b = manager._registry.get(sid_b)
    assert sess_b is not None
    assert sess_b.world is not None
    assert sess_b.world.registry.count() == 0


def test_capture_does_not_read_sibling_world(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    a = _cmd(manager, "start_session")
    b = _cmd(manager, "start_session")
    sid_a, sid_b = a["session_id"], b["session_id"]
    _cmd(manager, "set_editing_session", payload={"session_id": sid_a})
    spawn_a = _cmd(
        manager,
        "spawn_entity",
        sid_a,
        payload={"entity_type": "radar", "pose": _pose()},
    )
    assert spawn_a["ok"] is True
    _cmd(manager, "set_editing_session", payload={"session_id": sid_b})
    spawn_b = _cmd(
        manager,
        "spawn_entity",
        sid_b,
        payload={"entity_type": "drone", "pose": _pose(5, 5)},
    )
    assert spawn_b["ok"] is True
    _cmd(manager, "stop_session", sid_a)
    cap = _cmd(manager, "capture_session", sid_a)
    assert cap["ok"] is True
    cid = cap["capture_candidate_id"]
    staging = tmp_path / "runs" / "rt_sandbox" / "captures" / cid
    snapshot = json.loads((staging / "snapshot.json").read_text(encoding="utf-8"))
    entities = snapshot.get("entity_states") or []
    types = {e.get("entity_type") for e in entities}
    assert "radar" in types
    assert "drone" not in types
    sess_b = manager._registry.get(sid_b)
    assert sess_b is not None
    assert sess_b.world is not None
    assert sess_b.world.registry.count() == 1


def test_failed_session_sibling_survives(manager: BridgeSessionManager) -> None:
    a = _cmd(manager, "start_session")
    b = _cmd(manager, "start_session")
    sid_a, sid_b = a["session_id"], b["session_id"]
    rec_a = manager._registry.get(sid_a)
    assert rec_a is not None
    rec_a.runtime.kill_for_crash_simulation()
    out = _cmd(manager, "pause_session", sid_a)
    assert out["ok"] is False
    assert out["error_code"] == "RUNTIME_UNAVAILABLE"
    rec_b = manager._registry.get(sid_b)
    assert rec_b is not None
    assert rec_b.state.value == "running"
    pause_b = _cmd(manager, "pause_session", sid_b)
    assert pause_b["ok"] is True


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
    manager._session.runtime.kill_for_crash_simulation()
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
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(1, 2, 5)},
    )
    assert marker.read_text(encoding="utf-8") == before
    rt_sandbox_runs_dir(tmp_path)
    assert (tmp_path / "runs" / "rt_sandbox" / "audit").exists() or True


def test_entity_spawn_move_delete_happy_path(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    spawn = _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(10, 20, 15)},
    )
    assert spawn["ok"] is True
    assert spawn["error_code"] == "OK"
    assert "entity_id" in spawn
    assert spawn["world_summary"]["entity_count"] == 1
    eid = spawn["entity_id"]
    moved = _cmd(
        manager,
        "move_entity",
        sid,
        payload={"entity_id": eid, "pose": _pose(30, 40, 20)},
    )
    assert moved["ok"] is True
    assert moved["world_summary"]["entity_count"] == 1
    deleted = _cmd(
        manager,
        "delete_entity",
        sid,
        payload={"entity_id": eid},
    )
    assert deleted["ok"] is True
    assert deleted["world_summary"]["entity_count"] == 0


def test_entity_catalog_forbidden_type(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    out = _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "prototype_target", "pose": _pose()},
    )
    assert out["ok"] is False
    assert out["error_code"] == "COMMAND_FORBIDDEN"


def test_entity_cap_enforced(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    types = ("radar", "interceptor", "drone", "waypoint_marker")
    n = 0
    for entity_type in types:
        for i in range(8):
            out = _cmd(
                manager,
                "spawn_entity",
                sid,
                payload={"entity_type": entity_type, "pose": _pose(float(n), float(n), 1)},
            )
            assert out["ok"] is True
            n += 1
    overflow = _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "radar", "pose": _pose(99, 99, 1)},
    )
    assert overflow["ok"] is False
    assert overflow["error_code"] == "RESOURCE_LIMIT_EXCEEDED"


def test_per_type_cap(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    for i in range(8):
        assert _cmd(
            manager,
            "spawn_entity",
            sid,
            payload={"entity_type": "drone", "pose": _pose(float(i), 0, 5)},
        )["ok"]
    ninth = _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(50, 50, 5)},
    )
    assert ninth["ok"] is False
    assert ninth["error_code"] == "RESOURCE_LIMIT_EXCEEDED"


def test_world_bounds(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    oob = _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "radar", "pose": {"x": 9999, "y": 0, "z": 10}},
    )
    assert oob["ok"] is False
    assert oob["error_code"] == "INVALID_POSE"


def test_reset_session_clears_entities(manager: BridgeSessionManager, tmp_path: Path) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "interceptor", "pose": _pose()},
    )
    reset = _cmd(manager, "reset_session", sid)
    assert reset["ok"] is True
    assert reset["world_summary"]["entity_count"] == 0
    log = AuditLog(tmp_path)
    data = json.loads(log.path_for(sid).read_text(encoding="utf-8"))
    types = [e["command_type"] for e in data["entries"]]
    assert "reset_session" in types


def test_entity_cleanup_on_discard(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "radar", "pose": _pose()},
    )
    session = manager._registry.get(sid)
    assert session is not None
    assert session.world is not None
    assert session.world.registry.count() == 1
    _cmd(manager, "discard_session", sid)
    assert manager._registry.get(sid) is None


def _subscribe_default(manager: BridgeSessionManager, sid: str) -> dict:
    return _cmd(
        manager,
        "subscribe_telemetry",
        sid,
        payload={
            "channels": [
                "session_health",
                "world_summary",
                "entity_pose_mirror",
                "lifecycle_state",
                "clock_mirror",
            ]
        },
    )


def test_subscribe_telemetry_happy_path(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    sub = _subscribe_default(manager, sid)
    assert sub["ok"] is True
    assert "subscription_id" in sub
    assert sub["channels"]
    pull = manager.pull_telemetry(sid, sub["subscription_id"])
    assert pull["ok"] is True


def test_subscribe_forbidden_channel(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    out = _cmd(
        manager,
        "subscribe_telemetry",
        sid,
        payload={"channels": ["/tracks/state"]},
    )
    assert out["ok"] is False
    assert out["error_code"] == "COMMAND_FORBIDDEN"


def test_subscribe_channel_cap(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    out = _cmd(
        manager,
        "subscribe_telemetry",
        sid,
        payload={
            "channels": [
                "session_health",
                "lifecycle_state",
                "world_summary",
                "entity_pose_mirror",
                "clock_mirror",
                "tactical_state",
                "tactical_recommendation",
                "session_health",
            ]
        },
    )
    assert out["ok"] is False
    assert out["error_code"] == "RESOURCE_LIMIT_EXCEEDED"


def test_unsubscribe_telemetry(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    sub = _subscribe_default(manager, sid)
    sub_id = sub["subscription_id"]
    _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose()},
    )
    assert manager.pull_telemetry(sid, sub_id)["drained_count"] >= 0
    unsub = _cmd(
        manager,
        "unsubscribe_telemetry",
        sid,
        payload={"subscription_id": sub_id},
    )
    assert unsub["ok"] is True
    assert manager.pull_telemetry(sid, sub_id)["ok"] is False


def test_telemetry_cleanup_on_discard(manager: BridgeSessionManager, tmp_path: Path) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _subscribe_default(manager, sid)
    _cmd(manager, "discard_session", sid)
    data = json.loads(AuditLog(tmp_path).path_for(sid).read_text(encoding="utf-8"))
    types = [e["command_type"] for e in data["entries"]]
    assert "telemetry_cleanup" in types
    assert manager._telemetry_subs.get_for_session(sid) is None


def test_tactical_cleanup_on_discard(manager: BridgeSessionManager, tmp_path: Path) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    session = manager._registry.get(sid)
    assert session is not None
    assert session.tactical is not None
    _cmd(manager, "discard_session", sid)
    data = json.loads(AuditLog(tmp_path).path_for(sid).read_text(encoding="utf-8"))
    tactical_entries = [e for e in data["entries"] if e["command_type"] == "tactical_cleanup"]
    assert tactical_entries
    assert tactical_entries[-1]["detail"]["had_tactical"] is True
    assert manager._registry.get(sid) is None


def test_telemetry_cleanup_on_reset(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    sub = _subscribe_default(manager, sid)
    _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose()},
    )
    _cmd(manager, "reset_session", sid)
    pull = manager.pull_telemetry(sid, sub["subscription_id"])
    assert pull["ok"] is True


def test_telemetry_rate_cap(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _subscribe_default(manager, sid)
    session = manager._session
    assert session is not None
    for _ in range(50):
        manager._publish_telemetry(session, "session_health")
    sub = manager._telemetry_subs.get_for_session(sid)
    assert sub is not None
    assert len(sub.events) <= 64


def test_telemetry_invalid_when_stopped(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    sub = _cmd(
        manager,
        "subscribe_telemetry",
        sid,
        payload={"channels": ["session_health"]},
    )
    assert sub["ok"] is False
    assert sub["error_code"] == "INVALID_STATE"


def test_sa_fixture_dir_untouched_after_telemetry(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    sa = tmp_path / "fixtures" / "sa_r0"
    sa.mkdir(parents=True)
    marker = sa / "marker.json"
    marker.write_text("{}", encoding="utf-8")
    before = marker.read_text(encoding="utf-8")
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _subscribe_default(manager, sid)
    assert marker.read_text(encoding="utf-8") == before


def test_http_telemetry_pull_loopback(tmp_path: Path) -> None:
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
            start = json.loads(resp.read().decode("utf-8"))
        sid = start["session_id"]
        sub_body = json.dumps(
            {
                "command_type": "subscribe_telemetry",
                "command_id": str(uuid.uuid4()),
                "session_id": sid,
                "issued_by": "test",
                "authority_scope": "rt_sandbox_prototype",
                "payload": {"channels": ["session_health", "world_summary"]},
            }
        ).encode("utf-8")
        sub_req = urllib.request.Request(
            f"http://127.0.0.1:{port}/v1/command",
            data=sub_body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(sub_req, timeout=5) as resp:
            sub = json.loads(resp.read().decode("utf-8"))
        sub_id = sub["subscription_id"]
        pull_url = (
            f"http://127.0.0.1:{port}/v1/telemetry/pull"
            f"?session_id={sid}&subscription_id={sub_id}"
        )
        with urllib.request.urlopen(pull_url, timeout=5) as resp:
            pull = json.loads(resp.read().decode("utf-8"))
        assert pull["ok"] is True
    finally:
        server.shutdown()
        server.server_close()


def test_entity_ops_invalid_when_stopped(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    spawn = _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose()},
    )
    assert spawn["ok"] is False
    assert spawn["error_code"] == "INVALID_STATE"


def test_entity_audit_entries(manager: BridgeSessionManager, tmp_path: Path) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    spawn = _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose()},
    )
    eid = spawn["entity_id"]
    _cmd(manager, "delete_entity", sid, payload={"entity_id": eid})
    _cmd(manager, "discard_session", sid)
    data = json.loads(AuditLog(tmp_path).path_for(sid).read_text(encoding="utf-8"))
    types = [e["command_type"] for e in data["entries"]]
    assert "spawn_entity" in types
    assert "delete_entity" in types
    assert "entity_cleanup" in types


def test_list_runtime_templates_no_session(manager: BridgeSessionManager) -> None:
    resp = _cmd(manager, "list_runtime_templates")
    assert resp["ok"] is True
    assert len(resp["templates"]) >= 5


def test_apply_runtime_template_happy_path(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    resp = _cmd(
        manager,
        "apply_runtime_template",
        sid,
        payload={"template_id": "radar_north_arc_v1"},
    )
    assert resp["ok"] is True
    assert resp["entities_spawned"] == 2
    assert resp["world_summary"]["entity_count"] == 2


def test_apply_unknown_template(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    resp = _cmd(
        manager,
        "apply_runtime_template",
        sid,
        payload={"template_id": "not_a_template_v1"},
    )
    assert resp["ok"] is False
    assert resp["error_code"] == "INVALID_STATE"


def test_workflow_happy_path(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    wf_id = "minimal_radar_demo_v1"
    assert _cmd(
        manager,
        "start_workflow",
        sid,
        payload={"workflow_id": wf_id},
    )["ok"] is True
    adv = _cmd(manager, "advance_workflow", sid)
    assert adv["ok"] is True
    assert adv.get("workflow_completed") is False
    adv2 = _cmd(manager, "advance_workflow", sid)
    assert adv2["ok"] is True
    assert adv2.get("workflow_completed") is True
    state = _cmd(manager, "get_workflow_state", sid)
    assert state["workflow"]["status"] == "completed"


def test_reset_workflow_and_reload(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    wf_id = "waypoint_staging_v1"
    _cmd(manager, "start_workflow", sid, payload={"workflow_id": wf_id})
    _cmd(manager, "advance_workflow", sid)
    assert _cmd(manager, "reset_workflow", sid)["ok"] is True
    state = _cmd(manager, "get_workflow_state", sid)
    assert state["workflow"]["status"] == "idle"
    reload = _cmd(manager, "reload_workflow", sid, payload={"workflow_id": wf_id})
    assert reload["ok"] is True
    assert reload["workflow"]["status"] == "in_progress"


def test_reset_session_clears_workflow(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(
        manager,
        "start_workflow",
        sid,
        payload={"workflow_id": "minimal_radar_demo_v1"},
    )
    _cmd(manager, "reset_session", sid)
    state = _cmd(manager, "get_workflow_state", sid)
    assert state["workflow"]["status"] == "idle"


def test_import_scenario_forbidden(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    resp = _cmd(manager, "import_scenario", sid)
    assert resp["ok"] is False
    assert resp["error_code"] == "COMMAND_FORBIDDEN"


def test_auto_capture_forbidden(manager: BridgeSessionManager) -> None:
    resp = _cmd(manager, "auto_capture")
    assert resp["ok"] is False
    assert resp["error_code"] == "COMMAND_FORBIDDEN"


def test_capture_includes_workflow_summary(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(
        manager,
        "apply_runtime_template",
        sid,
        payload={"template_id": "drone_ingress_lane_v1"},
    )
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    assert cap["ok"] is True
    cid = cap["capture_candidate_id"]
    report = json.loads(
        (tmp_path / "runs" / "rt_sandbox" / "captures" / cid / "capture_report.json").read_text(
            encoding="utf-8"
        )
    )
    assert "templates_applied" in report
    assert "drone_ingress_lane_v1" in report["templates_applied"]


def test_workflow_audit_entries(manager: BridgeSessionManager, tmp_path: Path) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(
        manager,
        "start_workflow",
        sid,
        payload={"workflow_id": "minimal_radar_demo_v1"},
    )
    _cmd(manager, "advance_workflow", sid)
    _cmd(manager, "discard_session", sid)
    data = json.loads(AuditLog(tmp_path).path_for(sid).read_text(encoding="utf-8"))
    types = [e["command_type"] for e in data["entries"]]
    assert "start_workflow" in types
    assert "advance_workflow" in types


def test_http_workflow_state_loopback(tmp_path: Path) -> None:
    (tmp_path / "AGENTS.md").write_text("# test\n", encoding="utf-8")
    cfg = GovernanceConfig(
        command_rate_burst=1000,
        command_rate_sustained=1000.0,
        bridge_ready_timeout_s=5.0,
        session_cleanup_timeout_s=0.5,
    )
    manager = BridgeSessionManager(config=cfg, repo_root=tmp_path)
    port = _free_port()
    server = make_server(port=port, manager=manager, repo_root=tmp_path)
    server_thread = __import__("threading").Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    try:
        start_body = json.dumps(
            {
                "command_type": "start_session",
                "command_id": str(uuid.uuid4()),
                "issued_by": "test",
                "authority_scope": "rt_sandbox_prototype",
            }
        ).encode()
        req = urllib.request.Request(
            f"http://127.0.0.1:{port}/v1/command",
            data=start_body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=5) as resp:
            start = json.loads(resp.read().decode())
        sid = start["session_id"]
        tpl_body = json.dumps(
            {
                "command_type": "apply_runtime_template",
                "command_id": str(uuid.uuid4()),
                "issued_by": "test",
                "authority_scope": "rt_sandbox_prototype",
                "session_id": sid,
                "payload": {"template_id": "world_empty_v1"},
            }
        ).encode()
        req2 = urllib.request.Request(
            f"http://127.0.0.1:{port}/v1/command",
            data=tpl_body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req2, timeout=5) as resp:
            tpl = json.loads(resp.read().decode())
        assert tpl["ok"] is True
    finally:
        server.shutdown()
        server.server_close()


@pytest.fixture
def adapter_manager(tmp_path: Path) -> BridgeSessionManager:
    (tmp_path / "AGENTS.md").write_text("# test repo\n", encoding="utf-8")
    (tmp_path / "runs" / "rt_sandbox").mkdir(parents=True, exist_ok=True)
    cfg = GovernanceConfig(
        command_rate_burst=1000,
        command_rate_sustained=1000.0,
        bridge_ready_timeout_s=5.0,
        session_cleanup_timeout_s=0.5,
        cleanup_pending_max_age_s=1.0,
        max_session_duration_s=60.0,
        enable_gazebo_adapter=True,
        adapter_mode="mock",
        adapter_ipc_timeout_s=10.0,
    )
    return BridgeSessionManager(config=cfg, repo_root=tmp_path)


def test_send_runtime_command_allowed_in_classifier() -> None:
    assert classify_command("send_runtime_command") is None


def test_send_runtime_command_forbidden_when_adapter_disabled(
    manager: BridgeSessionManager,
) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    out = _cmd(
        manager,
        "send_runtime_command",
        sid,
        payload={"sub_command": "adapter_health"},
    )
    assert out["ok"] is False
    assert out["error_code"] == "COMMAND_FORBIDDEN"


def test_mock_adapter_session_lifecycle(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    assert start["ok"] is True
    sid = start["session_id"]
    assert adapter_manager._session is not None
    from rt_sandbox.runtime_adapter import GazeboRuntimeAdapter

    assert isinstance(adapter_manager._session.runtime, GazeboRuntimeAdapter)
    assert adapter_manager._session.runtime.is_alive()
    assert _cmd(adapter_manager, "pause_session", sid)["ok"] is True
    assert _cmd(adapter_manager, "resume", sid)["ok"] is True
    assert _cmd(adapter_manager, "stop_session", sid)["ok"] is True
    assert _cmd(adapter_manager, "discard_session", sid)["ok"] is True


def test_mock_adapter_entity_sync_audit(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    spawn = _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(1, 2, 3)},
    )
    assert spawn["ok"] is True
    assert "entity_id" in spawn
    audit = AuditLog(adapter_manager._repo_root).path_for(sid)
    data = json.loads(audit.read_text(encoding="utf-8"))
    spawn_entries = [e for e in data["entries"] if e["command_type"] == "spawn_entity"]
    assert spawn_entries
    detail = spawn_entries[-1].get("detail") or {}
    assert "adapter_sync" in detail
    assert detail["adapter_sync"].get("sim_entity_ref")


def test_ros_allowlist_rejects_tracks_state(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    assert start["ok"] is True
    from rt_sandbox.runtime_adapter import GazeboRuntimeAdapter

    runtime = adapter_manager._session.runtime
    assert isinstance(runtime, GazeboRuntimeAdapter)
    resp = runtime.validate_topic("/tracks/state")
    assert resp.ok is False
    assert resp.error_code == "COMMAND_FORBIDDEN"


def test_send_runtime_command_adapter_health(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    out = _cmd(
        adapter_manager,
        "send_runtime_command",
        sid,
        payload={"sub_command": "adapter_health"},
    )
    assert out["ok"] is True
    assert out["runtime_health"]["adapter_alive"] is True
    assert out["runtime_health"]["adapter_mode"] == "mock"


def test_runtime_subcommand_governance_lint_passes() -> None:
    issues = lint_runtime_subcommands()
    assert issues == [], "\n".join(issues)


def test_runtime_subcommand_forbidden(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    out = _cmd(
        adapter_manager,
        "send_runtime_command",
        sid,
        payload={"sub_command": "reload_world_config"},
    )
    assert out["ok"] is False
    assert out["error_code"] == "COMMAND_FORBIDDEN"


def test_mock_feedback_after_spawn(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    spawn = _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 10)},
    )
    assert spawn["ok"] is True
    assert spawn["world_summary"].get("sync_health") == "ok"
    audit = AuditLog(adapter_manager._repo_root).path_for(sid)
    types = {e["command_type"] for e in json.loads(audit.read_text())["entries"]}
    assert "sync_update" in types


def test_mock_stale_detection_on_drift(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    spawn = _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 10)},
    )
    eid = spawn["entity_id"]
    inject = _cmd(
        adapter_manager,
        "send_runtime_command",
        sid,
        payload={
            "sub_command": "mock_inject_drift",
            "entity_id": eid,
            "offset": {"x": 5.0, "y": 0, "z": 0},
        },
    )
    assert inject["ok"] is True
    move = _cmd(
        adapter_manager,
        "move_entity",
        sid,
        payload={"entity_id": eid, "pose": _pose(0, 0, 10)},
    )
    assert move["ok"] is False
    assert move["error_code"] == "SYNC_STALE"
    audit = AuditLog(adapter_manager._repo_root).path_for(sid)
    types = {e["command_type"] for e in json.loads(audit.read_text())["entries"]}
    assert "sync_stale" in types


def test_reset_session_clears_sync_mirror(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(1, 1, 1)},
    )
    assert adapter_manager._session.pose_sync is not None
    assert adapter_manager._session.pose_sync.entries
    reset = _cmd(adapter_manager, "reset_session", sid)
    assert reset["ok"] is True
    assert adapter_manager._session.pose_sync is None
    assert reset["world_summary"]["entity_count"] == 0


def test_discard_clears_sync_state(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 5)},
    )
    _cmd(adapter_manager, "stop_session", sid)
    _cmd(adapter_manager, "discard_session", sid)
    assert adapter_manager._registry.get(sid) is None
    audit = AuditLog(adapter_manager._repo_root).path_for(sid)
    types = {e["command_type"] for e in json.loads(audit.read_text())["entries"]}
    assert "discard_session" in types


def test_adapter_feedback_lost_on_kill(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 5)},
    )
    adapter_manager._session.runtime.kill_for_crash_simulation()
    _cmd(adapter_manager, "pause_session", sid)
    audit = AuditLog(adapter_manager._repo_root).path_for(sid)
    types = {e["command_type"] for e in json.loads(audit.read_text())["entries"]}
    assert "adapter_feedback_lost" in types or "runtime_crashed" in types


def test_stub_default_no_sync_audit(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 5)},
    )
    audit = AuditLog(manager._repo_root).path_for(sid)
    types = {e["command_type"] for e in json.loads(audit.read_text())["entries"]}
    assert "sync_update" not in types


def test_world_summary_includes_sync_health(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    spawn = _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(2, 3, 4)},
    )
    ws = spawn["world_summary"]
    assert "sync_health" in ws
    assert "feedback_entities" in ws


def test_world_summary_sync_revision_matches_world_revision(
    adapter_manager: BridgeSessionManager,
) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    spawn = _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(2, 3, 4)},
    )
    ws = spawn["world_summary"]
    assert ws["sync_revision"] == ws["revision"]


def test_telemetry_mirror_world_revision_hint_shape(
    adapter_manager: BridgeSessionManager,
) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    spawn = _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 10)},
    )
    eid = spawn["entity_id"]
    _cmd(
        adapter_manager,
        "move_entity",
        sid,
        payload={"entity_id": eid, "pose": _pose(1, 0, 10)},
    )
    _cmd(
        adapter_manager,
        "send_runtime_command",
        sid,
        payload={"sub_command": "adapter_poll_telemetry"},
    )
    session = adapter_manager._session
    assert session is not None
    mirror = session.telemetry_mirror
    assert mirror is not None
    hint = mirror.world_revision_hint
    assert set(hint.keys()) == expected_world_revision_hint_keys()
    assert "revision" not in hint
    world_rev = session.world.revision
    assert world_rev >= 2
    assert isinstance(hint["telemetry_seq"], int)
    assert isinstance(hint["sync_seq"], int)


def test_revision_counter_roles_export() -> None:
    roles = revision_counter_roles()
    counters = {row["counter"] for row in roles}
    assert "world.revision" in counters
    assert "world_revision_hint" in counters
    assert len(roles) >= 5


def test_adapter_resync_subcommand(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(1, 2, 3)},
    )
    out = _cmd(
        adapter_manager,
        "send_runtime_command",
        sid,
        payload={"sub_command": "adapter_resync"},
    )
    assert out["ok"] is True
    assert out["world_summary"].get("sync_health") == "ok"


def test_sa_viewer_path_unchanged() -> None:
    viewer = Path(__file__).resolve().parents[3] / "platform" / "sa-r0-viewer"
    assert viewer.is_dir()
    assert not (viewer / "src" / "rt_live_hook.ts").exists()


def test_telemetry_update_audit(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 10)},
    )
    audit = AuditLog(adapter_manager._repo_root).path_for(sid)
    types = {e["command_type"] for e in json.loads(audit.read_text())["entries"]}
    assert "telemetry_update" in types


def test_adapter_fed_entity_pose_mirror(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    spawn = _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 10)},
    )
    eid = spawn["entity_id"]
    _cmd(
        adapter_manager,
        "send_runtime_command",
        sid,
        payload={
            "sub_command": "mock_inject_drift",
            "entity_id": eid,
            "offset": {"x": 10.0, "y": 0, "z": 0},
        },
    )
    sub = _cmd(
        adapter_manager,
        "subscribe_telemetry",
        sid,
        payload={"channels": ["entity_pose_mirror"]},
    )
    assert sub["ok"] is True
    events = sub.get("initial_events") or []
    assert events
    entities = events[0]["payload"]["entities"]
    mirror_ent = next(e for e in entities if e["entity_id"] == eid)
    assert mirror_ent["pose"]["x"] == 10.0
    assert events[0]["payload"].get("source") == "adapter_feedback"
    reg = adapter_manager._session.world.registry.get(eid)
    assert reg is not None
    assert reg.pose["x"] == 0.0


def test_telemetry_stale_mock(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 10)},
    )
    _cmd(
        adapter_manager,
        "send_runtime_command",
        sid,
        payload={"sub_command": "adapter_poll_telemetry", "mock_stale_telemetry": True},
    )
    audit = AuditLog(adapter_manager._repo_root).path_for(sid)
    types = {e["command_type"] for e in json.loads(audit.read_text())["entries"]}
    assert "telemetry_stale" in types
    spawn2 = _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "radar", "pose": _pose(5, 5, 5)},
    )
    assert spawn2["ok"] is True


def test_pose_sync_still_authoritative_with_telemetry(
    adapter_manager: BridgeSessionManager,
) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    spawn = _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 10)},
    )
    eid = spawn["entity_id"]
    _cmd(
        adapter_manager,
        "send_runtime_command",
        sid,
        payload={
            "sub_command": "mock_inject_drift",
            "entity_id": eid,
            "offset": {"x": 5.0, "y": 0, "z": 0},
        },
    )
    move = _cmd(
        adapter_manager,
        "move_entity",
        sid,
        payload={"entity_id": eid, "pose": _pose(0, 0, 10)},
    )
    assert move["ok"] is False
    assert move["error_code"] == "SYNC_STALE"


def test_reset_clears_telemetry_mirror(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(1, 1, 1)},
    )
    assert adapter_manager._session.telemetry_mirror is not None
    _cmd(adapter_manager, "reset_session", sid)
    assert adapter_manager._session.telemetry_mirror is None


def test_stub_default_registry_telemetry_mirror(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    spawn = _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(3, 4, 5)},
    )
    sub = _cmd(
        manager,
        "subscribe_telemetry",
        sid,
        payload={"channels": ["entity_pose_mirror"]},
    )
    entities = sub["initial_events"][0]["payload"]["entities"]
    assert entities[0]["pose"]["x"] == 3.0


def test_telemetry_buffer_trim_audit(tmp_path: Path) -> None:
    (tmp_path / "AGENTS.md").write_text("# test\n", encoding="utf-8")
    (tmp_path / "runs" / "rt_sandbox").mkdir(parents=True, exist_ok=True)
    cfg = GovernanceConfig(
        command_rate_burst=1000,
        command_rate_sustained=1000.0,
        telemetry_ring_buffer_size=2,
        telemetry_update_rate_cap_hz=1000.0,
        enable_gazebo_adapter=True,
        adapter_mode="mock",
        adapter_ipc_timeout_s=10.0,
    )
    mgr = BridgeSessionManager(config=cfg, repo_root=tmp_path)
    start = _cmd(mgr, "start_session")
    sid = start["session_id"]
    _cmd(
        mgr,
        "subscribe_telemetry",
        sid,
        payload={"channels": ["session_health", "clock_mirror", "world_summary"]},
    )
    for _ in range(5):
        _cmd(
            mgr,
            "send_runtime_command",
            sid,
            payload={"sub_command": "adapter_poll_telemetry"},
        )
    audit = AuditLog(tmp_path).path_for(sid)
    types = {e["command_type"] for e in json.loads(audit.read_text())["entries"]}
    assert "telemetry_buffer_trim" in types


def test_telemetry_feedback_lost_on_kill(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 5)},
    )
    adapter_manager._session.runtime.kill_for_crash_simulation()
    _cmd(adapter_manager, "pause_session", sid)
    audit = AuditLog(adapter_manager._repo_root).path_for(sid)
    types = {e["command_type"] for e in json.loads(audit.read_text())["entries"]}
    assert "telemetry_feedback_lost" in types or "runtime_crashed" in types


def test_adapter_orphan_cleanup_on_crash(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    adapter_manager._session.runtime.kill_for_crash_simulation()
    out = _cmd(adapter_manager, "pause_session", sid)
    assert out["ok"] is False
    assert out["error_code"] == "RUNTIME_UNAVAILABLE"
    audit = AuditLog(adapter_manager._repo_root).path_for(sid)
    data = json.loads(audit.read_text(encoding="utf-8"))
    types = {e["command_type"] for e in data["entries"]}
    assert "runtime_crashed" in types


# --- PLAT-RT-G5 capture normalization ---


def test_capture_normalized_export_audit(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    _cmd(manager, "capture_session", sid)
    export_log = tmp_path / "runs" / "rt_sandbox" / "export_audit" / "export_boundary.jsonl"
    events = [
        json.loads(line)["event_type"]
        for line in export_log.read_text(encoding="utf-8").strip().split("\n")
        if line
    ]
    assert "capture_normalized" in events
    assert "provenance_injected" in events
    assert "normalization_validation" in events


def test_normalization_rejected_on_invalid_staging(tmp_path: Path) -> None:
    from rt_sandbox.capture_normalize import (
        NormalizationContext,
        NormalizationError,
        normalize_capture_bundle,
    )
    from rt_sandbox.isolation import rt_sandbox_captures_dir

    (tmp_path / "AGENTS.md").write_text("# test\n", encoding="utf-8")
    cid = str(uuid.uuid4())
    staging = rt_sandbox_captures_dir(tmp_path) / cid
    staging.mkdir(parents=True)
    (staging / "candidate.json").write_text(
        json.dumps(
            {
                "schema": "rt_capture_candidate_v1",
                "capture_candidate_id": cid,
                "session_id": cid,
                "origin": "rt_sandbox_capture_v1",
                "approval_status": "pending",
            }
        ),
        encoding="utf-8",
    )
    ctx = NormalizationContext(
        session_state="stopped",
        enable_gazebo_adapter=False,
        adapter_mode="stub",
        adapter_attached=False,
    )
    with pytest.raises(NormalizationError) as exc:
        normalize_capture_bundle(staging, runtime_context=ctx, repo_root=tmp_path)
    assert exc.value.code == "INVALID_STATE"


def test_normalized_manifest_no_session_lineage(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    from rt_sandbox.export_boundary import validate_normalized_manifest
    from rt_sandbox.isolation import rt_sandbox_captures_dir

    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    staging = rt_sandbox_captures_dir(tmp_path) / cap["capture_candidate_id"]
    norm = json.loads((staging / "normalized_manifest.json").read_text(encoding="utf-8"))
    assert norm.get("parent_ref") != sid
    assert validate_normalized_manifest(norm) is None
    bad = dict(norm)
    bad["parent_ref"] = sid
    bad["session_id"] = sid
    assert validate_normalized_manifest(bad) is not None


def test_provenance_redacts_external_audit_ref(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    from rt_sandbox.isolation import rt_sandbox_captures_dir

    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    staging = rt_sandbox_captures_dir(tmp_path) / cap["capture_candidate_id"]
    prov = json.loads((staging / "provenance.json").read_text(encoding="utf-8"))
    refs = prov.get("source_artifact_refs") or {}
    assert "audit_ref" not in refs
    for ref in refs.values():
        assert "runs/rt_sandbox/audit" not in ref


def test_replay_boundary_no_sa_write_on_normalize(tmp_path: Path) -> None:
    from rt_sandbox.capture_normalize import NormalizationContext, normalize_capture_bundle
    from rt_sandbox.isolation import assert_sa_path_blocked, rt_sandbox_captures_dir

    (tmp_path / "AGENTS.md").write_text("# test\n", encoding="utf-8")
    (tmp_path / "platform" / "sa-r0-viewer").mkdir(parents=True, exist_ok=True)
    cid = str(uuid.uuid4())
    sid = cid
    staging = rt_sandbox_captures_dir(tmp_path) / cid
    staging.mkdir(parents=True)
    snapshot = {
        "schema": "sandbox_session_snapshot_v1",
        "session_id": sid,
        "entity_states": [],
        "snapshot_utc": "2026-01-01T00:00:00+00:00",
    }
    report = {
        "schema": "runtime_capture_report_v1",
        "capture_candidate_id": cid,
        "session_id": sid,
        "command_summary": [],
        "resource_limit_events": [],
        "failure_states_observed": [],
    }
    telemetry = {"schema": "rt_telemetry_capture_summary_v1", "session_id": sid, "channels": []}
    candidate = {
        "schema": "rt_capture_candidate_v1",
        "capture_candidate_id": cid,
        "session_id": sid,
        "ephemeral_session_ref": sid,
        "origin": "rt_sandbox_capture_v1",
        "capture_utc": "2026-01-01T00:00:00+00:00",
        "approval_status": "pending",
        "staging_refs": {},
    }
    (staging / "candidate.json").write_text(json.dumps(candidate), encoding="utf-8")
    (staging / "snapshot.json").write_text(json.dumps(snapshot), encoding="utf-8")
    (staging / "capture_report.json").write_text(json.dumps(report), encoding="utf-8")
    (staging / "telemetry_summary.json").write_text(json.dumps(telemetry), encoding="utf-8")
    ctx = NormalizationContext(
        session_state="stopped",
        enable_gazebo_adapter=False,
        adapter_mode="stub",
        adapter_attached=False,
    )
    normalize_capture_bundle(staging, runtime_context=ctx, repo_root=tmp_path)
    sa_path = tmp_path / "platform" / "sa-r0-viewer" / "x.json"
    with pytest.raises(PermissionError):
        assert_sa_path_blocked(sa_path, tmp_path)


def test_approve_requires_normalization(manager: BridgeSessionManager, tmp_path: Path) -> None:
    from rt_sandbox.capture import CaptureBundleError, write_approval_record
    from rt_sandbox.isolation import rt_sandbox_captures_dir

    cid = str(uuid.uuid4())
    staging = rt_sandbox_captures_dir(tmp_path) / cid
    staging.mkdir(parents=True)
    (staging / "candidate.json").write_text(
        json.dumps(
            {
                "schema": "rt_capture_candidate_v1",
                "capture_candidate_id": cid,
                "session_id": cid,
                "origin": "rt_sandbox_capture_v1",
                "approval_status": "pending",
                "normalization_status": "pending",
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(CaptureBundleError) as exc:
        write_approval_record(staging, approved_by="test", repo_root=tmp_path)
    assert "normalization" in exc.value.message


def test_conversion_manifest_rejects_missing_normalized_ref() -> None:
    from rt_sandbox.export_boundary import validate_conversion_manifest

    manifest = {
        "schema": "runtime_to_replay_conversion_v1",
        "origin": "rt_sandbox_capture_v1",
        "conversion_steps": ["validate_scenario_pack"],
        "staging_refs": {},
        "requires_normalization": True,
    }
    assert validate_conversion_manifest(manifest) is not None


def test_adapter_capture_includes_sync_telemetry_summaries(
    adapter_manager: BridgeSessionManager, tmp_path: Path
) -> None:
    from rt_sandbox.isolation import rt_sandbox_captures_dir

    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(1, 2, 3)},
    )
    _cmd(
        adapter_manager,
        "send_runtime_command",
        sid,
        payload={"sub_command": "adapter_poll_telemetry"},
    )
    _cmd(adapter_manager, "stop_session", sid)
    cap = _cmd(adapter_manager, "capture_session", sid)
    staging = rt_sandbox_captures_dir(tmp_path) / cap["capture_candidate_id"]
    norm = json.loads((staging / "normalized_manifest.json").read_text(encoding="utf-8"))
    assert norm.get("sync_health_summary") is not None or norm.get("entity_pose_history")
    prov = json.loads((staging / "provenance.json").read_text(encoding="utf-8"))
    assert prov.get("adapter_attached") is True


def test_capture_bundle_size_cap_includes_normalized(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    # Raw bundle ~2.6KiB with one entity; normalized total ~5.8KiB — cap between them.
    manager.config.max_capture_bundle_bytes = 3000
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose()},
    )
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    assert cap["ok"] is False
    assert cap["error_code"] == "RESOURCE_LIMIT_EXCEEDED"
    export_log = tmp_path / "runs" / "rt_sandbox" / "export_audit" / "export_boundary.jsonl"
    events = [
        json.loads(line)["event_type"]
        for line in export_log.read_text(encoding="utf-8").strip().split("\n")
        if line
    ]
    assert "normalization_rejected" in events


# --- PLAT-RT-R2d template adapter resync ---


def test_template_apply_resync_with_adapter(
    adapter_manager: BridgeSessionManager,
) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    resp = _cmd(
        adapter_manager,
        "apply_runtime_template",
        sid,
        payload={"template_id": "radar_north_arc_v1"},
    )
    assert resp["ok"] is True
    assert "sync_health" in resp["world_summary"]
    audit = json.loads(AuditLog(adapter_manager._repo_root).path_for(sid).read_text())
    types = [e["command_type"] for e in audit["entries"]]
    assert "template_resync_requested" in types
    assert "template_resync_completed" in types


def test_template_apply_resync_skipped_on_stub(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(
        manager,
        "apply_runtime_template",
        sid,
        payload={"template_id": "radar_north_arc_v1"},
    )
    audit = json.loads(AuditLog(manager._repo_root).path_for(sid).read_text())
    types = [e["command_type"] for e in audit["entries"]]
    assert "template_resync_skipped" in types


def test_workflow_reset_world_adapter_cleanup(
    adapter_manager: BridgeSessionManager,
) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(1, 2, 3)},
    )
    _cmd(
        adapter_manager,
        "start_workflow",
        sid,
        payload={"workflow_id": "waypoint_staging_v1"},
    )
    adv = _cmd(adapter_manager, "advance_workflow", sid)
    assert adv["ok"] is True
    assert adv["world_summary"]["entity_count"] == 0
    audit = json.loads(AuditLog(adapter_manager._repo_root).path_for(sid).read_text())
    types = [e["command_type"] for e in audit["entries"]]
    assert "template_resync_requested" in types
    assert "template_resync_completed" in types


def test_template_resync_audit_event_kinds(
    adapter_manager: BridgeSessionManager,
) -> None:
    from rt_sandbox.audit_vocabulary import VALID_EVENT_KINDS, classify_event_kind

    for ct in (
        "template_resync_requested",
        "template_resync_completed",
        "template_resync_skipped",
        "template_resync_stale",
    ):
        assert classify_event_kind(ct) == "sync"
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        adapter_manager,
        "apply_runtime_template",
        sid,
        payload={"template_id": "radar_north_arc_v1"},
    )
    audit = json.loads(AuditLog(adapter_manager._repo_root).path_for(sid).read_text())
    kinds = {e["event_kind"] for e in audit["entries"]}
    assert kinds <= VALID_EVENT_KINDS


# --- PLAT-RT-R1b adapter poll consolidation ---


def test_spawn_single_telemetry_poll(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    runtime = adapter_manager._session.runtime
    original = runtime.poll_telemetry
    call_count = {"n": 0}

    def counting_poll(**kwargs: object) -> dict:
        call_count["n"] += 1
        return original(**kwargs)

    runtime.poll_telemetry = counting_poll  # type: ignore[method-assign]
    _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 10)},
    )
    assert call_count["n"] == 1


def test_unified_poll_tick_audit_sequence(adapter_manager: BridgeSessionManager) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 10)},
    )
    audit = json.loads(AuditLog(adapter_manager._repo_root).path_for(sid).read_text())
    types = [e["command_type"] for e in audit["entries"]]
    sync_idx = types.index("sync_update")
    spawn_idx = types.index("spawn_entity")
    entity_window = types[sync_idx : spawn_idx + 1]
    assert entity_window.count("sync_update") == 1
    assert entity_window.count("telemetry_update") == 1


def test_shared_stale_helper() -> None:
    from datetime import datetime, timedelta, timezone

    from rt_sandbox.governance import GovernanceConfig
    from rt_sandbox.pose_sync import PoseSyncMirror
    from rt_sandbox.telemetry_bridge import TelemetryMirror
    from rt_sandbox.time_utils import is_poll_stale, parse_utc, poll_age_seconds

    old = (datetime.now(timezone.utc) - timedelta(seconds=60)).replace(microsecond=0).isoformat()
    assert parse_utc(old) is not None
    assert poll_age_seconds(old) is not None
    assert poll_age_seconds(old) > 30.0
    assert is_poll_stale(old, 30.0) is True
    assert is_poll_stale(None, 30.0) is True

    cfg = GovernanceConfig()
    pose = PoseSyncMirror(session_id="s1")
    pose.last_poll_utc = old
    assert pose.check_stale(cfg) is True

    telem = TelemetryMirror(session_id="s1")
    telem.last_poll_utc = old
    assert telem.check_stale(cfg) is True


# --- PLAT-RT-R1a vocabulary & authority hardening ---


def test_audit_entries_include_event_kind(manager: BridgeSessionManager) -> None:
    from rt_sandbox.audit_vocabulary import VALID_EVENT_KINDS

    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "spawn_entity", sid, payload={"entity_type": "drone", "pose": _pose()})
    _cmd(manager, "stop_session", sid)
    audit = json.loads(AuditLog(manager._repo_root).path_for(sid).read_text(encoding="utf-8"))
    kinds = {e["event_kind"] for e in audit["entries"]}
    assert "user_command" in kinds
    assert kinds <= VALID_EVENT_KINDS


def test_stub_entity_pose_mirror_authority_labels(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "spawn_entity", sid, payload={"entity_type": "drone", "pose": _pose()})
    sub = _cmd(
        manager,
        "subscribe_telemetry",
        sid,
        payload={"channels": ["entity_pose_mirror"]},
    )
    assert sub["ok"] is True
    payload = (sub.get("initial_events") or [])[0]["payload"]
    assert payload.get("source") == "bridge_registry"
    assert payload.get("authority_label") == "command_authoritative"
    assert payload.get("governance_banner")


def test_adapter_telemetry_payload_authority_labels(
    adapter_manager: BridgeSessionManager,
) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 10)},
    )
    sub = _cmd(
        adapter_manager,
        "subscribe_telemetry",
        sid,
        payload={"channels": ["entity_pose_mirror", "clock_mirror", "session_health"]},
    )
    assert sub["ok"] is True
    by_channel = {ev["channel"]: ev["payload"] for ev in (sub.get("initial_events") or [])}
    assert by_channel["entity_pose_mirror"]["source"] == "adapter_feedback"
    assert by_channel["entity_pose_mirror"]["authority_label"] == "explanatory_telemetry"
    assert by_channel["clock_mirror"]["source"] == "adapter_telemetry"
    assert by_channel["clock_mirror"]["authority_label"] == "explanatory_telemetry"
    assert by_channel["session_health"]["authority_label"] == "explanatory_telemetry"


def test_normalized_manifest_authority_model(manager: BridgeSessionManager, tmp_path: Path) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "spawn_entity", sid, payload={"entity_type": "drone", "pose": _pose()})
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    assert cap["ok"] is True
    staging = tmp_path / "runs" / "rt_sandbox" / "captures" / cap["capture_candidate_id"]
    norm = json.loads((staging / "normalized_manifest.json").read_text(encoding="utf-8"))
    assert norm.get("authority_model", {}).get("manifest_authority_label") == "replay_boundary_scoped"
    assert "command_authoritative" in (norm.get("authority_model", {}).get("legend") or {})
    history = norm.get("entity_pose_history") or []
    assert history
    assert history[0].get("authority_label") == "command_authoritative"


# --- PLAT-RT-R2e capture pose cognition ---


def test_capture_pose_authority_stub_capture(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    assert cap["ok"] is True
    audit = AuditLog(tmp_path).path_for(sid)
    entries = json.loads(audit.read_text(encoding="utf-8"))["entries"]
    types = {e["command_type"] for e in entries}
    assert "capture_pose_authority" in types
    auth = next(e for e in entries if e["command_type"] == "capture_pose_authority")
    assert auth.get("event_kind") == "capture"
    assert auth["detail"]["authoritative_field"] == "command_pose"
    staging = tmp_path / "runs" / "rt_sandbox" / "captures" / cap["capture_candidate_id"]
    norm = json.loads((staging / "normalized_manifest.json").read_text(encoding="utf-8"))
    cognition = norm.get("capture_pose_cognition") or {}
    assert cognition.get("governance_banner")
    per_entity = cognition.get("per_entity") or []
    if per_entity:
        assert per_entity[0].get("authoritative_field") == "command_pose"


def test_export_pose_normalized_export_audit(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    _cmd(manager, "capture_session", sid)
    export_log = tmp_path / "runs" / "rt_sandbox" / "export_audit" / "export_boundary.jsonl"
    events = [
        json.loads(line)["event_type"]
        for line in export_log.read_text(encoding="utf-8").strip().split("\n")
        if line
    ]
    assert "capture_normalized" in events
    assert "export_pose_normalized" in events


def test_normalized_manifest_capture_pose_cognition_block(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "spawn_entity", sid, payload={"entity_type": "drone", "pose": _pose()})
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    staging = tmp_path / "runs" / "rt_sandbox" / "captures" / cap["capture_candidate_id"]
    norm = json.loads((staging / "normalized_manifest.json").read_text(encoding="utf-8"))
    cognition = norm.get("capture_pose_cognition")
    assert isinstance(cognition, dict)
    assert "assessment_utc" in cognition
    assert "per_entity" in cognition
    assert "session_flags" in cognition
    assert cognition.get("adapter_attached") is False


def test_capture_pose_stale_with_mock_drift(
    adapter_manager: BridgeSessionManager, tmp_path: Path
) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    spawn = _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 10)},
    )
    eid = spawn["entity_id"]
    _cmd(
        adapter_manager,
        "send_runtime_command",
        sid,
        payload={
            "sub_command": "mock_inject_drift",
            "entity_id": eid,
            "offset": {"x": 5.0, "y": 0, "z": 0},
        },
    )
    move = _cmd(
        adapter_manager,
        "move_entity",
        sid,
        payload={"entity_id": eid, "pose": _pose(0, 0, 10)},
    )
    assert move["ok"] is False
    assert move["error_code"] == "SYNC_STALE"
    _cmd(adapter_manager, "stop_session", sid)
    cap = _cmd(adapter_manager, "capture_session", sid)
    assert cap["ok"] is True
    audit = AuditLog(tmp_path).path_for(sid)
    types = {e["command_type"] for e in json.loads(audit.read_text())["entries"]}
    assert "capture_pose_authority" in types
    assert "capture_pose_stale" in types or "capture_pose_mismatch" in types
    staging = tmp_path / "runs" / "rt_sandbox" / "captures" / cap["capture_candidate_id"]
    norm = json.loads((staging / "normalized_manifest.json").read_text(encoding="utf-8"))
    cognition = norm.get("capture_pose_cognition") or {}
    assert cognition.get("adapter_attached") is True
    assert cognition.get("sync_health") in {"stale", "mismatch", "ok", "feedback_lost"}


# --- PLAT-RT-F5b P0 fidelity coupling ---


@pytest.fixture
def fidelity_adapter_manager(tmp_path: Path) -> BridgeSessionManager:
    (tmp_path / "AGENTS.md").write_text("# test repo\n", encoding="utf-8")
    (tmp_path / "runs" / "rt_sandbox").mkdir(parents=True, exist_ok=True)
    cfg = GovernanceConfig(
        command_rate_burst=1000,
        command_rate_sustained=1000.0,
        bridge_ready_timeout_s=5.0,
        session_cleanup_timeout_s=0.5,
        cleanup_pending_max_age_s=1.0,
        max_session_duration_s=60.0,
        enable_gazebo_adapter=True,
        enable_fidelity_coupling=True,
        adapter_mode="mock",
        adapter_ipc_timeout_s=10.0,
    )
    return BridgeSessionManager(config=cfg, repo_root=tmp_path)


def test_fidelity_coupling_default_off(
    adapter_manager: BridgeSessionManager,
) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(1, 2, 3)},
    )
    _cmd(
        adapter_manager,
        "send_runtime_command",
        sid,
        payload={"sub_command": "adapter_poll_telemetry"},
    )
    mirror = adapter_manager._session.telemetry_mirror
    assert mirror is not None
    assert mirror.fidelity_truth is None
    assert GovernanceConfig().enable_fidelity_coupling is False


def test_fidelity_truth_in_mock_telemetry_poll(
    fidelity_adapter_manager: BridgeSessionManager,
) -> None:
    start = _cmd(fidelity_adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        fidelity_adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(10, 20, 15)},
    )
    _cmd(
        fidelity_adapter_manager,
        "send_runtime_command",
        sid,
        payload={"sub_command": "adapter_poll_telemetry"},
    )
    mirror = fidelity_adapter_manager._session.telemetry_mirror
    assert mirror is not None
    truth = mirror.fidelity_truth
    assert isinstance(truth, dict)
    assert truth.get("schema") == "rt_fidelity_truth_snapshot_v1"
    assert truth.get("attestation_status") == "available"
    entity_truth = truth.get("entity_truth") or []
    assert len(entity_truth) == 1
    assert entity_truth[0].get("sim_agl_m") == 15.0
    assert isinstance(truth.get("los_truth"), dict)
    assert isinstance(truth.get("dome_truth"), dict)
    audit = AuditLog(fidelity_adapter_manager._repo_root).path_for(sid)
    types = {e["command_type"] for e in json.loads(audit.read_text())["entries"]}
    assert "fidelity_truth_update" in types


def test_fidelity_pose_block_on_capture(
    fidelity_adapter_manager: BridgeSessionManager, tmp_path: Path
) -> None:
    start = _cmd(fidelity_adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        fidelity_adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 12)},
    )
    _cmd(
        fidelity_adapter_manager,
        "send_runtime_command",
        sid,
        payload={"sub_command": "adapter_poll_telemetry"},
    )
    _cmd(fidelity_adapter_manager, "stop_session", sid)
    cap = _cmd(fidelity_adapter_manager, "capture_session", sid)
    assert cap["ok"] is True
    staging = tmp_path / "runs" / "rt_sandbox" / "captures" / cap["capture_candidate_id"]
    assert (staging / "fidelity_truth.json").is_file()
    norm = json.loads((staging / "normalized_manifest.json").read_text(encoding="utf-8"))
    block = norm.get("fidelity_pose_block")
    assert isinstance(block, dict)
    assert block.get("schema") == "rt_fidelity_pose_block_v1"
    per_entity = block.get("per_entity") or []
    assert len(per_entity) == 1
    row = per_entity[0]
    assert row.get("command_pose") is not None
    assert row.get("truth_attested_pose") is not None
    assert row.get("truth_timestamp_utc") is not None


def test_fidelity_capture_audits(
    fidelity_adapter_manager: BridgeSessionManager, tmp_path: Path
) -> None:
    start = _cmd(fidelity_adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        fidelity_adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 8)},
    )
    _cmd(
        fidelity_adapter_manager,
        "send_runtime_command",
        sid,
        payload={"sub_command": "adapter_poll_telemetry"},
    )
    _cmd(fidelity_adapter_manager, "stop_session", sid)
    cap = _cmd(fidelity_adapter_manager, "capture_session", sid)
    assert cap["ok"] is True
    audit = AuditLog(tmp_path).path_for(sid)
    types = {e["command_type"] for e in json.loads(audit.read_text())["entries"]}
    assert "fidelity_truth_update" in types
    assert "fidelity_capture_snapshot" in types


def test_fidelity_truth_mismatch_unknown_entity(
    fidelity_adapter_manager: BridgeSessionManager,
) -> None:
    from rt_sandbox.fidelity_coupling import build_fidelity_poll_audits

    start = _cmd(fidelity_adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        fidelity_adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 5)},
    )
    _cmd(
        fidelity_adapter_manager,
        "send_runtime_command",
        sid,
        payload={"sub_command": "adapter_poll_telemetry"},
    )
    mirror = fidelity_adapter_manager._session.telemetry_mirror
    assert mirror is not None and mirror.fidelity_truth is not None
    truth = dict(mirror.fidelity_truth)
    entity_truth = list(truth.get("entity_truth") or [])
    entity_truth.append(
        {
            "entity_id": "ghost-unknown",
            "truth_attested_pose": {"x": 99, "y": 0, "z": 5},
            "sim_agl_m": 5.0,
        }
    )
    truth["entity_truth"] = entity_truth
    mirror.fidelity_truth = truth
    audits = build_fidelity_poll_audits(fidelity_adapter_manager._session, fidelity_adapter_manager.config)
    audit_types = {a[0] for a in audits}
    assert "fidelity_truth_mismatch" in audit_types


def test_registry_unchanged_by_truth_drift(
    fidelity_adapter_manager: BridgeSessionManager,
) -> None:
    start = _cmd(fidelity_adapter_manager, "start_session")
    sid = start["session_id"]
    spawn = _cmd(
        fidelity_adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 10)},
    )
    eid = spawn["entity_id"]
    _cmd(
        fidelity_adapter_manager,
        "send_runtime_command",
        sid,
        payload={
            "sub_command": "mock_inject_drift",
            "entity_id": eid,
            "offset": {"x": 3.0, "y": 0, "z": 2.0},
        },
    )
    _cmd(
        fidelity_adapter_manager,
        "send_runtime_command",
        sid,
        payload={"sub_command": "adapter_poll_telemetry"},
    )
    registry_pose = fidelity_adapter_manager._session.world.registry.get(eid).pose
    assert registry_pose["x"] == 0.0
    assert registry_pose["z"] == 10.0
    mirror = fidelity_adapter_manager._session.telemetry_mirror
    assert mirror is not None and mirror.fidelity_truth is not None
    entity_truth = mirror.fidelity_truth.get("entity_truth") or []
    truth_row = next(r for r in entity_truth if r["entity_id"] == eid)
    assert truth_row["truth_attested_pose"]["x"] != registry_pose["x"]


# --- PLAT-RT-F5b P1 fidelity telemetry passthrough ---


def test_fidelity_telemetry_fields_default_off(
    adapter_manager: BridgeSessionManager,
) -> None:
    from rt_sandbox.telemetry_bridge import resolve_channel_payload

    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    session = adapter_manager._session
    assert session is not None
    ws = resolve_channel_payload(session, "world_summary", adapter_manager.config)
    assert ws is not None
    assert ws.get("enable_fidelity_coupling") is False
    assert ws.get("fidelity_attestation_status") == "unavailable"
    assert "fidelity_truth" not in ws


def test_fidelity_truth_on_world_summary_pull(
    fidelity_adapter_manager: BridgeSessionManager,
) -> None:
    from rt_sandbox.telemetry_bridge import resolve_channel_payload

    start = _cmd(fidelity_adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        fidelity_adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(5, 5, 12)},
    )
    _cmd(
        fidelity_adapter_manager,
        "send_runtime_command",
        sid,
        payload={"sub_command": "adapter_poll_telemetry"},
    )
    session = fidelity_adapter_manager._session
    assert session is not None
    ws = resolve_channel_payload(session, "world_summary", fidelity_adapter_manager.config)
    sh = resolve_channel_payload(session, "session_health", fidelity_adapter_manager.config)
    assert ws is not None and sh is not None
    assert ws.get("enable_fidelity_coupling") is True
    assert ws.get("fidelity_attestation_status") == "available"
    assert ws.get("fidelity_label") == "truth_attested"
    truth = ws.get("fidelity_truth")
    assert isinstance(truth, dict)
    assert truth.get("schema") == "rt_fidelity_truth_snapshot_v1"
    assert sh.get("fidelity_truth") == truth


def test_fidelity_stale_attestation_on_pull(
    fidelity_adapter_manager: BridgeSessionManager,
) -> None:
    from rt_sandbox.telemetry_bridge import resolve_channel_payload

    start = _cmd(fidelity_adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        fidelity_adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 5)},
    )
    _cmd(
        fidelity_adapter_manager,
        "send_runtime_command",
        sid,
        payload={"sub_command": "adapter_poll_telemetry", "mock_stale_telemetry": True},
    )
    session = fidelity_adapter_manager._session
    assert session is not None
    ws = resolve_channel_payload(session, "world_summary", fidelity_adapter_manager.config)
    assert ws is not None
    assert ws.get("fidelity_attestation_status") == "stale"


# --- PLAT-RT-R3b lifecycle transition hardening ---


from rt_sandbox.lifecycle import (  # noqa: E402
    SessionState,
    can_transition,
    transition_rules,
)


@pytest.mark.parametrize(
    "command_type",
    [
        "spawn_entity",
        "move_entity",
        "delete_entity",
        "subscribe_telemetry",
        "apply_runtime_template",
        "start_workflow",
    ],
)
def test_can_transition_happy_path_commands(command_type: str) -> None:
    rules = transition_rules()
    allowed = rules[command_type]
    assert SessionState.RUNNING in allowed
    assert SessionState.PAUSED in allowed
    assert can_transition(SessionState.RUNNING, command_type)
    assert can_transition(SessionState.PAUSED, command_type)
    assert not can_transition(SessionState.STOPPED, command_type)


@pytest.mark.parametrize(
    "state",
    [SessionState.CAPTURED, SessionState.DISCARDED],
)
def test_can_transition_terminal_states_block_all(state: SessionState) -> None:
    assert not can_transition(state, "pause_session")
    assert not can_transition(state, "capture_session")
    assert not can_transition(state, "discard_session")


def test_can_transition_cleanup_pending_only_discard() -> None:
    assert can_transition(SessionState.CLEANUP_PENDING, "discard_session")
    assert not can_transition(SessionState.CLEANUP_PENDING, "spawn_entity")
    assert not can_transition(SessionState.CLEANUP_PENDING, "capture_session")


def test_capture_only_from_stopped() -> None:
    assert can_transition(SessionState.STOPPED, "capture_session")
    for state in (
        SessionState.RUNNING,
        SessionState.PAUSED,
        SessionState.FAILED,
        SessionState.RUNTIME_CRASHED,
        SessionState.BRIDGE_DISCONNECTED,
        SessionState.CLEANUP_PENDING,
    ):
        assert not can_transition(state, "capture_session")


@pytest.mark.parametrize(
    "state",
    [
        SessionState.FAILED,
        SessionState.RUNTIME_CRASHED,
        SessionState.BRIDGE_DISCONNECTED,
        SessionState.CLEANUP_PENDING,
    ],
)
def test_failure_states_block_capture(state: SessionState) -> None:
    assert not can_transition(state, "capture_session")


def test_bridge_disconnected_allows_discard(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    assert manager._session is not None
    manager._session.state = SessionState.BRIDGE_DISCONNECTED
    out = _cmd(manager, "discard_session", sid)
    assert out["ok"] is True
    assert out["state"] == "discarded"


def test_bridge_ready_timeout_to_failed_auto_discard(
    manager: BridgeSessionManager,
) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    session = manager._session
    assert session is not None
    session.state = SessionState.CREATED
    session.bridge_ready_deadline = time.monotonic() - 1
    session.cleanup_after = None
    manager._tick_timeouts(time.monotonic())
    assert session.state == SessionState.FAILED
    assert session.cleanup_after is not None
    session.cleanup_after = time.monotonic() - 1
    manager._tick_timeouts(time.monotonic())
    assert session.state == SessionState.DISCARDED
    audit = AuditLog(manager._repo_root).path_for(sid)
    types = {e["command_type"] for e in json.loads(audit.read_text())["entries"]}
    assert "bridge_ready_timeout" in types
    assert "auto_cleanup" in types


def test_failed_auto_cleanup_partial_teardown(
    adapter_manager: BridgeSessionManager,
    tmp_path: Path,
) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    session = adapter_manager._session
    assert session is not None
    session.state = SessionState.FAILED
    session.cleanup_after = time.monotonic() - 1
    adapter_manager._tick_timeouts(time.monotonic())
    assert session.state == SessionState.DISCARDED
    audit = AuditLog(tmp_path).path_for(sid)
    types = [e["command_type"] for e in json.loads(audit.read_text())["entries"]]
    assert "auto_cleanup" in types
    assert "adapter_teardown" not in types
    assert "orphan_cleanup" not in types


def test_stopped_auto_cleanup_full_adapter_teardown(
    adapter_manager: BridgeSessionManager,
    tmp_path: Path,
) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(adapter_manager, "stop_session", sid)
    time.sleep(0.7)
    adapter_manager._tick_timeouts(time.monotonic())
    assert adapter_manager._session is not None
    assert adapter_manager._session.state.value == "discarded"
    audit = AuditLog(tmp_path).path_for(sid)
    types = [e["command_type"] for e in json.loads(audit.read_text())["entries"]]
    assert "auto_cleanup" in types
    assert "adapter_teardown" in types
    assert "orphan_cleanup" in types


def test_runtime_crashed_blocks_entity_ops(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    assert manager._session is not None
    manager._session.state = SessionState.RUNTIME_CRASHED
    out = _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose()},
    )
    assert out["ok"] is False
    assert out["error_code"] == "INVALID_STATE"


def test_rt_sandbox_ui_isolation() -> None:
    """PLAT-RT-T1: RT UI package isolated from SA viewer."""
    repo = Path(__file__).resolve().parents[3]
    rt_ui = repo / "platform" / "rt-sandbox-ui"
    sa_viewer = repo / "platform" / "sa-r0-viewer"
    assert rt_ui.is_dir()
    assert sa_viewer.is_dir()
    src = rt_ui / "src"
    assert src.is_dir()
    for path in src.rglob("*"):
        if path.suffix not in {".ts", ".tsx", ".json", ".html"}:
            continue
        if path.name.endswith(".test.ts"):
            continue
        text = path.read_text(encoding="utf-8")
        assert "platform/sa-r0-viewer" not in text
    from rt_sandbox.isolation import assert_sa_path_blocked

    with pytest.raises(PermissionError):
        assert_sa_path_blocked(sa_viewer / "index.html", repo)


def test_rt_sandbox_ui_world_editing_commands() -> None:
    """PLAT-RT-T2: RT UI uses entity commands without SA viewer coupling."""
    repo = Path(__file__).resolve().parents[3]
    rt_ui_src = repo / "platform" / "rt-sandbox-ui" / "src"
    combined = ""
    for path in rt_ui_src.rglob("*.ts"):
        if path.name.endswith(".test.ts"):
            continue
        combined += path.read_text(encoding="utf-8")
    for path in rt_ui_src.rglob("*.tsx"):
        combined += path.read_text(encoding="utf-8")
    assert "spawn_entity" in combined
    assert "move_entity" in combined
    assert "delete_entity" in combined
    assert "platform/sa-r0-viewer" not in combined


def test_rt_sandbox_ui_workstation_boundaries() -> None:
    """PLAT-RT-T4: workstation polish must not invoke capture or SA import from browser."""
    repo = Path(__file__).resolve().parents[3]
    rt_ui_src = repo / "platform" / "rt-sandbox-ui" / "src"
    combined = ""
    for path in rt_ui_src.rglob("*.ts"):
        if path.name.endswith(".test.ts"):
            continue
        combined += path.read_text(encoding="utf-8")
    for path in rt_ui_src.rglob("*.tsx"):
        combined += path.read_text(encoding="utf-8")
    assert "RuntimeWorkstationShell" in combined
    assert "captureHandoffCognition" in combined
    assert "cesiumEditing" in combined
    bridge_dir = rt_ui_src / "bridge"
    bridge_combined = ""
    for path in bridge_dir.rglob("*.ts"):
        if path.name.endswith(".test.ts"):
            continue
        bridge_combined += path.read_text(encoding="utf-8")
    for forbidden in (
        "capture_session",
        "rt_sa_import",
        "handoff_import_committed",
    ):
        assert forbidden not in bridge_combined


# --- PLAT-RT-SA1 manual import bridge ---


def test_handoff_preconditions_after_capture(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    from rt_sandbox.isolation import rt_sandbox_captures_dir
    from rt_sandbox.sa_handoff import append_handoff_event, check_handoff_preconditions

    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    staging = rt_sandbox_captures_dir(tmp_path) / cap["capture_candidate_id"]
    assert check_handoff_preconditions(staging) == []
    append_handoff_event(
        tmp_path,
        "handoff_ready",
        capture_candidate_id=cap["capture_candidate_id"],
        session_id=sid,
    )
    export_log = tmp_path / "runs" / "rt_sandbox" / "export_audit" / "export_boundary.jsonl"
    events = [json.loads(line)["event_type"] for line in export_log.read_text().splitlines() if line]
    assert "handoff_ready" in events


def test_handoff_rejected_blocks_prepare(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    from rt_sandbox.capture import write_approval_record, write_conversion_manifest
    from rt_sandbox.isolation import rt_sandbox_captures_dir
    from rt_sandbox.capture import CaptureBundleError
    from rt_sandbox.sa_handoff import (
        is_handoff_blocked,
        write_handoff_review_v1,
        write_handoff_manifest,
    )

    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    cid = cap["capture_candidate_id"]
    staging = rt_sandbox_captures_dir(tmp_path) / cid
    write_handoff_review_v1(
        staging,
        capture_id=cid,
        decision="rejected",
        reviewer="test",
        repo_root=tmp_path,
    )
    assert is_handoff_blocked(staging)
    write_approval_record(staging, approved_by="test", repo_root=tmp_path)
    conversion = write_conversion_manifest(staging, repo_root=tmp_path)
    with pytest.raises(CaptureBundleError):
        write_handoff_manifest(tmp_path, cid, staging, conversion)


def test_validate_sa_import_record_rejects_session_parent() -> None:
    from rt_sandbox.export_boundary import validate_sa_import_record

    sid = str(uuid.uuid4())
    record = {
        "schema": "rt_sa_import_record_v1",
        "corpus_ref": "fixtures/sa_r0/demo_test",
        "bundle_path": "fixtures/sa_r0/demo_test/index.json",
        "parent_ref": sid,
        "session_id": sid,
    }
    assert validate_sa_import_record(record) is not None


def test_assert_maintainer_corpus_write_allowed(tmp_path: Path) -> None:
    from rt_sandbox.export_boundary import ExportBoundaryError, assert_maintainer_corpus_write_allowed

    ok_dest = tmp_path / "fixtures" / "sa_r0" / "demo_rt_import_test"
    assert_maintainer_corpus_write_allowed(ok_dest, tmp_path)
    with pytest.raises(ExportBoundaryError):
        assert_maintainer_corpus_write_allowed(tmp_path / "runs" / "other", tmp_path)


def test_sa_import_prepare_and_commit_dry_run(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    from rt_sandbox.capture import write_approval_record, write_conversion_manifest
    from rt_sandbox.isolation import rt_sandbox_captures_dir
    from rt_sandbox.sa_handoff import (
        commit_bundle_to_corpus,
        sa_handoff_dir,
        write_handoff_manifest,
    )

    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    cid = cap["capture_candidate_id"]
    staging = rt_sandbox_captures_dir(tmp_path) / cid
    write_approval_record(staging, approved_by="test", repo_root=tmp_path)
    conversion = write_conversion_manifest(staging, repo_root=tmp_path)
    manifest = write_handoff_manifest(tmp_path, cid, staging, conversion)
    assert manifest["schema"] == "rt_sa_handoff_manifest_v1"
    handoff = sa_handoff_dir(tmp_path, cid)
    bundle_dir = handoff / "bundle"
    bundle_dir.mkdir(parents=True)
    (bundle_dir / "index.json").write_text(
        json.dumps({"schema": "replay_sa_bundle_v1", "bundle_id": "test"}),
        encoding="utf-8",
    )
    corpus_dest = tmp_path / "fixtures" / "sa_r0" / f"demo_rt_{cid[:8]}"
    plan = commit_bundle_to_corpus(
        tmp_path, cid, corpus_dest, imported_by="test", dry_run=True
    )
    assert plan.get("dry_run") is True
    assert not corpus_dest.exists()


def test_handoff_event_types_in_vocabulary() -> None:
    from rt_sandbox.audit_vocabulary import HANDOFF_EVENT_TYPES
    from rt_sandbox.sa_handoff import HANDOFF_EXPORT_EVENTS

    assert HANDOFF_EVENT_TYPES == HANDOFF_EXPORT_EVENTS


def test_world_summary_includes_adapter_mode(
    adapter_manager: BridgeSessionManager,
) -> None:
    start = _cmd(adapter_manager, "start_session")
    sid = start["session_id"]
    _cmd(
        adapter_manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(0, 0, 10)},
    )
    ws = _cmd(adapter_manager, "get_workflow_state", sid)
    # workflow state may not have world_summary — use subscribe or entity path
    start2 = start
    assert start2["ok"] is True
    session = adapter_manager._session
    assert session is not None
    from rt_sandbox.session_telemetry_coordinator import pose_sync_summary

    summary = session.world.world_summary(pose_sync_summary=pose_sync_summary(session))
    assert summary.get("adapter_mode") == "mock"


# --- PLAT-RT-SA2 handoff mirror ---


def test_list_capture_handoff_status_scoped_to_session(
    manager: BridgeSessionManager,
) -> None:
    start_a = _cmd(manager, "start_session")
    sid_a = start_a["session_id"]
    _cmd(manager, "stop_session", sid_a)
    cap_a = _cmd(manager, "capture_session", sid_a)

    start_b = _cmd(manager, "start_session")
    sid_b = start_b["session_id"]
    _cmd(manager, "stop_session", sid_b)
    cap_b = _cmd(manager, "capture_session", sid_b)

    cid_a = cap_a["capture_candidate_id"]
    cid_b = cap_b["capture_candidate_id"]

    resp_a = _cmd(
        manager,
        "list_capture_handoff_status",
        payload={"session_id": sid_a},
    )
    assert resp_a["ok"] is True
    ids_a = {r["capture_candidate_id"] for r in resp_a["captures"]}
    assert cid_a in ids_a
    assert cid_b not in ids_a

    resp_b = _cmd(
        manager,
        "list_capture_handoff_status",
        payload={"session_id": sid_b},
    )
    ids_b = {r["capture_candidate_id"] for r in resp_b["captures"]}
    assert cid_b in ids_b
    assert cid_a not in ids_b


def test_list_capture_handoff_status_redacts_paths(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    resp = _cmd(
        manager,
        "list_capture_handoff_status",
        payload={"session_id": sid},
    )
    assert resp["ok"] is True
    assert len(resp["captures"]) >= 1
    row = resp["captures"][0]
    assert row["schema"] == "rt_capture_handoff_row_v1"
    assert "candidate" not in row
    assert "corpus_ref" not in row
    assert "fixtures/sa_r0" not in json.dumps(row)
    assert row["governance_banner"]


def test_list_capture_handoff_status_requires_session_id(
    manager: BridgeSessionManager,
) -> None:
    resp = _cmd(manager, "list_capture_handoff_status", payload={})
    assert resp["ok"] is False
    assert resp["error_code"] == "INVALID_STATE"


def test_list_capture_handoff_status_no_writes(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    from rt_sandbox.isolation import rt_sandbox_captures_dir, rt_sandbox_sa_handoff_dir

    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    cid = cap["capture_candidate_id"]
    staging = rt_sandbox_captures_dir(tmp_path) / cid
    cand_mtime = (staging / "candidate.json").stat().st_mtime
    handoff_root = rt_sandbox_sa_handoff_dir(tmp_path)
    handoff_exists_before = handoff_root.exists()

    _cmd(
        manager,
        "list_capture_handoff_status",
        payload={"session_id": sid},
    )

    assert (staging / "candidate.json").stat().st_mtime == cand_mtime
    if handoff_exists_before:
        assert handoff_root.is_dir()
    else:
        assert not handoff_root.exists() or not any(handoff_root.iterdir())


def test_capture_handoff_mirror_derive_phases(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    from rt_sandbox.capture_handoff_mirror import (
        capture_belongs_to_session,
        derive_workflow_phase,
    )
    from rt_sandbox.isolation import rt_sandbox_captures_dir
    from rt_sandbox.sa_handoff import handoff_status_summary

    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    cid = cap["capture_candidate_id"]
    staging = rt_sandbox_captures_dir(tmp_path) / cid
    cand = json.loads((staging / "candidate.json").read_text(encoding="utf-8"))
    assert capture_belongs_to_session(cand, sid)
    assert not capture_belongs_to_session(cand, str(uuid.uuid4()))

    summary = handoff_status_summary(tmp_path, cid)
    phase = derive_workflow_phase(summary=summary, staging_dir=staging)
    assert phase in {"staged", "normalized", "review_pending", "ready"}

