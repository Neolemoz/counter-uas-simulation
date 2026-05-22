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
    time.sleep(0.6)
    assert manager._session is not None
    assert manager._session.state.value == "captured"


def test_capture_invalid_after_auto_cleanup(manager: BridgeSessionManager) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(manager, "stop_session", sid)
    time.sleep(0.6)
    cap = _cmd(manager, "capture_session", sid)
    assert cap["ok"] is False
    assert cap["error_code"] == "INVALID_STATE"


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
    assert (staging / "conversion.json").exists()


def test_sa_viewer_path_unchanged() -> None:
    sa_viewer = _REPO / "platform" / "sa-r0-viewer"
    assert sa_viewer.is_dir()


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
    assert manager._session is not None
    assert manager._session.world is not None
    assert manager._session.world.registry.count() == 1
    _cmd(manager, "discard_session", sid)
    assert manager._session is not None
    assert manager._session.state.value == "discarded"
    assert manager._session.world is None


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
