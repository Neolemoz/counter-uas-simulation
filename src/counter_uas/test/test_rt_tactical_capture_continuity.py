"""PLAT-RT-TAC5 — tactical capture continuity tests."""

from __future__ import annotations

import json
import sys
import uuid
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
if str(_BRIDGE_PKG) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_PKG))

from rt_sandbox.authority_labels import AUTHORITY_REPLAY_BOUNDARY  # noqa: E402
from rt_sandbox.capture_normalize import validate_normalized_capture  # noqa: E402
from rt_sandbox.governance import GovernanceConfig  # noqa: E402
from rt_sandbox.isolation import rt_sandbox_captures_dir  # noqa: E402
from rt_sandbox.session_manager import BridgeSessionManager  # noqa: E402
from rt_sandbox.tactical_capture_annex import validate_tactical_annex  # noqa: E402


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
    (tmp_path / "runs" / "rt_sandbox").mkdir(parents=True)
    cfg = GovernanceConfig(
        command_rate_burst=1000,
        command_rate_sustained=1000.0,
        bridge_ready_timeout_s=5.0,
        session_cleanup_timeout_s=0.5,
        cleanup_pending_max_age_s=1.0,
        max_session_duration_s=60.0,
    )
    return BridgeSessionManager(config=cfg, repo_root=tmp_path)


def _spawn_pair(manager: BridgeSessionManager) -> tuple[str, str, str]:
    start = _cmd(manager, "start_session")
    assert start["ok"] is True
    sid = start["session_id"]
    i = _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "interceptor", "pose": _pose(0, 0, 10)},
    )
    assert i["ok"] is True
    iid = i["entity_id"]
    t = _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(100, 0, 10)},
    )
    assert t["ok"] is True
    tid = t["entity_id"]
    return sid, iid, tid


def _capture(manager: BridgeSessionManager, sid: str) -> dict:
    _cmd(manager, "stop_session", sid)
    cap = _cmd(manager, "capture_session", sid)
    assert cap["ok"] is True
    return cap


def _staging(tmp_path: Path, cap: dict) -> Path:
    cid = cap["capture_candidate_id"]
    return rt_sandbox_captures_dir(tmp_path) / cid


def _audit_types(manager: BridgeSessionManager, sid: str) -> set[str]:
    audit_path = tmp_path_audit(manager, sid)
    if not audit_path.exists():
        return set()
    data = json.loads(audit_path.read_text(encoding="utf-8"))
    return {str(e.get("command_type", "")) for e in data.get("entries") or []}


def tmp_path_audit(manager: BridgeSessionManager, sid: str) -> Path:
    root = manager._repo_root
    return root / "runs" / "rt_sandbox" / "audit" / f"{sid}.json"


def test_manual_capture_annex_normalized(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    sid, iid, tid = _spawn_pair(manager)
    _cmd(
        manager,
        "select_candidate",
        sid,
        payload={"role": "interceptor", "entity_id": iid},
    )
    _cmd(
        manager,
        "select_candidate",
        sid,
        payload={"role": "target", "entity_id": tid},
    )
    assign = _cmd(
        manager,
        "assign_candidate",
        sid,
        payload={"interceptor_id": iid, "target_id": tid},
    )
    assert assign["ok"] is True
    cap = _capture(manager, sid)
    staging = _staging(tmp_path, cap)
    assert (staging / "tactical_annex.json").exists()
    annex = json.loads((staging / "tactical_annex.json").read_text(encoding="utf-8"))
    assert annex["schema"] == "rt_tactical_capture_annex_v1"
    assert annex["authority_label"] == AUTHORITY_REPLAY_BOUNDARY
    assert len(annex["assignment_timeline"]) >= 1
    norm = json.loads((staging / "normalized_manifest.json").read_text(encoding="utf-8"))
    assert norm.get("tactical_annex") is not None
    assert validate_tactical_annex(annex) is None
    assert validate_normalized_capture(staging) == []


def test_empty_tactical_capture_no_annex_file(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    start = _cmd(manager, "start_session")
    sid = start["session_id"]
    _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose()},
    )
    cap = _capture(manager, sid)
    staging = _staging(tmp_path, cap)
    assert not (staging / "tactical_annex.json").exists()
    norm = json.loads((staging / "normalized_manifest.json").read_text(encoding="utf-8"))
    assert norm.get("tactical_annex") is None
    types = _audit_types(manager, sid)
    assert "tactical_capture_annex_empty" in types


def test_assisted_recommendation_timeline(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    sid, iid, tid = _spawn_pair(manager)
    _cmd(manager, "set_tactical_mode", sid, payload={"mode": "assisted"})
    rec = _cmd(manager, "request_recommendation", sid, payload={})
    assert rec["ok"] is True
    rec_id = rec["tactical_recommendation"]["recommendation_id"]
    approve = _cmd(
        manager,
        "approve_recommendation",
        sid,
        payload={"recommendation_id": rec_id},
    )
    assert approve["ok"] is True
    cap = _capture(manager, sid)
    annex = json.loads(
        (_staging(tmp_path, cap) / "tactical_annex.json").read_text(encoding="utf-8")
    )
    assert len(annex["recommendation_timeline"]) >= 2
    events = {e["event"] for e in annex["recommendation_timeline"]}
    assert "issued" in events
    assert "approved" in events


def test_autonomous_pause_resume_and_lock_in_annex(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    sid, _, _ = _spawn_pair(manager)
    _cmd(manager, "set_tactical_mode", sid, payload={"mode": "autonomous"})
    _cmd(manager, "pause_autonomous_loop", sid, payload={})
    _cmd(manager, "resume_autonomous_loop", sid, payload={})
    for _ in range(3):
        _cmd(manager, "get_tactical_state", sid, payload={})
    cap = _capture(manager, sid)
    annex = json.loads(
        (_staging(tmp_path, cap) / "tactical_annex.json").read_text(encoding="utf-8")
    )
    assert len(annex["pause_resume_transitions"]) >= 2
    types = _audit_types(manager, sid)
    assert "tactical_capture_annex_written" in types
    assert "tactical_capture_snapshot" in types


def test_multi_session_capture_isolation(
    manager: BridgeSessionManager, tmp_path: Path
) -> None:
    sid_a, iid, tid = _spawn_pair(manager)
    _cmd(
        manager,
        "select_candidate",
        sid_a,
        payload={"role": "interceptor", "entity_id": iid},
    )
    _cmd(
        manager,
        "assign_candidate",
        sid_a,
        payload={"interceptor_id": iid, "target_id": tid},
    )
    start_b = _cmd(manager, "start_session")
    sid_b = start_b["session_id"]
    _cmd(
        manager,
        "spawn_entity",
        sid_b,
        payload={"entity_type": "drone", "pose": _pose(5, 5, 5)},
    )
    cap_b = _capture(manager, sid_b)
    annex_b_path = _staging(tmp_path, cap_b) / "tactical_annex.json"
    assert not annex_b_path.exists()
    cap_a = _capture(manager, sid_a)
    annex_a = json.loads(
        (_staging(tmp_path, cap_a) / "tactical_annex.json").read_text(encoding="utf-8")
    )
    assert len(annex_a["assignment_timeline"]) >= 1
