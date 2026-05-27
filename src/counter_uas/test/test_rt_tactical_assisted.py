"""PLAT-RT-TAC3 — assisted tactical recommendation tests."""

from __future__ import annotations

import sys
import uuid
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
if str(_BRIDGE_PKG) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_PKG))

from rt_sandbox.governance import classify_command  # noqa: E402
from rt_sandbox.session_manager import BridgeSessionManager, GovernanceConfig  # noqa: E402


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


def _start_with_two_interceptors(
    manager: BridgeSessionManager,
) -> tuple[str, str, str, str, str]:
    start = _cmd(manager, "start_session")
    assert start["ok"] is True
    sid = start["session_id"]
    i1 = _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "interceptor", "pose": _pose(95, 0, 10)},
    )
    assert i1["ok"] is True
    iid_near = i1["entity_id"]
    i2 = _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "interceptor", "pose": _pose(0, 0, 10)},
    )
    assert i2["ok"] is True
    iid_far = i2["entity_id"]
    t = _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "drone", "pose": _pose(100, 0, 10)},
    )
    assert t["ok"] is True
    tid = t["entity_id"]
    return sid, iid_near, iid_far, tid


def _assisted_session(
    manager: BridgeSessionManager,
) -> tuple[str, str, str, str, str]:
    sid, iid_near, iid_far, tid = _start_with_two_interceptors(manager)
    mode = _cmd(manager, "set_tactical_mode", sid, payload={"mode": "assisted"})
    assert mode["ok"] is True
    return sid, iid_near, iid_far, tid


def test_recommendation_verbs_classified() -> None:
    for verb in (
        "request_recommendation",
        "approve_recommendation",
        "reject_recommendation",
    ):
        assert classify_command(verb) is None


def test_request_forbidden_in_manual(manager: BridgeSessionManager) -> None:
    sid, _, _, tid = _start_with_two_interceptors(manager)
    bad = _cmd(manager, "request_recommendation", sid, payload={})
    assert bad["ok"] is False
    assert bad["error_code"] == "COMMAND_FORBIDDEN"


def test_assign_forbidden_in_assisted(manager: BridgeSessionManager) -> None:
    sid, iid_near, _, tid = _assisted_session(manager)
    bad = _cmd(
        manager,
        "assign_candidate",
        sid,
        payload={"interceptor_id": iid_near, "target_id": tid},
    )
    assert bad["ok"] is False
    assert bad["error_code"] == "COMMAND_FORBIDDEN"


def test_request_does_not_commit_assignment(manager: BridgeSessionManager) -> None:
    sid, _, _, _ = _assisted_session(manager)
    req = _cmd(manager, "request_recommendation", sid, payload={})
    assert req["ok"] is True
    assert req["tactical_state"]["assigned_interceptor_id"] is None
    assert req["tactical_state"]["assigned_target_id"] is None
    rec = req["tactical_recommendation"]
    assert rec["schema"] == "rt_tactical_recommendation_v1"
    assert rec["authority_label"] == "tactical_recommendation_explanatory"


def test_ranking_picks_nearest_interceptor(manager: BridgeSessionManager) -> None:
    sid, iid_near, iid_far, tid = _assisted_session(manager)
    req = _cmd(manager, "request_recommendation", sid, payload={})
    assert req["ok"] is True
    rec = req["tactical_recommendation"]
    assert rec["feasibility"]["feasible"] is True
    assert rec["recommended_interceptor_id"] == iid_near
    assert rec["recommended_target_id"] == tid
    assert rec["tti_s"] is not None


def test_approve_and_reject_flow(manager: BridgeSessionManager) -> None:
    sid, _, _, tid = _assisted_session(manager)
    req = _cmd(manager, "request_recommendation", sid, payload={})
    rec = req["tactical_recommendation"]
    rec_id = rec["recommendation_id"]
    assert rec_id
    bad = _cmd(
        manager,
        "approve_recommendation",
        sid,
        payload={"recommendation_id": "wrong-id"},
    )
    assert bad["ok"] is False
    approved = _cmd(
        manager,
        "approve_recommendation",
        sid,
        payload={"recommendation_id": rec_id},
    )
    assert approved["ok"] is True
    assert approved["tactical_state"]["authority_label"] == "user_approval_authoritative"
    assert approved["tactical_state"]["assigned_interceptor_id"] == rec[
        "recommended_interceptor_id"
    ]
    assert approved["tactical_state"]["assigned_target_id"] == tid

    req2 = _cmd(manager, "request_recommendation", sid, payload={})
    rec_id2 = req2["tactical_recommendation"]["recommendation_id"]
    rejected = _cmd(
        manager,
        "reject_recommendation",
        sid,
        payload={"recommendation_id": rec_id2},
    )
    assert rejected["ok"] is True
    assert rejected["tactical_state"]["pending_recommendation_id"] is None


def test_tactical_recommendation_telemetry_channel(
    manager: BridgeSessionManager,
) -> None:
    sid, _, _, _ = _assisted_session(manager)
    sub = _cmd(
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
            ]
        },
    )
    assert sub["ok"] is True
    sub_id = sub["subscription_id"]
    _cmd(manager, "request_recommendation", sid, payload={})
    events = manager.pull_telemetry(sid, sub_id, max_events=64)["events"]
    rec_events = [e for e in events if e.get("channel") == "tactical_recommendation"]
    assert rec_events
    assert rec_events[-1]["payload"].get("schema") == "rt_tactical_recommendation_v1"
