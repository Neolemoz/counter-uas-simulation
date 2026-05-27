"""PLAT-RT-TAC4 — autonomous tactical loop tests."""

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
from rt_sandbox.session_tactical_handlers import tick_tactical_autonomous_for_session  # noqa: E402
from rt_sandbox.tactical_state import (  # noqa: E402
    AUTONOMOUS_LOOP_STATUS_PAUSED,
    AUTONOMOUS_LOOP_STATUS_RUNNING,
)


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


def _spawn_scenario(manager: BridgeSessionManager) -> tuple[str, str, str]:
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


def _autonomous_running(manager: BridgeSessionManager) -> tuple[str, str, str]:
    sid, iid, tid = _spawn_scenario(manager)
    mode = _cmd(manager, "set_tactical_mode", sid, payload={"mode": "autonomous"})
    assert mode["ok"] is True
    assert mode["tactical_state"]["autonomous_loop_status"] == AUTONOMOUS_LOOP_STATUS_PAUSED
    resume = _cmd(manager, "resume_autonomous_loop", sid, payload={})
    assert resume["ok"] is True
    assert resume["tactical_state"]["autonomous_loop_status"] == AUTONOMOUS_LOOP_STATUS_RUNNING
    return sid, iid, tid


def test_pause_resume_verbs_classified() -> None:
    for verb in ("pause_autonomous_loop", "resume_autonomous_loop"):
        assert classify_command(verb) is None


def test_assisted_to_autonomous_forbidden(manager: BridgeSessionManager) -> None:
    sid, _, _ = _spawn_scenario(manager)
    _cmd(manager, "set_tactical_mode", sid, payload={"mode": "assisted"})
    bad = _cmd(manager, "set_tactical_mode", sid, payload={"mode": "autonomous"})
    assert bad["ok"] is False
    assert bad["error_code"] == "COMMAND_FORBIDDEN"


def test_assign_forbidden_in_autonomous(manager: BridgeSessionManager) -> None:
    sid, iid, tid = _autonomous_running(manager)
    bad = _cmd(
        manager,
        "assign_candidate",
        sid,
        payload={"interceptor_id": iid, "target_id": tid},
    )
    assert bad["ok"] is False
    assert bad["error_code"] == "COMMAND_FORBIDDEN"


def test_pause_resume_in_manual_forbidden(manager: BridgeSessionManager) -> None:
    sid, _, _ = _spawn_scenario(manager)
    for verb in ("pause_autonomous_loop", "resume_autonomous_loop"):
        bad = _cmd(manager, verb, sid, payload={})
        assert bad["ok"] is False
        assert bad["error_code"] == "COMMAND_FORBIDDEN"


def test_autonomous_tick_assigns_with_controller_authority(
    manager: BridgeSessionManager,
) -> None:
    sid, iid, tid = _autonomous_running(manager)
    session = manager._registry.get(sid)
    assert session is not None
    tactical = session.tactical
    assert tactical is not None
    now = 1000.0
    tactical.state.last_autonomous_tick_monotonic = None

    def publish(_ch: str) -> None:
        pass

    tick_tactical_autonomous_for_session(
        session,
        now,
        config=manager.config,
        audit=manager._audit,
        publish_channel=publish,
    )
    assert tactical.state.assigned_interceptor_id == iid
    assert tactical.state.assigned_target_id == tid
    state = _cmd(manager, "get_tactical_state", sid, payload={})
    assert state["tactical_state"]["authority_label"] == "tactical_controller_authoritative"


def test_assignment_lock_blocks_rapid_reassign(manager: BridgeSessionManager) -> None:
    sid, _, _ = _autonomous_running(manager)
    session = manager._registry.get(sid)
    assert session is not None
    tactical = session.tactical
    assert tactical is not None
    now = 2000.0
    tactical.state.last_autonomous_tick_monotonic = None

    def publish(_ch: str) -> None:
        pass

    tick_tactical_autonomous_for_session(
        session, now, config=manager.config, audit=manager._audit, publish_channel=publish
    )
    first_assigned = tactical.state.assigned_interceptor_id
    tick_tactical_autonomous_for_session(
        session,
        now + 0.1,
        config=manager.config,
        audit=manager._audit,
        publish_channel=publish,
    )
    assert tactical.state.assigned_interceptor_id == first_assigned
    assert tactical.state.tactical_health["summary"] == "assignment_lock_active"


def test_pause_stops_autonomous_commits(manager: BridgeSessionManager) -> None:
    sid, iid, tid = _autonomous_running(manager)
    paused = _cmd(manager, "pause_autonomous_loop", sid, payload={})
    assert paused["ok"] is True
    session = manager._registry.get(sid)
    assert session is not None
    tactical = session.tactical
    assert tactical is not None
    tactical.state.last_autonomous_tick_monotonic = None
    tactical.state.assignment_lock_until_monotonic = None
    tactical.clear_assignment_on_session(session)

    def publish(_ch: str) -> None:
        pass

    tick_tactical_autonomous_for_session(
        session,
        3000.0,
        config=manager.config,
        audit=manager._audit,
        publish_channel=publish,
    )
    assert tactical.state.assigned_interceptor_id is None


def test_manual_mode_stops_autonomous(manager: BridgeSessionManager) -> None:
    sid, _, _ = _autonomous_running(manager)
    manual = _cmd(manager, "set_tactical_mode", sid, payload={"mode": "manual"})
    assert manual["ok"] is True
    assert manual["tactical_state"]["tactical_mode"] == "manual"
    session = manager._registry.get(sid)
    assert session is not None
    tactical = session.tactical
    assert tactical is not None
    assert tactical.state.autonomous_loop_status == AUTONOMOUS_LOOP_STATUS_PAUSED


def test_autonomous_isolated_across_sessions(manager: BridgeSessionManager) -> None:
    sid_a, _, _ = _autonomous_running(manager)
    start_b = _cmd(manager, "start_session")
    sid_b = start_b["session_id"]
    session_a = manager._registry.get(sid_a)
    assert session_a is not None
    tactical_a = session_a.tactical
    assert tactical_a is not None
    tactical_a.state.last_autonomous_tick_monotonic = None
    tactical_a.state.assignment_lock_until_monotonic = None

    def publish(_ch: str) -> None:
        pass

    tick_tactical_autonomous_for_session(
        session_a,
        4000.0,
        config=manager.config,
        audit=manager._audit,
        publish_channel=publish,
    )
    snap_b = _cmd(manager, "get_tactical_state", sid_b, payload={})
    assert snap_b["tactical_state"]["assigned_interceptor_id"] is None


def test_pause_editing_session_mismatch(manager: BridgeSessionManager) -> None:
    sid_a, _, _ = _autonomous_running(manager)
    start_b = _cmd(manager, "start_session")
    sid_b = start_b["session_id"]
    _cmd(manager, "set_editing_session", sid_b, payload={"session_id": sid_b})
    bad = _cmd(manager, "pause_autonomous_loop", sid_a, payload={})
    assert bad["ok"] is False
    assert bad["error_code"] == "EDITING_SESSION_MISMATCH"
