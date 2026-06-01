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
    SWITCH_DWELL_DURATION_S,
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


def _spawn_target(manager: BridgeSessionManager, session_id: str, x: float) -> str:
    t = _cmd(
        manager,
        "spawn_entity",
        session_id,
        payload={"entity_type": "drone", "pose": _pose(x, 0, 10)},
    )
    assert t["ok"] is True
    return t["entity_id"]



def _spawn_interceptor(manager: BridgeSessionManager, session_id: str, x: float) -> str:
    i = _cmd(
        manager,
        "spawn_entity",
        session_id,
        payload={"entity_type": "interceptor", "pose": _pose(x, 0, 10)},
    )
    assert i["ok"] is True
    return i["entity_id"]


def _start_autonomous_with_entities(
    manager: BridgeSessionManager,
    *,
    interceptor_xs: list[float],
    target_xs: list[float],
) -> tuple[str, list[str], list[str]]:
    start = _cmd(manager, "start_session")
    assert start["ok"] is True
    sid = start["session_id"]
    interceptors = [_spawn_interceptor(manager, sid, x) for x in interceptor_xs]
    targets = [_spawn_target(manager, sid, x) for x in target_xs]
    mode = _cmd(manager, "set_tactical_mode", sid, payload={"mode": "autonomous"})
    assert mode["ok"] is True
    resume = _cmd(manager, "resume_autonomous_loop", sid, payload={})
    assert resume["ok"] is True
    return sid, interceptors, targets

def _prime_current_assignment(
    manager: BridgeSessionManager,
    sid: str,
    iid: str,
    tid: str,
    *,
    now: float,
    tti_s: float = 4.0,
) -> None:
    session = manager._registry.get(sid)
    assert session is not None
    tactical = session.tactical
    assert tactical is not None
    assert session.world is not None
    record, err = session.world.registry.move(iid, _pose(0, 0, 10))
    assert err is None
    assert record is not None
    tactical.state.assigned_interceptor_id = iid
    tactical.state.assigned_target_id = tid
    tactical.state.selected_interceptor_id = iid
    tactical.state.selected_target_id = tid
    tactical.state.tti_s = tti_s
    tactical.state.last_assignment_monotonic = now - SWITCH_DWELL_DURATION_S - 1.0
    tactical.state.assignment_lock_until_monotonic = None
    tactical.state.last_autonomous_tick_monotonic = None


def _publish_noop(_ch: str) -> None:
    pass


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


def test_autonomous_hysteresis_blocks_tiny_tti_improvement(
    manager: BridgeSessionManager,
) -> None:
    sid, iid, current_tid = _autonomous_running(manager)
    candidate_tid = _spawn_target(manager, sid, 99.0)
    session = manager._registry.get(sid)
    assert session is not None
    tactical = session.tactical
    assert tactical is not None
    now = 5000.0
    _prime_current_assignment(manager, sid, iid, current_tid, now=now)

    tick_tactical_autonomous_for_session(
        session, now, config=manager.config, audit=manager._audit, publish_channel=_publish_noop
    )

    assert tactical.state.assigned_target_id == current_tid
    assert tactical.state.assigned_target_id != candidate_tid
    assert tactical.state.switch_blocked_reason == "tti_margin"
    assert tactical.state.candidate_tti_delta_s == pytest.approx(0.04)
    snap = _cmd(manager, "get_tactical_state", sid, payload={})["tactical_state"]
    assert snap["switch_blocked_reason"] == "tti_margin"
    assert snap["candidate_tti_delta_s"] == pytest.approx(0.04)
    assert snap["assignment_lock_active"] is False


def test_autonomous_hysteresis_allows_significant_tti_improvement(
    manager: BridgeSessionManager,
) -> None:
    sid, iid, current_tid = _autonomous_running(manager)
    candidate_tid = _spawn_target(manager, sid, 60.0)
    session = manager._registry.get(sid)
    assert session is not None
    tactical = session.tactical
    assert tactical is not None
    now = 6000.0
    _prime_current_assignment(manager, sid, iid, current_tid, now=now)

    tick_tactical_autonomous_for_session(
        session, now, config=manager.config, audit=manager._audit, publish_channel=_publish_noop
    )

    assert tactical.state.assigned_target_id == candidate_tid
    assert tactical.state.switch_blocked_reason is None
    assert tactical.state.candidate_tti_delta_s == pytest.approx(1.6)


def test_autonomous_hysteresis_blocks_during_dwell(
    manager: BridgeSessionManager,
) -> None:
    sid, iid, current_tid = _autonomous_running(manager)
    candidate_tid = _spawn_target(manager, sid, 60.0)
    session = manager._registry.get(sid)
    assert session is not None
    tactical = session.tactical
    assert tactical is not None
    now = 7000.0
    _prime_current_assignment(manager, sid, iid, current_tid, now=now)
    tactical.state.last_assignment_monotonic = now - 1.0

    tick_tactical_autonomous_for_session(
        session, now, config=manager.config, audit=manager._audit, publish_channel=_publish_noop
    )

    assert tactical.state.assigned_target_id == current_tid
    assert tactical.state.assigned_target_id != candidate_tid
    assert tactical.state.switch_blocked_reason == "dwell_active"
    assert tactical.state.candidate_tti_delta_s == pytest.approx(1.6)


def test_autonomous_assignment_lock_blocks_target_switch(
    manager: BridgeSessionManager,
) -> None:
    sid, iid, current_tid = _autonomous_running(manager)
    candidate_tid = _spawn_target(manager, sid, 60.0)
    session = manager._registry.get(sid)
    assert session is not None
    tactical = session.tactical
    assert tactical is not None
    now = 8000.0
    _prime_current_assignment(manager, sid, iid, current_tid, now=now)
    tactical.state.assignment_lock_until_monotonic = now + 10.0

    tick_tactical_autonomous_for_session(
        session, now, config=manager.config, audit=manager._audit, publish_channel=_publish_noop
    )

    assert tactical.state.assigned_target_id == current_tid
    assert tactical.state.assigned_target_id != candidate_tid
    assert tactical.state.switch_blocked_reason == "assignment_lock_active"
    assert tactical.state.tactical_health["summary"] == "assignment_lock_active"
    snap = _cmd(manager, "get_tactical_state", sid, payload={})["tactical_state"]
    assert snap["assignment_lock_active"] is True


def test_autonomous_coordination_splits_two_defenders_two_targets(
    manager: BridgeSessionManager,
) -> None:
    sid, interceptors, targets = _start_autonomous_with_entities(
        manager, interceptor_xs=[0.0, 1.0], target_xs=[10.0, 100.0]
    )
    session = manager._registry.get(sid)
    assert session is not None
    tactical = session.tactical
    assert tactical is not None

    tick_tactical_autonomous_for_session(
        session, 9000.0, config=manager.config, audit=manager._audit, publish_channel=_publish_noop
    )

    assert len(tactical.state.assigned_pairs) == 2
    assert set(session.live_assignments) == set(interceptors)
    assert set(session.live_assignments.values()) == set(targets)
    assert tactical.state.coordination_state == "one_to_one"
    assert tactical.state.duplicate_target_blocked is True
    snap = _cmd(manager, "get_tactical_state", sid, payload={})["tactical_state"]
    assert len(snap["assigned_pairs"]) == 2
    assert snap["coordination_state"] == "one_to_one"
    assert snap["duplicate_target_blocked"] is True


def test_autonomous_coordination_prevents_duplicate_target_when_alternative_exists(
    manager: BridgeSessionManager,
) -> None:
    sid, interceptors, targets = _start_autonomous_with_entities(
        manager, interceptor_xs=[0.0, 1.0], target_xs=[10.0, 100.0]
    )
    session = manager._registry.get(sid)
    assert session is not None
    tactical = session.tactical
    assert tactical is not None

    tick_tactical_autonomous_for_session(
        session, 9100.0, config=manager.config, audit=manager._audit, publish_channel=_publish_noop
    )

    assigned_targets = list(session.live_assignments.values())
    assert len(assigned_targets) == len(set(assigned_targets))
    assert set(assigned_targets) == set(targets)
    assert set(session.live_assignments) == set(interceptors)
    assert tactical.state.duplicate_target_blocked is True


def test_autonomous_coordination_allows_single_target_fallback(
    manager: BridgeSessionManager,
) -> None:
    sid, interceptors, targets = _start_autonomous_with_entities(
        manager, interceptor_xs=[0.0, 10.0], target_xs=[100.0]
    )
    session = manager._registry.get(sid)
    assert session is not None
    tactical = session.tactical
    assert tactical is not None

    tick_tactical_autonomous_for_session(
        session, 9200.0, config=manager.config, audit=manager._audit, publish_channel=_publish_noop
    )

    assert set(session.live_assignments) == set(interceptors)
    assert list(session.live_assignments.values()) == [targets[0], targets[0]]
    assert tactical.state.coordination_state == "single_target_fallback"
    assert tactical.state.duplicate_target_blocked is False


def test_autonomous_coordination_respects_assignment_lock(
    manager: BridgeSessionManager,
) -> None:
    sid, interceptors, targets = _start_autonomous_with_entities(
        manager, interceptor_xs=[0.0, 1.0], target_xs=[10.0, 100.0]
    )
    session = manager._registry.get(sid)
    assert session is not None
    tactical = session.tactical
    assert tactical is not None
    session.live_assignments.clear()
    tactical.state.assigned_pairs = []
    tactical.state.assigned_interceptor_id = None
    tactical.state.assigned_target_id = None
    tactical.state.last_autonomous_tick_monotonic = None
    tactical.state.assignment_lock_until_monotonic = 9310.0

    tick_tactical_autonomous_for_session(
        session, 9300.0, config=manager.config, audit=manager._audit, publish_channel=_publish_noop
    )

    assert session.live_assignments == {}
    assert tactical.state.assigned_pairs == []
    assert tactical.state.switch_blocked_reason == "assignment_lock_active"
    assert tactical.state.coordination_state == "assignment_lock_active"
    assert targets

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
