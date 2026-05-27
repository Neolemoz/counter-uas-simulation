"""PLAT-RT-TAC2 — manual tactical controller tests."""

from __future__ import annotations

import sys
import uuid
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
_GZ_PKG = _REPO / "src" / "gazebo_target_sim"
if str(_BRIDGE_PKG) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_PKG))
if str(_GZ_PKG) not in sys.path:
    sys.path.insert(0, str(_GZ_PKG))

from rt_sandbox.governance import classify_command  # noqa: E402
from rt_sandbox.session_manager import BridgeSessionManager, GovernanceConfig  # noqa: E402
from rt_sandbox.tactical_geometry import compute_intercept, solve_intercept_time  # noqa: E402

from gazebo_target_sim.guidance_lib import (  # noqa: E402
    compute_intercept as gz_compute_intercept,
    solve_intercept_time as gz_solve_intercept_time,
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


def _start_with_entities(manager: BridgeSessionManager) -> tuple[str, str, str]:
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


def test_forbidden_tactical_and_intercept_commands() -> None:
    assert classify_command("tactical_foo") == "COMMAND_FORBIDDEN"
    assert classify_command("intercept") == "COMMAND_FORBIDDEN"
    assert classify_command("command_intercept") == "COMMAND_FORBIDDEN"


def test_set_tactical_mode_manual_and_assisted(manager: BridgeSessionManager) -> None:
    sid, _, _ = _start_with_entities(manager)
    ok_manual = _cmd(
        manager,
        "set_tactical_mode",
        sid,
        payload={"mode": "manual"},
    )
    assert ok_manual["ok"] is True
    ok_assisted = _cmd(
        manager,
        "set_tactical_mode",
        sid,
        payload={"mode": "assisted"},
    )
    assert ok_assisted["ok"] is True
    assert ok_assisted["tactical_state"]["tactical_mode"] == "assisted"
    ok_autonomous = _cmd(
        manager,
        "set_tactical_mode",
        sid,
        payload={"mode": "manual"},
    )
    assert ok_autonomous["ok"] is True
    auto = _cmd(manager, "set_tactical_mode", sid, payload={"mode": "autonomous"})
    assert auto["ok"] is True
    assert auto["tactical_state"]["tactical_mode"] == "autonomous"


def test_manual_assign_and_clear(manager: BridgeSessionManager) -> None:
    sid, iid, tid = _start_with_entities(manager)
    sel_i = _cmd(
        manager,
        "select_candidate",
        sid,
        payload={"role": "interceptor", "entity_id": iid},
    )
    assert sel_i["ok"] is True
    sel_t = _cmd(
        manager,
        "select_candidate",
        sid,
        payload={"role": "target", "entity_id": tid},
    )
    assert sel_t["ok"] is True
    state = sel_t["tactical_state"]
    assert state["tti_s"] is not None
    assign = _cmd(
        manager,
        "assign_candidate",
        sid,
        payload={"interceptor_id": iid, "target_id": tid},
    )
    assert assign["ok"] is True
    assert assign["tactical_state"]["assigned_interceptor_id"] == iid
    assert assign["tactical_state"]["assigned_target_id"] == tid
    cleared = _cmd(manager, "clear_assignment", sid, payload={})
    assert cleared["ok"] is True
    assert cleared["tactical_state"]["assigned_interceptor_id"] is None


def test_tactical_editing_session_mismatch(manager: BridgeSessionManager) -> None:
    sid_a, iid_a, tid_a = _start_with_entities(manager)
    start_b = _cmd(manager, "start_session")
    assert start_b["ok"] is True
    sid_b = start_b["session_id"]
    _cmd(
        manager,
        "set_editing_session",
        sid_b,
        payload={"session_id": sid_b},
    )
    bad = _cmd(
        manager,
        "assign_candidate",
        sid_a,
        payload={"interceptor_id": iid_a, "target_id": tid_a},
    )
    assert bad["ok"] is False
    assert bad["error_code"] == "EDITING_SESSION_MISMATCH"


def test_tactical_state_isolated_across_sessions(manager: BridgeSessionManager) -> None:
    sid_a, iid_a, tid_a = _start_with_entities(manager)
    start_b = _cmd(manager, "start_session")
    sid_b = start_b["session_id"]
    _cmd(
        manager,
        "spawn_entity",
        sid_b,
        payload={"entity_type": "interceptor", "pose": _pose(5, 5, 10)},
    )
    _cmd(
        manager,
        "select_candidate",
        sid_a,
        payload={"role": "interceptor", "entity_id": iid_a},
    )
    _cmd(
        manager,
        "assign_candidate",
        sid_a,
        payload={"interceptor_id": iid_a, "target_id": tid_a},
    )
    snap_b = _cmd(manager, "get_tactical_state", sid_b, payload={})
    assert snap_b["ok"] is True
    assert snap_b["tactical_state"]["assigned_interceptor_id"] is None


def test_tactical_state_telemetry_channel(manager: BridgeSessionManager) -> None:
    sid, iid, tid = _start_with_entities(manager)
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
            ]
        },
    )
    assert sub["ok"] is True
    sub_id = sub["subscription_id"]
    _cmd(
        manager,
        "assign_candidate",
        sid,
        payload={"interceptor_id": iid, "target_id": tid},
    )
    events = manager.pull_telemetry(sid, sub_id, max_events=32)["events"]
    tactical = [e for e in events if e.get("channel") == "tactical_state"]
    assert tactical
    assert tactical[-1]["payload"].get("schema") == "rt_tactical_state_v1"


def test_geometry_matches_guidance_lib() -> None:
    args = (100.0, 0.0, 10.0, 5.0, 0.0, 0.0, 0.0, 0.0, 10.0, 25.0)
    t_rt = solve_intercept_time(*args)
    t_gz = gz_solve_intercept_time(*args)
    assert t_rt is not None and t_gz is not None
    assert abs(t_rt - t_gz) < 1e-6
    c_rt = compute_intercept(*args)
    c_gz = gz_compute_intercept(*args)
    assert c_rt is not None and c_gz is not None
    assert abs(c_rt[0] - c_gz[0]) < 1e-6
