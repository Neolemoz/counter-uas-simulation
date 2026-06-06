"""PLAT-RT-INTEL-LIVE1 protected-center and live assembler tests."""

from __future__ import annotations

import sys
import time
import uuid
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
if str(_BRIDGE_PKG) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_PKG))

from rt_sandbox.adapter_poll import run_adapter_poll_tick  # noqa: E402
from rt_sandbox.governance import GovernanceConfig  # noqa: E402
from rt_sandbox.lifecycle import SessionState  # noqa: E402
from rt_sandbox.rt_intelligence_advisory_transport import (  # noqa: E402
    build_intelligence_advisory_transport,
)
from rt_sandbox.session_adapter_results import apply_adapter_poll_result  # noqa: E402
from rt_sandbox.session_manager import BridgeSessionManager  # noqa: E402
from rt_sandbox.session_record import SessionRecord  # noqa: E402
from rt_sandbox.tactical_controller import TacticalController  # noqa: E402
from rt_sandbox.telemetry_bridge import TelemetryMirror  # noqa: E402
from rt_sandbox.world_state import WorldStateStore  # noqa: E402


def _now(offset_s: float = 0.0) -> str:
    return (datetime.now(timezone.utc) + timedelta(seconds=offset_s)).isoformat()


def _pose(x: float, y: float, z: float) -> dict[str, float]:
    return {"x": x, "y": y, "z": z}


def _session(*, attackers: int = 1, defenders: int = 1) -> SessionRecord:
    session = SessionRecord(
        session_id="intel-live",
        state=SessionState.RUNNING,
        created_monotonic=time.monotonic(),
        bridge_ready_deadline=time.monotonic() + 10.0,
    )
    session.world = WorldStateStore(session.session_id)
    center, err = session.world.registry.spawn("radar", _pose(0.0, 0.0, 0.0), entity_id="center")
    assert err is None and center is not None
    session.protected_center_entity_id = center.entity_id
    rows = []
    for index in range(attackers):
        entity_id = f"attacker-{index}"
        record, err = session.world.registry.spawn(
            "drone", _pose(100.0 + index, 0.0, 20.0), entity_id=entity_id
        )
        assert err is None and record is not None
        rows.append({"entity_id": entity_id, "velocity": {"x": 0.0, "y": 0.0, "z": 0.0}})
    for index in range(defenders):
        record, err = session.world.registry.spawn(
            "interceptor",
            _pose(0.0, 10.0 + index, 10.0),
            entity_id=f"defender-{index}",
        )
        assert err is None and record is not None
    session.world.bump_revision()
    session.telemetry_mirror = TelemetryMirror(
        session_id=session.session_id,
        telemetry_revision=7,
        last_poll_utc=_now(),
        telemetry_health="ok",
        entity_pose_mirror={"entities": rows},
    )
    session.tactical = TacticalController(session.session_id)
    return session


def _transport(session: SessionRecord) -> dict:
    return build_intelligence_advisory_transport(session, now_utc=_now())


def _assert_stale(session: SessionRecord, reason: str) -> None:
    payload = _transport(session)
    assert payload["stale"] is True
    assert payload["stale_reason"] == reason
    assert payload["advisories"] == []


def test_protected_center_missing_and_deleted_fail_closed() -> None:
    session = _session()
    session.protected_center_entity_id = None
    _assert_stale(session, "protected_center_unavailable")
    session.protected_center_entity_id = "center"
    session.world.registry.delete("center")
    _assert_stale(session, "protected_center_unavailable")


def test_explicit_zero_velocity_is_valid() -> None:
    payload = _transport(_session())
    assert payload["stale"] is False
    descent = payload["advisories"][0]["threat_evaluation"]["threat_components"][
        "descent_factor"
    ]
    assert descent["value_mps"] == 0.0


def test_missing_duplicate_and_nonfinite_velocity_fail_closed() -> None:
    missing = _session()
    missing.telemetry_mirror.entity_pose_mirror = {"entities": [{"entity_id": "attacker-0"}]}
    _assert_stale(missing, "attacker_velocity_missing")

    duplicate = _session()
    row = {"entity_id": "attacker-0", "velocity": {"x": 0.0, "y": 0.0, "z": 0.0}}
    duplicate.telemetry_mirror.entity_pose_mirror = {"entities": [row, dict(row)]}
    _assert_stale(duplicate, "snapshot_validation_failed")

    nonfinite = _session()
    nonfinite.telemetry_mirror.entity_pose_mirror["entities"][0]["velocity"]["x"] = float("nan")
    _assert_stale(nonfinite, "attacker_velocity_nonfinite")


def test_velocity_stale_reason_and_global_precedence() -> None:
    session = _session(attackers=2)
    rows = session.telemetry_mirror.entity_pose_mirror["entities"]
    rows[0]["telemetry_revision"] = 6
    rows[1].pop("velocity")
    _assert_stale(session, "attacker_velocity_missing")
    rows[1]["velocity"] = {"x": 0.0, "y": 0.0, "z": 0.0}
    _assert_stale(session, "attacker_velocity_stale")


def test_stale_telemetry_and_feedback_lost_fail_closed() -> None:
    stale = _session()
    stale.telemetry_mirror.last_poll_utc = _now(-60.0)
    _assert_stale(stale, "telemetry_stale")
    lost = _session()
    lost.telemetry_mirror.telemetry_health = "feedback_lost"
    _assert_stale(lost, "telemetry_feedback_lost")


def test_stopped_session_fails_closed() -> None:
    session = _session()
    session.state = SessionState.STOPPED
    _assert_stale(session, "snapshot_validation_failed")


def test_revision_race_fails_closed() -> None:
    session = _session()
    original = session.world.registry.all_entities

    def racing_all_entities():
        rows = original()
        session.world.bump_revision()
        return rows

    session.world.registry.all_entities = racing_all_entities
    _assert_stale(session, "snapshot_validation_failed")


def test_empty_attacker_and_defender_sets_are_valid() -> None:
    no_attackers = _session(attackers=0)
    payload = _transport(no_attackers)
    assert payload["stale"] is False
    assert payload["advisories"] == []

    no_defenders = _session(defenders=0)
    payload = _transport(no_defenders)
    assert payload["stale"] is False
    assert payload["advisories"][0]["recommended_defender"]["defender_id"] is None


class _FailedTelemetryRuntime:
    kind = "adapter"

    def poll_telemetry(self, **_kwargs):
        return {"error_code": "IPC_FAILURE"}


class _Audit:
    def append(self, *_args, **_kwargs):
        return None


def test_poll_failure_publishes_fail_closed_advisory() -> None:
    session = _session()
    session.runtime = _FailedTelemetryRuntime()
    config = GovernanceConfig(enable_gazebo_adapter=True, telemetry_bridge_enabled=True)
    result = run_adapter_poll_tick(session, config, poll_telemetry=True)
    assert result.should_publish_channels is True
    published = {}

    def publish(channel: str) -> None:
        if channel == "intelligence_advisory":
            published[channel] = _transport(session)

    apply_adapter_poll_result(session, result, _Audit(), publish)
    assert published["intelligence_advisory"]["stale_reason"] == "telemetry_feedback_lost"
    assert published["intelligence_advisory"]["advisories"] == []


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


def test_explicit_designation_replacement_delete_and_reset(tmp_path: Path) -> None:
    (tmp_path / "AGENTS.md").write_text("# test repo\n")
    (tmp_path / "runs" / "rt_sandbox").mkdir(parents=True)
    manager = BridgeSessionManager(
        config=GovernanceConfig(command_rate_burst=1000, command_rate_sustained=1000.0),
        repo_root=tmp_path,
    )
    sid = _cmd(manager, "start_session")["session_id"]
    first = _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "radar", "entity_id": "center-a", "pose": _pose(0, 0, 0)},
    )
    second = _cmd(
        manager,
        "spawn_entity",
        sid,
        payload={"entity_type": "radar", "entity_id": "center-b", "pose": _pose(10, 0, 0)},
    )
    assert first["ok"] and second["ok"]
    assert _cmd(manager, "designate_protected_center", sid, payload={"entity_id": "center-a"})["ok"]
    assert not _cmd(
        manager, "designate_protected_center", sid, payload={"entity_id": "center-b"}
    )["ok"]
    assert _cmd(
        manager,
        "designate_protected_center",
        sid,
        payload={"entity_id": "center-b", "replace": True},
    )["ok"]
    session = manager._registry.get(sid)
    assert session.protected_center_entity_id == "center-b"
    assert _cmd(manager, "delete_entity", sid, payload={"entity_id": "center-b"})["ok"]
    assert session.protected_center_entity_id is None
    assert _cmd(manager, "designate_protected_center", sid, payload={"entity_id": "center-a"})["ok"]
    assert _cmd(manager, "reset_session", sid)["ok"]
    assert session.protected_center_entity_id is None
