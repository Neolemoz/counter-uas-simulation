"""Bridge command response shaping helpers (PLAT-RT-R3a)."""

from __future__ import annotations

from typing import Any, Callable

from rt_sandbox.governance import GOVERNANCE_BANNER
from rt_sandbox.session_record import SessionRecord
from rt_sandbox.telemetry import TelemetryBuffer


def response_base(command_id: str, session_id: Any) -> dict[str, Any]:
    return {
        "governance_banner": GOVERNANCE_BANNER,
        "session_id": session_id,
        "command_id": command_id,
        "ok": False,
        "error_code": None,
        "message": None,
    }


def ok(base: dict[str, Any], *, state: str | None) -> dict[str, Any]:
    out = dict(base)
    out["ok"] = True
    out["error_code"] = "OK"
    out["state"] = state
    return out


def fail(base: dict[str, Any], error_code: str, message: str) -> dict[str, Any]:
    out = dict(base)
    out["ok"] = False
    out["error_code"] = error_code
    out["message"] = message
    return out


def entity_ok(
    session: SessionRecord,
    base: dict[str, Any],
    telemetry: TelemetryBuffer,
    pose_sync_summary: Callable[[SessionRecord], dict[str, Any] | None],
    *,
    entity_id: str | None = None,
    detail: dict[str, Any] | None = None,
) -> dict[str, Any]:
    world = session.world
    summary = (
        world.world_summary(pose_sync_summary=pose_sync_summary(session))
        if world
        else {}
    )
    resp = ok(base, state=session.state.value)
    resp["world_summary"] = summary
    if entity_id:
        resp["entity_id"] = entity_id
    if detail:
        resp["entities"] = world.registry.poses_for_telemetry() if world else []
    hb = telemetry.emit_entity_pose_heartbeat(
        session.session_id,
        world.registry.poses_for_telemetry() if world else [],
        world_summary=summary,
    )
    if hb:
        resp["telemetry"] = hb
    return resp
