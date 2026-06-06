"""Entity command handlers for RT sandbox bridge (PLAT-RT-R3a)."""

from __future__ import annotations

from typing import Any, Callable

from rt_sandbox.adapter_poll import run_adapter_poll_tick
from rt_sandbox.adapter_sync import (
    run_post_entity_sync,
    sync_delete,
    sync_move,
    sync_spawn,
)
from rt_sandbox.audit_log import AuditLog
from rt_sandbox.governance import GovernanceConfig, validate_entity_payload
from rt_sandbox.lifecycle import can_transition
from rt_sandbox.session_adapter_results import apply_adapter_poll_result
from rt_sandbox.session_record import SessionRecord
from rt_sandbox.session_response import entity_ok, fail
from rt_sandbox.session_telemetry_coordinator import pose_sync_summary
from rt_sandbox.telemetry import TelemetryBuffer


def finish_entity_sync(
    session: SessionRecord,
    config: GovernanceConfig,
    audit: AuditLog,
    publish_channel: Callable[[str], None],
    *,
    entity_id: str,
    entity_type: str,
    command_pose: dict[str, float],
    sync_revision: int,
    push_result: dict[str, Any] | None,
    base: dict[str, Any],
    command_id: str,
    issued_by: str,
) -> tuple[dict[str, Any] | None, dict[str, Any] | None]:
    enriched, err_code, audit_event = run_post_entity_sync(
        session,
        config,
        entity_id=entity_id,
        entity_type=entity_type,
        command_pose=command_pose,
        sync_revision=sync_revision,
        push_result=push_result,
    )
    if audit_event:
        detail = enriched.get("sync_detail") if enriched else None
        if audit_event == "sync_update" and enriched:
            detail = detail or {
                "entity_id": entity_id,
                "sync_revision": sync_revision,
                "sim_entity_ref": enriched.get("sim_entity_ref"),
            }
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type=audit_event,
            issued_by=issued_by,
            result="OK" if not err_code else err_code,
            detail=detail,
        )
    if err_code == "SYNC_MISMATCH" and getattr(session.runtime, "mode", None) == "live":
        err_code = None
    if err_code:
        return None, fail(
            base,
            err_code,
            f"pose sync: {audit_event or err_code}",
        )
    poll_result = run_adapter_poll_tick(
        session,
        config,
        poll_feedback=False,
        poll_telemetry=True,
    )
    apply_adapter_poll_result(
        session,
        poll_result,
        audit,
        publish_channel,
        command_id=command_id,
        issued_by=issued_by,
    )
    return enriched, None


def handle_entity(
    session: SessionRecord,
    command_type: str,
    payload: Any,
    base: dict[str, Any],
    *,
    config: GovernanceConfig,
    audit: AuditLog,
    telemetry: TelemetryBuffer,
    telemetry_subs: Any,
    publish_channel: Callable[[str], None],
    publish_transition: Callable[[str, Any, bool], None],
    runtime_crashed: Callable[[], dict[str, Any]],
    command_id: str,
    issued_by: str,
    total_entity_count: Callable[[], int] | None = None,
) -> dict[str, Any]:
    if not can_transition(session.state, command_type):
        return fail(base, "INVALID_STATE", session.state.value)
    if not session.runtime.is_alive():
        return runtime_crashed()
    if session.world is None:
        return fail(base, "INVALID_STATE", "world not initialized")

    payload_err = validate_entity_payload(command_type, payload)
    if payload_err:
        return fail(base, payload_err, f"invalid payload for {command_type}")

    assert isinstance(payload, dict)
    world = session.world
    registry = world.registry

    if command_type == "designate_protected_center":
        entity_id = str(payload["entity_id"])
        if registry.get(entity_id) is None:
            return fail(base, "ENTITY_NOT_FOUND", entity_id)
        current = session.protected_center_entity_id
        if (
            current is not None
            and current != entity_id
            and payload.get("replace") is not True
        ):
            return fail(base, "INVALID_STATE", "explicit replace required")
        session.protected_center_entity_id = entity_id
        detail = {
            "protected_center_entity_id": entity_id,
            "replaced_entity_id": current if current != entity_id else None,
            "state": session.state.value,
        }
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="designate_protected_center",
            issued_by=issued_by,
            result="OK",
            detail=detail,
        )
        publish_transition("designate_protected_center", session.state, False)
        return entity_ok(session, base, telemetry, pose_sync_summary, detail=detail)

    if command_type == "spawn_entity":
        if total_entity_count is not None:
            if total_entity_count() >= config.max_total_entities_across_sessions:
                return fail(
                    base,
                    "RESOURCE_LIMIT_EXCEEDED",
                    "aggregate entity cap across sessions",
                )
        entity_type = str(payload["entity_type"])
        pose = {k: float(payload["pose"][k]) for k in ("x", "y", "z")}
        if "yaw_deg" in payload.get("pose", {}):
            pose["yaw_deg"] = float(payload["pose"]["yaw_deg"])
        eid_opt = payload.get("entity_id")
        entity_id = str(eid_opt) if eid_opt else None
        record, err = registry.spawn(entity_type, pose, entity_id=entity_id)
        if err:
            return fail(base, err, err)
        world.bump_revision()
        detail = {
            "entity_id": record.entity_id,
            "entity_type": record.entity_type,
            "pose": dict(record.pose),
            "state": session.state.value,
        }
        sync_result = sync_spawn(
            session.runtime,
            config,
            record.entity_id,
            record.entity_type,
            dict(record.pose),
            bridge_revision=world.revision,
        )
        if sync_result and sync_result.get("error_code"):
            return fail(
                base,
                str(sync_result["error_code"]),
                str(sync_result.get("error_message", "adapter sync failed")),
            )
        enriched, sync_fail = finish_entity_sync(
            session,
            config,
            audit,
            publish_channel,
            entity_id=record.entity_id,
            entity_type=record.entity_type,
            command_pose=dict(record.pose),
            sync_revision=world.revision,
            push_result=sync_result,
            base=base,
            command_id=command_id,
            issued_by=issued_by,
        )
        if sync_fail:
            return sync_fail
        if enriched:
            detail["adapter_sync"] = enriched
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="spawn_entity",
            issued_by=issued_by,
            result="OK",
            detail=detail,
        )
        publish_transition("spawn_entity", session.state, True)
        return entity_ok(
            session,
            base,
            telemetry,
            pose_sync_summary,
            entity_id=record.entity_id,
            detail=detail,
        )

    if command_type == "move_entity":
        entity_id = str(payload["entity_id"])
        pose = {k: float(payload["pose"][k]) for k in ("x", "y", "z")}
        if "yaw_deg" in payload.get("pose", {}):
            pose["yaw_deg"] = float(payload["pose"]["yaw_deg"])
        record, err = registry.move(entity_id, pose)
        if err:
            return fail(base, err, err)
        world.bump_revision()
        detail = {
            "entity_id": record.entity_id,
            "entity_type": record.entity_type,
            "pose": dict(record.pose),
            "state": session.state.value,
        }
        sync_result = sync_move(
            session.runtime,
            config,
            record.entity_id,
            record.entity_type,
            dict(record.pose),
            bridge_revision=world.revision,
        )
        if sync_result and sync_result.get("error_code"):
            return fail(
                base,
                str(sync_result["error_code"]),
                str(sync_result.get("error_message", "adapter sync failed")),
            )
        enriched, sync_fail = finish_entity_sync(
            session,
            config,
            audit,
            publish_channel,
            entity_id=record.entity_id,
            entity_type=record.entity_type,
            command_pose=dict(record.pose),
            sync_revision=world.revision,
            push_result=sync_result,
            base=base,
            command_id=command_id,
            issued_by=issued_by,
        )
        if sync_fail:
            return sync_fail
        if enriched:
            detail["adapter_sync"] = enriched
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="move_entity",
            issued_by=issued_by,
            result="OK",
            detail=detail,
        )
        publish_transition("move_entity", session.state, True)
        return entity_ok(
            session,
            base,
            telemetry,
            pose_sync_summary,
            detail=detail,
        )

    if command_type == "delete_entity":
        entity_id = str(payload["entity_id"])
        record, err = registry.delete(entity_id)
        if err:
            return fail(base, err, err)
        world.bump_revision()
        if session.protected_center_entity_id == entity_id:
            session.protected_center_entity_id = None
        detail = {
            "entity_id": record.entity_id,
            "entity_type": record.entity_type,
            "state": session.state.value,
        }
        sync_result = sync_delete(
            session.runtime,
            config,
            record.entity_id,
        )
        if sync_result and sync_result.get("error_code"):
            return fail(
                base,
                str(sync_result["error_code"]),
                str(sync_result.get("error_message", "adapter sync failed")),
            )
        if session.pose_sync is not None:
            session.pose_sync.remove_entity(record.entity_id)
        session.live_assignments.pop(record.entity_id, None)
        for defender_id, target_id in list(session.live_assignments.items()):
            if target_id == record.entity_id:
                session.live_assignments.pop(defender_id, None)
        if sync_result:
            detail["adapter_sync"] = sync_result
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="delete_entity",
            issued_by=issued_by,
            result="OK",
            detail=detail,
        )
        publish_transition("delete_entity", session.state, False)
        return entity_ok(
            session,
            base,
            telemetry,
            pose_sync_summary,
            detail=detail,
        )

    return fail(base, "COMMAND_FORBIDDEN", command_type)
