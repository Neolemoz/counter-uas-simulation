"""Tactical command handlers (PLAT-RT-TAC2 manual + PLAT-RT-TAC3 assisted)."""

from __future__ import annotations

from typing import Any, Callable

from rt_sandbox.adapter_sync import sync_move
from rt_sandbox.audit_log import AuditLog
from rt_sandbox.governance import GovernanceConfig
from rt_sandbox.lifecycle import can_transition
from rt_sandbox.session_entity_handlers import finish_entity_sync
from rt_sandbox.session_record import SessionRecord
from rt_sandbox.session_response import fail, ok
from rt_sandbox.session_telemetry_coordinator import pose_sync_summary
from rt_sandbox.tactical_controller import TacticalController
from rt_sandbox.tactical_autonomous import monotonic_now, tick_autonomous_session
from rt_sandbox.tactical_state import (
    ALLOWED_TACTICAL_MODES_TAC4,
    TACTICAL_MODE_ASSISTED,
    TACTICAL_MODE_AUTONOMOUS,
    TACTICAL_MODE_MANUAL,
)
from rt_sandbox.tactical_telemetry import (
    publish_tactical_recommendation,
    publish_tactical_state,
    tactical_recommendation_snapshot_for_response,
    tactical_state_snapshot_for_response,
)
from rt_sandbox.telemetry import TelemetryBuffer


def _tactical_audit(
    audit: AuditLog,
    session: SessionRecord,
    *,
    command_id: str,
    command_type: str,
    issued_by: str,
    result: str,
    detail: dict[str, Any] | None = None,
) -> None:
    audit.append(
        session.session_id,
        command_id=command_id,
        command_type=command_type,
        issued_by=issued_by,
        result=result,
        detail=detail or {},
    )


def _apply_interceptor_move(
    session: SessionRecord,
    interceptor_id: str,
    pose: dict[str, float],
    base: dict[str, Any],
    *,
    config: GovernanceConfig,
    audit: AuditLog,
    publish_channel: Callable[[str], None],
    command_id: str,
    issued_by: str,
) -> dict[str, Any] | None:
    if session.world is None:
        return fail(base, "INVALID_STATE", "world not initialized")
    world = session.world
    registry = world.registry
    record, err = registry.move(interceptor_id, pose)
    if err:
        return fail(base, err, err)
    world.bump_revision()
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
    return None


def apply_interceptor_move_autonomous(
    session: SessionRecord,
    interceptor_id: str,
    pose: dict[str, float],
    *,
    config: GovernanceConfig,
    audit: AuditLog,
    publish_channel: Callable[[str], None],
) -> str | None:
    """Apply intercept move from autonomous tick; returns error_code or None."""
    if session.world is None:
        return "INVALID_STATE"
    world = session.world
    registry = world.registry
    record, err = registry.move(interceptor_id, pose)
    if err:
        return err
    world.bump_revision()
    sync_result = sync_move(
        session.runtime,
        config,
        record.entity_id,
        record.entity_type,
        dict(record.pose),
        bridge_revision=world.revision,
    )
    if sync_result and sync_result.get("error_code"):
        return str(sync_result["error_code"])
    base_stub: dict[str, Any] = {
        "ok": True,
        "session_id": session.session_id,
        "command_id": "autonomous_tick",
    }
    _, sync_fail = finish_entity_sync(
        session,
        config,
        audit,
        publish_channel,
        entity_id=record.entity_id,
        entity_type=record.entity_type,
        command_pose=dict(record.pose),
        sync_revision=world.revision,
        push_result=sync_result,
        base=base_stub,
        command_id="autonomous_tick",
        issued_by="bridge_autonomous_tick",
    )
    if sync_fail:
        return str(sync_fail.get("error_code", "SYNC_FAILED"))
    return None


def tick_tactical_autonomous_for_session(
    session: SessionRecord,
    now: float,
    *,
    config: GovernanceConfig,
    audit: AuditLog,
    publish_channel: Callable[[str], None],
) -> None:
    tactical = session.tactical
    if tactical is None or not isinstance(tactical, TacticalController):
        return

    def apply_move(iid: str, pose: dict[str, float]) -> str | None:
        return apply_interceptor_move_autonomous(
            session,
            iid,
            pose,
            config=config,
            audit=audit,
            publish_channel=publish_channel,
        )

    tick_autonomous_session(
        session,
        tactical,
        now,
        audit=audit,
        publish_channel=publish_channel,
        apply_move=apply_move,
    )


def _tactical_ok_response(
    session: SessionRecord,
    base: dict[str, Any],
    tactical: TacticalController,
    *,
    user_approval: bool = False,
    include_recommendation: bool = False,
) -> dict[str, Any]:
    resp = ok(base, state=session.state.value)
    resp["tactical_state"] = tactical_state_snapshot_for_response(
        session, user_approval=user_approval
    )
    if include_recommendation:
        resp["tactical_recommendation"] = tactical_recommendation_snapshot_for_response(
            session
        )
    return resp


def handle_tactical(
    session: SessionRecord,
    command_type: str,
    payload: Any,
    base: dict[str, Any],
    *,
    config: GovernanceConfig,
    audit: AuditLog,
    telemetry: TelemetryBuffer,
    publish_channel: Callable[[str], None],
    command_id: str,
    issued_by: str,
) -> dict[str, Any]:
    if not can_transition(session.state, command_type):
        return fail(base, "INVALID_STATE", session.state.value)
    if not session.runtime.is_alive():
        return fail(base, "RUNTIME_UNAVAILABLE", "runtime backend exited")
    if session.tactical is None:
        return fail(base, "INVALID_STATE", "tactical controller not initialized")
    tactical = session.tactical
    if not isinstance(payload, dict) and command_type != "get_tactical_state":
        return fail(base, "INVALID_PAYLOAD", command_type)
    payload = payload if isinstance(payload, dict) else {}

    if command_type == "set_tactical_mode":
        mode = str(payload.get("mode", "")).lower()
        if mode not in ALLOWED_TACTICAL_MODES_TAC4:
            return fail(base, "COMMAND_FORBIDDEN", "invalid tactical mode")
        if (
            mode == TACTICAL_MODE_AUTONOMOUS
            and tactical.state.mode == TACTICAL_MODE_ASSISTED
        ):
            return fail(
                base,
                "COMMAND_FORBIDDEN",
                "switch to manual before autonomous mode",
            )
        prev_mode = tactical.state.mode
        err = tactical.set_mode(mode)
        if err:
            return fail(base, err, err)
        _tactical_audit(
            audit,
            session,
            command_id=command_id,
            command_type="tactical_mode_changed",
            issued_by=issued_by,
            result="OK",
            detail={"mode": mode, "from_mode": prev_mode},
        )
        publish_tactical_state(session, publish_channel)
        if mode != TACTICAL_MODE_ASSISTED:
            publish_tactical_recommendation(session, publish_channel)
        return _tactical_ok_response(
            session,
            base,
            tactical,
            include_recommendation=mode == TACTICAL_MODE_ASSISTED,
        )

    if command_type == "pause_autonomous_loop":
        if tactical.state.mode != TACTICAL_MODE_AUTONOMOUS:
            return fail(
                base,
                "COMMAND_FORBIDDEN",
                "pause_autonomous_loop requires autonomous mode",
            )
        err = tactical.pause_autonomous_loop()
        if err:
            return fail(base, err, err)
        tactical.capture_buffer.record_pause_resume(action="paused", initiator=issued_by)
        _tactical_audit(
            audit,
            session,
            command_id=command_id,
            command_type="tactical_autonomous_paused",
            issued_by=issued_by,
            result="OK",
            detail={},
        )
        publish_tactical_state(session, publish_channel)
        return _tactical_ok_response(session, base, tactical)

    if command_type == "resume_autonomous_loop":
        if tactical.state.mode != TACTICAL_MODE_AUTONOMOUS:
            return fail(
                base,
                "COMMAND_FORBIDDEN",
                "resume_autonomous_loop requires autonomous mode",
            )
        err = tactical.resume_autonomous_loop()
        if err:
            return fail(base, err, err)
        tactical.capture_buffer.record_pause_resume(action="resumed", initiator=issued_by)
        _tactical_audit(
            audit,
            session,
            command_id=command_id,
            command_type="tactical_autonomous_resumed",
            issued_by=issued_by,
            result="OK",
            detail={},
        )
        publish_tactical_state(session, publish_channel)
        tick_tactical_autonomous_for_session(
            session,
            monotonic_now(),
            config=config,
            audit=audit,
            publish_channel=publish_channel,
        )
        return _tactical_ok_response(session, base, tactical)

    if command_type == "select_candidate":
        if tactical.state.mode == TACTICAL_MODE_AUTONOMOUS:
            return fail(
                base,
                "COMMAND_FORBIDDEN",
                "select_candidate forbidden in autonomous mode",
            )
        role = str(payload.get("role", ""))
        entity_id = str(payload.get("entity_id", ""))
        if not role or not entity_id:
            return fail(base, "INVALID_PAYLOAD", "role and entity_id required")
        err = tactical.select_candidate_on_session(session, role, entity_id)
        if err:
            return fail(base, err, err)
        _tactical_audit(
            audit,
            session,
            command_id=command_id,
            command_type="tactical_candidate_selected",
            issued_by=issued_by,
            result="OK",
            detail={"role": role, "entity_id": entity_id},
        )
        publish_tactical_state(session, publish_channel)
        return _tactical_ok_response(session, base, tactical)

    if command_type == "assign_candidate":
        if tactical.state.mode == TACTICAL_MODE_ASSISTED:
            return fail(
                base,
                "COMMAND_FORBIDDEN",
                "use approve_recommendation in assisted mode",
            )
        if tactical.state.mode == TACTICAL_MODE_AUTONOMOUS:
            return fail(
                base,
                "COMMAND_FORBIDDEN",
                "assign_candidate forbidden in autonomous mode",
            )
        interceptor_id = payload.get("interceptor_id")
        target_id = payload.get("target_id")
        iid = str(interceptor_id) if interceptor_id else None
        tid = str(target_id) if target_id else None
        err, move_pose = tactical.assign_candidate_on_session(session, iid, tid)
        if err:
            return fail(base, err, err)
        assert move_pose is not None
        iid = tactical.state.assigned_interceptor_id
        assert iid is not None
        move_fail = _apply_interceptor_move(
            session,
            iid,
            move_pose,
            base,
            config=config,
            audit=audit,
            publish_channel=publish_channel,
            command_id=command_id,
            issued_by=issued_by,
        )
        if move_fail is not None:
            return move_fail
        _tactical_audit(
            audit,
            session,
            command_id=command_id,
            command_type="tactical_assignment_committed",
            issued_by=issued_by,
            result="OK",
            detail={
                "assigned_interceptor_id": tactical.state.assigned_interceptor_id,
                "assigned_target_id": tactical.state.assigned_target_id,
                "tti_s": tactical.state.tti_s,
            },
        )
        publish_tactical_state(session, publish_channel)
        resp = _tactical_ok_response(session, base, tactical)
        if session.world:
            resp["world_summary"] = session.world.world_summary(
                pose_sync_summary=pose_sync_summary(session)
            )
        return resp

    if command_type == "clear_assignment":
        if tactical.state.mode == TACTICAL_MODE_AUTONOMOUS:
            return fail(
                base,
                "COMMAND_FORBIDDEN",
                "clear_assignment forbidden in autonomous mode",
            )
        tactical.clear_assignment_on_session(session)
        _tactical_audit(
            audit,
            session,
            command_id=command_id,
            command_type="tactical_assignment_cleared",
            issued_by=issued_by,
            result="OK",
            detail={},
        )
        publish_tactical_state(session, publish_channel)
        return _tactical_ok_response(session, base, tactical)

    if command_type == "request_recommendation":
        if tactical.state.mode != TACTICAL_MODE_ASSISTED:
            return fail(
                base,
                "COMMAND_FORBIDDEN",
                "request_recommendation requires assisted mode",
            )
        hint_i = payload.get("interceptor_id") or payload.get("selected_interceptor_id")
        hint_t = payload.get("target_id") or payload.get("selected_target_id")
        iid = str(hint_i) if hint_i else None
        tid = str(hint_t) if hint_t else None
        err = tactical.request_recommendation_on_session(
            session, hint_interceptor_id=iid, hint_target_id=tid
        )
        if err:
            return fail(base, err, err)
        rec = tactical.state.pending_recommendation
        _tactical_audit(
            audit,
            session,
            command_id=command_id,
            command_type="tactical_recommendation_issued",
            issued_by=issued_by,
            result="OK",
            detail=rec.to_dict() if rec else {},
        )
        publish_tactical_state(session, publish_channel)
        publish_tactical_recommendation(session, publish_channel)
        return _tactical_ok_response(
            session, base, tactical, include_recommendation=True
        )

    if command_type == "approve_recommendation":
        if tactical.state.mode != TACTICAL_MODE_ASSISTED:
            return fail(
                base,
                "COMMAND_FORBIDDEN",
                "approve_recommendation requires assisted mode",
            )
        rec_id = str(payload.get("recommendation_id", ""))
        if not rec_id:
            return fail(base, "INVALID_PAYLOAD", "recommendation_id required")
        err, move_pose = tactical.approve_recommendation_on_session(session, rec_id)
        if err:
            return fail(base, err, err)
        assert move_pose is not None
        iid = tactical.state.assigned_interceptor_id
        assert iid is not None
        move_fail = _apply_interceptor_move(
            session,
            iid,
            move_pose,
            base,
            config=config,
            audit=audit,
            publish_channel=publish_channel,
            command_id=command_id,
            issued_by=issued_by,
        )
        if move_fail is not None:
            return move_fail
        _tactical_audit(
            audit,
            session,
            command_id=command_id,
            command_type="tactical_recommendation_approved",
            issued_by=issued_by,
            result="OK",
            detail={
                "recommendation_id": rec_id,
                "assigned_interceptor_id": tactical.state.assigned_interceptor_id,
                "assigned_target_id": tactical.state.assigned_target_id,
            },
        )
        publish_tactical_state(session, publish_channel)
        publish_tactical_recommendation(session, publish_channel)
        resp = _tactical_ok_response(session, base, tactical, user_approval=True)
        if session.world:
            resp["world_summary"] = session.world.world_summary(
                pose_sync_summary=pose_sync_summary(session)
            )
        return resp

    if command_type == "reject_recommendation":
        if tactical.state.mode != TACTICAL_MODE_ASSISTED:
            return fail(
                base,
                "COMMAND_FORBIDDEN",
                "reject_recommendation requires assisted mode",
            )
        rec_id = payload.get("recommendation_id")
        rid = str(rec_id) if rec_id else None
        err = tactical.reject_recommendation_on_session(rid)
        if err:
            return fail(base, err, err)
        _tactical_audit(
            audit,
            session,
            command_id=command_id,
            command_type="tactical_recommendation_rejected",
            issued_by=issued_by,
            result="OK",
            detail={"recommendation_id": rid},
        )
        publish_tactical_state(session, publish_channel)
        publish_tactical_recommendation(session, publish_channel)
        return _tactical_ok_response(
            session, base, tactical, include_recommendation=True
        )

    if command_type == "get_tactical_state":
        publish_tactical_state(session, publish_channel)
        if tactical.state.mode == TACTICAL_MODE_ASSISTED:
            publish_tactical_recommendation(session, publish_channel)
        resp = _tactical_ok_response(session, base, tactical)
        if tactical.state.mode == TACTICAL_MODE_ASSISTED:
            resp["tactical_recommendation"] = tactical_recommendation_snapshot_for_response(
                session
            )
        return resp

    return fail(base, "COMMAND_FORBIDDEN", command_type)
