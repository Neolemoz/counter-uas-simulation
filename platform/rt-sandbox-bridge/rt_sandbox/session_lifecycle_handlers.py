"""Session lifecycle command handlers (PLAT-RT-R3a)."""

from __future__ import annotations

import time
import uuid
from typing import Any, Callable

from rt_sandbox.adapter_sync import clear_pose_sync, sync_reset_world
from rt_sandbox.audit_log import AuditLog
from rt_sandbox.export_audit_log import ExportAuditLog
from rt_sandbox.governance import GovernanceConfig
from rt_sandbox.lifecycle import SessionState, can_transition
from rt_sandbox.runtime_handle import create_runtime, runtime_is_adapter
from rt_sandbox.session_record import SessionRecord
from rt_sandbox.session_response import fail, ok
from rt_sandbox.session_teardown import (
    TeardownDeps,
    clear_adapter_mirrors,
    clear_tactical_state,
    clear_workflow_state,
    clear_world_with_audit,
    clear_telemetry_with_audit,
    teardown_discarded_session,
    teardown_failed_auto_cleanup,
    teardown_stopped_auto_cleanup,
)
from rt_sandbox.session_telemetry_coordinator import pose_sync_summary
from rt_sandbox.telemetry import TelemetryBuffer
from rt_sandbox.telemetry_bridge import clear_telemetry_mirror
from rt_sandbox.tactical_controller import TacticalController
from rt_sandbox.world_state import WorldStateStore


def runtime_crashed(
    session: SessionRecord,
    base: dict[str, Any],
    audit: AuditLog,
    config: GovernanceConfig,
    *,
    command_id: str,
    issued_by: str,
) -> dict[str, Any]:
    session.state = SessionState.RUNTIME_CRASHED
    session.cleanup_after = time.monotonic() + config.cleanup_pending_max_age_s
    audit.append(
        session.session_id,
        command_id=command_id,
        command_type="runtime_crashed",
        issued_by=issued_by,
        result="RUNTIME_UNAVAILABLE",
        detail={"state": session.state.value},
    )
    if runtime_is_adapter(session.runtime):
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="adapter_teardown",
            issued_by=issued_by,
            result="RUNTIME_UNAVAILABLE",
            detail={"trigger": "runtime_crashed"},
        )
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="adapter_feedback_lost",
            issued_by=issued_by,
            result="RUNTIME_UNAVAILABLE",
            detail={"trigger": "runtime_crashed"},
        )
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="telemetry_feedback_lost",
            issued_by=issued_by,
            result="RUNTIME_UNAVAILABLE",
            detail={"trigger": "runtime_crashed"},
        )
    clear_adapter_mirrors(session)
    return fail(base, "RUNTIME_UNAVAILABLE", "runtime backend exited")


def start_session(
    base: dict[str, Any],
    *,
    config: GovernanceConfig,
    audit: AuditLog,
    telemetry: TelemetryBuffer,
    publish_transition: Callable[[SessionRecord, str, SessionState], None],
    non_terminal_count: Callable[[], int],
    register_session: Callable[[SessionRecord], None],
    command_id: str,
    issued_by: str,
    now: float,
    runtime_profile: str = "stub",
) -> dict[str, Any]:
    if non_terminal_count() >= config.max_concurrent_sessions:
        return fail(base, "SESSION_CAPACITY_EXCEEDED", "session capacity reached")

    session_id = str(uuid.uuid4())
    runtime = create_runtime(config, session_id)
    record = SessionRecord(
        session_id=session_id,
        state=SessionState.CREATED,
        created_monotonic=now,
        bridge_ready_deadline=now + config.bridge_ready_timeout_s,
        issued_by=issued_by,
        world=WorldStateStore(session_id=session_id),
        runtime=runtime,
        runtime_profile=runtime_profile,
    )
    record.world.registry.max_entity_count = config.max_entity_count
    record.tactical = TacticalController(session_id)
    register_session(record)
    base["session_id"] = session_id

    try:
        pid = record.runtime.start()
        record.state = SessionState.RUNNING
        runtime_detail: dict[str, Any] = {
            "runtime_pid": pid,
            "runtime_kind": getattr(record.runtime, "kind", "unknown"),
            "state": record.state.value,
        }
        if runtime_is_adapter(record.runtime):
            runtime_detail["adapter_mode"] = getattr(record.runtime, "mode", None)
        audit.append(
            session_id,
            command_id=command_id,
            command_type="start_session",
            issued_by=issued_by,
            result="OK",
            detail=runtime_detail,
        )
        if runtime_is_adapter(record.runtime):
            audit.append(
                session_id,
                command_id=command_id,
                command_type="adapter_attach",
                issued_by=issued_by,
                result="OK",
                detail=runtime_detail,
            )
        summary = record.world.world_summary() if record.world else {}
        health = record.runtime.health_payload()
        hb = telemetry.maybe_emit(
            session_id,
            record.state.value,
            health.get("stub_alive") or health.get("adapter_alive", False),
            world_summary=summary,
        )
        publish_transition(record, "start_session", SessionState.CREATED)
        resp = ok(base, state=record.state.value)
        resp["world_summary"] = summary
        if hb:
            resp["heartbeat"] = hb
        return resp
    except OSError as exc:
        record.state = SessionState.FAILED
        audit.append(
            session_id,
            command_id=command_id,
            command_type="start_session",
            issued_by=issued_by,
            result="RUNTIME_UNAVAILABLE",
            detail={"error": str(exc)},
        )
        return fail(base, "RUNTIME_UNAVAILABLE", str(exc))


def pause_session(
    session: SessionRecord,
    base: dict[str, Any],
    *,
    audit: AuditLog,
    publish_transition: Callable[[SessionRecord, str, SessionState], None],
    runtime_crashed_fn: Callable[[], dict[str, Any]],
    command_id: str,
    issued_by: str,
) -> dict[str, Any]:
    if not can_transition(session.state, "pause_session"):
        return fail(base, "INVALID_STATE", session.state.value)
    if not session.runtime.is_alive():
        return runtime_crashed_fn()
    prev = session.state
    session.runtime.pause()
    session.state = SessionState.PAUSED
    audit.append(
        session.session_id,
        command_id=command_id,
        command_type="pause_session",
        issued_by=issued_by,
        result="OK",
        detail={"state": session.state.value},
    )
    publish_transition(session, "pause_session", prev)
    return ok(base, state=session.state.value)


def resume_session(
    session: SessionRecord,
    base: dict[str, Any],
    *,
    audit: AuditLog,
    publish_transition: Callable[[SessionRecord, str, SessionState], None],
    runtime_crashed_fn: Callable[[], dict[str, Any]],
    command_id: str,
    issued_by: str,
) -> dict[str, Any]:
    if not can_transition(session.state, "resume"):
        return fail(base, "INVALID_STATE", session.state.value)
    if not session.runtime.is_alive():
        return runtime_crashed_fn()
    prev = session.state
    session.runtime.resume()
    session.state = SessionState.RUNNING
    audit.append(
        session.session_id,
        command_id=command_id,
        command_type="resume",
        issued_by=issued_by,
        result="OK",
        detail={"state": session.state.value},
    )
    publish_transition(session, "resume", prev)
    return ok(base, state=session.state.value)


def reset_session(
    session: SessionRecord,
    base: dict[str, Any],
    *,
    config: GovernanceConfig,
    audit: AuditLog,
    telemetry: TelemetryBuffer,
    publish_transition: Callable[[SessionRecord, str, SessionState], None],
    command_id: str,
    issued_by: str,
) -> dict[str, Any]:
    if not can_transition(session.state, "reset_session"):
        return fail(base, "INVALID_STATE", session.state.value)
    if session.world is None:
        return fail(base, "INVALID_STATE", "world not initialized")
    pre_count = session.world.registry.count()
    pre_snapshot = session.world.snapshot().to_dict() if pre_count else None
    removed = session.world.reset()
    post_snapshot = session.world.snapshot().to_dict()
    detail = {
        "pre_entity_count": pre_count,
        "entities_removed": removed,
        "world_snapshot": post_snapshot,
        "state": session.state.value,
    }
    if pre_snapshot:
        detail["pre_reset_snapshot"] = pre_snapshot
    if session.workflow is not None:
        clear_workflow_state(
            session, audit, issued_by=issued_by, command_id=command_id
        )
        detail["workflow_reset"] = True
    sync_reset_world(session.runtime, config)
    clear_pose_sync(session)
    session.pose_sync = None
    clear_telemetry_mirror(session)
    session.telemetry_mirror = None
    if session.tactical is not None:
        session.tactical.reset()
    session.live_assignments.clear()
    audit.append(
        session.session_id,
        command_id=command_id,
        command_type="reset_session",
        issued_by=issued_by,
        result="OK",
        detail=detail,
    )
    publish_transition(session, "reset_session", session.state)
    resp = ok(base, state=session.state.value)
    resp["world_summary"] = session.world.world_summary(
        pose_sync_summary=pose_sync_summary(session)
    )
    telemetry.emit_world_summary(session.session_id, resp["world_summary"])
    return resp


def stop_session(
    session: SessionRecord,
    base: dict[str, Any],
    *,
    config: GovernanceConfig,
    audit: AuditLog,
    publish_transition: Callable[[SessionRecord, str, SessionState], None],
    command_id: str,
    issued_by: str,
    now: float,
    terminate_runtime: bool = False,
) -> dict[str, Any]:
    if not can_transition(session.state, "stop_session"):
        return fail(base, "INVALID_STATE", session.state.value)
    prev = session.state
    session.runtime.stop()
    if terminate_runtime:
        session.runtime.terminate()
    session.state = SessionState.STOPPED
    session.cleanup_after = now + config.session_cleanup_timeout_s
    audit.append(
        session.session_id,
        command_id=command_id,
        command_type="stop_session",
        issued_by=issued_by,
        result="OK",
        detail={
            "state": session.state.value,
            "cleanup_after_s": config.session_cleanup_timeout_s,
        },
    )
    publish_transition(session, "stop_session", prev)
    return ok(base, state=session.state.value)


def discard_session(
    session: SessionRecord,
    base: dict[str, Any],
    *,
    audit: AuditLog,
    export_audit: ExportAuditLog,
    telemetry_subs: Any,
    command_id: str,
    issued_by: str,
) -> dict[str, Any]:
    if not can_transition(session.state, "discard_session"):
        return fail(base, "INVALID_STATE", session.state.value)
    if session.state == SessionState.CAPTURED:
        export_audit.append(
            "capture_discarded",
            session_id=session.session_id,
            result="OK",
            detail={"trigger": "discard_session"},
        )
    session.state = SessionState.CLEANUP_PENDING
    teardown_discarded_session(
        session,
        TeardownDeps(audit=audit, telemetry_subs=telemetry_subs),
        command_id=command_id,
        issued_by=issued_by,
    )
    session.state = SessionState.DISCARDED
    session.cleanup_after = None
    audit.append(
        session.session_id,
        command_id=command_id,
        command_type="discard_session",
        issued_by=issued_by,
        result="OK",
        detail={"state": session.state.value, "cleanup": "complete"},
    )
    return ok(base, state=session.state.value)


def tick_timeouts(
    session: SessionRecord | None,
    *,
    config: GovernanceConfig,
    audit: AuditLog,
    telemetry_subs: Any,
    now: float,
) -> None:
    if session is None:
        return

    deps = TeardownDeps(audit=audit, telemetry_subs=telemetry_subs)

    if session.state == SessionState.CREATED and now > session.bridge_ready_deadline:
        session.state = SessionState.FAILED
        session.runtime.terminate()
        clear_world_with_audit(
            session, audit, extra_detail={"trigger": "bridge_ready_timeout"}
        )
        clear_telemetry_with_audit(
            session,
            telemetry_subs,
            audit,
            extra_detail={"trigger": "bridge_ready_timeout"},
        )
        clear_tactical_state(
            session,
            audit,
            trigger="bridge_ready_timeout",
            issued_by="bridge",
        )
        session.world = None
        session.cleanup_after = now + config.session_cleanup_timeout_s
        audit.append(
            session.session_id,
            command_id=None,
            command_type="bridge_ready_timeout",
            issued_by="bridge",
            result="failed",
            detail={"state": session.state.value},
        )

    if (
        session.state == SessionState.RUNNING
        and now - session.created_monotonic > config.max_session_duration_s
    ):
        session.runtime.stop()
        session.state = SessionState.STOPPED
        session.cleanup_after = now + config.session_cleanup_timeout_s
        audit.append(
            session.session_id,
            command_id=None,
            command_type="max_session_duration",
            issued_by="bridge",
            result="RESOURCE_LIMIT_EXCEEDED",
            detail={"state": session.state.value},
        )

    if session.state in {SessionState.RUNNING, SessionState.PAUSED} and not session.runtime.is_alive():
        session.state = SessionState.RUNTIME_CRASHED
        session.cleanup_after = now + config.cleanup_pending_max_age_s
        audit.append(
            session.session_id,
            command_id=None,
            command_type="runtime_crashed",
            issued_by="bridge",
            result="RUNTIME_UNAVAILABLE",
            detail={"state": session.state.value},
        )

    if session.state == SessionState.CAPTURED:
        return

    if session.cleanup_after is not None and now >= session.cleanup_after:
        if session.state == SessionState.STOPPED:
            session.state = SessionState.CLEANUP_PENDING
            teardown_stopped_auto_cleanup(session, deps)
            session.state = SessionState.DISCARDED
            session.cleanup_after = None
            audit.append(
                session.session_id,
                command_id=None,
                command_type="auto_cleanup",
                issued_by="bridge",
                result="OK",
                detail={"state": session.state.value},
            )
        elif session.state in {
            SessionState.FAILED,
            SessionState.RUNTIME_CRASHED,
            SessionState.CLEANUP_PENDING,
        }:
            teardown_failed_auto_cleanup(session, deps)
            session.state = SessionState.DISCARDED
            session.cleanup_after = None
            audit.append(
                session.session_id,
                command_id=None,
                command_type="auto_cleanup",
                issued_by="bridge",
                result="OK",
                detail={"state": session.state.value},
            )
