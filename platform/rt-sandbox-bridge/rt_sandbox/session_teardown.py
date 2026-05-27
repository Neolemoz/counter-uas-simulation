"""Session teardown and cleanup sequencing (PLAT-RT-R3a)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from rt_sandbox.adapter_sync import clear_pose_sync
from rt_sandbox.audit_log import AuditLog
from rt_sandbox.runtime_handle import runtime_is_adapter
from rt_sandbox.session_record import SessionRecord
from rt_sandbox.telemetry_bridge import clear_telemetry_mirror
from rt_sandbox.telemetry_subscriptions import TelemetrySubscriptionStore
from rt_sandbox.tactical_controller import TacticalController


def clear_adapter_mirrors(session: SessionRecord) -> None:
    clear_pose_sync(session)
    session.pose_sync = None
    clear_telemetry_mirror(session)
    session.telemetry_mirror = None


def reset_template_counters(session: SessionRecord) -> None:
    session.template_apply_count = 0
    session.templates_applied = []


def clear_world_with_audit(
    session: SessionRecord,
    audit: AuditLog,
    *,
    command_type: str = "entity_cleanup",
    issued_by: str = "bridge",
    command_id: str | None = None,
    extra_detail: dict[str, Any] | None = None,
) -> int:
    if session.world is None:
        return 0
    pre_count = session.world.registry.count()
    snapshot = session.world.snapshot().to_dict() if pre_count else None
    removed = session.world.clear()
    detail: dict[str, Any] = {
        "entities_removed": removed,
        "pre_entity_count": pre_count,
        "state": session.state.value,
    }
    if snapshot:
        detail["world_snapshot"] = snapshot
    if extra_detail:
        detail.update(extra_detail)
    audit.append(
        session.session_id,
        command_id=command_id,
        command_type=command_type,
        issued_by=issued_by,
        result="OK",
        detail=detail,
    )
    return removed


def clear_telemetry_with_audit(
    session: SessionRecord,
    telemetry_subs: TelemetrySubscriptionStore,
    audit: AuditLog,
    *,
    issued_by: str = "bridge",
    command_id: str | None = None,
    extra_detail: dict[str, Any] | None = None,
) -> int:
    removed = telemetry_subs.clear_session(session.session_id)
    if removed == 0:
        return 0
    detail: dict[str, Any] = {
        "subscriptions_removed": removed,
        "state": session.state.value,
    }
    if extra_detail:
        detail.update(extra_detail)
    audit.append(
        session.session_id,
        command_id=command_id,
        command_type="telemetry_cleanup",
        issued_by=issued_by,
        result="OK",
        detail=detail,
    )
    return removed


def _buffer_timeline_counts(tactical: TacticalController) -> dict[str, int]:
    capture = tactical.capture_buffer
    return {
        "mode_switches": len(capture.mode_switches),
        "assignment_timeline": len(capture.assignment_timeline),
        "selected_timeline": len(capture.selected_timeline),
        "tti_timeline": len(capture.tti_timeline),
        "recommendation_timeline": len(capture.recommendation_timeline),
        "pause_resume_transitions": len(capture.pause_resume_transitions),
        "assignment_lock_events": len(capture.assignment_lock_events),
        "target_switch_events": len(capture.target_switch_events),
    }


def clear_tactical_state(
    session: SessionRecord,
    audit: AuditLog,
    *,
    trigger: str,
    command_id: str | None = None,
    issued_by: str = "bridge",
    extra_detail: dict[str, Any] | None = None,
) -> bool:
    tactical = session.tactical
    if not isinstance(tactical, TacticalController):
        session.tactical = None
        return False
    buffer_was_empty = tactical.capture_buffer.is_empty()
    timeline_counts = _buffer_timeline_counts(tactical)
    tactical.capture_buffer.reset()
    session.tactical = None
    detail: dict[str, Any] = {
        "trigger": trigger,
        "had_tactical": True,
        "buffer_was_empty": buffer_was_empty,
        "timeline_counts": timeline_counts,
        "state": session.state.value,
    }
    if extra_detail:
        detail.update(extra_detail)
    audit.append(
        session.session_id,
        command_id=command_id,
        command_type="tactical_cleanup",
        issued_by=issued_by,
        result="OK",
        detail=detail,
    )
    return True


def clear_workflow_state(
    session: SessionRecord,
    audit: AuditLog,
    *,
    issued_by: str,
    command_id: str | None,
    transition: str = "workflow_reset",
) -> None:
    if session.workflow is None:
        return
    wf_id = session.workflow.workflow_id
    session.workflow = None
    audit.append(
        session.session_id,
        command_id=command_id,
        command_type="reset_workflow",
        issued_by=issued_by,
        result="OK",
        detail={
            "workflow_id": wf_id,
            "transition": transition,
            "state": session.state.value,
        },
    )


def terminate_runtime_with_audit(
    session: SessionRecord,
    audit: AuditLog,
    *,
    trigger: str,
    command_id: str | None = None,
    issued_by: str = "bridge",
) -> None:
    was_adapter = runtime_is_adapter(session.runtime)
    adapter_pid = getattr(session.runtime, "pid", None)
    if was_adapter and session.pose_sync is not None:
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="adapter_feedback_lost",
            issued_by=issued_by,
            result="OK",
            detail={"trigger": trigger, "reason": "adapter_teardown"},
        )
    if was_adapter and session.telemetry_mirror is not None:
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="telemetry_feedback_lost",
            issued_by=issued_by,
            result="OK",
            detail={"trigger": trigger, "reason": "adapter_teardown"},
        )
    clear_adapter_mirrors(session)
    session.runtime.terminate()
    if was_adapter:
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="adapter_teardown",
            issued_by=issued_by,
            result="OK",
            detail={"trigger": trigger, "adapter_pid": adapter_pid},
        )
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="orphan_cleanup",
            issued_by=issued_by,
            result="OK",
            detail={"trigger": trigger},
        )


@dataclass
class TeardownDeps:
    audit: AuditLog
    telemetry_subs: TelemetrySubscriptionStore


def teardown_capture_post_session(
    session: SessionRecord,
    deps: TeardownDeps,
    *,
    command_id: str,
    issued_by: str,
) -> None:
    clear_world_with_audit(
        session,
        deps.audit,
        command_type="entity_cleanup",
        issued_by=issued_by,
        command_id=command_id,
        extra_detail={"trigger": "capture_session"},
    )
    clear_telemetry_with_audit(
        session,
        deps.telemetry_subs,
        deps.audit,
        issued_by=issued_by,
        command_id=command_id,
        extra_detail={"trigger": "capture_session"},
    )
    clear_tactical_state(
        session,
        deps.audit,
        trigger="capture_session",
        command_id=command_id,
        issued_by=issued_by,
    )
    terminate_runtime_with_audit(
        session,
        deps.audit,
        trigger="capture_session",
        command_id=command_id,
        issued_by=issued_by,
    )
    session.world = None
    session.workflow = None
    reset_template_counters(session)


def teardown_discarded_session(
    session: SessionRecord,
    deps: TeardownDeps,
    *,
    command_id: str,
    issued_by: str,
) -> None:
    clear_world_with_audit(
        session,
        deps.audit,
        command_type="entity_cleanup",
        issued_by=issued_by,
        command_id=command_id,
        extra_detail={"trigger": "discard_session"},
    )
    clear_telemetry_with_audit(
        session,
        deps.telemetry_subs,
        deps.audit,
        issued_by=issued_by,
        command_id=command_id,
        extra_detail={"trigger": "discard_session"},
    )
    clear_tactical_state(
        session,
        deps.audit,
        trigger="discard_session",
        command_id=command_id,
        issued_by=issued_by,
    )
    terminate_runtime_with_audit(
        session,
        deps.audit,
        trigger="discard_session",
        command_id=command_id,
        issued_by=issued_by,
    )
    session.world = None
    session.workflow = None
    reset_template_counters(session)


def teardown_stopped_auto_cleanup(
    session: SessionRecord,
    deps: TeardownDeps,
) -> None:
    clear_world_with_audit(
        session, deps.audit, extra_detail={"trigger": "auto_cleanup"}
    )
    clear_telemetry_with_audit(
        session,
        deps.telemetry_subs,
        deps.audit,
        extra_detail={"trigger": "auto_cleanup"},
    )
    clear_tactical_state(session, deps.audit, trigger="auto_cleanup")
    terminate_runtime_with_audit(session, deps.audit, trigger="auto_cleanup")
    session.world = None


def teardown_failed_auto_cleanup(
    session: SessionRecord,
    deps: TeardownDeps,
) -> None:
    clear_world_with_audit(
        session, deps.audit, extra_detail={"trigger": "auto_cleanup"}
    )
    clear_telemetry_with_audit(
        session,
        deps.telemetry_subs,
        deps.audit,
        extra_detail={"trigger": "auto_cleanup"},
    )
    clear_tactical_state(session, deps.audit, trigger="auto_cleanup")
    session.runtime.terminate()
    session.world = None
