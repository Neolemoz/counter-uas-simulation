"""Template and workflow command handlers (PLAT-RT-R3a)."""

from __future__ import annotations

from typing import Any, Callable

from rt_sandbox.audit_log import AuditLog
from rt_sandbox.governance import (
    GovernanceConfig,
    validate_template_command_payload,
    validate_workflow_command_payload,
)
from rt_sandbox.lifecycle import can_transition
from rt_sandbox.session_adapter_results import apply_template_resync_result
from rt_sandbox.session_record import SessionRecord
from rt_sandbox.session_response import fail, ok
from rt_sandbox.session_teardown import clear_workflow_state
from rt_sandbox.session_telemetry_coordinator import pose_sync_summary
from rt_sandbox.template_catalog import catalog_size as template_catalog_size
from rt_sandbox.template_catalog import list_templates_metadata
from rt_sandbox.template_resync import run_template_adapter_resync
from rt_sandbox.templates import apply_template
from rt_sandbox.workflow import (
    WorkflowStatus,
    catalog_size as workflow_catalog_size,
    execute_workflow_step,
    new_workflow_state,
)


def list_runtime_templates(
    base: dict[str, Any],
    config: GovernanceConfig,
) -> dict[str, Any]:
    if template_catalog_size() > config.max_runtime_templates_in_catalog:
        return fail(
            base,
            "RESOURCE_LIMIT_EXCEEDED",
            "template catalog exceeds cap",
        )
    resp = ok(base, state=None)
    resp["templates"] = list_templates_metadata()
    resp["catalog_size"] = template_catalog_size()
    return resp


def workflow_state_response(
    session: SessionRecord,
    base: dict[str, Any],
) -> dict[str, Any]:
    resp = ok(base, state=session.state.value)
    if session.workflow is None:
        resp["workflow"] = {
            "schema": "rt_sandbox_workflow_v1",
            "status": WorkflowStatus.IDLE.value,
        }
    else:
        resp["workflow"] = session.workflow.to_dict()
    resp["templates_applied"] = list(session.templates_applied)
    resp["template_apply_count"] = session.template_apply_count
    return resp


def start_workflow_inner(
    session: SessionRecord,
    workflow_id: str,
    base: dict[str, Any],
    *,
    config: GovernanceConfig,
    audit: AuditLog,
    command_id: str,
    issued_by: str,
    transition: str,
) -> dict[str, Any]:
    wf_state = new_workflow_state(workflow_id)
    if wf_state is None:
        return fail(base, "INVALID_STATE", "unknown workflow_id")
    if wf_state.step_count > config.max_workflow_steps:
        return fail(base, "RESOURCE_LIMIT_EXCEEDED", "workflow steps exceed cap")
    session.workflow = wf_state
    audit.append(
        session.session_id,
        command_id=command_id,
        command_type="start_workflow",
        issued_by=issued_by,
        result="OK",
        detail={
            "workflow_id": workflow_id,
            "step_count": wf_state.step_count,
            "current_step": 0,
            "transition": transition,
            "state": session.state.value,
        },
    )
    return workflow_state_response(session, base)


def handle_template(
    session: SessionRecord,
    command_type: str,
    payload: Any,
    base: dict[str, Any],
    *,
    config: GovernanceConfig,
    audit: AuditLog,
    publish_channel: Callable[[str], None],
    publish_transition: Callable[[str, Any, bool], None],
    runtime_crashed: Callable[[], dict[str, Any]],
    command_id: str,
    issued_by: str,
) -> dict[str, Any]:
    if command_type == "list_runtime_templates":
        if session is not None and can_transition(session.state, command_type):
            pass
        return list_runtime_templates(base, config)

    if not can_transition(session.state, command_type):
        return fail(base, "INVALID_STATE", session.state.value)
    if not session.runtime.is_alive():
        return runtime_crashed()
    if session.world is None:
        return fail(base, "INVALID_STATE", "world not initialized")

    payload_err = validate_template_command_payload(command_type, payload)
    if payload_err:
        return fail(base, payload_err, f"invalid payload for {command_type}")

    assert isinstance(payload, dict)
    template_id = str(payload["template_id"])
    if session.template_apply_count >= config.max_template_applies_per_session:
        return fail(base, "RESOURCE_LIMIT_EXCEEDED", "max template applies exceeded")

    result, err = apply_template(
        session.world,
        template_id,
        max_entities_per_apply=config.max_entities_per_template_apply,
    )
    if err:
        return fail(base, err, err)
    assert result is not None
    session.template_apply_count += 1
    session.templates_applied.append(template_id)
    detail = {
        "template_id": template_id,
        "entities_spawned": result.entities_spawned,
        "entity_ids": result.entity_ids,
        "revision": result.revision,
        "state": session.state.value,
        "transition": "template_applied",
    }
    audit.append(
        session.session_id,
        command_id=command_id,
        command_type="apply_runtime_template",
        issued_by=issued_by,
        result="OK",
        detail=detail,
    )
    resync_result = run_template_adapter_resync(
        session,
        config,
        trigger="apply_runtime_template",
        template_id=template_id,
        entities_spawned=result.entities_spawned,
    )
    channels_published = apply_template_resync_result(
        session,
        resync_result,
        audit,
        publish_channel,
        command_id=command_id,
        issued_by=issued_by,
    )
    publish_transition("apply_runtime_template", session.state, channels_published)
    resp = ok(base, state=session.state.value)
    resp["world_summary"] = session.world.world_summary(
        pose_sync_summary=pose_sync_summary(session)
    )
    resp["template_id"] = template_id
    resp["entities_spawned"] = result.entities_spawned
    return resp


def handle_workflow(
    session: SessionRecord,
    command_type: str,
    payload: Any,
    base: dict[str, Any],
    *,
    config: GovernanceConfig,
    audit: AuditLog,
    publish_channel: Callable[[str], None],
    publish_transition: Callable[[str, Any, bool], None],
    runtime_crashed: Callable[[], dict[str, Any]],
    command_id: str,
    issued_by: str,
) -> dict[str, Any]:
    if command_type == "get_workflow_state":
        return workflow_state_response(session, base)

    if not can_transition(session.state, command_type):
        return fail(base, "INVALID_STATE", session.state.value)
    if not session.runtime.is_alive():
        return runtime_crashed()
    if session.world is None:
        return fail(base, "INVALID_STATE", "world not initialized")

    payload_err = validate_workflow_command_payload(command_type, payload)
    if payload_err:
        return fail(base, payload_err, f"invalid payload for {command_type}")

    if workflow_catalog_size() > config.max_workflows_in_catalog:
        return fail(base, "RESOURCE_LIMIT_EXCEEDED", "workflow catalog exceeds cap")

    if command_type == "reset_workflow":
        clear_workflow_state(
            session, audit, issued_by=issued_by, command_id=command_id
        )
        return workflow_state_response(session, base)

    if command_type == "reload_workflow":
        if not isinstance(payload, dict):
            return fail(base, "INVALID_STATE", "workflow_id required")
        workflow_id = str(payload.get("workflow_id", ""))
        clear_workflow_state(
            session,
            audit,
            issued_by=issued_by,
            command_id=command_id,
            transition="workflow_reload",
        )
        return start_workflow_inner(
            session,
            workflow_id,
            base,
            config=config,
            audit=audit,
            command_id=command_id,
            issued_by=issued_by,
            transition="workflow_reloaded",
        )

    if command_type == "start_workflow":
        if (
            session.workflow is not None
            and session.workflow.status == WorkflowStatus.IN_PROGRESS
        ):
            return fail(base, "INVALID_STATE", "workflow already in progress")
        if not isinstance(payload, dict):
            return fail(base, "INVALID_STATE", "workflow_id required")
        workflow_id = str(payload.get("workflow_id", ""))
        return start_workflow_inner(
            session,
            workflow_id,
            base,
            config=config,
            audit=audit,
            command_id=command_id,
            issued_by=issued_by,
            transition="workflow_started",
        )

    if command_type == "advance_workflow":
        if (
            session.workflow is None
            or session.workflow.status != WorkflowStatus.IN_PROGRESS
        ):
            return fail(base, "INVALID_STATE", "no workflow in progress")
        if session.workflow.step_count > config.max_workflow_steps:
            return fail(base, "RESOURCE_LIMIT_EXCEEDED", "workflow steps exceed cap")
        step_result = execute_workflow_step(
            session.workflow,
            session.world,
            max_entities_per_apply=config.max_entities_per_template_apply,
        )
        detail: dict[str, Any] = {
            "workflow_id": session.workflow.workflow_id,
            "step_index": step_result.step_index,
            "transition": step_result.transition,
            "state": session.state.value,
            "workflow_status": session.workflow.status.value,
        }
        if step_result.staged_setup:
            detail["staged_setup"] = True
        if step_result.template_apply:
            ta = step_result.template_apply
            session.template_apply_count += 1
            session.templates_applied.append(ta.template_id)
            detail["template_id"] = ta.template_id
            detail["entities_spawned"] = ta.entities_spawned
        if step_result.world_reset:
            detail["world_reset"] = True

        if not step_result.ok:
            audit.append(
                session.session_id,
                command_id=command_id,
                command_type="advance_workflow",
                issued_by=issued_by,
                result="WORKFLOW_STEP_FAILED",
                detail=detail,
            )
            return fail(
                base,
                step_result.error_code or "WORKFLOW_STEP_FAILED",
                "workflow step failed",
            )

        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="advance_workflow",
            issued_by=issued_by,
            result="OK",
            detail=detail,
        )
        channels_published = False
        if step_result.transition in {"apply_template", "reset_world"}:
            resync_result = run_template_adapter_resync(
                session,
                config,
                trigger="advance_workflow",
                template_id=detail.get("template_id"),
                entities_spawned=int(detail.get("entities_spawned") or 0),
                world_reset=bool(step_result.world_reset),
            )
            channels_published = apply_template_resync_result(
                session,
                resync_result,
                audit,
                publish_channel,
                command_id=command_id,
                issued_by=issued_by,
            )
            publish_transition(
                "advance_workflow", session.state, channels_published
            )
        resp = workflow_state_response(session, base)
        resp["step_completed"] = step_result.step_index
        resp["workflow_completed"] = step_result.completed
        if session.world:
            resp["world_summary"] = session.world.world_summary(
                pose_sync_summary=pose_sync_summary(session)
            )
        return resp

    return fail(base, "COMMAND_FORBIDDEN", command_type)
