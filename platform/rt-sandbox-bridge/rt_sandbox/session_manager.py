"""Transient session manager for RT sandbox bridge prototype."""

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from rt_sandbox.audit_log import AuditLog
from rt_sandbox.capture import CaptureBundleError, build_capture_bundle
from rt_sandbox.export_audit_log import ExportAuditLog
from rt_sandbox.export_boundary import CAPTURE_GOVERNANCE_BANNER
from rt_sandbox.governance import (
    ENTITY_COMMANDS,
    GOVERNANCE_BANNER,
    GovernanceConfig,
    RateLimiter,
    TEMPLATE_COMMANDS,
    TELEMETRY_COMMANDS,
    WORKFLOW_COMMANDS,
    classify_command,
    validate_capture_payload,
    validate_entity_payload,
    validate_template_command_payload,
    validate_telemetry_payload,
    validate_workflow_command_payload,
)
from rt_sandbox.template_catalog import catalog_size as template_catalog_size
from rt_sandbox.template_catalog import list_templates_metadata
from rt_sandbox.templates import apply_template
from rt_sandbox.workflow import (
    WorkflowState,
    WorkflowStatus,
    catalog_size as workflow_catalog_size,
    execute_workflow_step,
    list_workflows_metadata,
    new_workflow_state,
)
from rt_sandbox.lifecycle import SessionState, can_transition, is_active_state
from rt_sandbox.runtime_stub import RuntimeStub
from rt_sandbox.telemetry import TelemetryBuffer
from rt_sandbox.telemetry_subscriptions import (
    TelemetrySubscriptionStore,
    build_channel_payload,
)
from rt_sandbox.isolation import repo_root_from
from rt_sandbox.world_state import WorldStateStore


@dataclass
class SessionRecord:
    session_id: str
    state: SessionState
    created_monotonic: float
    bridge_ready_deadline: float
    cleanup_after: float | None = None
    stub: RuntimeStub = field(default_factory=RuntimeStub)
    world: WorldStateStore | None = None
    issued_by: str = "rt_ui_prototype"
    workflow: WorkflowState | None = None
    template_apply_count: int = 0
    templates_applied: list[str] = field(default_factory=list)


class BridgeSessionManager:
    def __init__(
        self,
        config: GovernanceConfig | None = None,
        repo_root: Path | None = None,
    ) -> None:
        self.config = config or GovernanceConfig()
        self._repo_root = repo_root
        self._session: SessionRecord | None = None
        self._rate_limiter = RateLimiter(
            burst=self.config.command_rate_burst,
            sustained_per_s=self.config.command_rate_sustained,
        )
        self._audit = AuditLog(repo_root)
        self._export_audit = ExportAuditLog(repo_root)
        hz = self.config.telemetry_update_rate_cap_hz
        self._telemetry = TelemetryBuffer(max_hz=hz)
        self._telemetry_subs = TelemetrySubscriptionStore(
            max_hz=hz,
            ring_size=self.config.telemetry_ring_buffer_size,
        )

    def handle_command(self, body: dict[str, Any]) -> dict[str, Any]:
        now = time.monotonic()
        if self._session is not None and self._session.state in {
            SessionState.RUNNING,
            SessionState.PAUSED,
        }:
            if not self._session.stub.is_alive():
                base = self._response_base(
                    str(body.get("command_id", "")),
                    self._session.session_id,
                )
                return self._runtime_crashed(
                    self._session,
                    base,
                    str(body.get("command_id", "")),
                    str(body.get("issued_by", "rt_ui_prototype")),
                )
        self._tick_timeouts(now)

        command_type = str(body.get("command_type", ""))
        command_id = str(body.get("command_id", ""))
        issued_by = str(body.get("issued_by", "rt_ui_prototype"))
        authority_scope = str(body.get("authority_scope", ""))
        session_id = body.get("session_id")
        payload = body.get("payload")

        base = self._response_base(command_id, session_id)

        if authority_scope and authority_scope != self.config.authority_scope:
            return self._fail(base, "COMMAND_FORBIDDEN", "invalid authority_scope")

        forbidden = classify_command(command_type)
        if forbidden:
            return self._fail(base, forbidden, f"command not allowed: {command_type}")

        cap_err = validate_capture_payload(command_type, payload)
        if cap_err:
            return self._fail(base, cap_err, "invalid capture payload")

        if not self._rate_limiter.check(now):
            return self._fail(base, "RESOURCE_LIMIT_EXCEEDED", "command rate limit")

        if command_type == "start_session":
            return self._start_session(base, command_id, issued_by, now)

        if command_type == "list_runtime_templates":
            return self._list_runtime_templates(base)

        sid = session_id or (self._session.session_id if self._session else None)
        if not sid or self._session is None or self._session.session_id != sid:
            return self._fail(base, "SESSION_NOT_FOUND", "unknown session_id")

        session = self._session
        base["session_id"] = session.session_id

        if command_type == "pause_session":
            return self._pause(session, base, command_id, issued_by)
        if command_type == "resume":
            return self._resume(session, base, command_id, issued_by)
        if command_type == "stop_session":
            return self._stop(session, base, command_id, issued_by, now)
        if command_type == "discard_session":
            return self._discard(session, base, command_id, issued_by, now)
        if command_type == "capture_session":
            return self._capture_session(
                session, base, command_id, issued_by, payload, now
            )
        if command_type == "reset_session":
            return self._reset_session(session, base, command_id, issued_by)
        if command_type in ENTITY_COMMANDS:
            return self._handle_entity(
                session,
                command_type,
                payload,
                base,
                command_id,
                issued_by,
            )
        if command_type in TELEMETRY_COMMANDS:
            return self._handle_telemetry(
                session,
                command_type,
                payload,
                base,
                command_id,
                issued_by,
            )
        if command_type in TEMPLATE_COMMANDS:
            return self._handle_template(
                session,
                command_type,
                payload,
                base,
                command_id,
                issued_by,
            )
        if command_type in WORKFLOW_COMMANDS:
            return self._handle_workflow(
                session,
                command_type,
                payload,
                base,
                command_id,
                issued_by,
            )

        return self._fail(base, "COMMAND_FORBIDDEN", command_type)

    def pull_telemetry(
        self,
        session_id: str,
        subscription_id: str,
        max_events: int = 10,
    ) -> dict[str, Any]:
        sub = self._telemetry_subs.get(subscription_id)
        if sub is None or sub.session_id != session_id:
            return {
                "governance_banner": GOVERNANCE_BANNER,
                "ok": False,
                "error_code": "SESSION_NOT_FOUND",
                "message": "unknown subscription_id",
                "events": [],
                "drained_count": 0,
            }
        events = self._telemetry_subs.drain(subscription_id, max_events=max_events)
        return {
            "governance_banner": GOVERNANCE_BANNER,
            "ok": True,
            "error_code": "OK",
            "session_id": session_id,
            "subscription_id": subscription_id,
            "events": events,
            "drained_count": len(events),
        }

    def _publish_telemetry(
        self,
        session: SessionRecord,
        channel: str,
        *,
        command_type: str | None = None,
        previous_state: str | None = None,
    ) -> None:
        payload = build_channel_payload(session, channel)
        if payload is None:
            return
        if channel == "lifecycle_state":
            if command_type is not None:
                payload["command_type"] = command_type
            if previous_state is not None:
                payload["previous_state"] = previous_state
        self._telemetry_subs.record(session.session_id, channel, payload)

    def _publish_channels_for_transition(
        self,
        session: SessionRecord,
        command_type: str,
        previous_state: SessionState,
    ) -> None:
        prev = previous_state.value
        self._publish_telemetry(
            session,
            "lifecycle_state",
            command_type=command_type,
            previous_state=prev,
        )
        self._publish_telemetry(session, "session_health")
        self._publish_telemetry(session, "clock_mirror")
        if command_type in {
            "spawn_entity",
            "move_entity",
            "delete_entity",
            "reset_session",
            "apply_runtime_template",
            "advance_workflow",
        }:
            self._publish_telemetry(session, "world_summary")
            self._publish_telemetry(session, "entity_pose_mirror")

    def _clear_telemetry_with_audit(
        self,
        session: SessionRecord,
        *,
        issued_by: str = "bridge",
        command_id: str | None = None,
        extra_detail: dict[str, Any] | None = None,
    ) -> int:
        removed = self._telemetry_subs.clear_session(session.session_id)
        if removed == 0:
            return 0
        detail: dict[str, Any] = {
            "subscriptions_removed": removed,
            "state": session.state.value,
        }
        if extra_detail:
            detail.update(extra_detail)
        self._audit.append(
            session.session_id,
            command_id=command_id,
            command_type="telemetry_cleanup",
            issued_by=issued_by,
            result="OK",
            detail=detail,
        )
        return removed

    def _handle_telemetry(
        self,
        session: SessionRecord,
        command_type: str,
        payload: Any,
        base: dict[str, Any],
        command_id: str,
        issued_by: str,
    ) -> dict[str, Any]:
        if not can_transition(session.state, command_type):
            return self._fail(base, "INVALID_STATE", session.state.value)
        payload_err = validate_telemetry_payload(command_type, payload)
        if payload_err:
            return self._fail(base, payload_err, f"invalid payload for {command_type}")
        assert isinstance(payload, dict)

        if command_type == "subscribe_telemetry":
            channels = [str(c) for c in payload["channels"]]
            sub_id, err = self._telemetry_subs.subscribe(session.session_id, channels)
            if err:
                return self._fail(base, err, err)
            initial = self._telemetry_subs.build_initial_events(session)
            for ev in initial:
                self._telemetry_subs.record(
                    session.session_id,
                    ev["channel"],
                    ev["payload"],
                )
            self._audit.append(
                session.session_id,
                command_id=command_id,
                command_type="subscribe_telemetry",
                issued_by=issued_by,
                result="OK",
                detail={
                    "subscription_id": sub_id,
                    "channels": channels,
                    "state": session.state.value,
                },
            )
            self._audit.append(
                session.session_id,
                command_id=command_id,
                command_type="telemetry_snapshot",
                issued_by=issued_by,
                result="OK",
                detail={"subscription_id": sub_id, "channel_count": len(channels)},
            )
            resp = self._ok(base, state=session.state.value)
            resp["subscription_id"] = sub_id
            resp["channels"] = channels
            resp["initial_events"] = initial
            return resp

        sub_id = str(payload["subscription_id"])
        if not self._telemetry_subs.unsubscribe(sub_id):
            return self._fail(base, "SESSION_NOT_FOUND", "unknown subscription_id")
        self._audit.append(
            session.session_id,
            command_id=command_id,
            command_type="unsubscribe_telemetry",
            issued_by=issued_by,
            result="OK",
            detail={"subscription_id": sub_id, "state": session.state.value},
        )
        return self._ok(base, state=session.state.value)

    def _handle_entity(
        self,
        session: SessionRecord,
        command_type: str,
        payload: Any,
        base: dict[str, Any],
        command_id: str,
        issued_by: str,
    ) -> dict[str, Any]:
        if not can_transition(session.state, command_type):
            return self._fail(base, "INVALID_STATE", session.state.value)
        if not session.stub.is_alive():
            return self._runtime_crashed(session, base, command_id, issued_by)
        if session.world is None:
            return self._fail(base, "INVALID_STATE", "world not initialized")

        payload_err = validate_entity_payload(command_type, payload)
        if payload_err:
            return self._fail(base, payload_err, f"invalid payload for {command_type}")

        assert isinstance(payload, dict)
        world = session.world
        registry = world.registry

        if command_type == "spawn_entity":
            entity_type = str(payload["entity_type"])
            pose = {k: float(payload["pose"][k]) for k in ("x", "y", "z")}
            if "yaw_deg" in payload.get("pose", {}):
                pose["yaw_deg"] = float(payload["pose"]["yaw_deg"])
            eid_opt = payload.get("entity_id")
            entity_id = str(eid_opt) if eid_opt else None
            record, err = registry.spawn(entity_type, pose, entity_id=entity_id)
            if err:
                return self._fail(base, err, err)
            world.bump_revision()
            detail = {
                "entity_id": record.entity_id,
                "entity_type": record.entity_type,
                "pose": dict(record.pose),
                "state": session.state.value,
            }
            self._audit.append(
                session.session_id,
                command_id=command_id,
                command_type="spawn_entity",
                issued_by=issued_by,
                result="OK",
                detail=detail,
            )
            self._publish_channels_for_transition(
                session, "spawn_entity", session.state
            )
            return self._entity_ok(session, base, entity_id=record.entity_id, detail=detail)

        if command_type == "move_entity":
            entity_id = str(payload["entity_id"])
            pose = {k: float(payload["pose"][k]) for k in ("x", "y", "z")}
            if "yaw_deg" in payload.get("pose", {}):
                pose["yaw_deg"] = float(payload["pose"]["yaw_deg"])
            record, err = registry.move(entity_id, pose)
            if err:
                return self._fail(base, err, err)
            world.bump_revision()
            detail = {
                "entity_id": record.entity_id,
                "entity_type": record.entity_type,
                "pose": dict(record.pose),
                "state": session.state.value,
            }
            self._audit.append(
                session.session_id,
                command_id=command_id,
                command_type="move_entity",
                issued_by=issued_by,
                result="OK",
                detail=detail,
            )
            self._publish_channels_for_transition(
                session, "move_entity", session.state
            )
            return self._entity_ok(session, base, detail=detail)

        if command_type == "delete_entity":
            entity_id = str(payload["entity_id"])
            record, err = registry.delete(entity_id)
            if err:
                return self._fail(base, err, err)
            world.bump_revision()
            detail = {
                "entity_id": record.entity_id,
                "entity_type": record.entity_type,
                "state": session.state.value,
            }
            self._audit.append(
                session.session_id,
                command_id=command_id,
                command_type="delete_entity",
                issued_by=issued_by,
                result="OK",
                detail=detail,
            )
            self._publish_channels_for_transition(
                session, "delete_entity", session.state
            )
            return self._entity_ok(session, base, detail=detail)

        return self._fail(base, "COMMAND_FORBIDDEN", command_type)

    def _entity_ok(
        self,
        session: SessionRecord,
        base: dict[str, Any],
        *,
        entity_id: str | None = None,
        detail: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        world = session.world
        summary = world.world_summary() if world else {}
        resp = self._ok(base, state=session.state.value)
        resp["world_summary"] = summary
        if entity_id:
            resp["entity_id"] = entity_id
        if detail:
            resp["entities"] = world.registry.poses_for_telemetry() if world else []
        hb = self._telemetry.emit_entity_pose_heartbeat(
            session.session_id,
            world.registry.poses_for_telemetry() if world else [],
            world_summary=summary,
        )
        if hb:
            resp["telemetry"] = hb
        return resp

    def _clear_world_with_audit(
        self,
        session: SessionRecord,
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
        self._audit.append(
            session.session_id,
            command_id=command_id,
            command_type=command_type,
            issued_by=issued_by,
            result="OK",
            detail=detail,
        )
        return removed

    def _clear_workflow_state(
        self,
        session: SessionRecord,
        *,
        issued_by: str,
        command_id: str | None,
        transition: str = "workflow_reset",
    ) -> None:
        if session.workflow is None:
            return
        wf_id = session.workflow.workflow_id
        session.workflow = None
        self._audit.append(
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

    def _list_runtime_templates(self, base: dict[str, Any]) -> dict[str, Any]:
        if template_catalog_size() > self.config.max_runtime_templates_in_catalog:
            return self._fail(
                base,
                "RESOURCE_LIMIT_EXCEEDED",
                "template catalog exceeds cap",
            )
        resp = self._ok(base, state=None)
        resp["templates"] = list_templates_metadata()
        resp["catalog_size"] = template_catalog_size()
        return resp

    def _handle_template(
        self,
        session: SessionRecord,
        command_type: str,
        payload: Any,
        base: dict[str, Any],
        command_id: str,
        issued_by: str,
    ) -> dict[str, Any]:
        if command_type == "list_runtime_templates":
            if session is not None and can_transition(session.state, command_type):
                pass
            return self._list_runtime_templates(base)

        if not can_transition(session.state, command_type):
            return self._fail(base, "INVALID_STATE", session.state.value)
        if not session.stub.is_alive():
            return self._runtime_crashed(session, base, command_id, issued_by)
        if session.world is None:
            return self._fail(base, "INVALID_STATE", "world not initialized")

        payload_err = validate_template_command_payload(command_type, payload)
        if payload_err:
            return self._fail(base, payload_err, f"invalid payload for {command_type}")

        assert isinstance(payload, dict)
        template_id = str(payload["template_id"])
        if session.template_apply_count >= self.config.max_template_applies_per_session:
            return self._fail(base, "RESOURCE_LIMIT_EXCEEDED", "max template applies exceeded")

        result, err = apply_template(
            session.world,
            template_id,
            max_entities_per_apply=self.config.max_entities_per_template_apply,
        )
        if err:
            return self._fail(base, err, err)
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
        self._audit.append(
            session.session_id,
            command_id=command_id,
            command_type="apply_runtime_template",
            issued_by=issued_by,
            result="OK",
            detail=detail,
        )
        self._publish_channels_for_transition(
            session, "apply_runtime_template", session.state
        )
        resp = self._ok(base, state=session.state.value)
        resp["world_summary"] = session.world.world_summary()
        resp["template_id"] = template_id
        resp["entities_spawned"] = result.entities_spawned
        return resp

    def _handle_workflow(
        self,
        session: SessionRecord,
        command_type: str,
        payload: Any,
        base: dict[str, Any],
        command_id: str,
        issued_by: str,
    ) -> dict[str, Any]:
        if command_type == "get_workflow_state":
            return self._workflow_state_response(session, base)

        if not can_transition(session.state, command_type):
            return self._fail(base, "INVALID_STATE", session.state.value)
        if not session.stub.is_alive():
            return self._runtime_crashed(session, base, command_id, issued_by)
        if session.world is None:
            return self._fail(base, "INVALID_STATE", "world not initialized")

        payload_err = validate_workflow_command_payload(command_type, payload)
        if payload_err:
            return self._fail(base, payload_err, f"invalid payload for {command_type}")

        if workflow_catalog_size() > self.config.max_workflows_in_catalog:
            return self._fail(base, "RESOURCE_LIMIT_EXCEEDED", "workflow catalog exceeds cap")

        if command_type == "reset_workflow":
            self._clear_workflow_state(session, issued_by=issued_by, command_id=command_id)
            return self._workflow_state_response(session, base)

        if command_type == "reload_workflow":
            if not isinstance(payload, dict):
                return self._fail(base, "INVALID_STATE", "workflow_id required")
            workflow_id = str(payload.get("workflow_id", ""))
            self._clear_workflow_state(
                session,
                issued_by=issued_by,
                command_id=command_id,
                transition="workflow_reload",
            )
            return self._start_workflow_inner(
                session,
                workflow_id,
                base,
                command_id,
                issued_by,
                transition="workflow_reloaded",
            )

        if command_type == "start_workflow":
            if session.workflow is not None and session.workflow.status == WorkflowStatus.IN_PROGRESS:
                return self._fail(base, "INVALID_STATE", "workflow already in progress")
            if not isinstance(payload, dict):
                return self._fail(base, "INVALID_STATE", "workflow_id required")
            workflow_id = str(payload.get("workflow_id", ""))
            return self._start_workflow_inner(
                session,
                workflow_id,
                base,
                command_id,
                issued_by,
                transition="workflow_started",
            )

        if command_type == "advance_workflow":
            if session.workflow is None or session.workflow.status != WorkflowStatus.IN_PROGRESS:
                return self._fail(base, "INVALID_STATE", "no workflow in progress")
            if session.workflow.step_count > self.config.max_workflow_steps:
                return self._fail(base, "RESOURCE_LIMIT_EXCEEDED", "workflow steps exceed cap")
            step_result = execute_workflow_step(
                session.workflow,
                session.world,
                max_entities_per_apply=self.config.max_entities_per_template_apply,
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
                self._audit.append(
                    session.session_id,
                    command_id=command_id,
                    command_type="advance_workflow",
                    issued_by=issued_by,
                    result="WORKFLOW_STEP_FAILED",
                    detail=detail,
                )
                return self._fail(
                    base,
                    step_result.error_code or "WORKFLOW_STEP_FAILED",
                    "workflow step failed",
                )

            self._audit.append(
                session.session_id,
                command_id=command_id,
                command_type="advance_workflow",
                issued_by=issued_by,
                result="OK",
                detail=detail,
            )
            if step_result.transition in {"apply_template", "reset_world"}:
                self._publish_channels_for_transition(
                    session, "advance_workflow", session.state
                )
            resp = self._workflow_state_response(session, base)
            resp["step_completed"] = step_result.step_index
            resp["workflow_completed"] = step_result.completed
            if session.world:
                resp["world_summary"] = session.world.world_summary()
            return resp

        return self._fail(base, "COMMAND_FORBIDDEN", command_type)

    def _start_workflow_inner(
        self,
        session: SessionRecord,
        workflow_id: str,
        base: dict[str, Any],
        command_id: str,
        issued_by: str,
        *,
        transition: str,
    ) -> dict[str, Any]:
        wf_state = new_workflow_state(workflow_id)
        if wf_state is None:
            return self._fail(base, "INVALID_STATE", "unknown workflow_id")
        if wf_state.step_count > self.config.max_workflow_steps:
            return self._fail(base, "RESOURCE_LIMIT_EXCEEDED", "workflow steps exceed cap")
        session.workflow = wf_state
        self._audit.append(
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
        return self._workflow_state_response(session, base)

    def _workflow_state_response(
        self,
        session: SessionRecord,
        base: dict[str, Any],
    ) -> dict[str, Any]:
        resp = self._ok(base, state=session.state.value)
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

    def _start_session(
        self,
        base: dict[str, Any],
        command_id: str,
        issued_by: str,
        now: float,
    ) -> dict[str, Any]:
        if self._session is not None and self._session.state not in {
            SessionState.DISCARDED,
            SessionState.CAPTURED,
        }:
            if is_active_state(self._session.state) or self._session.state in {
                SessionState.STOPPED,
                SessionState.CLEANUP_PENDING,
                SessionState.FAILED,
            }:
                return self._fail(base, "INVALID_STATE", "session already active")

        session_id = str(uuid.uuid4())
        record = SessionRecord(
            session_id=session_id,
            state=SessionState.CREATED,
            created_monotonic=now,
            bridge_ready_deadline=now + self.config.bridge_ready_timeout_s,
            issued_by=issued_by,
            world=WorldStateStore(session_id=session_id),
        )
        record.world.registry.max_entity_count = self.config.max_entity_count
        self._session = record
        base["session_id"] = session_id

        try:
            pid = record.stub.start()
            record.state = SessionState.RUNNING
            self._audit.append(
                session_id,
                command_id=command_id,
                command_type="start_session",
                issued_by=issued_by,
                result="OK",
                detail={"stub_pid": pid, "state": record.state.value},
            )
            summary = record.world.world_summary() if record.world else {}
            hb = self._telemetry.maybe_emit(
                session_id,
                record.state.value,
                record.stub.is_alive(),
                world_summary=summary,
            )
            self._publish_channels_for_transition(
                record, "start_session", SessionState.CREATED
            )
            resp = self._ok(base, state=record.state.value)
            resp["world_summary"] = summary
            if hb:
                resp["heartbeat"] = hb
            return resp
        except OSError as exc:
            record.state = SessionState.FAILED
            self._audit.append(
                session_id,
                command_id=command_id,
                command_type="start_session",
                issued_by=issued_by,
                result="RUNTIME_UNAVAILABLE",
                detail={"error": str(exc)},
            )
            return self._fail(base, "RUNTIME_UNAVAILABLE", str(exc))

    def _pause(
        self,
        session: SessionRecord,
        base: dict[str, Any],
        command_id: str,
        issued_by: str,
    ) -> dict[str, Any]:
        if not can_transition(session.state, "pause_session"):
            return self._fail(base, "INVALID_STATE", session.state.value)
        if not session.stub.is_alive():
            return self._runtime_crashed(session, base, command_id, issued_by)
        prev = session.state
        session.stub.pause()
        session.state = SessionState.PAUSED
        self._audit.append(
            session.session_id,
            command_id=command_id,
            command_type="pause_session",
            issued_by=issued_by,
            result="OK",
            detail={"state": session.state.value},
        )
        self._publish_channels_for_transition(session, "pause_session", prev)
        return self._ok(base, state=session.state.value)

    def _resume(
        self,
        session: SessionRecord,
        base: dict[str, Any],
        command_id: str,
        issued_by: str,
    ) -> dict[str, Any]:
        if not can_transition(session.state, "resume"):
            return self._fail(base, "INVALID_STATE", session.state.value)
        if not session.stub.is_alive():
            return self._runtime_crashed(session, base, command_id, issued_by)
        prev = session.state
        session.stub.resume()
        session.state = SessionState.RUNNING
        self._audit.append(
            session.session_id,
            command_id=command_id,
            command_type="resume",
            issued_by=issued_by,
            result="OK",
            detail={"state": session.state.value},
        )
        self._publish_channels_for_transition(session, "resume", prev)
        return self._ok(base, state=session.state.value)

    def _reset_session(
        self,
        session: SessionRecord,
        base: dict[str, Any],
        command_id: str,
        issued_by: str,
    ) -> dict[str, Any]:
        if not can_transition(session.state, "reset_session"):
            return self._fail(base, "INVALID_STATE", session.state.value)
        if session.world is None:
            return self._fail(base, "INVALID_STATE", "world not initialized")
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
            self._clear_workflow_state(session, issued_by=issued_by, command_id=command_id)
            detail["workflow_reset"] = True
        self._audit.append(
            session.session_id,
            command_id=command_id,
            command_type="reset_session",
            issued_by=issued_by,
            result="OK",
            detail=detail,
        )
        self._publish_channels_for_transition(session, "reset_session", session.state)
        resp = self._ok(base, state=session.state.value)
        resp["world_summary"] = session.world.world_summary()
        self._telemetry.emit_world_summary(session.session_id, resp["world_summary"])
        return resp

    def _stop(
        self,
        session: SessionRecord,
        base: dict[str, Any],
        command_id: str,
        issued_by: str,
        now: float,
    ) -> dict[str, Any]:
        if not can_transition(session.state, "stop_session"):
            return self._fail(base, "INVALID_STATE", session.state.value)
        prev = session.state
        session.stub.stop()
        session.state = SessionState.STOPPED
        session.cleanup_after = now + self.config.session_cleanup_timeout_s
        self._audit.append(
            session.session_id,
            command_id=command_id,
            command_type="stop_session",
            issued_by=issued_by,
            result="OK",
            detail={"state": session.state.value, "cleanup_after_s": self.config.session_cleanup_timeout_s},
        )
        self._publish_channels_for_transition(session, "stop_session", prev)
        return self._ok(base, state=session.state.value)

    def _capture_session(
        self,
        session: SessionRecord,
        base: dict[str, Any],
        command_id: str,
        issued_by: str,
        payload: Any,
        now: float,
    ) -> dict[str, Any]:
        if not can_transition(session.state, "capture_session"):
            self._export_audit.append(
                "capture_rejected",
                session_id=session.session_id,
                result="INVALID_STATE",
                detail={"state": session.state.value},
            )
            return self._fail(base, "INVALID_STATE", session.state.value)

        self._export_audit.append(
            "capture_requested",
            session_id=session.session_id,
            result="pending",
            detail={"issued_by": issued_by},
        )

        repo_root = self._repo_root or repo_root_from()
        world_snapshot = (
            session.world.snapshot().to_dict() if session.world else None
        )
        audit_path = self._audit.path_for(session.session_id)

        workflow_summary = (
            session.workflow.to_dict() if session.workflow else None
        )
        templates_applied = list(session.templates_applied)

        try:
            bundle = build_capture_bundle(
                repo_root=repo_root,
                session_id=session.session_id,
                world_snapshot=world_snapshot,
                audit_path=audit_path,
                telemetry_store=self._telemetry_subs,
                payload=payload if isinstance(payload, dict) else None,
                max_bundle_bytes=self.config.max_capture_bundle_bytes,
                max_staged=self.config.max_staged_captures,
                workflow_summary=workflow_summary,
                templates_applied=templates_applied,
            )
        except CaptureBundleError as exc:
            self._export_audit.append(
                "capture_rejected",
                session_id=session.session_id,
                result=exc.code,
                detail={"message": exc.message},
            )
            self._audit.append(
                session.session_id,
                command_id=command_id,
                command_type="capture_session",
                issued_by=issued_by,
                result=exc.code,
                detail={"message": exc.message},
            )
            return self._fail(base, exc.code, exc.message)

        session.cleanup_after = None
        session.state = SessionState.CAPTURED
        self._clear_world_with_audit(
            session,
            command_type="entity_cleanup",
            issued_by=issued_by,
            command_id=command_id,
            extra_detail={"trigger": "capture_session"},
        )
        self._clear_telemetry_with_audit(
            session,
            issued_by=issued_by,
            command_id=command_id,
            extra_detail={"trigger": "capture_session"},
        )
        session.stub.terminate()
        session.world = None
        session.workflow = None
        session.template_apply_count = 0
        session.templates_applied = []

        detail = {
            "state": session.state.value,
            "capture_candidate_id": bundle.capture_candidate_id,
            "staging_refs": bundle.staging_refs,
        }
        if workflow_summary:
            detail["workflow_summary"] = workflow_summary
        if templates_applied:
            detail["templates_applied"] = templates_applied
        self._audit.append(
            session.session_id,
            command_id=command_id,
            command_type="capture_session",
            issued_by=issued_by,
            result="OK",
            detail=detail,
        )
        self._export_audit.append(
            "capture_validated",
            capture_candidate_id=bundle.capture_candidate_id,
            session_id=session.session_id,
            result="OK",
            detail={"staging_refs": bundle.staging_refs},
        )

        resp = self._ok(base, state=session.state.value)
        resp["governance_banner"] = CAPTURE_GOVERNANCE_BANNER
        resp["capture_candidate_id"] = bundle.capture_candidate_id
        resp["staging_refs"] = bundle.staging_refs
        return resp

    def _discard(
        self,
        session: SessionRecord,
        base: dict[str, Any],
        command_id: str,
        issued_by: str,
        now: float,
    ) -> dict[str, Any]:
        if not can_transition(session.state, "discard_session"):
            return self._fail(base, "INVALID_STATE", session.state.value)
        if session.state == SessionState.CAPTURED:
            self._export_audit.append(
                "capture_discarded",
                session_id=session.session_id,
                result="OK",
                detail={"trigger": "discard_session"},
            )
        session.state = SessionState.CLEANUP_PENDING
        self._clear_world_with_audit(
            session,
            command_type="entity_cleanup",
            issued_by=issued_by,
            command_id=command_id,
            extra_detail={"trigger": "discard_session"},
        )
        self._clear_telemetry_with_audit(
            session,
            issued_by=issued_by,
            command_id=command_id,
            extra_detail={"trigger": "discard_session"},
        )
        session.stub.terminate()
        session.state = SessionState.DISCARDED
        session.cleanup_after = None
        session.world = None
        session.workflow = None
        session.template_apply_count = 0
        session.templates_applied = []
        self._audit.append(
            session.session_id,
            command_id=command_id,
            command_type="discard_session",
            issued_by=issued_by,
            result="OK",
            detail={"state": session.state.value, "cleanup": "complete"},
        )
        return self._ok(base, state=session.state.value)

    def _runtime_crashed(
        self,
        session: SessionRecord,
        base: dict[str, Any],
        command_id: str,
        issued_by: str,
    ) -> dict[str, Any]:
        session.state = SessionState.RUNTIME_CRASHED
        session.cleanup_after = time.monotonic() + self.config.cleanup_pending_max_age_s
        self._audit.append(
            session.session_id,
            command_id=command_id,
            command_type="runtime_crashed",
            issued_by=issued_by,
            result="RUNTIME_UNAVAILABLE",
            detail={"state": session.state.value},
        )
        return self._fail(base, "RUNTIME_UNAVAILABLE", "runtime stub exited")

    def _tick_timeouts(self, now: float) -> None:
        session = self._session
        if session is None:
            return

        if session.state == SessionState.CREATED and now > session.bridge_ready_deadline:
            session.state = SessionState.FAILED
            session.stub.terminate()
            self._clear_world_with_audit(session, extra_detail={"trigger": "bridge_ready_timeout"})
            self._clear_telemetry_with_audit(
                session, extra_detail={"trigger": "bridge_ready_timeout"}
            )
            session.world = None
            session.cleanup_after = now + self.config.session_cleanup_timeout_s
            self._audit.append(
                session.session_id,
                command_id=None,
                command_type="bridge_ready_timeout",
                issued_by="bridge",
                result="failed",
                detail={"state": session.state.value},
            )

        if session.state == SessionState.RUNNING and now - session.created_monotonic > self.config.max_session_duration_s:
            session.stub.stop()
            session.state = SessionState.STOPPED
            session.cleanup_after = now + self.config.session_cleanup_timeout_s
            self._audit.append(
                session.session_id,
                command_id=None,
                command_type="max_session_duration",
                issued_by="bridge",
                result="RESOURCE_LIMIT_EXCEEDED",
                detail={"state": session.state.value},
            )

        if session.state in {SessionState.RUNNING, SessionState.PAUSED} and not session.stub.is_alive():
            session.state = SessionState.RUNTIME_CRASHED
            session.cleanup_after = now + self.config.cleanup_pending_max_age_s
            self._audit.append(
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
                self._clear_world_with_audit(session, extra_detail={"trigger": "auto_cleanup"})
                self._clear_telemetry_with_audit(
                    session, extra_detail={"trigger": "auto_cleanup"}
                )
                session.stub.terminate()
                session.state = SessionState.DISCARDED
                session.world = None
                session.cleanup_after = None
                self._audit.append(
                    session.session_id,
                    command_id=None,
                    command_type="auto_cleanup",
                    issued_by="bridge",
                    result="OK",
                    detail={"state": session.state.value},
                )
            elif session.state in {SessionState.FAILED, SessionState.RUNTIME_CRASHED, SessionState.CLEANUP_PENDING}:
                self._clear_world_with_audit(session, extra_detail={"trigger": "auto_cleanup"})
                self._clear_telemetry_with_audit(
                    session, extra_detail={"trigger": "auto_cleanup"}
                )
                session.stub.terminate()
                session.state = SessionState.DISCARDED
                session.world = None
                session.cleanup_after = None
                self._audit.append(
                    session.session_id,
                    command_id=None,
                    command_type="auto_cleanup",
                    issued_by="bridge",
                    result="OK",
                    detail={"state": session.state.value},
                )

    def _response_base(self, command_id: str, session_id: Any) -> dict[str, Any]:
        return {
            "governance_banner": GOVERNANCE_BANNER,
            "session_id": session_id,
            "command_id": command_id,
            "ok": False,
            "error_code": None,
            "message": None,
        }

    def _ok(self, base: dict[str, Any], *, state: str) -> dict[str, Any]:
        out = dict(base)
        out["ok"] = True
        out["error_code"] = "OK"
        out["state"] = state
        return out

    def _fail(self, base: dict[str, Any], error_code: str, message: str) -> dict[str, Any]:
        out = dict(base)
        out["ok"] = False
        out["error_code"] = error_code
        out["message"] = message
        return out
