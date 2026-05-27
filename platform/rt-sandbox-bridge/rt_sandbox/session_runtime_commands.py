"""Runtime adapter subcommand handlers (PLAT-RT-R3a)."""

from __future__ import annotations

from typing import Any, Callable

from rt_sandbox.adapter_poll import run_adapter_poll_tick
from rt_sandbox.audit_log import AuditLog
from rt_sandbox.governance import GovernanceConfig, validate_runtime_subcommand
from rt_sandbox.runtime_handle import runtime_is_adapter
from rt_sandbox.session_adapter_results import (
    apply_adapter_poll_result,
    apply_template_resync_result,
)
from rt_sandbox.session_record import SessionRecord
from rt_sandbox.session_response import fail, ok
from rt_sandbox.session_teardown import clear_adapter_mirrors
from rt_sandbox.session_telemetry_coordinator import pose_sync_summary
from rt_sandbox.template_resync import run_template_adapter_resync


def handle_runtime_command(
    session: SessionRecord,
    payload: Any,
    base: dict[str, Any],
    *,
    config: GovernanceConfig,
    audit: AuditLog,
    publish_channel: Callable[[str], None],
    runtime_crashed: Callable[[], dict[str, Any]],
    command_id: str,
    issued_by: str,
) -> dict[str, Any]:
    if not config.enable_gazebo_adapter:
        return fail(
            base,
            "COMMAND_FORBIDDEN",
            "gazebo adapter disabled",
        )
    err = validate_runtime_subcommand(payload)
    if err:
        return fail(base, err, "invalid runtime sub_command")
    assert isinstance(payload, dict)
    sub = str(payload["sub_command"])
    if not runtime_is_adapter(session.runtime):
        return fail(base, "INVALID_STATE", "adapter not active")
    if not session.runtime.is_alive():
        return runtime_crashed()

    if sub == "adapter_health":
        health = session.runtime.health_payload()
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="adapter_health",
            issued_by=issued_by,
            result="OK",
            detail=health,
        )
        resp = ok(base, state=session.state.value)
        resp["runtime_health"] = health
        return resp

    if sub == "adapter_attach":
        try:
            pid = session.runtime.start()
        except OSError as exc:
            return fail(base, "RUNTIME_UNAVAILABLE", str(exc))
        detail = {"runtime_pid": pid, "adapter_mode": session.runtime.mode}
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="adapter_attach",
            issued_by=issued_by,
            result="OK",
            detail=detail,
        )
        resp = ok(base, state=session.state.value)
        resp["runtime_health"] = session.runtime.health_payload()
        return resp

    if sub == "adapter_poll_telemetry":
        mock_stale = bool(payload.get("mock_stale_telemetry"))
        poll_result = run_adapter_poll_tick(
            session,
            config,
            poll_feedback=False,
            poll_telemetry=True,
            mock_stale_telemetry=mock_stale,
        )
        apply_adapter_poll_result(
            session,
            poll_result,
            audit,
            publish_channel,
            command_id=command_id,
            issued_by=issued_by,
        )
        resp = ok(base, state=session.state.value)
        if session.world:
            resp["world_summary"] = session.world.world_summary(
                pose_sync_summary=pose_sync_summary(session)
            )
        return resp

    if sub == "adapter_detach":
        session.runtime.terminate()
        clear_adapter_mirrors(session)
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="adapter_detach",
            issued_by=issued_by,
            result="OK",
            detail={"detached": True},
        )
        return ok(base, state=session.state.value)

    if sub == "adapter_poll_feedback":
        poll_result = run_adapter_poll_tick(
            session,
            config,
            poll_feedback=True,
            poll_telemetry=False,
        )
        apply_adapter_poll_result(
            session,
            poll_result,
            audit,
            publish_channel,
            command_id=command_id,
            issued_by=issued_by,
        )
        if poll_result.sync_error_code:
            return fail(
                base,
                poll_result.sync_error_code,
                poll_result.audits[-1].command_type
                if poll_result.audits
                else poll_result.sync_error_code,
            )
        resp = ok(base, state=session.state.value)
        if session.world:
            resp["world_summary"] = session.world.world_summary(
                pose_sync_summary=pose_sync_summary(session)
            )
        return resp

    if sub == "adapter_resync":
        world = session.world
        if world is None:
            return fail(base, "INVALID_STATE", "world not initialized")
        resync_result = run_template_adapter_resync(
            session,
            config,
            trigger="adapter_resync",
            manual=True,
        )
        if resync_result.resync_ipc_result and resync_result.resync_ipc_result.get(
            "error_code"
        ):
            return fail(
                base,
                str(resync_result.resync_ipc_result["error_code"]),
                str(
                    resync_result.resync_ipc_result.get(
                        "error_message", "resync failed"
                    )
                ),
            )
        apply_template_resync_result(
            session,
            resync_result,
            audit,
            publish_channel,
            command_id=command_id,
            issued_by=issued_by,
        )
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="sync_update",
            issued_by=issued_by,
            result="OK",
            detail={
                "resync": True,
                "entity_count": resync_result.entity_count,
                **(resync_result.resync_ipc_result or {}),
            },
        )
        resp = ok(base, state=session.state.value)
        resp["world_summary"] = world.world_summary(
            pose_sync_summary=pose_sync_summary(session)
        )
        return resp

    if sub == "mock_inject_drift":
        if config.adapter_mode != "mock":
            return fail(base, "COMMAND_FORBIDDEN", "mock only")
        entity_id = str(payload.get("entity_id", ""))
        offset = dict(payload.get("offset") or {})
        fn = getattr(session.runtime, "mock_inject_drift", None)
        if fn is None:
            return fail(base, "INVALID_STATE", "adapter required")
        result = fn(entity_id, offset)
        if result and result.get("error_code"):
            return fail(
                base,
                str(result["error_code"]),
                str(result.get("error_message", "drift inject failed")),
            )
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="mock_inject_drift",
            issued_by=issued_by,
            result="OK",
            detail=result or {},
        )
        return ok(base, state=session.state.value)

    return fail(base, "COMMAND_FORBIDDEN", sub)  # unreachable when R3c lint passes
