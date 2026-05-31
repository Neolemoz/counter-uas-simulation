"""Telemetry publish and subscription coordination (PLAT-RT-R3a)."""

from __future__ import annotations

from typing import Any, Callable

from rt_sandbox.adapter_poll import run_adapter_poll_tick
from rt_sandbox.audit_log import AuditLog
from rt_sandbox.governance import (
    GovernanceConfig,
    validate_telemetry_payload,
)
from rt_sandbox.lifecycle import SessionState, can_transition
from rt_sandbox.runtime_handle import runtime_is_adapter
from rt_sandbox.session_adapter_results import apply_adapter_poll_result
from rt_sandbox.session_record import SessionRecord
from rt_sandbox.runtime_capture import record_runtime_capture_frame
from rt_sandbox.session_response import fail, ok
from rt_sandbox.telemetry_subscriptions import (
    TelemetrySubscriptionStore,
    build_channel_payload,
)


def pose_sync_summary(session: SessionRecord) -> dict[str, Any] | None:
    if session.pose_sync is None or session.world is None:
        return None
    adapter_mode = None
    if runtime_is_adapter(session.runtime):
        adapter_mode = getattr(session.runtime, "mode", None)
    return session.pose_sync.summary(
        session.world.revision,
        adapter_mode=adapter_mode,
    )


def publish_telemetry(
    session: SessionRecord,
    channel: str,
    *,
    config: GovernanceConfig,
    telemetry_subs: TelemetrySubscriptionStore,
    audit: AuditLog,
    command_type: str | None = None,
    previous_state: str | None = None,
    command_id: str | None = None,
    issued_by: str = "bridge",
) -> None:
    payload = build_channel_payload(
        session,
        channel,
        config=config,
        pose_sync_summary=pose_sync_summary(session),
    )
    if payload is None:
        return
    if channel == "lifecycle_state":
        if command_type is not None:
            payload["command_type"] = command_type
        if previous_state is not None:
            payload["previous_state"] = previous_state
    record_runtime_capture_frame(session, channel, payload)
    trimmed = telemetry_subs.record(session.session_id, channel, payload)
    if trimmed > 0:
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="telemetry_buffer_trim",
            issued_by=issued_by,
            result="OK",
            detail={
                "trimmed": trimmed,
                "ring_size": config.telemetry_ring_buffer_size,
                "channel": channel,
            },
        )


def poll_telemetry_bridge(
    session: SessionRecord,
    config: GovernanceConfig,
    audit: AuditLog,
    publish_channel: Callable[[str], None],
    *,
    command_id: str | None = None,
    issued_by: str = "bridge",
    mock_stale_telemetry: bool = False,
) -> None:
    poll_result = run_adapter_poll_tick(
        session,
        config,
        poll_feedback=False,
        poll_telemetry=True,
        mock_stale_telemetry=mock_stale_telemetry,
    )
    apply_adapter_poll_result(
        session,
        poll_result,
        audit,
        publish_channel,
        command_id=command_id,
        issued_by=issued_by,
    )


def publish_channels_for_transition(
    session: SessionRecord,
    command_type: str,
    previous_state: SessionState,
    *,
    config: GovernanceConfig,
    telemetry_subs: TelemetrySubscriptionStore,
    audit: AuditLog,
    publish_channel: Callable[[str], None],
    skip_adapter_poll: bool = False,
) -> None:
    if (
        not skip_adapter_poll
        and command_type != "reset_session"
        and config.enable_gazebo_adapter
        and runtime_is_adapter(session.runtime)
    ):
        poll_telemetry_bridge(
            session,
            config,
            audit,
            publish_channel,
        )
    prev = previous_state.value
    publish_telemetry(
        session,
        "lifecycle_state",
        config=config,
        telemetry_subs=telemetry_subs,
        audit=audit,
        command_type=command_type,
        previous_state=prev,
    )
    publish_telemetry(
        session,
        "session_health",
        config=config,
        telemetry_subs=telemetry_subs,
        audit=audit,
    )
    publish_telemetry(
        session,
        "clock_mirror",
        config=config,
        telemetry_subs=telemetry_subs,
        audit=audit,
    )
    if command_type in {
        "spawn_entity",
        "move_entity",
        "delete_entity",
        "reset_session",
        "apply_runtime_template",
        "apply_scenario",
        "assign_target",
        "cancel_assignment",
        "advance_workflow",
    }:
        publish_telemetry(
            session,
            "world_summary",
            config=config,
            telemetry_subs=telemetry_subs,
            audit=audit,
        )
        publish_telemetry(
            session,
            "entity_pose_mirror",
            config=config,
            telemetry_subs=telemetry_subs,
            audit=audit,
        )


def handle_telemetry(
    session: SessionRecord,
    command_type: str,
    payload: Any,
    base: dict[str, Any],
    *,
    config: GovernanceConfig,
    telemetry_subs: TelemetrySubscriptionStore,
    audit: AuditLog,
    publish_channel: Callable[[str], None],
    poll_bridge: Callable[[], None],
    command_id: str,
    issued_by: str,
) -> dict[str, Any]:
    if not can_transition(session.state, command_type):
        return fail(base, "INVALID_STATE", session.state.value)
    payload_err = validate_telemetry_payload(command_type, payload)
    if payload_err:
        return fail(base, payload_err, f"invalid payload for {command_type}")
    assert isinstance(payload, dict)

    if command_type == "subscribe_telemetry":
        poll_bridge()
        channels = [str(c) for c in payload["channels"]]
        sub_id, err = telemetry_subs.subscribe(session.session_id, channels)
        if err:
            return fail(base, err, err)
        initial = telemetry_subs.build_initial_events(
            session,
            config=config,
            pose_sync_summary=pose_sync_summary(session),
        )
        for ev in initial:
            telemetry_subs.record(
                session.session_id,
                ev["channel"],
                ev["payload"],
            )
        audit.append(
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
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="telemetry_snapshot",
            issued_by=issued_by,
            result="OK",
            detail={"subscription_id": sub_id, "channel_count": len(channels)},
        )
        resp = ok(base, state=session.state.value)
        resp["subscription_id"] = sub_id
        resp["channels"] = channels
        resp["initial_events"] = initial
        return resp

    sub_id = str(payload["subscription_id"])
    if not telemetry_subs.unsubscribe(sub_id):
        return fail(base, "SESSION_NOT_FOUND", "unknown subscription_id")
    audit.append(
        session.session_id,
        command_id=command_id,
        command_type="unsubscribe_telemetry",
        issued_by=issued_by,
        result="OK",
        detail={"subscription_id": sub_id, "state": session.state.value},
    )
    return ok(base, state=session.state.value)
