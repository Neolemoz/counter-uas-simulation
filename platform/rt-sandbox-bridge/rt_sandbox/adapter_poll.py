"""Unified adapter poll tick — feedback + telemetry (PLAT-RT-R1b).

Polling vs sync semantics: docs/evaluation/rt_poll_sync_semantics_v1.md
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from rt_sandbox.adapter_sync import (
    ensure_pose_sync_mirror,
    poll_feedback,
    sync_audit_event,
)
from rt_sandbox.fidelity_coupling import build_fidelity_poll_audits
from rt_sandbox.governance import GovernanceConfig
from rt_sandbox.runtime_handle import runtime_is_adapter
from rt_sandbox.telemetry_bridge import poll_and_update_mirror


@dataclass
class PollAuditEntry:
    command_type: str
    result: str
    detail: dict[str, Any] | None = None


@dataclass
class AdapterPollResult:
    audits: list[PollAuditEntry] = field(default_factory=list)
    sync_error_code: str | None = None
    sync_enriched: dict[str, Any] | None = None
    telemetry_error_code: str | None = None
    should_publish_channels: bool = False


def adapter_poll_active(session: Any, config: GovernanceConfig) -> bool:
    if not config.enable_gazebo_adapter:
        return False
    return runtime_is_adapter(session.runtime)


def _append_feedback_poll(
    session: Any,
    config: GovernanceConfig,
    result: AdapterPollResult,
    *,
    emit_audit: bool = True,
) -> None:
    if not config.pose_sync_enabled:
        return
    feedback = poll_feedback(session.runtime, config)
    if feedback is None:
        return
    if feedback.get("error_code"):
        mirror = ensure_pose_sync_mirror(session)
        mirror.sync_health = "feedback_lost"
        err = str(feedback["error_code"])
        result.audits.append(
            PollAuditEntry("adapter_feedback_lost", err, dict(feedback))
        )
        result.sync_error_code = err
        return
    world = session.world
    if world is None:
        return
    mirror = ensure_pose_sync_mirror(session)
    registry_ids = {e.entity_id for e in world.registry.all_entities()}
    health, err_code, audit_detail = mirror.update_from_feedback(
        feedback, registry_ids, config
    )
    if config.pose_sync_enabled and mirror.check_feedback_stale(config):
        if health == "ok":
            health = "feedback_lost"
        mirror.sync_health = health
        if not err_code:
            err_code = "ADAPTER_FEEDBACK_LOST"
            audit_detail = {
                "reason": "feedback_poll_stale",
                "last_poll_utc": mirror.last_poll_utc,
                "stale_s": config.adapter_feedback_stale_s,
            }
    event = sync_audit_event(err_code, health)
    if event and emit_audit:
        result.audits.append(
            PollAuditEntry(
                event,
                err_code or "OK",
                audit_detail or {"sync_health": health},
            )
        )
        result.sync_error_code = err_code


def _append_telemetry_poll(
    session: Any,
    config: GovernanceConfig,
    result: AdapterPollResult,
    *,
    mock_stale_telemetry: bool = False,
) -> None:
    audit_event, err_code, detail = poll_and_update_mirror(
        session,
        config,
        mock_stale_telemetry=mock_stale_telemetry,
    )
    if not audit_event:
        return
    result.audits.append(
        PollAuditEntry(audit_event, err_code or "OK", detail)
    )
    if err_code:
        result.telemetry_error_code = err_code
    else:
        result.should_publish_channels = True
    for cmd_type, res, fdetail in build_fidelity_poll_audits(session, config):
        result.audits.append(PollAuditEntry(cmd_type, res, fdetail))


def run_adapter_poll_tick(
    session: Any,
    config: GovernanceConfig,
    *,
    poll_feedback: bool = False,
    poll_telemetry: bool = True,
    mock_stale_telemetry: bool = False,
    emit_feedback_audit: bool = True,
) -> AdapterPollResult:
    """Single adapter poll tick for feedback and/or telemetry mirrors."""
    result = AdapterPollResult()
    if not adapter_poll_active(session, config):
        return result
    if poll_feedback:
        _append_feedback_poll(
            session,
            config,
            result,
            emit_audit=emit_feedback_audit,
        )
    if poll_telemetry:
        _append_telemetry_poll(
            session,
            config,
            result,
            mock_stale_telemetry=mock_stale_telemetry,
        )
    return result
