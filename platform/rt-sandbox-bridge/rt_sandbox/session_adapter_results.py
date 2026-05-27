"""Adapter poll/resync result application (PLAT-RT-R3a)."""

from __future__ import annotations

from typing import Any, Callable

from rt_sandbox.audit_log import AuditLog
from rt_sandbox.session_record import SessionRecord
from rt_sandbox.telemetry_bridge import publish_all_telemetry_channels


def apply_adapter_poll_result(
    session: SessionRecord,
    poll_result: Any,
    audit: AuditLog,
    publish_channel: Callable[[str], None],
    *,
    command_id: str | None = None,
    issued_by: str = "bridge",
) -> None:
    from rt_sandbox.adapter_poll import AdapterPollResult

    assert isinstance(poll_result, AdapterPollResult)
    for entry in poll_result.audits:
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type=entry.command_type,
            issued_by=issued_by,
            result=entry.result,
            detail=entry.detail,
        )
    if poll_result.should_publish_channels:
        publish_all_telemetry_channels(session, publish_channel)


def apply_template_resync_result(
    session: SessionRecord,
    resync_result: Any,
    audit: AuditLog,
    publish_channel: Callable[[str], None],
    *,
    command_id: str | None = None,
    issued_by: str = "bridge",
) -> bool:
    """Apply template resync audits. Returns True if telemetry already published."""
    from rt_sandbox.template_resync import TemplateResyncResult

    assert isinstance(resync_result, TemplateResyncResult)
    for entry in resync_result.audits:
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type=entry.command_type,
            issued_by=issued_by,
            result=entry.result,
            detail=entry.detail,
        )
    if resync_result.should_publish_channels:
        publish_all_telemetry_channels(session, publish_channel)
    return resync_result.should_publish_channels
