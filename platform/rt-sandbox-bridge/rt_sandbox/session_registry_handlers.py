"""Session registry command handlers (PLAT-RT-M2)."""

from __future__ import annotations

from typing import Any

from rt_sandbox.audit_log import AuditLog
from rt_sandbox.governance import GovernanceConfig
from rt_sandbox.lifecycle import SessionState
from rt_sandbox.session_record import SessionRecord
from rt_sandbox.session_registry import SessionRegistry, TERMINAL_STATES
from rt_sandbox.session_response import fail, ok


def list_sessions(
    base: dict[str, Any],
    *,
    registry: SessionRegistry,
    editing_session_id: str | None,
    config: GovernanceConfig,
) -> dict[str, Any]:
    sessions = registry.list_snapshot(editing_session_id=editing_session_id)
    resp = ok(base, state="registry")
    resp["sessions"] = sessions
    resp["editing_session_id"] = editing_session_id
    resp["capacity"] = config.max_concurrent_sessions
    resp["non_terminal_count"] = registry.non_terminal_count()
    return resp


def set_editing_session(
    base: dict[str, Any],
    payload: Any,
    *,
    registry: SessionRegistry,
    audit: AuditLog,
    command_id: str,
    issued_by: str,
) -> dict[str, Any]:
    if not isinstance(payload, dict):
        return fail(base, "INVALID_STATE", "payload required")
    target_id = payload.get("session_id")
    if not isinstance(target_id, str) or not target_id:
        return fail(base, "SESSION_NOT_FOUND", "session_id required")
    session = registry.get(target_id)
    if session is None:
        return fail(base, "SESSION_NOT_FOUND", "unknown session_id")
    if session.state in TERMINAL_STATES:
        return fail(base, "INVALID_STATE", session.state.value)
    audit.append(
        target_id,
        command_id=command_id,
        command_type="set_editing_session",
        issued_by=issued_by,
        result="OK",
        detail={"session_id": target_id, "state": session.state.value},
    )
    resp = ok(base, state=session.state.value)
    resp["editing_session_id"] = target_id
    return resp


def pick_editing_session_after_evict(
    registry: SessionRegistry,
    evicted_id: str,
    current_editing: str | None,
) -> str | None:
    if current_editing != evicted_id:
        return current_editing
    return registry.first_non_terminal_id()
