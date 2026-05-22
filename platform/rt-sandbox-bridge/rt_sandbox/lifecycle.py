"""Session lifecycle states and transition rules (rt_session_lifecycle_v1)."""

from __future__ import annotations

from enum import StrEnum


class SessionState(StrEnum):
    CREATED = "created"
    RUNNING = "running"
    PAUSED = "paused"
    STOPPED = "stopped"
    CAPTURED = "captured"
    DISCARDED = "discarded"
    FAILED = "failed"
    RUNTIME_CRASHED = "runtime_crashed"
    BRIDGE_DISCONNECTED = "bridge_disconnected"
    CLEANUP_PENDING = "cleanup_pending"


TERMINAL_STATES = frozenset({SessionState.DISCARDED, SessionState.CAPTURED})

# command_type -> allowed source states
_TRANSITIONS: dict[str, frozenset[SessionState]] = {
    "start_session": frozenset(),  # handled when no session
    "pause_session": frozenset({SessionState.RUNNING}),
    "resume": frozenset({SessionState.PAUSED}),
    "stop_session": frozenset({SessionState.RUNNING, SessionState.PAUSED}),
    "discard_session": frozenset(
        {
            SessionState.CREATED,
            SessionState.RUNNING,
            SessionState.PAUSED,
            SessionState.STOPPED,
            SessionState.FAILED,
            SessionState.RUNTIME_CRASHED,
            SessionState.BRIDGE_DISCONNECTED,
            SessionState.CLEANUP_PENDING,
        }
    ),
}


def can_transition(state: SessionState, command_type: str) -> bool:
    if command_type == "capture_session":
        return False
    if state in TERMINAL_STATES:
        return False
    if state == SessionState.CLEANUP_PENDING:
        return command_type == "discard_session"
    if command_type == "discard_session":
        allowed = _TRANSITIONS["discard_session"]
        return state in allowed
    allowed = _TRANSITIONS.get(command_type)
    if allowed is None:
        return False
    return state in allowed


def is_active_state(state: SessionState) -> bool:
    return state in {SessionState.RUNNING, SessionState.PAUSED, SessionState.CREATED}
