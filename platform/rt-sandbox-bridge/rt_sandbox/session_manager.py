"""Transient session manager for RT-S2 bridge prototype."""

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from rt_sandbox.audit_log import AuditLog
from rt_sandbox.governance import (
    GOVERNANCE_BANNER,
    GovernanceConfig,
    RateLimiter,
    classify_command,
)
from rt_sandbox.lifecycle import SessionState, can_transition, is_active_state
from rt_sandbox.runtime_stub import RuntimeStub
from rt_sandbox.telemetry import TelemetryBuffer


@dataclass
class SessionRecord:
    session_id: str
    state: SessionState
    created_monotonic: float
    bridge_ready_deadline: float
    cleanup_after: float | None = None
    stub: RuntimeStub = field(default_factory=RuntimeStub)
    issued_by: str = "rt_ui_prototype"


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
        self._telemetry = TelemetryBuffer(max_hz=10.0)

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

        base = self._response_base(command_id, session_id)

        if authority_scope and authority_scope != self.config.authority_scope:
            return self._fail(base, "COMMAND_FORBIDDEN", "invalid authority_scope")

        forbidden = classify_command(command_type)
        if forbidden:
            return self._fail(base, forbidden, f"command not allowed in RT-S2: {command_type}")

        if not self._rate_limiter.check(now):
            return self._fail(base, "RESOURCE_LIMIT_EXCEEDED", "command rate limit")

        if command_type == "start_session":
            return self._start_session(base, command_id, issued_by, now)

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

        return self._fail(base, "COMMAND_FORBIDDEN", command_type)

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
        )
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
            hb = self._telemetry.maybe_emit(session_id, record.state.value, record.stub.is_alive())
            resp = self._ok(base, state=record.state.value)
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
        return self._ok(base, state=session.state.value)

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
        return self._ok(base, state=session.state.value)

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
        session.state = SessionState.CLEANUP_PENDING
        session.stub.terminate()
        session.state = SessionState.DISCARDED
        session.cleanup_after = None
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

        if session.cleanup_after is not None and now >= session.cleanup_after:
            if session.state == SessionState.STOPPED:
                session.state = SessionState.CLEANUP_PENDING
                session.stub.terminate()
                session.state = SessionState.DISCARDED
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
                session.stub.terminate()
                session.state = SessionState.DISCARDED
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
