"""Transient session manager for RT sandbox bridge prototype (PLAT-RT-R3a facade, PLAT-RT-M2 registry)."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

from rt_sandbox.audit_log import AuditLog
from rt_sandbox.export_audit_log import ExportAuditLog
from rt_sandbox.governance import (
    ENTITY_COMMANDS,
    GOVERNANCE_BANNER,
    GovernanceConfig,
    RateLimiter,
    TACTICAL_COMMANDS,
    TEMPLATE_COMMANDS,
    TELEMETRY_COMMANDS,
    WORKFLOW_COMMANDS,
    classify_command,
    validate_capture_payload,
)
from rt_sandbox.lifecycle import SessionState
from rt_sandbox.session_capture_handler import capture_session as handle_capture_session
from rt_sandbox.session_entity_handlers import handle_entity
from rt_sandbox.session_lifecycle_handlers import (
    discard_session,
    pause_session,
    reset_session,
    resume_session,
    runtime_crashed as lifecycle_runtime_crashed,
    start_session,
    stop_session,
    tick_timeouts,
)
from rt_sandbox.session_record import SessionRecord
from rt_sandbox.session_registry import SessionRegistry, TERMINAL_STATES
from rt_sandbox.session_handoff_handlers import (
    handle_list_capture_handoff_status,
)
from rt_sandbox.session_registry_handlers import (
    list_sessions as handle_list_sessions,
    pick_editing_session_after_evict,
    set_editing_session as handle_set_editing_session,
)
from rt_sandbox.session_response import fail, ok, response_base
from rt_sandbox.session_runtime_commands import handle_runtime_command
from rt_sandbox.session_tactical_handlers import (
    handle_tactical,
    tick_tactical_autonomous_for_session,
)
from rt_sandbox.tactical_autonomous import monotonic_now
from rt_sandbox.session_telemetry_coordinator import (
    handle_telemetry,
    poll_telemetry_bridge,
    publish_channels_for_transition,
    publish_telemetry,
)
from rt_sandbox.session_workflow_handlers import (
    handle_template,
    handle_workflow,
    list_runtime_templates,
)
from rt_sandbox.telemetry import TelemetryBuffer
from rt_sandbox.telemetry_subscriptions import TelemetrySubscriptionStore

# Re-export for backward compatibility
__all__ = ["BridgeSessionManager", "GovernanceConfig", "SessionRecord"]


class BridgeSessionManager:
    def __init__(
        self,
        config: GovernanceConfig | None = None,
        repo_root: Path | None = None,
    ) -> None:
        self.config = config or GovernanceConfig()
        self._repo_root = repo_root
        self._registry = SessionRegistry()
        self._editing_session_id: str | None = None
        self._rate_limiters: dict[str, RateLimiter] = {}
        self._global_rate_limiter = RateLimiter(
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

    @property
    def _session(self) -> SessionRecord | None:
        """Backward-compatible single-session accessor."""
        non_terminal = self._registry.iter_non_terminal()
        if non_terminal:
            return non_terminal[0]
        all_sessions = list(self._registry.iter_all())
        if len(all_sessions) == 1:
            return all_sessions[0]
        return None

    def handle_command(self, body: dict[str, Any]) -> dict[str, Any]:
        now = time.monotonic()
        command_type = str(body.get("command_type", ""))
        command_id = str(body.get("command_id", ""))
        issued_by = str(body.get("issued_by", "rt_ui_prototype"))
        authority_scope = str(body.get("authority_scope", ""))
        session_id = body.get("session_id")
        payload = body.get("payload")

        base = response_base(command_id, session_id)

        if isinstance(session_id, str):
            pre_session = self._registry.get(session_id)
            if (
                pre_session is not None
                and pre_session.state in {SessionState.RUNNING, SessionState.PAUSED}
                and not pre_session.runtime.is_alive()
            ):
                return self._runtime_crashed(
                    pre_session,
                    base,
                    command_id,
                    issued_by,
                )

        self._tick_timeouts(now)
        self._evict_terminal_sessions()

        if authority_scope and authority_scope != self.config.authority_scope:
            return fail(base, "COMMAND_FORBIDDEN", "invalid authority_scope")

        forbidden = classify_command(command_type)
        if forbidden:
            return fail(base, forbidden, f"command not allowed: {command_type}")

        cap_err = validate_capture_payload(command_type, payload)
        if cap_err:
            return fail(base, cap_err, "invalid capture payload")

        if command_type == "start_session":
            if not self._global_rate_limiter.check(now):
                return fail(base, "RESOURCE_LIMIT_EXCEEDED", "command rate limit")
            result = start_session(
                base,
                config=self.config,
                audit=self._audit,
                telemetry=self._telemetry,
                publish_transition=self._publish_channels_for_transition,
                non_terminal_count=self._registry.non_terminal_count,
                register_session=self._register_session,
                command_id=command_id,
                issued_by=issued_by,
                now=now,
            )
            if result.get("ok") and result.get("session_id"):
                sid = str(result["session_id"])
                if self._editing_session_id is None:
                    self._editing_session_id = sid
            return result

        if command_type == "list_sessions":
            if not self._global_rate_limiter.check(now):
                return fail(base, "RESOURCE_LIMIT_EXCEEDED", "command rate limit")
            return handle_list_sessions(
                base,
                registry=self._registry,
                editing_session_id=self._editing_session_id,
                config=self.config,
            )

        if command_type == "set_editing_session":
            if not self._global_rate_limiter.check(now):
                return fail(base, "RESOURCE_LIMIT_EXCEEDED", "command rate limit")
            result = handle_set_editing_session(
                base,
                payload,
                registry=self._registry,
                audit=self._audit,
                command_id=command_id,
                issued_by=issued_by,
            )
            if result.get("ok") and result.get("editing_session_id"):
                self._editing_session_id = str(result["editing_session_id"])
            return result

        if command_type == "list_capture_handoff_status":
            if not self._global_rate_limiter.check(now):
                return fail(base, "RESOURCE_LIMIT_EXCEEDED", "command rate limit")
            return handle_list_capture_handoff_status(
                base,
                payload,
                repo_root=self._repo_root,
            )

        if command_type == "list_runtime_templates":
            return list_runtime_templates(base, self.config)

        sid = session_id
        if not sid or not isinstance(sid, str):
            return fail(base, "SESSION_NOT_FOUND", "unknown session_id")

        session = self._registry.get(sid)
        if session is None:
            return fail(base, "SESSION_NOT_FOUND", "unknown session_id")

        base["session_id"] = session.session_id

        if not self._rate_limiter_for(session.session_id).check(now):
            return fail(base, "RESOURCE_LIMIT_EXCEEDED", "command rate limit")

        if command_type in ENTITY_COMMANDS or command_type == "apply_runtime_template":
            if session.session_id != self._editing_session_id:
                return fail(
                    base,
                    "EDITING_SESSION_MISMATCH",
                    "entity mutations require editing session",
                )
        if command_type in TACTICAL_COMMANDS and command_type != "get_tactical_state":
            if session.session_id != self._editing_session_id:
                return fail(
                    base,
                    "EDITING_SESSION_MISMATCH",
                    "tactical mutations require editing session",
                )

        if command_type == "pause_session":
            return pause_session(
                session,
                base,
                audit=self._audit,
                publish_transition=self._publish_channels_for_transition,
                runtime_crashed_fn=lambda: self._runtime_crashed(
                    session, base, command_id, issued_by
                ),
                command_id=command_id,
                issued_by=issued_by,
            )
        if command_type == "resume":
            return resume_session(
                session,
                base,
                audit=self._audit,
                publish_transition=self._publish_channels_for_transition,
                runtime_crashed_fn=lambda: self._runtime_crashed(
                    session, base, command_id, issued_by
                ),
                command_id=command_id,
                issued_by=issued_by,
            )
        if command_type == "stop_session":
            return stop_session(
                session,
                base,
                config=self.config,
                audit=self._audit,
                publish_transition=self._publish_channels_for_transition,
                command_id=command_id,
                issued_by=issued_by,
                now=now,
            )
        if command_type == "discard_session":
            result = discard_session(
                session,
                base,
                audit=self._audit,
                export_audit=self._export_audit,
                telemetry_subs=self._telemetry_subs,
                command_id=command_id,
                issued_by=issued_by,
            )
            if result.get("ok"):
                self._unregister_session(session.session_id)
            return result
        if command_type == "capture_session":
            result = handle_capture_session(
                session,
                base,
                config=self.config,
                audit=self._audit,
                export_audit=self._export_audit,
                telemetry_subs=self._telemetry_subs,
                repo_root=self._repo_root,
                command_id=command_id,
                issued_by=issued_by,
                payload=payload,
            )
            if result.get("ok"):
                self._unregister_session(session.session_id)
            return result
        if command_type == "reset_session":
            return reset_session(
                session,
                base,
                config=self.config,
                audit=self._audit,
                telemetry=self._telemetry,
                publish_transition=self._publish_channels_for_transition,
                command_id=command_id,
                issued_by=issued_by,
            )
        if command_type in ENTITY_COMMANDS:
            return handle_entity(
                session,
                command_type,
                payload,
                base,
                config=self.config,
                audit=self._audit,
                telemetry=self._telemetry,
                telemetry_subs=self._telemetry_subs,
                publish_channel=lambda ch: self._publish_telemetry(session, ch),
                publish_transition=lambda ct, st, skip: self._publish_channels_for_transition(
                    session, ct, st, skip_adapter_poll=skip
                ),
                runtime_crashed=lambda: self._runtime_crashed(
                    session, base, command_id, issued_by
                ),
                command_id=command_id,
                issued_by=issued_by,
                total_entity_count=self._registry.total_entity_count,
            )
        if command_type in TELEMETRY_COMMANDS:
            return handle_telemetry(
                session,
                command_type,
                payload,
                base,
                config=self.config,
                telemetry_subs=self._telemetry_subs,
                audit=self._audit,
                publish_channel=lambda ch: self._publish_telemetry(
                    session,
                    ch,
                    command_id=command_id,
                    issued_by=issued_by,
                ),
                poll_bridge=lambda: poll_telemetry_bridge(
                    session,
                    self.config,
                    self._audit,
                    lambda ch: self._publish_telemetry(
                        session,
                        ch,
                        command_id=command_id,
                        issued_by=issued_by,
                    ),
                    command_id=command_id,
                    issued_by=issued_by,
                ),
                command_id=command_id,
                issued_by=issued_by,
            )
        if command_type in TEMPLATE_COMMANDS:
            return handle_template(
                session,
                command_type,
                payload,
                base,
                config=self.config,
                audit=self._audit,
                publish_channel=lambda ch: self._publish_telemetry(
                    session,
                    ch,
                    command_id=command_id,
                    issued_by=issued_by,
                ),
                publish_transition=lambda ct, st, skip: self._publish_channels_for_transition(
                    session, ct, st, skip_adapter_poll=skip
                ),
                runtime_crashed=lambda: self._runtime_crashed(
                    session, base, command_id, issued_by
                ),
                command_id=command_id,
                issued_by=issued_by,
            )
        if command_type in WORKFLOW_COMMANDS:
            return handle_workflow(
                session,
                command_type,
                payload,
                base,
                config=self.config,
                audit=self._audit,
                publish_channel=lambda ch: self._publish_telemetry(
                    session,
                    ch,
                    command_id=command_id,
                    issued_by=issued_by,
                ),
                publish_transition=lambda ct, st, skip: self._publish_channels_for_transition(
                    session, ct, st, skip_adapter_poll=skip
                ),
                runtime_crashed=lambda: self._runtime_crashed(
                    session, base, command_id, issued_by
                ),
                command_id=command_id,
                issued_by=issued_by,
            )
        if command_type == "send_runtime_command":
            return handle_runtime_command(
                session,
                payload,
                base,
                config=self.config,
                audit=self._audit,
                publish_channel=lambda ch: self._publish_telemetry(
                    session,
                    ch,
                    command_id=command_id,
                    issued_by=issued_by,
                ),
                runtime_crashed=lambda: self._runtime_crashed(
                    session, base, command_id, issued_by
                ),
                command_id=command_id,
                issued_by=issued_by,
            )
        if command_type in TACTICAL_COMMANDS:
            return handle_tactical(
                session,
                command_type,
                payload,
                base,
                config=self.config,
                audit=self._audit,
                telemetry=self._telemetry,
                publish_channel=lambda ch: self._publish_telemetry(
                    session,
                    ch,
                    command_id=command_id,
                    issued_by=issued_by,
                ),
                command_id=command_id,
                issued_by=issued_by,
            )

        return fail(base, "COMMAND_FORBIDDEN", command_type)

    def _rate_limiter_for(self, session_id: str) -> RateLimiter:
        if session_id not in self._rate_limiters:
            self._rate_limiters[session_id] = RateLimiter(
                burst=self.config.command_rate_burst,
                sustained_per_s=self.config.command_rate_sustained,
            )
        return self._rate_limiters[session_id]

    def _register_session(self, record: SessionRecord) -> None:
        self._registry.register(record)
        _ = self._rate_limiter_for(record.session_id)

    def _unregister_session(self, session_id: str) -> None:
        self._registry.evict(session_id)
        self._rate_limiters.pop(session_id, None)
        self._editing_session_id = pick_editing_session_after_evict(
            self._registry,
            session_id,
            self._editing_session_id,
        )

    def _evict_terminal_sessions(self) -> None:
        for session in list(self._registry.iter_all()):
            if session.state in TERMINAL_STATES:
                self._unregister_session(session.session_id)

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
        command_id: str | None = None,
        issued_by: str = "bridge",
    ) -> None:
        publish_telemetry(
            session,
            channel,
            config=self.config,
            telemetry_subs=self._telemetry_subs,
            audit=self._audit,
            command_type=command_type,
            previous_state=previous_state,
            command_id=command_id,
            issued_by=issued_by,
        )

    def _publish_channels_for_transition(
        self,
        session: SessionRecord,
        command_type: str,
        previous_state: SessionState,
        *,
        skip_adapter_poll: bool = False,
    ) -> None:
        publish_channels_for_transition(
            session,
            command_type,
            previous_state,
            config=self.config,
            telemetry_subs=self._telemetry_subs,
            audit=self._audit,
            publish_channel=lambda ch: self._publish_telemetry(session, ch),
            skip_adapter_poll=skip_adapter_poll,
        )

    def _runtime_crashed(
        self,
        session: SessionRecord,
        base: dict[str, Any],
        command_id: str,
        issued_by: str,
    ) -> dict[str, Any]:
        return lifecycle_runtime_crashed(
            session,
            base,
            self._audit,
            self.config,
            command_id=command_id,
            issued_by=issued_by,
        )

    def _tick_timeouts(self, now: float) -> None:
        mono = monotonic_now()
        for session in self._registry.iter_non_terminal():
            tick_timeouts(
                session,
                config=self.config,
                audit=self._audit,
                telemetry_subs=self._telemetry_subs,
                now=now,
            )
            tick_tactical_autonomous_for_session(
                session,
                mono,
                config=self.config,
                audit=self._audit,
                publish_channel=lambda ch, s=session: self._publish_telemetry(s, ch),
            )

    # Backward-compatible private aliases for tests
    def _response_base(self, command_id: str, session_id: Any) -> dict[str, Any]:
        return response_base(command_id, session_id)

    def _ok(self, base: dict[str, Any], *, state: str) -> dict[str, Any]:
        return ok(base, state=state)

    def _fail(self, base: dict[str, Any], error_code: str, message: str) -> dict[str, Any]:
        return fail(base, error_code, message)
