"""Internal heartbeat/state events only (not subscribe_telemetry)."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any


@dataclass
class TelemetryBuffer:
    max_hz: float = 10.0
    _events: list[dict[str, Any]] = field(default_factory=list)
    _last_emit: float = 0.0

    def _throttled(self) -> bool:
        now = time.monotonic()
        min_interval = 1.0 / self.max_hz
        if now - self._last_emit < min_interval:
            return True
        self._last_emit = now
        return False

    def maybe_emit(
        self,
        session_id: str,
        state: str,
        stub_alive: bool,
        *,
        world_summary: dict[str, Any] | None = None,
    ) -> dict[str, Any] | None:
        if self._throttled():
            return None
        event: dict[str, Any] = {
            "event_type": "session_heartbeat",
            "session_id": session_id,
            "state": state,
            "stub_alive": stub_alive,
            "governance_banner": "RT SANDBOX — experimental simulation; not operational state",
        }
        if world_summary is not None:
            event["world_summary"] = world_summary
        self._events.append(event)
        return event

    def emit_entity_pose_heartbeat(
        self,
        session_id: str,
        entities: list[dict[str, Any]],
        *,
        world_summary: dict[str, Any] | None = None,
    ) -> dict[str, Any] | None:
        if self._throttled():
            return None
        event: dict[str, Any] = {
            "event_type": "entity_pose_heartbeat",
            "session_id": session_id,
            "entities": entities,
            "governance_banner": "RT SANDBOX — experimental simulation; not operational state",
        }
        if world_summary is not None:
            event["world_summary"] = world_summary
        self._events.append(event)
        return event

    def emit_world_summary(self, session_id: str, world_summary: dict[str, Any]) -> dict[str, Any]:
        event = {
            "event_type": "world_summary",
            "session_id": session_id,
            "world_summary": world_summary,
            "governance_banner": "RT SANDBOX — experimental simulation; not operational state",
        }
        self._events.append(event)
        return event

    def recent(self, limit: int = 20) -> list[dict[str, Any]]:
        return self._events[-limit:]
