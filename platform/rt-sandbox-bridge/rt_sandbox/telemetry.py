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

    def maybe_emit(self, session_id: str, state: str, stub_alive: bool) -> dict[str, Any] | None:
        now = time.monotonic()
        min_interval = 1.0 / self.max_hz
        if now - self._last_emit < min_interval:
            return None
        self._last_emit = now
        event = {
            "event_type": "session_heartbeat",
            "session_id": session_id,
            "state": state,
            "stub_alive": stub_alive,
            "governance_banner": "RT SANDBOX — experimental simulation; not operational state",
        }
        self._events.append(event)
        return event

    def recent(self, limit: int = 20) -> list[dict[str, Any]]:
        return self._events[-limit:]
