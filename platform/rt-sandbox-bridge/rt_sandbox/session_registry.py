"""Session registry for multi-session RT bridge (PLAT-RT-M2)."""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any, Iterator

from rt_sandbox.lifecycle import SessionState
from rt_sandbox.session_record import SessionRecord

TERMINAL_STATES = frozenset({SessionState.DISCARDED, SessionState.CAPTURED})


@dataclass
class SessionRegistry:
    """Thread-safe map of session_id -> SessionRecord."""

    _sessions: dict[str, SessionRecord] = field(default_factory=dict)
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def get(self, session_id: str) -> SessionRecord | None:
        with self._lock:
            return self._sessions.get(session_id)

    def register(self, record: SessionRecord) -> None:
        with self._lock:
            self._sessions[record.session_id] = record

    def evict(self, session_id: str) -> SessionRecord | None:
        with self._lock:
            return self._sessions.pop(session_id, None)

    def non_terminal_count(self) -> int:
        with self._lock:
            return sum(
                1 for s in self._sessions.values() if s.state not in TERMINAL_STATES
            )

    def total_entity_count(self) -> int:
        with self._lock:
            total = 0
            for s in self._sessions.values():
                if s.state in TERMINAL_STATES or s.world is None:
                    continue
                total += s.world.registry.count()
            return total

    def iter_all(self) -> Iterator[SessionRecord]:
        with self._lock:
            return iter(list(self._sessions.values()))

    def iter_non_terminal(self) -> list[SessionRecord]:
        with self._lock:
            return [s for s in self._sessions.values() if s.state not in TERMINAL_STATES]

    def list_snapshot(self, *, editing_session_id: str | None) -> list[dict[str, Any]]:
        with self._lock:
            rows: list[dict[str, Any]] = []
            for s in self._sessions.values():
                entity_count = s.world.registry.count() if s.world else 0
                rows.append(
                    {
                        "session_id": s.session_id,
                        "state": s.state.value,
                        "created_monotonic": s.created_monotonic,
                        "entity_count": entity_count,
                        "is_editing": s.session_id == editing_session_id,
                    }
                )
            return rows

    def first_non_terminal_id(self) -> str | None:
        with self._lock:
            for s in self._sessions.values():
                if s.state not in TERMINAL_STATES:
                    return s.session_id
            return None
