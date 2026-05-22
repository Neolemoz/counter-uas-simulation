"""Session-scoped telemetry subscriptions (PLAT-RT-S4)."""

from __future__ import annotations

import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, TYPE_CHECKING

from rt_sandbox.governance import GOVERNANCE_BANNER

if TYPE_CHECKING:
    from rt_sandbox.session_manager import SessionRecord

TELEMETRY_CHANNELS = frozenset(
    {
        "session_health",
        "lifecycle_state",
        "world_summary",
        "entity_pose_mirror",
        "clock_mirror",
    }
)

MAX_CHANNELS_PER_SUBSCRIPTION = 5
MAX_SUBSCRIPTIONS_PER_SESSION = 1
DEFAULT_RING_SIZE = 64


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


@dataclass
class TelemetryEvent:
    channel: str
    session_id: str
    timestamp_utc: str
    payload: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return {
            "channel": self.channel,
            "session_id": self.session_id,
            "timestamp_utc": self.timestamp_utc,
            "payload": self.payload,
            "governance_banner": GOVERNANCE_BANNER,
        }


@dataclass
class TelemetrySubscription:
    subscription_id: str
    session_id: str
    channels: frozenset[str]
    events: deque[TelemetryEvent] = field(default_factory=deque)
    created_monotonic: float = field(default_factory=time.monotonic)


@dataclass
class TelemetrySubscriptionStore:
    max_hz: float = 10.0
    ring_size: int = DEFAULT_RING_SIZE
    _by_session: dict[str, TelemetrySubscription] = field(default_factory=dict)
    _by_id: dict[str, TelemetrySubscription] = field(default_factory=dict)
    _last_emit_monotonic: float = 0.0

    def _rate_ok(self) -> bool:
        now = time.monotonic()
        min_interval = 1.0 / self.max_hz
        if now - self._last_emit_monotonic < min_interval:
            return False
        self._last_emit_monotonic = now
        return True

    def subscribe(
        self,
        session_id: str,
        channels: list[str],
    ) -> tuple[str | None, str | None]:
        """Return (subscription_id, error_code)."""
        if not channels:
            return None, "COMMAND_FORBIDDEN"
        if len(channels) > MAX_CHANNELS_PER_SUBSCRIPTION:
            return None, "RESOURCE_LIMIT_EXCEEDED"
        for ch in channels:
            if ch not in TELEMETRY_CHANNELS:
                return None, "COMMAND_FORBIDDEN"
        existing = self._by_session.get(session_id)
        if existing is not None:
            self.unsubscribe(existing.subscription_id)
        sub_id = str(uuid.uuid4())
        sub = TelemetrySubscription(
            subscription_id=sub_id,
            session_id=session_id,
            channels=frozenset(channels),
        )
        self._by_session[session_id] = sub
        self._by_id[sub_id] = sub
        return sub_id, None

    def unsubscribe(self, subscription_id: str) -> bool:
        sub = self._by_id.pop(subscription_id, None)
        if sub is None:
            return False
        if self._by_session.get(sub.session_id) is sub:
            del self._by_session[sub.session_id]
        return True

    def get(self, subscription_id: str) -> TelemetrySubscription | None:
        return self._by_id.get(subscription_id)

    def get_for_session(self, session_id: str) -> TelemetrySubscription | None:
        return self._by_session.get(session_id)

    def clear_session(self, session_id: str) -> int:
        sub = self._by_session.pop(session_id, None)
        if sub is None:
            return 0
        self._by_id.pop(sub.subscription_id, None)
        return 1

    def record(self, session_id: str, channel: str, payload: dict[str, Any]) -> None:
        if channel not in TELEMETRY_CHANNELS:
            return
        sub = self._by_session.get(session_id)
        if sub is None or channel not in sub.channels:
            return
        if not self._rate_ok():
            return
        event = TelemetryEvent(
            channel=channel,
            session_id=session_id,
            timestamp_utc=_utc_now(),
            payload=payload,
        )
        sub.events.append(event)
        while len(sub.events) > self.ring_size:
            sub.events.popleft()

    def drain(self, subscription_id: str, max_events: int = 10) -> list[dict[str, Any]]:
        sub = self._by_id.get(subscription_id)
        if sub is None:
            return []
        out: list[dict[str, Any]] = []
        while sub.events and len(out) < max_events:
            out.append(sub.events.popleft().to_dict())
        return out

    def build_initial_events(self, session: SessionRecord) -> list[dict[str, Any]]:
        """Snapshot events for all subscribed channels on subscribe."""
        sub = self._by_session.get(session.session_id)
        if sub is None:
            return []
        events: list[dict[str, Any]] = []
        for ch in sorted(sub.channels):
            payload = build_channel_payload(session, ch)
            if payload is None:
                continue
            events.append(
                TelemetryEvent(
                    channel=ch,
                    session_id=session.session_id,
                    timestamp_utc=_utc_now(),
                    payload=payload,
                ).to_dict()
            )
        return events


def build_channel_payload(session: SessionRecord, channel: str) -> dict[str, Any] | None:
    """Build read-only payload for a channel from current session state."""
    from rt_sandbox.lifecycle import SessionState

    if channel == "session_health":
        return {
            "state": session.state.value,
            "stub_alive": session.stub.is_alive(),
            "governance_banner": GOVERNANCE_BANNER,
        }
    if channel == "lifecycle_state":
        return {
            "state": session.state.value,
            "previous_state": None,
            "command_type": None,
        }
    if channel == "clock_mirror":
        return {"paused": session.state == SessionState.PAUSED}
    if channel == "world_summary":
        if session.world is None:
            return {"entity_count": 0, "revision": 0, "by_type": {}, "bounds": {}}
        return session.world.world_summary()
    if channel == "entity_pose_mirror":
        if session.world is None:
            return {"entities": []}
        return {"entities": session.world.registry.poses_for_telemetry()}
    return None
