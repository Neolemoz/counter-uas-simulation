"""Deny-by-default command governance (rt_bridge_contract_v1, rt_runtime_governance_v1)."""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field

GOVERNANCE_BANNER = "RT SANDBOX — experimental simulation; not operational state"

SESSION_COMMANDS = frozenset(
    {
        "start_session",
        "pause_session",
        "resume",
        "stop_session",
        "discard_session",
    }
)

RT_S2_FORBIDDEN_COMMANDS = frozenset(
    {
        "capture_session",
        "spawn_entity",
        "move_entity",
        "delete_entity",
        "subscribe_telemetry",
        "unsubscribe_telemetry",
        "send_runtime_command",
        "engage",
        "intercept",
        "strike",
        "corpus_promote",
        "federation_register",
        "publish_to_collection",
        "launch_queue",
        "run_experiment_queue",
        "publish_topic",
        "alter_parser_contract",
    }
)

FORBIDDEN_SUBSTRINGS = (
    "corpus_",
    "federation_",
    "orchestration",
    "replay_sa",
    "publish_topic",
)


@dataclass
class GovernanceConfig:
    max_concurrent_sessions: int = 1
    command_rate_burst: int = 5
    command_rate_sustained: float = 1.0
    bridge_ready_timeout_s: float = 60.0
    session_cleanup_timeout_s: float = 120.0
    cleanup_pending_max_age_s: float = 300.0
    max_session_duration_s: float = 3600.0
    authority_scope: str = "rt_sandbox_prototype"


@dataclass
class RateLimiter:
    burst: int
    sustained_per_s: float
    _timestamps: deque[float] = field(default_factory=deque)

    def check(self, now: float | None = None) -> bool:
        t = now if now is not None else time.monotonic()
        window = 1.0
        while self._timestamps and t - self._timestamps[0] > window:
            self._timestamps.popleft()
        if len(self._timestamps) >= self.burst:
            return False
        if len(self._timestamps) >= self.burst - 1 and self._timestamps:
            elapsed = t - self._timestamps[-1]
            if elapsed < 1.0 / self.sustained_per_s:
                return False
        self._timestamps.append(t)
        return True


def classify_command(command_type: str) -> str | None:
    """Return error_code if forbidden, else None."""
    if command_type in RT_S2_FORBIDDEN_COMMANDS:
        return "COMMAND_FORBIDDEN"
    low = command_type.lower()
    for sub in FORBIDDEN_SUBSTRINGS:
        if sub in low:
            return "COMMAND_FORBIDDEN"
    if command_type == "resume_session":
        return "COMMAND_FORBIDDEN"
    if command_type not in SESSION_COMMANDS:
        return "COMMAND_FORBIDDEN"
    return None
