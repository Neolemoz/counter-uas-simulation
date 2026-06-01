"""Per-session tactical state bag (PLAT-RT-TAC2 / TAC3)."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any


TACTICAL_MODE_MANUAL = "manual"
TACTICAL_MODE_ASSISTED = "assisted"
TACTICAL_MODE_AUTONOMOUS = "autonomous"

ALLOWED_TACTICAL_MODES_TAC2 = frozenset({TACTICAL_MODE_MANUAL})
ALLOWED_TACTICAL_MODES_TAC3 = frozenset({TACTICAL_MODE_MANUAL, TACTICAL_MODE_ASSISTED})
ALLOWED_TACTICAL_MODES_TAC4 = frozenset(
    {TACTICAL_MODE_MANUAL, TACTICAL_MODE_ASSISTED, TACTICAL_MODE_AUTONOMOUS}
)

AUTONOMOUS_LOOP_STATUS_RUNNING = "running"
AUTONOMOUS_LOOP_STATUS_PAUSED = "paused"

AUTONOMOUS_TICK_INTERVAL_S = 2.0
ASSIGNMENT_LOCK_DURATION_S = 1.5
SWITCH_TTI_IMPROVEMENT_MARGIN_S = 1.0
SWITCH_DWELL_DURATION_S = 4.0

TARGET_ENTITY_TYPES = frozenset({"drone", "waypoint_marker"})
INTERCEPTOR_ENTITY_TYPE = "interceptor"

DEFAULT_INTERCEPTOR_SPEED_CAP_M_S = 25.0

TACTICAL_TELEMETRY_BANNER = (
    "TACTICAL TELEMETRY — sandbox simulation; not operational state"
)


@dataclass
class TacticalRecommendation:
    recommendation_id: str
    recommended_interceptor_id: str | None
    recommended_target_id: str | None
    tti_s: float | None
    feasibility: dict[str, Any]
    explanation: str
    expires_at_utc: str
    pairs_evaluated: int = 0
    ranked_pairs: list[dict[str, Any]] = field(default_factory=list)

    def is_expired(self) -> bool:
        try:
            exp = datetime.fromisoformat(self.expires_at_utc.replace("Z", "+00:00"))
        except ValueError:
            return True
        return datetime.now(timezone.utc) >= exp

    def to_dict(self) -> dict[str, Any]:
        return {
            "recommendation_id": self.recommendation_id,
            "recommended_interceptor_id": self.recommended_interceptor_id,
            "recommended_target_id": self.recommended_target_id,
            "candidate_id": self.recommended_interceptor_id,
            "tti_s": self.tti_s,
            "feasibility": dict(self.feasibility),
            "explanation": self.explanation,
            "expires_at_utc": self.expires_at_utc,
            "pairs_evaluated": self.pairs_evaluated,
            "ranked_pairs": [dict(pair) for pair in self.ranked_pairs],
        }


@dataclass
class TacticalState:
    mode: str = TACTICAL_MODE_MANUAL
    selected_interceptor_id: str | None = None
    selected_target_id: str | None = None
    assigned_interceptor_id: str | None = None
    assigned_target_id: str | None = None
    tti_s: float | None = None
    eta_s: float | None = None
    tactical_health: dict[str, Any] = field(
        default_factory=lambda: {
            "feasible": False,
            "summary": "no_selection",
            "stale": False,
        }
    )
    last_intercept_pose: dict[str, float] | None = None
    predicted_path_enu_m: list[dict[str, float]] | None = None
    interceptor_speed_cap_m_s: float = DEFAULT_INTERCEPTOR_SPEED_CAP_M_S
    pending_recommendation: TacticalRecommendation | None = None
    autonomous_loop_status: str = AUTONOMOUS_LOOP_STATUS_PAUSED
    assignment_lock_until_monotonic: float | None = None
    last_autonomous_tick_monotonic: float | None = None
    last_assignment_monotonic: float | None = None
    switch_blocked_reason: str | None = None
    candidate_tti_delta_s: float | None = None
    assigned_pairs: list[dict[str, str]] = field(default_factory=list)
    duplicate_target_blocked: bool = False
    coordination_state: str = "idle"

    def reset(self) -> None:
        self.mode = TACTICAL_MODE_MANUAL
        self.selected_interceptor_id = None
        self.selected_target_id = None
        self.assigned_interceptor_id = None
        self.assigned_target_id = None
        self.tti_s = None
        self.eta_s = None
        self.tactical_health = {
            "feasible": False,
            "summary": "no_selection",
            "stale": False,
        }
        self.last_intercept_pose = None
        self.predicted_path_enu_m = None
        self.pending_recommendation = None
        self.autonomous_loop_status = AUTONOMOUS_LOOP_STATUS_PAUSED
        self.assignment_lock_until_monotonic = None
        self.last_autonomous_tick_monotonic = None
        self.last_assignment_monotonic = None
        self.switch_blocked_reason = None
        self.candidate_tti_delta_s = None
        self.assigned_pairs = []
        self.duplicate_target_blocked = False
        self.coordination_state = "idle"

    def assignment_lock_active(self, now: float) -> bool:
        until = self.assignment_lock_until_monotonic
        return until is not None and now < until

    def clear_autonomous_scheduler(self) -> None:
        self.autonomous_loop_status = AUTONOMOUS_LOOP_STATUS_PAUSED
        self.assignment_lock_until_monotonic = None
        self.last_autonomous_tick_monotonic = None
        self.last_assignment_monotonic = None
        self.switch_blocked_reason = None
        self.candidate_tti_delta_s = None
        self.assigned_pairs = []
        self.duplicate_target_blocked = False
        self.coordination_state = "idle"

    @property
    def pending_recommendation_id(self) -> str | None:
        rec = self.pending_recommendation
        if rec is None or not rec.recommendation_id:
            return None
        return rec.recommendation_id

    def clear_pending_recommendation(self) -> None:
        self.pending_recommendation = None
