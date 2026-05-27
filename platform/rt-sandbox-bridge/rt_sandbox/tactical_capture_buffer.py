"""Session-scoped tactical timeline buffer for capture annex (PLAT-RT-TAC5)."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any

from rt_sandbox.authority_labels import AUTHORITY_REPLAY_BOUNDARY
from rt_sandbox.tactical_state import ASSIGNMENT_LOCK_DURATION_S

DEFAULT_MAX_TIMELINE = 256
DEFAULT_MAX_TTI_SAMPLES = 64
TTI_SAMPLE_MIN_INTERVAL_S = 0.5

TACTICAL_ANNEX_GOVERNANCE_BANNER = (
    "TACTICAL CAPTURE ANNEX — explanatory sandbox record; not SA replay authority"
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _append_bounded(lst: list[dict[str, Any]], entry: dict[str, Any], cap: int) -> None:
    lst.append(entry)
    if len(lst) > cap:
        del lst[: len(lst) - cap]


@dataclass
class TacticalCaptureBuffer:
    """Append-only per-session timelines frozen at capture_session."""

    max_timeline: int = DEFAULT_MAX_TIMELINE
    max_tti_samples: int = DEFAULT_MAX_TTI_SAMPLES
    selected_timeline: list[dict[str, Any]] = field(default_factory=list)
    assignment_timeline: list[dict[str, Any]] = field(default_factory=list)
    tti_timeline: list[dict[str, Any]] = field(default_factory=list)
    recommendation_timeline: list[dict[str, Any]] = field(default_factory=list)
    mode_switches: list[dict[str, Any]] = field(default_factory=list)
    pause_resume_transitions: list[dict[str, Any]] = field(default_factory=list)
    assignment_lock_events: list[dict[str, Any]] = field(default_factory=list)
    target_switch_events: list[dict[str, Any]] = field(default_factory=list)
    _last_selected_interceptor: str | None = None
    _last_selected_target: str | None = None
    _last_assigned_interceptor: str | None = None
    _last_assigned_target: str | None = None
    _last_tti_sample_monotonic: float | None = None

    def reset(self) -> None:
        self.selected_timeline.clear()
        self.assignment_timeline.clear()
        self.tti_timeline.clear()
        self.recommendation_timeline.clear()
        self.mode_switches.clear()
        self.pause_resume_transitions.clear()
        self.assignment_lock_events.clear()
        self.target_switch_events.clear()
        self._last_selected_interceptor = None
        self._last_selected_target = None
        self._last_assigned_interceptor = None
        self._last_assigned_target = None
        self._last_tti_sample_monotonic = None

    def is_empty(self) -> bool:
        return not any(
            (
                self.selected_timeline,
                self.assignment_timeline,
                self.tti_timeline,
                self.recommendation_timeline,
                self.mode_switches,
                self.pause_resume_transitions,
                self.assignment_lock_events,
                self.target_switch_events,
            )
        )

    def record_mode_switch(
        self,
        *,
        from_mode: str,
        to_mode: str,
        initiator: str,
        reason: str,
    ) -> None:
        _append_bounded(
            self.mode_switches,
            {
                "t_utc": _utc_now(),
                "from_mode": from_mode,
                "to_mode": to_mode,
                "initiator": initiator,
                "reason": reason,
                "authority_label": AUTHORITY_REPLAY_BOUNDARY,
            },
            self.max_timeline,
        )

    def record_pause_resume(self, *, action: str, initiator: str) -> None:
        _append_bounded(
            self.pause_resume_transitions,
            {
                "t_utc": _utc_now(),
                "action": action,
                "initiator": initiator,
                "authority_label": AUTHORITY_REPLAY_BOUNDARY,
            },
            self.max_timeline,
        )

    def record_selection(
        self,
        *,
        candidate_id: str | None,
        role: str,
        source: str,
    ) -> None:
        _append_bounded(
            self.selected_timeline,
            {
                "t_utc": _utc_now(),
                "candidate_id": candidate_id,
                "role": role,
                "source": source,
                "authority_label": AUTHORITY_REPLAY_BOUNDARY,
            },
            self.max_timeline,
        )
        if role == "interceptor":
            new_i, new_t = candidate_id, self._last_selected_target
        else:
            new_i, new_t = self._last_selected_interceptor, candidate_id
        self._maybe_target_switch(
            selected_interceptor_id=new_i,
            selected_target_id=new_t,
            assigned_interceptor_id=self._last_assigned_interceptor,
            assigned_target_id=self._last_assigned_target,
            reason="selection_change",
        )
        if role == "interceptor":
            self._last_selected_interceptor = candidate_id
        else:
            self._last_selected_target = candidate_id

    def record_assignment(
        self,
        *,
        assigned_interceptor_id: str | None,
        assigned_target_id: str | None,
        previous_interceptor_id: str | None,
        previous_target_id: str | None,
        reason: str,
        authority_label: str = AUTHORITY_REPLAY_BOUNDARY,
    ) -> None:
        assigned_candidate_id = assigned_interceptor_id
        _append_bounded(
            self.assignment_timeline,
            {
                "t_utc": _utc_now(),
                "assigned_candidate_id": assigned_candidate_id,
                "assigned_interceptor_id": assigned_interceptor_id,
                "assigned_target_id": assigned_target_id,
                "previous_id": previous_interceptor_id,
                "previous_interceptor_id": previous_interceptor_id,
                "previous_target_id": previous_target_id,
                "reason": reason,
                "authority_label": authority_label,
            },
            self.max_timeline,
        )
        self._maybe_target_switch(
            selected_interceptor_id=self._last_selected_interceptor,
            selected_target_id=self._last_selected_target,
            assigned_interceptor_id=assigned_interceptor_id,
            assigned_target_id=assigned_target_id,
            reason=reason,
        )
        self._last_assigned_interceptor = assigned_interceptor_id
        self._last_assigned_target = assigned_target_id

    def record_recommendation(
        self,
        *,
        event: str,
        recommendation_id: str | None = None,
        detail: dict[str, Any] | None = None,
    ) -> None:
        entry: dict[str, Any] = {
            "t_utc": _utc_now(),
            "event": event,
            "recommendation_id": recommendation_id,
            "authority_label": AUTHORITY_REPLAY_BOUNDARY,
        }
        if detail:
            entry["detail"] = detail
        _append_bounded(self.recommendation_timeline, entry, self.max_timeline)

    def record_assignment_lock(
        self,
        *,
        assigned_candidate_id: str,
        duration_s: float = ASSIGNMENT_LOCK_DURATION_S,
        released_at_utc: str | None = None,
    ) -> None:
        _append_bounded(
            self.assignment_lock_events,
            {
                "t_utc": _utc_now(),
                "duration_s": duration_s,
                "assigned_candidate_id": assigned_candidate_id,
                "released_at_utc": released_at_utc,
                "authority_label": AUTHORITY_REPLAY_BOUNDARY,
            },
            self.max_timeline,
        )

    def record_tti_sample(
        self,
        *,
        candidate_id: str,
        tti_s: float | None,
        feasible: bool,
        reason: str,
        force: bool = False,
    ) -> None:
        now = time.monotonic()
        if not force and self._last_tti_sample_monotonic is not None:
            if now - self._last_tti_sample_monotonic < TTI_SAMPLE_MIN_INTERVAL_S:
                return
        self._last_tti_sample_monotonic = now
        _append_bounded(
            self.tti_timeline,
            {
                "t_utc": _utc_now(),
                "candidate_id": candidate_id,
                "tti_s": tti_s,
                "feasible": feasible,
                "reason": reason,
                "authority_label": AUTHORITY_REPLAY_BOUNDARY,
            },
            self.max_tti_samples,
        )

    def _maybe_target_switch(
        self,
        *,
        selected_interceptor_id: str | None,
        selected_target_id: str | None,
        assigned_interceptor_id: str | None,
        assigned_target_id: str | None,
        reason: str,
    ) -> None:
        prev_sel = (self._last_selected_interceptor, self._last_selected_target)
        new_sel = (selected_interceptor_id, selected_target_id)
        prev_asg = (self._last_assigned_interceptor, self._last_assigned_target)
        new_asg = (assigned_interceptor_id, assigned_target_id)
        if new_sel != prev_sel and (prev_sel != (None, None) or new_sel != (None, None)):
            if new_sel != prev_sel:
                _append_bounded(
                    self.target_switch_events,
                    {
                        "t_utc": _utc_now(),
                        "switch_kind": "selected_pair",
                        "interceptor_id": selected_interceptor_id,
                        "target_id": selected_target_id,
                        "previous_interceptor_id": self._last_selected_interceptor,
                        "previous_target_id": self._last_selected_target,
                        "reason": reason,
                        "authority_label": AUTHORITY_REPLAY_BOUNDARY,
                    },
                    self.max_timeline,
                )
        if new_asg != prev_asg and (prev_asg != (None, None) or new_asg != (None, None)):
            if new_asg != prev_asg:
                _append_bounded(
                    self.target_switch_events,
                    {
                        "t_utc": _utc_now(),
                        "switch_kind": "assigned_pair",
                        "interceptor_id": assigned_interceptor_id,
                        "target_id": assigned_target_id,
                        "previous_interceptor_id": self._last_assigned_interceptor,
                        "previous_target_id": self._last_assigned_target,
                        "reason": reason,
                        "authority_label": AUTHORITY_REPLAY_BOUNDARY,
                    },
                    self.max_timeline,
                )

    def summary_counts(self) -> dict[str, int]:
        return {
            "selected_timeline": len(self.selected_timeline),
            "assignment_timeline": len(self.assignment_timeline),
            "tti_timeline": len(self.tti_timeline),
            "recommendation_timeline": len(self.recommendation_timeline),
            "mode_switches": len(self.mode_switches),
            "pause_resume_transitions": len(self.pause_resume_transitions),
            "assignment_lock_events": len(self.assignment_lock_events),
            "target_switch_events": len(self.target_switch_events),
        }
