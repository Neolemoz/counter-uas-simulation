"""Autonomous tactical loop tick (PLAT-RT-TAC4)."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any, Callable

from rt_sandbox.lifecycle import SessionState
from rt_sandbox.tactical_recommendation import rank_recommendation
from rt_sandbox.tactical_state import (
    AUTONOMOUS_LOOP_STATUS_RUNNING,
    AUTONOMOUS_TICK_INTERVAL_S,
    ASSIGNMENT_LOCK_DURATION_S,
    SWITCH_DWELL_DURATION_S,
    SWITCH_TTI_IMPROVEMENT_MARGIN_S,
    TACTICAL_MODE_AUTONOMOUS,
)

if TYPE_CHECKING:
    from rt_sandbox.audit_log import AuditLog
    from rt_sandbox.session_record import SessionRecord
    from rt_sandbox.tactical_controller import TacticalController




def _coordinated_pairs(
    ranked_pairs: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], bool, str]:
    target_ids = {str(pair["target_id"]) for pair in ranked_pairs}
    allow_duplicate_targets = len(target_ids) <= 1
    used_interceptors: set[str] = set()
    used_targets: set[str] = set()
    selected: list[dict[str, Any]] = []
    duplicate_blocked = False

    for pair in ranked_pairs:
        interceptor_id = str(pair["interceptor_id"])
        target_id = str(pair["target_id"])
        if interceptor_id in used_interceptors:
            continue
        if not allow_duplicate_targets and target_id in used_targets:
            duplicate_blocked = True
            continue
        selected.append(pair)
        used_interceptors.add(interceptor_id)
        used_targets.add(target_id)

    if allow_duplicate_targets and selected:
        return selected, duplicate_blocked, "single_target_fallback"
    if selected:
        return selected, duplicate_blocked, "one_to_one"
    return selected, duplicate_blocked, "no_feasible_pair"

def tick_autonomous_session(
    session: SessionRecord,
    tactical: TacticalController,
    now: float,
    *,
    audit: AuditLog,
    publish_channel: Callable[[str], None],
    apply_move: Callable[[str, dict[str, float]], str | None],
) -> bool:
    """Run one autonomous cycle if due. Returns True if a commit+move ran."""
    state = tactical.state
    if state.mode != TACTICAL_MODE_AUTONOMOUS:
        return False
    if state.autonomous_loop_status != AUTONOMOUS_LOOP_STATUS_RUNNING:
        return False
    if session.state != SessionState.RUNNING:
        return False
    if not session.runtime.is_alive():
        return False

    if state.last_autonomous_tick_monotonic is not None:
        elapsed = now - state.last_autonomous_tick_monotonic
        if elapsed < AUTONOMOUS_TICK_INTERVAL_S:
            return False

    state.last_autonomous_tick_monotonic = now

    if state.assignment_lock_active(now):
        state.switch_blocked_reason = "assignment_lock_active"
        state.candidate_tti_delta_s = None
        state.coordination_state = "assignment_lock_active"
        state.tactical_health = {
            "feasible": state.tactical_health.get("feasible", False),
            "summary": "assignment_lock_active",
            "stale": False,
        }
        publish_channel("tactical_state")
        return False

    rec = rank_recommendation(
        session,
        speed_cap_m_s=state.interceptor_speed_cap_m_s,
    )
    if not rec.feasibility.get("feasible"):
        state.tti_s = None
        state.switch_blocked_reason = None
        state.candidate_tti_delta_s = None
        state.tactical_health = {
            "feasible": False,
            "summary": str(rec.feasibility.get("reason", "infeasible")),
            "stale": False,
        }
        publish_channel("tactical_state")
        return False

    ranked_pairs = list(rec.ranked_pairs)
    selected_pairs, duplicate_blocked, coordination_state = _coordinated_pairs(
        ranked_pairs
    )
    state.duplicate_target_blocked = duplicate_blocked
    state.coordination_state = coordination_state
    if not selected_pairs:
        publish_channel("tactical_state")
        return False

    primary = selected_pairs[0]
    iid = str(primary["interceptor_id"])
    tid = str(primary["target_id"])
    current_tid = state.assigned_target_id
    switching_target = current_tid is not None and current_tid != tid
    if switching_target:
        current_tti = state.tti_s
        candidate_tti = float(primary["tti_s"])
        if current_tti is not None:
            state.candidate_tti_delta_s = current_tti - candidate_tti
        else:
            state.candidate_tti_delta_s = None

        if state.last_assignment_monotonic is not None:
            dwell_s = now - state.last_assignment_monotonic
            if dwell_s < SWITCH_DWELL_DURATION_S:
                state.switch_blocked_reason = "dwell_active"
                state.tactical_health = {
                    "feasible": True,
                    "summary": "switch_blocked_dwell_active",
                    "stale": False,
                }
                publish_channel("tactical_state")
                return False

        if (
            state.candidate_tti_delta_s is not None
            and state.candidate_tti_delta_s < SWITCH_TTI_IMPROVEMENT_MARGIN_S
        ):
            state.switch_blocked_reason = "tti_margin"
            state.tactical_health = {
                "feasible": True,
                "summary": "switch_blocked_tti_margin",
                "stale": False,
            }
            publish_channel("tactical_state")
            return False
    else:
        state.candidate_tti_delta_s = 0.0 if current_tid == tid else None

    state.switch_blocked_reason = None
    committed_pairs: list[dict[str, str]] = []
    for pair in selected_pairs:
        pair_iid = str(pair["interceptor_id"])
        pair_tid = str(pair["target_id"])
        state.selected_interceptor_id = pair_iid
        state.selected_target_id = pair_tid
        err, pose = tactical._commit_assignment_on_session(
            session, pair_iid, pair_tid, assignment_reason="autonomous_commit"
        )
        if err or pose is None:
            if err == "TACTICAL_INFEASIBLE":
                state.tactical_health = {
                    "feasible": False,
                    "summary": "no_intercept_solution_in_window",
                    "stale": False,
                }
            continue

        move_err = apply_move(pair_iid, pose)
        if move_err:
            continue
        committed_pairs.append({"interceptor_id": pair_iid, "target_id": pair_tid})

    if not committed_pairs:
        publish_channel("tactical_state")
        return False

    state.assigned_pairs = [dict(pair) for pair in committed_pairs]
    state.last_assignment_monotonic = now
    state.assignment_lock_until_monotonic = now + ASSIGNMENT_LOCK_DURATION_S
    tactical.capture_buffer.record_assignment_lock(
        assigned_candidate_id=committed_pairs[-1]["interceptor_id"],
        duration_s=ASSIGNMENT_LOCK_DURATION_S,
    )
    audit.append(
        session.session_id,
        command_id=None,
        command_type="tactical_assignment_committed",
        issued_by="bridge_autonomous_tick",
        result="OK",
        detail={
            "reason": "autonomous_commit",
            "assigned_interceptor_id": state.assigned_interceptor_id,
            "assigned_target_id": state.assigned_target_id,
            "assigned_pairs": [dict(pair) for pair in committed_pairs],
            "coordination_state": state.coordination_state,
            "duplicate_target_blocked": state.duplicate_target_blocked,
            "tti_s": state.tti_s,
        },
    )
    publish_channel("tactical_state")
    return True


def monotonic_now() -> float:
    return time.monotonic()
