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
    TACTICAL_MODE_AUTONOMOUS,
)

if TYPE_CHECKING:
    from rt_sandbox.audit_log import AuditLog
    from rt_sandbox.session_record import SessionRecord
    from rt_sandbox.tactical_controller import TacticalController


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
        state.tactical_health = {
            "feasible": False,
            "summary": str(rec.feasibility.get("reason", "infeasible")),
            "stale": False,
        }
        publish_channel("tactical_state")
        return False

    iid = rec.recommended_interceptor_id
    tid = rec.recommended_target_id
    if not iid or not tid:
        publish_channel("tactical_state")
        return False

    state.selected_interceptor_id = iid
    state.selected_target_id = tid
    err, pose = tactical._commit_assignment_on_session(
        session, iid, tid, assignment_reason="autonomous_commit"
    )
    if err or pose is None:
        if err == "TACTICAL_INFEASIBLE":
            state.tactical_health = {
                "feasible": False,
                "summary": "no_intercept_solution_in_window",
                "stale": False,
            }
        publish_channel("tactical_state")
        return False

    move_err = apply_move(iid, pose)
    if move_err:
        publish_channel("tactical_state")
        return False

    state.assignment_lock_until_monotonic = now + ASSIGNMENT_LOCK_DURATION_S
    tactical.capture_buffer.record_assignment_lock(
        assigned_candidate_id=iid,
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
            "assigned_interceptor_id": iid,
            "assigned_target_id": tid,
            "tti_s": state.tti_s,
        },
    )
    publish_channel("tactical_state")
    return True


def monotonic_now() -> float:
    return time.monotonic()
