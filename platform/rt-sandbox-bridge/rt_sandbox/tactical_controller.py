"""RT sandbox tactical controller — manual, assisted, autonomous (TAC2–TAC4)."""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING, Any

from rt_sandbox.entity_registry import EntityRecord
from rt_sandbox.tactical_geometry import compute_intercept
from rt_sandbox.tactical_recommendation import (
    cleared_recommendation,
    rank_recommendation,
)

if TYPE_CHECKING:
    from rt_sandbox.session_record import SessionRecord
from rt_sandbox.tactical_capture_buffer import TacticalCaptureBuffer
from rt_sandbox.tactical_state import (
    ALLOWED_TACTICAL_MODES_TAC4,
    AUTONOMOUS_LOOP_STATUS_PAUSED,
    AUTONOMOUS_LOOP_STATUS_RUNNING,
    INTERCEPTOR_ENTITY_TYPE,
    TACTICAL_MODE_ASSISTED,
    TACTICAL_MODE_AUTONOMOUS,
    TACTICAL_MODE_MANUAL,
    TARGET_ENTITY_TYPES,
    TacticalState,
)


class TacticalController:
    def __init__(self, session_id: str) -> None:
        self.session_id = session_id
        self.state = TacticalState()
        self.capture_buffer = TacticalCaptureBuffer()

    def reset(self) -> None:
        self.state.reset()
        self.capture_buffer.reset()

    def set_mode(self, mode: str) -> str | None:
        if mode not in ALLOWED_TACTICAL_MODES_TAC4:
            return "COMMAND_FORBIDDEN"
        prev = self.state.mode
        if mode == TACTICAL_MODE_AUTONOMOUS and prev == TACTICAL_MODE_ASSISTED:
            return "COMMAND_FORBIDDEN"
        if mode != TACTICAL_MODE_ASSISTED:
            self.state.clear_pending_recommendation()
        if mode == TACTICAL_MODE_AUTONOMOUS:
            self.state.autonomous_loop_status = AUTONOMOUS_LOOP_STATUS_PAUSED
            self.state.assignment_lock_until_monotonic = None
            self.state.last_autonomous_tick_monotonic = None
        elif prev == TACTICAL_MODE_AUTONOMOUS:
            self.state.clear_autonomous_scheduler()
        self.capture_buffer.record_mode_switch(
            from_mode=prev,
            to_mode=mode,
            initiator="bridge",
            reason="set_tactical_mode",
        )
        self.state.mode = mode
        return None

    def pause_autonomous_loop(self) -> str | None:
        if self.state.mode != TACTICAL_MODE_AUTONOMOUS:
            return "COMMAND_FORBIDDEN"
        self.state.autonomous_loop_status = AUTONOMOUS_LOOP_STATUS_PAUSED
        return None

    def resume_autonomous_loop(self) -> str | None:
        if self.state.mode != TACTICAL_MODE_AUTONOMOUS:
            return "COMMAND_FORBIDDEN"
        self.state.autonomous_loop_status = AUTONOMOUS_LOOP_STATUS_RUNNING
        self.state.last_autonomous_tick_monotonic = None
        return None

    def clear_assignment_on_session(self, session: SessionRecord) -> None:
        prev_i = self.state.assigned_interceptor_id
        prev_t = self.state.assigned_target_id
        self.state.assigned_interceptor_id = None
        self.state.assigned_target_id = None
        self.state.last_intercept_pose = None
        self.capture_buffer.record_assignment(
            assigned_interceptor_id=None,
            assigned_target_id=None,
            previous_interceptor_id=prev_i,
            previous_target_id=prev_t,
            reason="clear",
        )
        self._refresh_explanatory_for(session)

    def snapshot_dict(self) -> dict[str, Any]:
        s = self.state
        now = time.monotonic()
        return {
            "schema": "rt_tactical_state_v1",
            "session_id": self.session_id,
            "tactical_mode": s.mode,
            "selected_interceptor_id": s.selected_interceptor_id,
            "selected_target_id": s.selected_target_id,
            "selected_candidate_id": s.selected_interceptor_id,
            "assigned_interceptor_id": s.assigned_interceptor_id,
            "assigned_target_id": s.assigned_target_id,
            "assigned_candidate_id": s.assigned_interceptor_id,
            "tti_s": s.tti_s,
            "tactical_health": dict(s.tactical_health),
            "assignment_lock_active": s.assignment_lock_active(now),
            "autonomous_loop_status": (
                s.autonomous_loop_status
                if s.mode == TACTICAL_MODE_AUTONOMOUS
                else None
            ),
            "pending_recommendation_id": s.pending_recommendation_id,
            "last_intercept_pose": (
                dict(s.last_intercept_pose) if s.last_intercept_pose else None
            ),
            "interceptor_speed_cap_m_s": s.interceptor_speed_cap_m_s,
        }

    def recommendation_dict(self) -> dict[str, Any] | None:
        rec = self.state.pending_recommendation
        if rec is None:
            return None
        return rec.to_dict()

    def _registry(self, session: SessionRecord):
        if session.world is None:
            return None
        return session.world.registry

    def _entity_record_for(
        self, session: SessionRecord, entity_id: str
    ) -> tuple[EntityRecord | None, str | None]:
        registry = self._registry(session)
        if registry is None:
            return None, "WORLD_NOT_READY"
        record = registry.get(entity_id)
        if record is None:
            return None, "ENTITY_NOT_FOUND"
        return record, None

    def select_candidate_on_session(
        self, session: SessionRecord, role: str, entity_id: str
    ) -> str | None:
        if self.state.mode == TACTICAL_MODE_AUTONOMOUS:
            return "COMMAND_FORBIDDEN"
        record, err = self._entity_record_for(session, entity_id)
        if err:
            return err
        assert record is not None
        if role == "interceptor":
            if record.entity_type != INTERCEPTOR_ENTITY_TYPE:
                return "INVALID_ENTITY_TYPE"
            self.state.selected_interceptor_id = entity_id
        elif role == "target":
            if record.entity_type not in TARGET_ENTITY_TYPES:
                return "INVALID_ENTITY_TYPE"
            self.state.selected_target_id = entity_id
        else:
            return "INVALID_PAYLOAD"
        self.capture_buffer.record_selection(
            candidate_id=entity_id,
            role=role,
            source="user",
        )
        self._refresh_explanatory_for(session)
        return None

    def request_recommendation_on_session(
        self,
        session: SessionRecord,
        *,
        hint_interceptor_id: str | None = None,
        hint_target_id: str | None = None,
    ) -> str | None:
        if self.state.mode != TACTICAL_MODE_ASSISTED:
            return "COMMAND_FORBIDDEN"
        rec = rank_recommendation(
            session,
            speed_cap_m_s=self.state.interceptor_speed_cap_m_s,
            hint_interceptor_id=hint_interceptor_id,
            hint_target_id=hint_target_id,
        )
        self.state.pending_recommendation = rec
        if rec.recommended_interceptor_id:
            self.state.selected_interceptor_id = rec.recommended_interceptor_id
        if rec.recommended_target_id:
            self.state.selected_target_id = rec.recommended_target_id
        if rec.feasibility.get("feasible"):
            self.state.tti_s = rec.tti_s
            self.state.tactical_health = {
                "feasible": True,
                "summary": "feasible",
                "stale": False,
            }
        else:
            self.state.tti_s = None
            self.state.tactical_health = {
                "feasible": False,
                "summary": str(rec.feasibility.get("reason", "infeasible")),
                "stale": False,
            }
        cid = rec.recommended_interceptor_id or "none"
        self.capture_buffer.record_recommendation(
            event="issued",
            recommendation_id=rec.recommendation_id,
            detail=rec.to_dict(),
        )
        self._record_tti_from_state(candidate_id=cid, reason="recommendation_rank")
        return None

    def approve_recommendation_on_session(
        self,
        session: SessionRecord,
        recommendation_id: str,
    ) -> tuple[str | None, dict[str, float] | None]:
        if self.state.mode != TACTICAL_MODE_ASSISTED:
            return "COMMAND_FORBIDDEN", None
        rec = self.state.pending_recommendation
        if rec is None or not rec.recommendation_id:
            return "INVALID_STATE", None
        if rec.recommendation_id != recommendation_id:
            return "INVALID_PAYLOAD", None
        if rec.is_expired():
            self.state.clear_pending_recommendation()
            return "INVALID_STATE", None
        if not rec.feasibility.get("feasible"):
            return "TACTICAL_INFEASIBLE", None
        iid = rec.recommended_interceptor_id
        tid = rec.recommended_target_id
        if not iid or not tid:
            return "TACTICAL_INFEASIBLE", None
        err, pose = self._commit_assignment_on_session(
            session, iid, tid, assignment_reason="approved_recommendation"
        )
        if err:
            return err, None
        self.capture_buffer.record_recommendation(
            event="approved",
            recommendation_id=rec.recommendation_id,
        )
        self.state.clear_pending_recommendation()
        return None, pose

    def reject_recommendation_on_session(
        self,
        recommendation_id: str | None = None,
    ) -> str | None:
        if self.state.mode != TACTICAL_MODE_ASSISTED:
            return "COMMAND_FORBIDDEN"
        rec = self.state.pending_recommendation
        if rec is None:
            return None
        if recommendation_id and rec.recommendation_id != recommendation_id:
            return "INVALID_PAYLOAD"
        self.capture_buffer.record_recommendation(
            event="rejected",
            recommendation_id=rec.recommendation_id,
        )
        self.state.clear_pending_recommendation()
        return None

    def assign_candidate_on_session(
        self,
        session: SessionRecord,
        interceptor_id: str | None,
        target_id: str | None,
    ) -> tuple[str | None, dict[str, float] | None]:
        if self.state.mode in (TACTICAL_MODE_ASSISTED, TACTICAL_MODE_AUTONOMOUS):
            return "COMMAND_FORBIDDEN", None
        iid = interceptor_id or self.state.selected_interceptor_id
        tid = target_id or self.state.selected_target_id
        if not iid or not tid:
            return "INVALID_PAYLOAD", None
        return self._commit_assignment_on_session(
            session, iid, tid, assignment_reason="user_assign"
        )

    def _commit_assignment_on_session(
        self,
        session: SessionRecord,
        interceptor_id: str,
        target_id: str,
        *,
        assignment_reason: str = "user_assign",
    ) -> tuple[str | None, dict[str, float] | None]:
        intercept, err = self._compute_intercept_on_session(session, interceptor_id, target_id)
        if err:
            return err, None
        assert intercept is not None
        tti, pose = intercept
        prev_i = self.state.assigned_interceptor_id
        prev_t = self.state.assigned_target_id
        self.state.assigned_interceptor_id = interceptor_id
        self.state.assigned_target_id = target_id
        self.state.selected_interceptor_id = interceptor_id
        self.state.selected_target_id = target_id
        self.state.tti_s = tti
        self.state.last_intercept_pose = pose
        self.state.tactical_health = {
            "feasible": True,
            "summary": "feasible",
            "stale": False,
        }
        self.capture_buffer.record_assignment(
            assigned_interceptor_id=interceptor_id,
            assigned_target_id=target_id,
            previous_interceptor_id=prev_i,
            previous_target_id=prev_t,
            reason=assignment_reason,
        )
        self.capture_buffer.record_tti_sample(
            candidate_id=interceptor_id,
            tti_s=tti,
            feasible=True,
            reason=assignment_reason,
            force=True,
        )
        return None, pose

    def _compute_intercept_on_session(
        self,
        session: SessionRecord,
        interceptor_id: str,
        target_id: str,
    ) -> tuple[tuple[float, dict[str, float]] | None, str | None]:
        i_rec, err = self._entity_record_for(session, interceptor_id)
        if err:
            return None, err
        t_rec, err = self._entity_record_for(session, target_id)
        if err:
            return None, err
        assert i_rec is not None and t_rec is not None
        return self._compute_from_records(i_rec, t_rec)

    def _compute_from_records(
        self,
        interceptor: EntityRecord,
        target: EntityRecord,
    ) -> tuple[tuple[float, dict[str, float]] | None, str | None]:
        ip = interceptor.pose
        tp = target.pose
        cap = self.state.interceptor_speed_cap_m_s
        result = compute_intercept(
            float(tp["x"]),
            float(tp["y"]),
            float(tp["z"]),
            0.0,
            0.0,
            0.0,
            float(ip["x"]),
            float(ip["y"]),
            float(ip["z"]),
            cap,
        )
        if result is None:
            self.state.tti_s = None
            self.state.tactical_health = {
                "feasible": False,
                "summary": "no_intercept_solution_in_window",
                "stale": False,
            }
            return None, "TACTICAL_INFEASIBLE"
        t, phx, phy, phz, ux, uy, uz = result
        yaw_deg = math.degrees(math.atan2(uy, ux))
        pose = {
            "x": phx,
            "y": phy,
            "z": phz,
            "yaw_deg": yaw_deg,
        }
        return (t, pose), None

    def _refresh_explanatory_for(self, session: SessionRecord) -> None:
        iid = self.state.selected_interceptor_id
        tid = self.state.selected_target_id
        if not iid or not tid:
            self.state.tti_s = None
            self.state.tactical_health = {
                "feasible": False,
                "summary": "no_selection" if not (iid or tid) else "incomplete_selection",
                "stale": False,
            }
            return
        intercept, err = self._compute_intercept_on_session(session, iid, tid)
        if err or intercept is None:
            return
        tti, _pose = intercept
        self.state.tti_s = tti
        self.state.tactical_health = {
            "feasible": True,
            "summary": "feasible",
            "stale": False,
        }
        self._record_tti_from_state(candidate_id=iid, reason="selection_refresh")

    def _record_tti_from_state(self, *, candidate_id: str, reason: str) -> None:
        health = self.state.tactical_health
        self.capture_buffer.record_tti_sample(
            candidate_id=candidate_id,
            tti_s=self.state.tti_s,
            feasible=bool(health.get("feasible")),
            reason=str(health.get("summary", reason)),
        )

    def cleared_recommendation_for_telemetry(self) -> dict[str, Any]:
        return cleared_recommendation(self.session_id).to_dict()
