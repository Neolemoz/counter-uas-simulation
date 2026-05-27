"""Assisted recommendation ranking (PLAT-RT-TAC3)."""

from __future__ import annotations

import uuid
from datetime import datetime, timedelta, timezone
from typing import TYPE_CHECKING

from rt_sandbox.tactical_geometry import compute_intercept
from rt_sandbox.tactical_state import (
    INTERCEPTOR_ENTITY_TYPE,
    TARGET_ENTITY_TYPES,
    TacticalRecommendation,
)

if TYPE_CHECKING:
    from rt_sandbox.session_record import SessionRecord

RECOMMENDATION_TTL_S = 30.0
RECOMMENDATION_GOVERNANCE_BANNER = (
    "RECOMMENDATION — requires user approval; not assignment authority"
)


def _expires_at() -> str:
    return (
        datetime.now(timezone.utc) + timedelta(seconds=RECOMMENDATION_TTL_S)
    ).replace(microsecond=0).isoformat()


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def rank_recommendation(
    session: SessionRecord,
    *,
    speed_cap_m_s: float,
    hint_interceptor_id: str | None = None,
    hint_target_id: str | None = None,
) -> TacticalRecommendation:
    rec_id = str(uuid.uuid4())
    if session.world is None:
        return TacticalRecommendation(
            recommendation_id=rec_id,
            recommended_interceptor_id=None,
            recommended_target_id=None,
            tti_s=None,
            feasibility={"feasible": False, "reason": "world_not_ready"},
            explanation="World not ready for recommendation ranking",
            expires_at_utc=_expires_at(),
            pairs_evaluated=0,
        )

    registry = session.world.registry
    interceptors = [
        e
        for e in registry.all_entities()
        if e.entity_type == INTERCEPTOR_ENTITY_TYPE
    ]
    targets = [
        e for e in registry.all_entities() if e.entity_type in TARGET_ENTITY_TYPES
    ]

    best_tti: float | None = None
    best_iid: str | None = None
    best_tid: str | None = None
    pairs_evaluated = 0

    for i_rec in interceptors:
        if hint_interceptor_id and i_rec.entity_id != hint_interceptor_id:
            continue
        for t_rec in targets:
            if hint_target_id and t_rec.entity_id != hint_target_id:
                continue
            pairs_evaluated += 1
            ip = i_rec.pose
            tp = t_rec.pose
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
                speed_cap_m_s,
            )
            if result is None:
                continue
            tti = result[0]
            if best_tti is None:
                best_tti, best_iid, best_tid = tti, i_rec.entity_id, t_rec.entity_id
                continue
            if tti < best_tti - 1e-9:
                best_tti, best_iid, best_tid = tti, i_rec.entity_id, t_rec.entity_id
            elif abs(tti - best_tti) <= 1e-9:
                candidate = (i_rec.entity_id, t_rec.entity_id)
                current = (best_iid or "", best_tid or "")
                if candidate < current:
                    best_tti, best_iid, best_tid = tti, i_rec.entity_id, t_rec.entity_id

    if best_tti is None or best_iid is None or best_tid is None:
        return TacticalRecommendation(
            recommendation_id=rec_id,
            recommended_interceptor_id=None,
            recommended_target_id=None,
            tti_s=None,
            feasibility={
                "feasible": False,
                "reason": "no_intercept_solution_in_window",
            },
            explanation=(
                f"No feasible interceptor–target pair among {pairs_evaluated} evaluated"
            ),
            expires_at_utc=_expires_at(),
            pairs_evaluated=pairs_evaluated,
        )

    return TacticalRecommendation(
        recommendation_id=rec_id,
        recommended_interceptor_id=best_iid,
        recommended_target_id=best_tid,
        tti_s=best_tti,
        feasibility={"feasible": True, "reason": "feasible"},
        explanation=(
            f"Lowest cap-speed TTI among {pairs_evaluated} pair"
            f"{'s' if pairs_evaluated != 1 else ''} evaluated"
        ),
        expires_at_utc=_expires_at(),
        pairs_evaluated=pairs_evaluated,
    )


def cleared_recommendation(session_id: str) -> TacticalRecommendation:
    return TacticalRecommendation(
        recommendation_id="",
        recommended_interceptor_id=None,
        recommended_target_id=None,
        tti_s=None,
        feasibility={"feasible": False, "reason": "no_pending_recommendation"},
        explanation="No pending recommendation",
        expires_at_utc=_utc_now(),
        pairs_evaluated=0,
    )
