"""Assisted recommendation ranking (PLAT-RT-TAC3)."""

from __future__ import annotations

import uuid
from datetime import datetime, timedelta, timezone
from typing import TYPE_CHECKING, Any

from rt_sandbox.tactical_geometry import compute_intercept, velocity_from_pose
from rt_sandbox.tactical_state import (
    INTERCEPTOR_ENTITY_TYPE,
    TARGET_ENTITY_TYPES,
    TacticalRecommendation,
)

if TYPE_CHECKING:
    from rt_sandbox.session_record import SessionRecord



def _pose_with_runtime_velocity(session: SessionRecord, entity: Any) -> dict[str, Any]:
    pose: dict[str, Any] = dict(entity.pose)
    mirror = getattr(session, "telemetry_mirror", None)
    if mirror is None:
        return pose
    entity_pose_mirror = getattr(mirror, "entity_pose_mirror", {}) or {}
    for entry in entity_pose_mirror.get("entities", []):
        if str(entry.get("entity_id") or "") != entity.entity_id:
            continue
        velocity = entry.get("velocity")
        if isinstance(velocity, dict):
            pose["velocity"] = dict(velocity)
        for key in ("vx", "vy", "vz", "speed_mps", "heading_deg"):
            if key in entry:
                pose[key] = entry[key]
        return pose
    return pose

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

    feasible_pairs: list[dict[str, Any]] = []
    pairs_evaluated = 0

    for i_rec in interceptors:
        if hint_interceptor_id and i_rec.entity_id != hint_interceptor_id:
            continue
        for t_rec in targets:
            if hint_target_id and t_rec.entity_id != hint_target_id:
                continue
            pairs_evaluated += 1
            ip = i_rec.pose
            tp = _pose_with_runtime_velocity(session, t_rec)
            tvx, tvy, tvz = velocity_from_pose(tp)
            result = compute_intercept(
                float(tp["x"]),
                float(tp["y"]),
                float(tp["z"]),
                tvx,
                tvy,
                tvz,
                float(ip["x"]),
                float(ip["y"]),
                float(ip["z"]),
                speed_cap_m_s,
            )
            if result is None:
                continue
            tti = result[0]
            feasible_pairs.append(
                {
                    "interceptor_id": i_rec.entity_id,
                    "target_id": t_rec.entity_id,
                    "candidate_id": i_rec.entity_id,
                    "tti_s": tti,
                }
            )

    feasible_pairs.sort(
        key=lambda pair: (
            float(pair["tti_s"]),
            str(pair["interceptor_id"]),
            str(pair["target_id"]),
        )
    )
    if not feasible_pairs:
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

    best = feasible_pairs[0]
    return TacticalRecommendation(
        recommendation_id=rec_id,
        recommended_interceptor_id=str(best["interceptor_id"]),
        recommended_target_id=str(best["target_id"]),
        tti_s=float(best["tti_s"]),
        feasibility={"feasible": True, "reason": "feasible"},
        explanation=(
            f"Lowest cap-speed TTI among {pairs_evaluated} pair"
            f"{'s' if pairs_evaluated != 1 else ''} evaluated"
        ),
        expires_at_utc=_expires_at(),
        pairs_evaluated=pairs_evaluated,
        ranked_pairs=feasible_pairs,
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
