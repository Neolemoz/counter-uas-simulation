"""Capture pose cognition assessment (PLAT-RT-R2e).

See docs/evaluation/rt_capture_pose_cognition_v1.md
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any

from rt_sandbox.pose_sync import pose_drift_m

COGNITION_BANNER = (
    "CAPTURE POSE COGNITION — explanatory flags; command_pose is authoritative"
)

AUTHORITY_FIELD = "command_pose"


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


@dataclass
class CapturePoseAssessment:
    per_entity: list[dict[str, Any]] = field(default_factory=list)
    session_flags: list[str] = field(default_factory=list)
    sync_health: str | None = None
    adapter_attached: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "per_entity": list(self.per_entity),
            "session_flags": list(self.session_flags),
            "sync_health": self.sync_health,
            "adapter_attached": self.adapter_attached,
        }


def _poses_diverge(
    command_pose: dict[str, Any] | None,
    other: dict[str, Any] | None,
    *,
    threshold_m: float = 0.001,
) -> bool:
    if not command_pose or not other:
        return False
    try:
        return pose_drift_m(command_pose, other) > threshold_m
    except (KeyError, TypeError, ValueError):
        return False


def assess_capture_pose_cognition(
    entity_pose_history: list[dict[str, Any]],
    pose_sync_summary: dict[str, Any] | None,
    *,
    adapter_attached: bool,
) -> CapturePoseAssessment:
    sync_health = None
    if pose_sync_summary:
        sync_health = str(pose_sync_summary.get("sync_health") or "ok")

    session_flags: list[str] = []
    if adapter_attached and sync_health and sync_health != "ok":
        session_flags.append(f"sync_{sync_health}")

    feedback_ids = set()
    if pose_sync_summary:
        for ent in pose_sync_summary.get("feedback_entities") or []:
            eid = str(ent.get("entity_id", ""))
            if eid:
                feedback_ids.add(eid)

    per_entity: list[dict[str, Any]] = []
    for record in entity_pose_history:
        eid = str(record.get("entity_id", ""))
        if not eid:
            continue
        flags: list[str] = []
        command_pose = record.get("command_pose")
        feedback_pose = record.get("feedback_pose")
        telemetry_pose = record.get("telemetry_mirror_pose")

        if adapter_attached:
            if feedback_pose is None:
                flags.append("partial_feedback")
            elif record.get("drift_m") is not None:
                flags.append("feedback_drift_recorded")
            if eid not in feedback_ids and feedback_pose is None:
                flags.append("missing_sync_feedback")

        if telemetry_pose is not None and feedback_pose is None and adapter_attached:
            flags.append("telemetry_without_sync")

        if _poses_diverge(command_pose, telemetry_pose):
            flags.append("telemetry_divergence")

        per_entity.append(
            {
                "entity_id": eid,
                "authoritative_field": AUTHORITY_FIELD,
                "authority_label": record.get("authority_label"),
                "flags": flags,
            }
        )

    if adapter_attached:
        if any("partial_feedback" in e.get("flags", []) for e in per_entity):
            if "partial_feedback" not in session_flags:
                session_flags.append("partial_feedback")

    return CapturePoseAssessment(
        per_entity=per_entity,
        session_flags=session_flags,
        sync_health=sync_health,
        adapter_attached=adapter_attached,
    )


def cognition_to_manifest_block(assessment: CapturePoseAssessment) -> dict[str, Any]:
    return {
        "assessment_utc": _utc_now(),
        "adapter_attached": assessment.adapter_attached,
        "sync_health": assessment.sync_health,
        "session_flags": list(assessment.session_flags),
        "per_entity": list(assessment.per_entity),
        "governance_banner": COGNITION_BANNER,
    }


def export_pose_normalized_detail(
    assessment: CapturePoseAssessment,
    *,
    conversion_revision: int,
    input_content_hash: str,
) -> dict[str, Any]:
    return {
        "conversion_revision": conversion_revision,
        "input_content_hash": input_content_hash,
        "sync_health": assessment.sync_health,
        "session_flags": list(assessment.session_flags),
        "entity_count": len(assessment.per_entity),
        "ambiguous_entity_count": sum(
            1 for e in assessment.per_entity if e.get("flags")
        ),
    }


def append_capture_pose_audits(
    audit: Any,
    session_id: str,
    assessment: CapturePoseAssessment,
    *,
    capture_candidate_id: str,
    command_id: str | None,
    issued_by: str,
) -> None:
    base_detail: dict[str, Any] = {
        "capture_candidate_id": capture_candidate_id,
        "adapter_attached": assessment.adapter_attached,
        "sync_health": assessment.sync_health,
        "entity_count": len(assessment.per_entity),
    }

    audit.append(
        session_id,
        command_id=command_id,
        command_type="capture_pose_authority",
        issued_by=issued_by,
        result="OK",
        detail={
            **base_detail,
            "authoritative_field": AUTHORITY_FIELD,
            "per_entity": assessment.per_entity,
        },
    )

    if not assessment.adapter_attached:
        return

    stale_entities = [
        e["entity_id"]
        for e in assessment.per_entity
        if "partial_feedback" in (e.get("flags") or [])
        or "missing_sync_feedback" in (e.get("flags") or [])
    ]
    if assessment.sync_health in {"stale", "feedback_lost"} or stale_entities:
        audit.append(
            session_id,
            command_id=command_id,
            command_type="capture_pose_stale",
            issued_by=issued_by,
            result="OK",
            detail={
                **base_detail,
                "session_flags": assessment.session_flags,
                "entity_ids": stale_entities,
            },
        )

    if assessment.sync_health == "mismatch" or any(
        "feedback_drift_recorded" in (e.get("flags") or []) for e in assessment.per_entity
    ):
        mismatch_entities = [
            e["entity_id"]
            for e in assessment.per_entity
            if assessment.sync_health == "mismatch"
            or "feedback_drift_recorded" in (e.get("flags") or [])
        ]
        if assessment.sync_health == "mismatch" or mismatch_entities:
            audit.append(
                session_id,
                command_id=command_id,
                command_type="capture_pose_mismatch",
                issued_by=issued_by,
                result="OK",
                detail={
                    **base_detail,
                    "entity_ids": mismatch_entities,
                },
            )
