"""Capture fidelity coupling block (PLAT-RT-F5b P0).

See docs/evaluation/rt_runtime_fidelity_coupling_v1.md
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any

from rt_sandbox.fidelity_coupling import (
    FIDELITY_BANNER,
    pose_truth_drift_m,
    sim_agl_by_entity,
    truth_pose_by_entity,
)

POSE_TRUTH_DRIFT_FLAG_M = 0.001


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


@dataclass
class FidelityPoseAssessment:
    enable_fidelity_coupling: bool = False
    adapter_attached: bool = False
    truth_timestamp_utc: str | None = None
    attestation_status: str | None = None
    per_entity: list[dict[str, Any]] = field(default_factory=list)
    session_flags: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "enable_fidelity_coupling": self.enable_fidelity_coupling,
            "adapter_attached": self.adapter_attached,
            "truth_timestamp_utc": self.truth_timestamp_utc,
            "attestation_status": self.attestation_status,
            "per_entity": list(self.per_entity),
            "session_flags": list(self.session_flags),
        }


def assess_fidelity_pose_block(
    entity_pose_history: list[dict[str, Any]],
    fidelity_truth: dict[str, Any] | None,
    *,
    enable_fidelity_coupling: bool,
    adapter_attached: bool,
    drift_threshold_m: float = POSE_TRUTH_DRIFT_FLAG_M,
) -> FidelityPoseAssessment | None:
    if not enable_fidelity_coupling or not adapter_attached:
        return None

    truth_poses = truth_pose_by_entity(fidelity_truth)
    agl_map = sim_agl_by_entity(fidelity_truth)
    truth_ts = None
    attestation = None
    if fidelity_truth:
        truth_ts = fidelity_truth.get("timestamp_utc")
        attestation = fidelity_truth.get("attestation_status")

    session_flags: list[str] = []
    if fidelity_truth is None:
        session_flags.append("fidelity_truth_unavailable")
    elif attestation == "stale":
        session_flags.append("fidelity_truth_stale")

    per_entity: list[dict[str, Any]] = []
    for record in entity_pose_history:
        eid = str(record.get("entity_id", ""))
        if not eid:
            continue
        command_pose = record.get("command_pose")
        truth_pose = truth_poses.get(eid)
        drift = pose_truth_drift_m(command_pose, truth_pose)
        flags: list[str] = []
        if truth_pose is None and fidelity_truth is not None:
            flags.append("partial_truth")
        if drift is not None and drift > drift_threshold_m:
            flags.append("pose_truth_drift")
        per_entity.append(
            {
                "entity_id": eid,
                "command_pose": command_pose,
                "truth_attested_pose": truth_pose,
                "pose_truth_drift_m": drift,
                "sim_agl_m": agl_map.get(eid),
                "truth_timestamp_utc": truth_ts,
                "flags": flags,
            }
        )

    if any("partial_truth" in (e.get("flags") or []) for e in per_entity):
        if "partial_truth" not in session_flags:
            session_flags.append("partial_truth")

    return FidelityPoseAssessment(
        enable_fidelity_coupling=True,
        adapter_attached=adapter_attached,
        truth_timestamp_utc=truth_ts,
        attestation_status=attestation,
        per_entity=per_entity,
        session_flags=session_flags,
    )


def fidelity_to_manifest_block(assessment: FidelityPoseAssessment) -> dict[str, Any]:
    return {
        "schema": "rt_fidelity_pose_block_v1",
        "governance_banner": FIDELITY_BANNER,
        "enable_fidelity_coupling": assessment.enable_fidelity_coupling,
        "adapter_attached": assessment.adapter_attached,
        "truth_timestamp_utc": assessment.truth_timestamp_utc,
        "attestation_status": assessment.attestation_status,
        "session_flags": list(assessment.session_flags),
        "per_entity": list(assessment.per_entity),
    }


def export_fidelity_capture_detail(
    assessment: FidelityPoseAssessment,
    *,
    conversion_revision: int,
    input_content_hash: str,
) -> dict[str, Any]:
    return {
        "conversion_revision": conversion_revision,
        "input_content_hash": input_content_hash,
        "attestation_status": assessment.attestation_status,
        "session_flags": list(assessment.session_flags),
        "entity_count": len(assessment.per_entity),
        "truth_entity_count": sum(
            1 for e in assessment.per_entity if e.get("truth_attested_pose")
        ),
    }


def append_fidelity_capture_audits(
    audit: Any,
    session_id: str,
    assessment: FidelityPoseAssessment,
    *,
    capture_candidate_id: str,
    command_id: str | None,
    issued_by: str,
) -> None:
    detail: dict[str, Any] = {
        "capture_candidate_id": capture_candidate_id,
        "attestation_status": assessment.attestation_status,
        "truth_timestamp_utc": assessment.truth_timestamp_utc,
        "entity_count": len(assessment.per_entity),
        "session_flags": list(assessment.session_flags),
    }
    audit.append(
        session_id,
        command_id=command_id,
        command_type="fidelity_capture_snapshot",
        issued_by=issued_by,
        result="OK",
        detail={**detail, "per_entity": assessment.per_entity},
    )
