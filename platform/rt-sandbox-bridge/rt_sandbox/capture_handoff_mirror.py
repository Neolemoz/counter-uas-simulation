"""Read-only capture/handoff staging mirror for UI (PLAT-RT-SA2)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from rt_sandbox.capture import list_staged_capture_ids
from rt_sandbox.capture_normalize import validate_normalized_capture
from rt_sandbox.isolation import repo_root_from, rt_sandbox_sa_handoff_dir
from rt_sandbox.sa_handoff import (
    capture_staging_dir,
    check_handoff_preconditions,
    handoff_status_summary,
)

ROW_GOVERNANCE_BANNER = "HANDOFF MIRROR — read-only; not SA replay authority"
RESPONSE_GOVERNANCE_BANNER = "HANDOFF MIRROR — read-only; not SA replay authority"
LINEAGE_NOTE = (
    "session_id is correlation only; SA lineage begins at rt_sa_import commit only"
)

WORKFLOW_PHASES = frozenset(
    {
        "none",
        "staged",
        "normalized",
        "review_pending",
        "ready",
        "deferred",
        "rejected",
        "prepared",
        "committed",
    }
)

MAX_VALIDATION_ERRORS = 8

RELEVANT_EXPORT_EVENTS = frozenset(
    {
        "handoff_ready",
        "handoff_reviewed",
        "handoff_rejected",
        "handoff_import_deferred",
        "handoff_import_prepared",
        "handoff_import_committed",
        "capture_approved",
        "conversion_manifest_written",
        "capture_normalized",
    }
)


def capture_belongs_to_session(candidate: dict[str, Any], session_id: str) -> bool:
    ref = candidate.get("ephemeral_session_ref") or candidate.get("session_id")
    return isinstance(ref, str) and ref == session_id


def derive_workflow_phase(
    *,
    summary: dict[str, Any],
    staging_dir: Path,
) -> str:
    if summary.get("import_record"):
        return "committed"

    candidate = summary.get("candidate") or {}
    review = summary.get("handoff_review") or {}
    decision = review.get("decision")
    approval = candidate.get("approval_status", "pending")
    norm_status = candidate.get("normalization_status", "pending")

    if decision == "rejected" or approval == "rejected":
        return "rejected"
    if decision == "deferred":
        return "deferred"
    if summary.get("handoff_manifest"):
        return "prepared"

    if norm_status == "normalized":
        pre = check_handoff_preconditions(staging_dir)
        if (
            not pre
            and approval == "approved"
            and decision == "ready_for_approval"
        ):
            return "ready"
        if decision == "ready_for_approval" and not pre:
            return "ready"
        return "review_pending"

    if summary.get("staging_exists") and candidate:
        return "staged" if norm_status != "normalized" else "normalized"

    return "none"


def _last_export_event_type(summary: dict[str, Any]) -> str | None:
    events = summary.get("export_events") or []
    for ev in reversed(events):
        et = ev.get("event_type")
        if isinstance(et, str) and et in RELEVANT_EXPORT_EVENTS:
            return et
    return None


def build_handoff_row_redacted(
    repo_root: Path,
    capture_id: str,
    *,
    session_id: str,
) -> dict[str, Any] | None:
    summary = handoff_status_summary(repo_root, capture_id)
    candidate = summary.get("candidate")
    if not isinstance(candidate, dict):
        return None
    if not capture_belongs_to_session(candidate, session_id):
        return None

    staging_dir = capture_staging_dir(repo_root, capture_id)
    val_errors = validate_normalized_capture(staging_dir)
    if len(val_errors) > MAX_VALIDATION_ERRORS:
        val_errors = val_errors[:MAX_VALIDATION_ERRORS]

    review = summary.get("handoff_review") or {}
    handoff_path = rt_sandbox_sa_handoff_dir(repo_root) / capture_id
    staging_refs = candidate.get("staging_refs") or {}
    has_tactical_annex = bool(staging_refs.get("tactical_annex_ref")) or (
        staging_dir / "tactical_annex.json"
    ).is_file()

    return {
        "schema": "rt_capture_handoff_row_v1",
        "capture_candidate_id": capture_id,
        "approval_status": candidate.get("approval_status", "pending"),
        "normalization_status": candidate.get("normalization_status", "pending"),
        "handoff_decision": review.get("decision"),
        "workflow_phase": derive_workflow_phase(summary=summary, staging_dir=staging_dir),
        "validation_ok": len(val_errors) == 0,
        "validation_errors": val_errors if val_errors else None,
        "has_handoff_manifest": (handoff_path / "handoff_manifest.json").is_file(),
        "has_tactical_annex": has_tactical_annex,
        "has_import_record": summary.get("import_record") is not None,
        "last_export_event_type": _last_export_event_type(summary),
        "source_origin": candidate.get("origin", "rt_sandbox_capture_v1"),
        "lineage_note": LINEAGE_NOTE,
        "governance_banner": ROW_GOVERNANCE_BANNER,
    }


def list_capture_handoff_for_session(
    repo_root: Path,
    session_id: str,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for capture_id in list_staged_capture_ids(repo_root):
        row = build_handoff_row_redacted(repo_root, capture_id, session_id=session_id)
        if row is not None:
            rows.append(row)
    rows.sort(key=lambda r: r["capture_candidate_id"])
    return rows


def list_capture_handoff_status_response(
    repo_root: Path | None,
    session_id: str,
) -> dict[str, Any]:
    root = repo_root or repo_root_from(Path.cwd())
    return {
        "session_id": session_id,
        "captures": list_capture_handoff_for_session(root, session_id),
        "governance_banner": RESPONSE_GOVERNANCE_BANNER,
    }
