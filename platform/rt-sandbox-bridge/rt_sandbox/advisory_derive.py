"""Read-only RT→SA workflow advisory derive (PLAT-RT-F6 P0)."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from rt_sandbox.capture_handoff_mirror import derive_workflow_phase
from rt_sandbox.sa_handoff import (
    capture_staging_dir,
    check_handoff_preconditions,
    handoff_status_summary,
    is_handoff_blocked,
)

ADVISORY_GOVERNANCE_BANNER = (
    "SA WORKFLOW ADVISORY — explanatory; maintainer CLIs are authority"
)

ADVISORY_STATES = frozenset(
    {
        "capture_ready",
        "review_complete",
        "approval_ready",
        "handoff_ready",
        "import_ready",
        "blocked",
    }
)

STATE_LABELS: dict[str, str] = {
    "capture_ready": "Capture ready (advisory)",
    "review_complete": "Review complete (advisory)",
    "approval_ready": "Approval ready (advisory)",
    "handoff_ready": "Advisory: handoff packaging ready",
    "import_ready": "Import ready (advisory)",
    "blocked": "Blocked (advisory)",
}


def _event_types(export_events: list[Any]) -> list[str]:
    out: list[str] = []
    for ev in export_events or []:
        if isinstance(ev, str):
            out.append(ev)
        elif isinstance(ev, dict):
            et = ev.get("event_type")
            if isinstance(et, str):
                out.append(et)
    return out


def _has_event(events: list[str], name: str) -> bool:
    return name in events


_REQUIRED_EXPORT_EVENTS = (
    "capture_validated",
    "capture_normalized",
    "export_pose_normalized",
)


def _build_checklist(inp: dict[str, Any]) -> list[dict[str, Any]]:
    candidate = inp.get("candidate") or {}
    norm_val = inp.get("normalization_validation") or {}
    review = inp.get("handoff_review") or {}
    events = _event_types(inp.get("export_events") or [])
    checklist: list[dict[str, Any]] = []

    norm_status = candidate.get("normalization_status")
    if norm_status == "normalized":
        checklist.append({"id": "normalization", "status": "pass"})
    elif candidate:
        checklist.append({"id": "normalization", "status": "fail"})
    else:
        checklist.append({"id": "normalization", "status": "unknown"})

    if norm_val.get("valid") is True:
        checklist.append({"id": "validation_doc", "status": "pass"})
    elif norm_val:
        checklist.append({"id": "validation_doc", "status": "fail"})
    else:
        checklist.append({"id": "validation_doc", "status": "unknown"})

    notes = review.get("notes") if isinstance(review.get("notes"), str) else ""
    if inp.get("pose_attested") or (notes and "pose" in notes.lower()):
        checklist.append({"id": "pose_cognition", "status": "pass"})
    else:
        checklist.append(
            {
                "id": "pose_cognition",
                "status": "warn",
                "detail": "maintainer attestation required",
            }
        )

    origin = candidate.get("origin") or inp.get("origin") or ""
    if isinstance(origin, str) and "rt_sandbox_capture_v1" in origin:
        checklist.append({"id": "origin", "status": "pass"})
    elif origin:
        checklist.append(
            {"id": "origin", "status": "fail", "detail": "unexpected origin"}
        )
    else:
        checklist.append({"id": "origin", "status": "unknown"})

    lifecycle = (inp.get("session_lifecycle_state") or "").lower()
    if not lifecycle:
        checklist.append({"id": "session_state", "status": "unknown"})
    elif lifecycle in ("failed", "discarded"):
        checklist.append(
            {"id": "session_state", "status": "fail", "detail": lifecycle}
        )
    else:
        checklist.append({"id": "session_state", "status": "pass"})

    pack_ref = inp.get("scenario_pack_ref") or candidate.get("scenario_pack_ref")
    if not pack_ref:
        checklist.append({"id": "scenario_pack", "status": "pass"})
    else:
        checklist.append(
            {
                "id": "scenario_pack",
                "status": "warn",
                "detail": "pack ref present — validate via maintainer CLI",
            }
        )

    missing = [e for e in _REQUIRED_EXPORT_EVENTS if e not in events]
    workflow = inp.get("workflow_phase")
    if not events:
        checklist.append({"id": "export_audit", "status": "unknown"})
    elif not missing:
        checklist.append({"id": "export_audit", "status": "pass"})
    elif "capture_normalized" in events or workflow in ("normalized", "ready"):
        checklist.append(
            {
                "id": "export_audit",
                "status": "warn",
                "detail": f"missing: {', '.join(missing)}",
            }
        )
    else:
        checklist.append(
            {
                "id": "export_audit",
                "status": "fail",
                "detail": f"missing: {', '.join(missing)}",
            }
        )

    lineage_errors = inp.get("lineage_lint_errors")
    if isinstance(lineage_errors, list) and lineage_errors:
        checklist.append(
            {"id": "lineage", "status": "fail", "detail": lineage_errors[0]}
        )
    else:
        checklist.append({"id": "lineage", "status": "pass"})

    return checklist


def _block_reasons(inp: dict[str, Any], events: list[str]) -> list[str]:
    reasons: list[str] = []
    review = inp.get("handoff_review") or {}
    decision = review.get("decision")
    if _has_event(events, "handoff_rejected") or decision == "rejected":
        reasons.append("handoff_rejected")
    if _has_event(events, "handoff_import_deferred") or decision == "deferred":
        reasons.append("handoff_import_deferred")
    if inp.get("handoff_blocked"):
        if "handoff_rejected" not in reasons and decision == "rejected":
            reasons.append("handoff_rejected")
        if "handoff_import_deferred" not in reasons and decision == "deferred":
            reasons.append("handoff_import_deferred")
    lineage = inp.get("lineage_lint_errors")
    if isinstance(lineage, list) and lineage:
        reasons.extend([f"lineage:{e}" for e in lineage[:2]])
    return reasons


def _is_capture_ready(inp: dict[str, Any], events: list[str]) -> bool:
    candidate = inp.get("candidate") or {}
    workflow = inp.get("workflow_phase")
    if candidate.get("normalization_status") == "normalized":
        norm_ok = True
    elif workflow == "normalized" and _has_event(events, "capture_normalized"):
        norm_ok = True
    elif _has_event(events, "capture_normalized"):
        norm_ok = True
    else:
        return False
    norm_val = inp.get("normalization_validation")
    if isinstance(norm_val, dict) and norm_val.get("valid") is False:
        return False
    if not candidate and not _has_event(events, "capture_normalized"):
        return False
    return norm_ok


def _is_review_complete(events: list[str], inp: dict[str, Any]) -> bool:
    if not _has_event(events, "handoff_reviewed"):
        return False
    review = inp.get("handoff_review")
    if isinstance(review, dict) and review:
        return True
    return False


def _is_approval_ready(inp: dict[str, Any], events: list[str]) -> bool:
    candidate = inp.get("candidate") or {}
    if candidate.get("approval_status") == "approved":
        return False
    if not _has_event(events, "handoff_reviewed"):
        return False
    if isinstance(inp.get("handoff_review"), dict) and inp.get("handoff_review"):
        return False
    pre_errors = inp.get("handoff_preconditions_errors")
    if pre_errors == []:
        return True
    workflow = inp.get("workflow_phase")
    if workflow == "ready" and candidate.get("normalization_status") == "normalized":
        return True
    if pre_errors is None and workflow == "ready":
        return True
    return False


def _is_handoff_ready(inp: dict[str, Any], events: list[str]) -> bool:
    candidate = inp.get("candidate") or {}
    if candidate.get("approval_status") != "approved":
        return False
    if not inp.get("conversion_manifest_present") and not _has_event(
        events, "conversion_manifest_written"
    ):
        return False
    return _has_event(events, "capture_approved") or candidate.get("approval_status") == "approved"


def _is_import_ready(inp: dict[str, Any], events: list[str]) -> bool:
    if not _has_event(events, "handoff_import_prepared"):
        return False
    if not inp.get("handoff_manifest_present") and not inp.get("handoff_manifest"):
        return False
    steps_pass = inp.get("conversion_steps_advisory_pass")
    if steps_pass is False:
        return False
    lineage = inp.get("lineage_lint_errors")
    if isinstance(lineage, list) and lineage:
        return False
    return True


def derive_advisory_status(inp: dict[str, Any]) -> dict[str, Any]:
    """Pure derive — returns rt_sa_workflow_advisory_status_v1."""
    capture_id = inp.get("capture_candidate_id") or "unknown"
    events = _event_types(inp.get("export_events") or [])
    candidate = inp.get("candidate") or {}
    approval_status = candidate.get("approval_status", "pending")

    upstream = {
        "workflow_phase": inp.get("workflow_phase"),
        "last_export_event": events[-1] if events else None,
        "approval_status": approval_status,
    }

    base: dict[str, Any] = {
        "schema": "rt_sa_workflow_advisory_status_v1",
        "capture_candidate_id": capture_id,
        "blocked": False,
        "block_reasons": [],
        "checklist": _build_checklist(inp),
        "upstream": upstream,
        "governance_banner": ADVISORY_GOVERNANCE_BANNER,
    }

    if inp.get("import_record_present") or _has_event(events, "handoff_import_committed"):
        base["advisory_state"] = None
        base["advisory_state_label"] = "Committed — SA lineage active"
        base["terminal"] = "handoff_import_committed"
        return base

    block_reasons = _block_reasons(inp, events)
    if block_reasons:
        base["advisory_state"] = "blocked"
        base["advisory_state_label"] = STATE_LABELS["blocked"]
        base["blocked"] = True
        base["block_reasons"] = block_reasons
        return base

    if _is_import_ready(inp, events):
        base["advisory_state"] = "import_ready"
        base["advisory_state_label"] = STATE_LABELS["import_ready"]
        return base

    if _is_handoff_ready(inp, events):
        base["advisory_state"] = "handoff_ready"
        base["advisory_state_label"] = STATE_LABELS["handoff_ready"]
        return base

    if _is_approval_ready(inp, events):
        base["advisory_state"] = "approval_ready"
        base["advisory_state_label"] = STATE_LABELS["approval_ready"]
        return base

    if _is_review_complete(events, inp):
        base["advisory_state"] = "review_complete"
        base["advisory_state_label"] = STATE_LABELS["review_complete"]
        return base

    if _is_capture_ready(inp, events):
        base["advisory_state"] = "capture_ready"
        base["advisory_state_label"] = STATE_LABELS["capture_ready"]
        return base

    base["advisory_state"] = None
    base["advisory_state_label"] = "Not ready (advisory)"
    return base


def build_advisory_input(repo_root: Path, capture_id: str) -> dict[str, Any]:
    """Build derive input from staging + export audit (read-only)."""
    summary = handoff_status_summary(repo_root, capture_id)
    staging_dir = capture_staging_dir(repo_root, capture_id)
    candidate = summary.get("candidate") if isinstance(summary.get("candidate"), dict) else None

    export_events = summary.get("export_events") or []
    event_types = _event_types(export_events)

    norm_val: dict[str, Any] | None = None
    val_path = staging_dir / "normalization_validation.json"
    if val_path.is_file():
        norm_val = json.loads(val_path.read_text(encoding="utf-8"))

    handoff_path = repo_root / "runs" / "rt_sandbox" / "sa_handoff" / capture_id
    conversion_present = (staging_dir / "conversion.json").is_file()
    manifest_present = (handoff_path / "handoff_manifest.json").is_file()

    pre_errors: list[str] | None = None
    if staging_dir.is_dir() and (staging_dir / "candidate.json").exists():
        pre_errors = check_handoff_preconditions(staging_dir)

    workflow_phase = None
    if staging_dir.is_dir():
        workflow_phase = derive_workflow_phase(summary=summary, staging_dir=staging_dir)

    steps_pass: bool | None = None
    steps = summary.get("steps") or {}
    if manifest_present and steps:
        steps_pass = all(
            isinstance(v, dict) and v.get("ok", v.get("status") == "ok")
            for v in steps.values()
        )
    elif manifest_present:
        steps_pass = True

    scenario_pack_ref = None
    if isinstance(candidate, dict):
        scenario_pack_ref = candidate.get("scenario_pack_ref")
    if not scenario_pack_ref and conversion_present:
        conv_path = staging_dir / "conversion.json"
        if conv_path.is_file():
            conv = json.loads(conv_path.read_text(encoding="utf-8"))
            if isinstance(conv, dict):
                scenario_pack_ref = conv.get("scenario_pack_ref")

    return {
        "capture_candidate_id": capture_id,
        "candidate": candidate,
        "handoff_review": summary.get("handoff_review"),
        "export_events": event_types,
        "normalization_validation": norm_val,
        "handoff_preconditions_errors": pre_errors,
        "handoff_blocked": is_handoff_blocked(staging_dir) if staging_dir.is_dir() else False,
        "conversion_manifest_present": conversion_present,
        "handoff_manifest_present": manifest_present,
        "import_record_present": summary.get("import_record") is not None,
        "conversion_steps_advisory_pass": steps_pass,
        "lineage_lint_errors": [],
        "workflow_phase": workflow_phase,
        "origin": candidate.get("origin") if isinstance(candidate, dict) else None,
        "scenario_pack_ref": scenario_pack_ref,
    }


def derive_advisory_status_for_capture(repo_root: Path, capture_id: str) -> dict[str, Any]:
    """Convenience: build input from repo and derive."""
    if not capture_staging_dir(repo_root, capture_id).is_dir():
        raise FileNotFoundError(f"capture not found: {capture_id}")
    inp = build_advisory_input(repo_root, capture_id)
    return derive_advisory_status(inp)
