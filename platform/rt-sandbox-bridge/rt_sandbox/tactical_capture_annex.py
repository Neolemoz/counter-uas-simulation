"""Build rt_tactical_capture_annex_v1 and capture-time audits (PLAT-RT-TAC5)."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from rt_sandbox.authority_labels import AUTHORITY_REPLAY_BOUNDARY
from rt_sandbox.isolation import assert_capture_writable, repo_root_from
from rt_sandbox.tactical_capture_buffer import (
    TACTICAL_ANNEX_GOVERNANCE_BANNER,
    TacticalCaptureBuffer,
)
from rt_sandbox.tactical_controller import TacticalController

ORIGIN_RT_SANDBOX_TACTICAL = "rt_sandbox_tactical_v1"


def build_tactical_capture_annex_v1(
    *,
    session_id: str,
    tactical: TacticalController,
    capture_candidate_id: str,
) -> dict[str, Any] | None:
    buf = tactical.capture_buffer
    if buf.is_empty():
        return None
    s = tactical.state
    return {
        "schema": "rt_tactical_capture_annex_v1",
        "origin": ORIGIN_RT_SANDBOX_TACTICAL,
        "capture_candidate_id": capture_candidate_id,
        "ephemeral_session_ref": session_id,
        "final_tactical_mode": s.mode,
        "selected_id": s.selected_interceptor_id,
        "selected_interceptor_id": s.selected_interceptor_id,
        "selected_target_id": s.selected_target_id,
        "assigned_target": s.assigned_target_id,
        "assigned_interceptor_id": s.assigned_interceptor_id,
        "assigned_target_id": s.assigned_target_id,
        "selected_timeline": list(buf.selected_timeline),
        "assignment_timeline": list(buf.assignment_timeline),
        "tti_timeline": list(buf.tti_timeline),
        "recommendation_timeline": list(buf.recommendation_timeline),
        "mode_switches": list(buf.mode_switches),
        "pause_resume_transitions": list(buf.pause_resume_transitions),
        "assignment_lock_events": list(buf.assignment_lock_events),
        "target_switch_events": list(buf.target_switch_events),
        "authority_label": AUTHORITY_REPLAY_BOUNDARY,
        "governance_banner": TACTICAL_ANNEX_GOVERNANCE_BANNER,
    }


def write_tactical_annex_file(
    staging_dir: Path,
    annex: dict[str, Any],
    *,
    repo_root: Path | None = None,
) -> Path:
    root = repo_root or repo_root_from(staging_dir)
    path = staging_dir / "tactical_annex.json"
    assert_capture_writable(path, root)
    path.write_text(json.dumps(annex, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def validate_tactical_annex(annex: dict[str, Any]) -> str | None:
    if annex.get("schema") != "rt_tactical_capture_annex_v1":
        return "invalid tactical annex schema"
    if annex.get("authority_label") != AUTHORITY_REPLAY_BOUNDARY:
        return "tactical annex authority_label must be replay_boundary_scoped"
    if not annex.get("governance_banner"):
        return "tactical annex governance_banner required"
    if annex.get("authoritative_parent_ref"):
        return "authoritative_parent_ref forbidden on tactical annex"
    return None


def annex_rollup_detail(
    annex: dict[str, Any] | None,
    *,
    tactical_annex_ref: str | None,
) -> dict[str, Any]:
    if annex is None:
        return {"has_annex": False, "tactical_annex_ref": tactical_annex_ref}
    counts = {
        "selected_timeline": len(annex.get("selected_timeline") or []),
        "assignment_timeline": len(annex.get("assignment_timeline") or []),
        "tti_timeline": len(annex.get("tti_timeline") or []),
        "recommendation_timeline": len(annex.get("recommendation_timeline") or []),
        "mode_switches": len(annex.get("mode_switches") or []),
        "pause_resume_transitions": len(annex.get("pause_resume_transitions") or []),
        "assignment_lock_events": len(annex.get("assignment_lock_events") or []),
        "target_switch_events": len(annex.get("target_switch_events") or []),
    }
    return {
        "has_annex": True,
        "tactical_annex_ref": tactical_annex_ref,
        "final_tactical_mode": annex.get("final_tactical_mode"),
        "timeline_counts": counts,
    }


def append_tactical_capture_audits(
    audit: Any,
    export_audit: Any,
    *,
    session_id: str,
    capture_candidate_id: str,
    command_id: str,
    issued_by: str,
    annex: dict[str, Any] | None,
    tactical_annex_ref: str | None,
    buffer: TacticalCaptureBuffer,
) -> None:
    rollup = annex_rollup_detail(annex, tactical_annex_ref=tactical_annex_ref)
    base = {
        "capture_candidate_id": capture_candidate_id,
        **rollup,
    }

    if annex is None:
        audit.append(
            session_id,
            command_id=command_id,
            command_type="tactical_capture_annex_empty",
            issued_by=issued_by,
            result="OK",
            detail=base,
        )
        export_audit.append(
            "tactical_capture_annex_empty",
            capture_candidate_id=capture_candidate_id,
            session_id=session_id,
            result="OK",
            detail=base,
        )
        return

    audit.append(
        session_id,
        command_id=command_id,
        command_type="tactical_capture_annex_written",
        issued_by=issued_by,
        result="OK",
        detail={**base, "timeline_counts": rollup.get("timeline_counts")},
    )
    export_audit.append(
        "tactical_capture_annex_written",
        capture_candidate_id=capture_candidate_id,
        session_id=session_id,
        result="OK",
        detail={**base, "timeline_counts": rollup.get("timeline_counts")},
    )

    audit.append(
        session_id,
        command_id=command_id,
        command_type="tactical_capture_snapshot",
        issued_by=issued_by,
        result="OK",
        detail=base,
    )
    export_audit.append(
        "tactical_capture_snapshot",
        capture_candidate_id=capture_candidate_id,
        session_id=session_id,
        result="OK",
        detail=base,
    )

    mode_switches = annex.get("mode_switches") or []
    if mode_switches:
        last = mode_switches[-1]
        switch_detail = {
            **base,
            "count": len(mode_switches),
            "last_transition": last,
        }
        audit.append(
            session_id,
            command_id=command_id,
            command_type="tactical_switch",
            issued_by=issued_by,
            result="OK",
            detail=switch_detail,
        )
        export_audit.append(
            "tactical_switch",
            capture_candidate_id=capture_candidate_id,
            session_id=session_id,
            result="OK",
            detail=switch_detail,
        )

    assignments = annex.get("assignment_timeline") or []
    assign_detail = {
        **base,
        "count": len(assignments),
        "final_assigned_interceptor_id": annex.get("assigned_interceptor_id"),
        "final_assigned_target_id": annex.get("assigned_target_id"),
    }
    audit.append(
        session_id,
        command_id=command_id,
        command_type="tactical_assignment",
        issued_by=issued_by,
        result="OK",
        detail=assign_detail,
    )
    export_audit.append(
        "tactical_assignment",
        capture_candidate_id=capture_candidate_id,
        session_id=session_id,
        result="OK",
        detail=assign_detail,
    )

    locks = annex.get("assignment_lock_events") or []
    if locks:
        lock_detail = {**base, "count": len(locks)}
        audit.append(
            session_id,
            command_id=command_id,
            command_type="tactical_lock",
            issued_by=issued_by,
            result="OK",
            detail=lock_detail,
        )
        export_audit.append(
            "tactical_lock",
            capture_candidate_id=capture_candidate_id,
            session_id=session_id,
            result="OK",
            detail=lock_detail,
        )

    pause_resume = annex.get("pause_resume_transitions") or []
    if pause_resume:
        pr_detail = {**base, "count": len(pause_resume)}
        audit.append(
            session_id,
            command_id=command_id,
            command_type="tactical_pause_resume",
            issued_by=issued_by,
            result="OK",
            detail=pr_detail,
        )
        export_audit.append(
            "tactical_pause_resume",
            capture_candidate_id=capture_candidate_id,
            session_id=session_id,
            result="OK",
            detail=pr_detail,
        )

    _ = buffer  # buffer referenced for future extensions
