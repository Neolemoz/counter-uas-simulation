"""F7 advisory queue, blocker groups, cohorts, and batch aggregation (PLAT-RT-F7 P0)."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from rt_sandbox.export_boundary import (
    validate_conversion_manifest,
    validate_normalized_manifest,
)
from rt_sandbox.sa_handoff import capture_staging_dir

ADVISORY_BATCH_SUMMARY_SCHEMA = "rt_advisory_batch_summary_v1"
ADVISORY_BATCH_SUMMARY_V2_SCHEMA = "rt_advisory_batch_summary_v2"

STALE_AGE_HOURS = 24

FILTER_PRESET_IDS = (
    "blocked_today",
    "defer_queue",
    "import_advisory_only",
    "review_backlog",
    "normalize_failures",
    "experiment_warn_only",
    "lineage_review",
    "all_staged",
)

TEMPLATE_PACK_IDS = (
    "standup_json_v2",
    "standup_md_daily",
    "standup_md_minimal",
)

STANDUP_PASSES: tuple[dict[str, Any], ...] = (
    {
        "pass_id": "pass_a_firefight",
        "label": "Pass A — Firefight",
        "preset_ids": ["blocked_today", "defer_queue"],
    },
    {
        "pass_id": "pass_b_normalize",
        "label": "Pass B — Normalize",
        "preset_ids": ["normalize_failures"],
    },
    {
        "pass_id": "pass_c_review",
        "label": "Pass C — Review",
        "preset_ids": ["review_backlog"],
    },
    {
        "pass_id": "pass_d_package",
        "label": "Pass D — Package",
        "preset_ids": [],
        "cohort_hint": "needs_prepare",
    },
    {
        "pass_id": "pass_e_import_advisory",
        "label": "Pass E — Import advisory",
        "preset_ids": ["import_advisory_only"],
    },
)

BLOCKER_GROUP_ORDER = (
    "normalization",
    "review_attestation",
    "approval_gate",
    "packaging",
    "lineage",
    "experiment_warn",
    "terminal_block",
)

MAX_GROUP_EXEMPLARS = 5


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _staging_generated_at(repo_root: Path, capture_id: str) -> str | None:
    staging = capture_staging_dir(repo_root, capture_id)
    candidate_path = staging / "candidate.json"
    if candidate_path.is_file():
        data = _load_json(candidate_path)
        gen = data.get("generated_at")
        if isinstance(gen, str):
            return gen
    if staging.is_dir():
        return datetime.fromtimestamp(
            candidate_path.stat().st_mtime if candidate_path.is_file() else staging.stat().st_mtime,
            tz=timezone.utc,
        ).replace(microsecond=0).isoformat()
    return None


def detect_lineage_warnings(
    candidate: dict[str, Any] | None,
    conversion: dict[str, Any] | None = None,
    *,
    normalized: dict[str, Any] | None = None,
) -> list[str]:
    """Detect-only lineage signals (LIN-01..03)."""
    warnings: list[str] = []
    norm = normalized or candidate
    if isinstance(norm, dict):
        err = validate_normalized_manifest(norm)
        if err:
            warnings.append(f"LIN-01: normalized manifest — {err}")
        parent = norm.get("parent_ref")
        session_id = norm.get("session_id")
        if parent and session_id and parent == session_id:
            warnings.append(
                "LIN-01: parent_ref resembles session_id — not SA lineage authority"
            )
    if isinstance(conversion, dict):
        err = validate_conversion_manifest(conversion)
        if err:
            warnings.append(f"LIN-02: conversion manifest — {err}")
        origin = conversion.get("origin")
        if origin and "rt_sandbox_capture_v1" not in str(origin):
            warnings.append("LIN-02: conversion missing rt_sandbox_capture_v1 origin")
    elif isinstance(candidate, dict) and candidate.get("normalization_status") == "normalized":
        origin = candidate.get("origin")
        if origin and "rt_sandbox_capture_v1" not in str(origin):
            warnings.append("LIN-02: candidate origin missing rt_sandbox_capture_v1")
    return warnings


def classify_blocker_groups(
    advisory: dict[str, Any],
    *,
    workflow_phase: str | None = None,
    experiment_warn: bool = False,
) -> list[str]:
    """Map advisory signals to blocker group IDs."""
    groups: list[str] = []
    block_reasons = advisory.get("block_reasons") or []
    checklist = advisory.get("checklist") or []

    if advisory.get("blocked"):
        if "handoff_rejected" in block_reasons:
            groups.append("terminal_block")
        if "handoff_import_deferred" in block_reasons:
            if "terminal_block" not in groups:
                groups.append("terminal_block")

    for item in checklist:
        if not isinstance(item, dict):
            continue
        cid = item.get("id")
        status = item.get("status")
        if status == "fail":
            if cid == "normalization" or cid == "validation_doc":
                groups.append("normalization")
            elif cid in ("pose_cognition", "export_audit"):
                groups.append("review_attestation")
            elif cid == "lineage":
                groups.append("lineage")
            elif cid == "origin":
                groups.append("lineage")
        elif status == "warn" and cid == "pose_cognition":
            groups.append("review_attestation")

    for reason in block_reasons:
        if reason.startswith("lineage:"):
            groups.append("lineage")
        if "normalization" in reason:
            groups.append("normalization")

    state = advisory.get("advisory_state")
    if state in (None,) and not advisory.get("terminal"):
        norm_status = (advisory.get("upstream") or {}).get("approval_status")
        if workflow_phase in ("staged", "none", None) and state is None:
            groups.append("normalization")
        if norm_status == "pending" and state is None:
            pass

    if state == "capture_ready" and workflow_phase == "review_pending":
        groups.append("review_attestation")

    pre = advisory.get("upstream") or {}
    if state == "approval_ready":
        groups.append("approval_gate")
    if state == "handoff_ready":
        groups.append("packaging")
    if state in (None,) and not advisory.get("terminal"):
        candidate_norm = False
        for item in checklist:
            if isinstance(item, dict) and item.get("id") == "normalization" and item.get("status") == "fail":
                candidate_norm = True
        if candidate_norm:
            groups.append("normalization")

    lineage_warnings = advisory.get("lineage_warnings") or []
    if lineage_warnings:
        groups.append("lineage")

    if experiment_warn:
        groups.append("experiment_warn")

    # Deduplicate preserving order
    seen: set[str] = set()
    ordered: list[str] = []
    for g in BLOCKER_GROUP_ORDER:
        if g in groups and g not in seen:
            seen.add(g)
            ordered.append(g)
    return ordered


def primary_blocker_group(groups: list[str]) -> str | None:
    for g in BLOCKER_GROUP_ORDER:
        if g in groups:
            return g
    return None


def readiness_cohort(advisory: dict[str, Any], *, error: str | None = None) -> str:
    if error:
        return "error"
    if advisory.get("terminal"):
        return "terminal"
    if advisory.get("blocked"):
        return "blocked"
    state = advisory.get("advisory_state")
    mapping = {
        None: "needs_normalize",
        "capture_ready": "needs_review",
        "review_complete": "needs_review",
        "approval_ready": "needs_approve",
        "handoff_ready": "needs_prepare",
        "import_ready": "ready_for_commit_advisory",
        "blocked": "blocked",
    }
    if state is None and not advisory.get("terminal"):
        checklist = advisory.get("checklist") or []
        for item in checklist:
            if isinstance(item, dict) and item.get("id") == "normalization" and item.get("status") == "fail":
                return "needs_normalize"
        return "needs_normalize"
    return mapping.get(state, "needs_normalize")


def compute_queue_priority(
    advisory: dict[str, Any],
    *,
    capture_id: str,
    error: str | None = None,
    generated_at: str | None = None,
    workflow_phase: str | None = None,
) -> dict[str, Any]:
    """Return queue_priority {rank, band, rationale}. Lower rank = higher urgency."""
    if error:
        return {
            "rank": 100,
            "band": "P1_error",
            "rationale": error,
        }

    block_reasons = advisory.get("block_reasons") or []
    if advisory.get("blocked"):
        sub = 10
        if "handoff_rejected" in block_reasons:
            sub = 0
        elif "handoff_import_deferred" in block_reasons:
            sub = 10
        return {
            "rank": sub,
            "band": "P0_block",
            "rationale": "blocked: " + ", ".join(block_reasons[:2]),
        }

    if advisory.get("terminal"):
        return {
            "rank": 900,
            "band": "P7_terminal",
            "rationale": "terminal committed",
        }

    state = advisory.get("advisory_state")
    band_map = {
        None: ("P2_normalize", 250),
        "capture_ready": ("P3_review", 320),
        "review_complete": ("P3_review", 340),
        "approval_ready": ("P4_approve", 410),
        "handoff_ready": ("P5_package", 520),
        "import_ready": ("P6_import", 610),
    }
    band, base = band_map.get(state, ("P2_normalize", 250))

    if state is None:
        checklist = advisory.get("checklist") or []
        for item in checklist:
            if isinstance(item, dict) and item.get("id") == "normalization" and item.get("status") == "fail":
                band, base = "P2_normalize", 220
                break

    ts_part = 0
    if generated_at:
        try:
            ts = generated_at.replace("Z", "+00:00")
            dt = datetime.fromisoformat(ts)
            ts_part = int(dt.timestamp()) % 50
        except ValueError:
            ts_part = 0

    rank = base + min(ts_part, 49)
    rationale = state or "not_ready"
    if workflow_phase:
        rationale = f"{rationale}; phase={workflow_phase}"

    return {"rank": rank, "band": band, "rationale": rationale}


def enrich_row_dict(
    row_dict: dict[str, Any],
    *,
    repo_root: Path | None = None,
    experiment_warn: bool = False,
) -> dict[str, Any]:
    """Add F7 fields to a capture row dict (from CaptureAdvisoryRow.to_dict())."""
    advisory = row_dict.get("advisory") or {}
    capture_id = row_dict.get("capture_candidate_id", "")
    error = row_dict.get("error")

    if error:
        row_dict["queue_priority"] = compute_queue_priority(
            {}, capture_id=capture_id, error=error
        )
        row_dict["blocker_groups"] = []
        row_dict["readiness_cohort"] = "error"
        return row_dict

    if experiment_warn and "experiment_warn" not in (advisory.get("block_reasons") or []):
        groups = classify_blocker_groups(
            advisory,
            workflow_phase=row_dict.get("workflow_phase"),
            experiment_warn=True,
        )
    else:
        groups = classify_blocker_groups(
            advisory,
            workflow_phase=row_dict.get("workflow_phase"),
            experiment_warn=experiment_warn,
        )

    gen_at = None
    if repo_root is not None:
        gen_at = _staging_generated_at(repo_root, capture_id)

    row_dict["queue_priority"] = compute_queue_priority(
        advisory,
        capture_id=capture_id,
        workflow_phase=row_dict.get("workflow_phase"),
        generated_at=gen_at,
    )
    row_dict["blocker_groups"] = groups
    row_dict["readiness_cohort"] = readiness_cohort(advisory)
    return row_dict


def sort_rows_by_queue(
    row_dicts: list[dict[str, Any]],
    *,
    sort_key: str = "queue",
) -> list[dict[str, Any]]:
    if sort_key == "capture_id":
        return sorted(row_dicts, key=lambda r: r.get("capture_candidate_id", ""))

    def key_fn(r: dict[str, Any]) -> tuple[int, str, str]:
        qp = r.get("queue_priority") or {}
        rank = qp.get("rank", 9999)
        gen = ""
        cid = r.get("capture_candidate_id", "")
        return (int(rank), gen, cid)

    return sorted(row_dicts, key=key_fn)


def rollup_blocker_groups(row_dicts: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    counts: dict[str, list[str]] = {}
    for row in row_dicts:
        cid = row.get("capture_candidate_id", "")
        for g in row.get("blocker_groups") or []:
            counts.setdefault(g, [])
            if len(counts[g]) < MAX_GROUP_EXEMPLARS:
                counts[g].append(cid)
    return {
        g: {"count": len(ids), "exemplar_capture_ids": ids}
        for g, ids in counts.items()
    }


def rollup_readiness_cohorts(row_dicts: list[dict[str, Any]]) -> dict[str, int]:
    out: dict[str, int] = {}
    for row in row_dicts:
        cohort = row.get("readiness_cohort") or "needs_normalize"
        out[cohort] = out.get(cohort, 0) + 1
    return out


def build_experiment_rollup(
    manifest_path: Path,
    row_dicts: list[dict[str, Any]],
    repo_root: Path,
) -> dict[str, Any] | None:
    """Warn-only experiment handoff rollup."""
    path = manifest_path if manifest_path.is_absolute() else repo_root / manifest_path
    if not path.is_file():
        return {
            "manifest_ref": str(manifest_path),
            "handoff_eligibility": "ineligible",
            "warn_capture_ids": [],
            "note": "manifest not found — experiment eligibility is warn-only",
        }

    manifest = _load_json(path)
    runs = manifest.get("runs") or []
    if not isinstance(runs, list):
        runs = []

    by_id = {r.get("capture_candidate_id"): r for r in row_dicts if r.get("capture_candidate_id")}

    warn_ids: list[str] = []
    eligible_runs = 0
    for run in runs:
        if not isinstance(run, dict):
            continue
        cid = run.get("capture_candidate_id")
        if not isinstance(cid, str):
            continue
        row = by_id.get(cid)
        if not row:
            continue
        adv = row.get("advisory") or {}
        exp_hint = run.get("handoff_eligibility_hint") or run.get("handoff_eligibility")
        if exp_hint == "eligible" and adv.get("advisory_state") != "import_ready":
            warn_ids.append(cid)
        if run.get("has_capture", True):
            eligible_runs += 1

    level = "ineligible"
    if eligible_runs == len(runs) and runs and not warn_ids:
        level = "eligible"
    elif warn_ids or eligible_runs > 0:
        level = "partial"

    return {
        "manifest_ref": str(manifest_path),
        "handoff_eligibility": level,
        "eligible_count": eligible_runs,
        "warn_capture_ids": warn_ids,
        "note": "experiment eligibility is warn-only; per-capture advisory is authority",
    }


def _parse_generated_at(iso: str | None) -> datetime | None:
    if not iso:
        return None
    try:
        return datetime.fromisoformat(iso.replace("Z", "+00:00"))
    except ValueError:
        return None


def stale_age_hours(
    row_dict: dict[str, Any],
    *,
    repo_root: Path | None = None,
    threshold_hours: float = STALE_AGE_HOURS,
) -> float | None:
    """Warn-only staging age in hours."""
    gen = row_dict.get("generated_at")
    if gen is None and repo_root is not None:
        cid = row_dict.get("capture_candidate_id", "")
        if isinstance(cid, str) and cid:
            gen = _staging_generated_at(repo_root, cid)
    dt = _parse_generated_at(gen if isinstance(gen, str) else None)
    if dt is None:
        return None
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    age = (datetime.now(timezone.utc) - dt).total_seconds() / 3600.0
    return round(age, 2) if age >= threshold_hours else None


def _row_matches_preset(row: dict[str, Any], preset_id: str) -> bool:
    if preset_id == "all_staged":
        return True
    advisory = row.get("advisory") or {}
    groups = row.get("blocker_groups") or []
    if not groups and advisory:
        groups = classify_blocker_groups(
            advisory,
            workflow_phase=row.get("workflow_phase"),
        )

    if preset_id == "blocked_today":
        if advisory.get("blocked"):
            return True
        return "terminal_block" in groups

    if preset_id == "defer_queue":
        reasons = advisory.get("block_reasons") or []
        return "handoff_import_deferred" in reasons

    if preset_id == "import_advisory_only":
        return advisory.get("advisory_state") == "import_ready"

    if preset_id == "review_backlog":
        return advisory.get("advisory_state") in ("capture_ready", "review_complete")

    if preset_id == "normalize_failures":
        return "normalization" in groups or primary_blocker_group(groups) == "normalization"

    if preset_id == "experiment_warn_only":
        return "experiment_warn" in groups

    if preset_id == "lineage_review":
        lw = row.get("lineage_warnings") or advisory.get("lineage_warnings") or []
        return bool(lw)

    return True


def apply_filter_preset(
    row_dicts: list[dict[str, Any]],
    preset_id: str | None,
) -> list[dict[str, Any]]:
    """Filter-only — does not invoke maintainer CLIs."""
    if not preset_id or preset_id == "all_staged":
        return list(row_dicts)
    if preset_id not in FILTER_PRESET_IDS:
        raise ValueError(f"unknown preset: {preset_id}")
    return [r for r in row_dicts if _row_matches_preset(r, preset_id)]


def apply_focus_set(
    row_dicts: list[dict[str, Any]],
    focus_ids: list[str] | None,
) -> list[dict[str, Any]]:
    """Intersect rows and mark in_focus_set — no cross-session authority merge."""
    if not focus_ids:
        for row in row_dicts:
            row.pop("in_focus_set", None)
        return list(row_dicts)
    focus = set(focus_ids)
    out: list[dict[str, Any]] = []
    for row in row_dicts:
        cid = row.get("capture_candidate_id", "")
        if cid in focus:
            row = dict(row)
            row["in_focus_set"] = True
            out.append(row)
        else:
            row = dict(row)
            row["in_focus_set"] = False
    return [r for r in out if r.get("capture_candidate_id") in focus]


def readiness_cohort_v2(
    row_dict: dict[str, Any],
    *,
    experiment_warn_ids: set[str] | None = None,
) -> str:
    """Additive v2 cohort label — not operational readiness."""
    base = row_dict.get("readiness_cohort")
    if not base:
        base = readiness_cohort(row_dict.get("advisory") or {}, error=row_dict.get("error"))
    if row_dict.get("in_focus_set"):
        return "focus_highlight"
    cid = row_dict.get("capture_candidate_id", "")
    if experiment_warn_ids and cid in experiment_warn_ids:
        return "experiment_handoff_warn"
    groups = row_dict.get("blocker_groups") or []
    if len(groups) >= 2:
        return "multi_blocker"
    if row_dict.get("stale_age_hours") is not None:
        if base in ("needs_review",):
            return "stale_review"
        if base == "needs_approve":
            return "stale_approve"
    return base


def enrich_row_dict_v2(
    row_dict: dict[str, Any],
    *,
    repo_root: Path | None = None,
    experiment_warn: bool = False,
    experiment_warn_ids: set[str] | None = None,
) -> dict[str, Any]:
    """F7 enrich + F8 v2 fields."""
    enrich_row_dict(row_dict, repo_root=repo_root, experiment_warn=experiment_warn)
    if repo_root is not None and not row_dict.get("error"):
        gen = _staging_generated_at(repo_root, row_dict.get("capture_candidate_id", ""))
        if gen:
            row_dict["generated_at"] = gen
    row_dict["stale_age_hours"] = stale_age_hours(row_dict, repo_root=repo_root)
    row_dict["readiness_cohort_v2"] = readiness_cohort_v2(
        row_dict,
        experiment_warn_ids=experiment_warn_ids,
    )
    return row_dict


def rollup_readiness_cohorts_v2(row_dicts: list[dict[str, Any]]) -> dict[str, int]:
    out: dict[str, int] = {}
    for row in row_dicts:
        cohort = row.get("readiness_cohort_v2") or row.get("readiness_cohort") or "needs_normalize"
        out[cohort] = out.get(cohort, 0) + 1
    return out


def _lane_for_cohort(cohort: str) -> str | None:
    mapping = {
        "needs_normalize": "normalize",
        "needs_review": "review",
        "needs_approve": "approve",
        "needs_prepare": "prepare",
        "ready_for_commit_advisory": "import_advisory",
        "stale_review": "review",
        "stale_approve": "approve",
    }
    return mapping.get(cohort)


def rollup_multi_capture_cohorts(row_dicts: list[dict[str, Any]]) -> dict[str, Any]:
    by_lane: dict[str, int] = {}
    stale_warn = 0
    for row in row_dicts:
        cohort = row.get("readiness_cohort") or "needs_normalize"
        lane = _lane_for_cohort(cohort)
        if lane:
            by_lane[lane] = by_lane.get(lane, 0) + 1
        if row.get("stale_age_hours") is not None:
            stale_warn += 1
    return {
        "by_primary_lane": by_lane,
        "stale_age_warn_count": stale_warn,
        "note": "lane counts are advisory cognition only",
    }


def rollup_handoff_rollup(row_dicts: list[dict[str, Any]]) -> dict[str, Any]:
    by_stage: dict[str, int] = {
        "normalize": 0,
        "review": 0,
        "approve": 0,
        "prepare": 0,
        "import_advisory": 0,
    }
    blocked_count = 0
    terminal_count = 0
    for row in row_dicts:
        cohort = row.get("readiness_cohort") or "needs_normalize"
        if cohort == "blocked":
            blocked_count += 1
            continue
        if cohort == "terminal":
            terminal_count += 1
            continue
        if cohort == "error":
            blocked_count += 1
            continue
        lane = _lane_for_cohort(cohort)
        if lane:
            by_stage[lane] = by_stage.get(lane, 0) + 1
    return {
        "by_stage": by_stage,
        "blocked_count": blocked_count,
        "terminal_count": terminal_count,
    }


def build_experiment_handoff_rollup(
    manifest_path: Path | None,
    row_dicts: list[dict[str, Any]],
    repo_root: Path,
    *,
    cohort_index_ref: Path | None = None,
) -> dict[str, Any] | None:
    """Warn-only experiment + X2 cohort index adjacency — no packet generation."""
    if manifest_path is None:
        return None
    base = build_experiment_rollup(manifest_path, row_dicts, repo_root)
    if base is None:
        return None
    out: dict[str, Any] = {
        "manifest_ref": base.get("manifest_ref"),
        "handoff_eligibility": base.get("handoff_eligibility"),
        "warn_capture_ids": list(base.get("warn_capture_ids") or []),
        "review_packet_paths": [],
        "note": (
            "X2 cohort and packet paths are read-only adjacency; not commit authority"
        ),
    }
    if cohort_index_ref is not None:
        idx_path = (
            cohort_index_ref
            if cohort_index_ref.is_absolute()
            else repo_root / cohort_index_ref
        )
        out["cohort_index_ref"] = str(cohort_index_ref)
        if idx_path.is_file():
            idx = _load_json(idx_path)
            out["cohort_status"] = idx.get("status") or "indexed"
            entries = idx.get("entries") or idx.get("cohorts") or []
            if isinstance(entries, list):
                for entry in entries:
                    if not isinstance(entry, dict):
                        continue
                    mref = entry.get("manifest_ref") or entry.get("experiment_manifest_ref")
                    if mref and str(mref) == str(manifest_path):
                        pkt = entry.get("review_packet_path") or entry.get("packet_path")
                        if isinstance(pkt, str):
                            out["review_packet_paths"].append(pkt)
        else:
            out["cohort_status"] = "missing_index"
    else:
        out["cohort_status"] = None
    out["eligible_count"] = base.get("eligible_count")
    return out

