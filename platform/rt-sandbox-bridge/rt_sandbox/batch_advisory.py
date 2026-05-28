"""Batch RT→SA handoff advisory reports (PLAT-RT-F6 P2)."""

from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from rt_sandbox.advisory_derive import (
    ADVISORY_GOVERNANCE_BANNER,
    build_advisory_input,
    derive_advisory_status,
)
from rt_sandbox.advisory_queue import (
    ADVISORY_BATCH_SUMMARY_SCHEMA,
    ADVISORY_BATCH_SUMMARY_V2_SCHEMA,
    BLOCKER_GROUP_ORDER,
    STANDUP_PASSES,
    TEMPLATE_PACK_IDS,
    apply_filter_preset,
    apply_focus_set,
    build_experiment_handoff_rollup,
    build_experiment_rollup,
    enrich_row_dict,
    enrich_row_dict_v2,
    rollup_blocker_groups,
    rollup_handoff_rollup,
    rollup_multi_capture_cohorts,
    rollup_readiness_cohorts,
    rollup_readiness_cohorts_v2,
    sort_rows_by_queue,
)
from rt_sandbox.capture import list_staged_capture_ids
from rt_sandbox.sa_handoff import capture_staging_dir
from rt_sandbox.isolation import repo_root_from

BATCH_REVIEW_SCHEMA = "rt_handoff_batch_review_v1"
ADVISORY_BATCH_REVIEW_V2_SCHEMA = "rt_advisory_batch_review_v2"
ADVISORY_DRY_RUN_REVIEW_SCHEMA = "rt_advisory_dry_run_review_v1"
CORPUS_PREVIEW_SCHEMA = "rt_handoff_corpus_preview_v1"

QUEUE_BAND_ORDER = (
    "P0_block",
    "P1_error",
    "P2_normalize",
    "P3_review",
    "P4_approve",
    "P5_package",
    "P6_import",
    "P7_terminal",
)

PRIORITY_STANDUP_BANDS = frozenset({"P0_block", "P1_error", "P2_normalize"})

FORBIDDEN_EXPORT_LEXICON = (
    "readiness_score",
    "auto_import",
    "operational_ready",
    "commit-all",
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def next_maintainer_cli_hint(status: dict[str, Any]) -> str | None:
    """Mirror UI nextStepCli — maintainer hint only, not authority."""
    if status.get("terminal"):
        return None
    if status.get("blocked"):
        return "scripts/rt/rt_handoff_review.py ready <capture_id>"
    state = status.get("advisory_state")
    hints = {
        "capture_ready": "scripts/rt/rt_handoff_review.py ready <capture_id>",
        "review_complete": "scripts/rt/rt_handoff_review.py reviewed <capture_id>",
        "approval_ready": "scripts/rt/rt_capture_approve.py <capture_id>",
        "handoff_ready": "scripts/rt/rt_sa_import.py prepare <capture_id>",
        "import_ready": (
            "scripts/rt/rt_sa_import.py commit --corpus-dest <path> <capture_id>"
        ),
    }
    hint = hints.get(state)
    if not hint:
        return None
    cid = status.get("capture_candidate_id", "<capture_id>")
    return hint.replace("<capture_id>", cid)


@dataclass
class CaptureAdvisoryRow:
    capture_candidate_id: str
    advisory: dict[str, Any]
    next_cli: str | None = None
    workflow_phase: str | None = None
    error: str | None = None

    def to_dict(self, *, repo_root: Path | None = None, experiment_warn: bool = False) -> dict[str, Any]:
        out: dict[str, Any] = {
            "capture_candidate_id": self.capture_candidate_id,
            "advisory": self.advisory,
            "next_cli": self.next_cli,
            "workflow_phase": self.workflow_phase,
        }
        if self.error:
            out["error"] = self.error
        return enrich_row_dict(out, repo_root=repo_root, experiment_warn=experiment_warn)


@dataclass
class BatchFilters:
    state: str | None = None
    blocked_only: bool = False
    reject_only: bool = False
    defer_only: bool = False
    terminal_committed: bool = False


def derive_capture_row(repo_root: Path, capture_id: str) -> CaptureAdvisoryRow:
    staging = capture_staging_dir(repo_root, capture_id)
    if not staging.is_dir():
        return CaptureAdvisoryRow(
            capture_candidate_id=capture_id,
            advisory={},
            error="staging_not_found",
        )
    try:
        inp = build_advisory_input(repo_root, capture_id)
        status = derive_advisory_status(inp)
        status["capture_candidate_id"] = capture_id
        return CaptureAdvisoryRow(
            capture_candidate_id=capture_id,
            advisory=status,
            next_cli=next_maintainer_cli_hint(status),
            workflow_phase=inp.get("workflow_phase"),
        )
    except (OSError, json.JSONDecodeError, TypeError, ValueError) as exc:
        return CaptureAdvisoryRow(
            capture_candidate_id=capture_id,
            advisory={},
            error=str(exc),
        )


def resolve_capture_ids(repo_root: Path, capture_ids: list[str] | None) -> list[str]:
    if capture_ids:
        return sorted(set(capture_ids))
    return list_staged_capture_ids(repo_root)


def scan_staged(
    repo_root: Path,
    capture_ids: list[str] | None = None,
) -> list[CaptureAdvisoryRow]:
    rows: list[CaptureAdvisoryRow] = []
    for cid in resolve_capture_ids(repo_root, capture_ids):
        rows.append(derive_capture_row(repo_root, cid))
    return rows


def _state_key(row: CaptureAdvisoryRow) -> str:
    adv = row.advisory
    if adv.get("terminal"):
        return "committed"
    if adv.get("blocked"):
        return "blocked"
    state = adv.get("advisory_state")
    return state if isinstance(state, str) else "not_ready"


def _review_decision(row: CaptureAdvisoryRow) -> str | None:
    blockers = row.advisory.get("block_reasons") or []
    if "handoff_rejected" in blockers or row.workflow_phase == "rejected":
        return "rejected"
    if "handoff_import_deferred" in blockers or row.workflow_phase == "deferred":
        return "deferred"
    return None


def filter_rows(rows: list[CaptureAdvisoryRow], filters: BatchFilters) -> list[CaptureAdvisoryRow]:
    out: list[CaptureAdvisoryRow] = []
    for row in rows:
        if row.error:
            if not filters.state and not filters.blocked_only:
                if not filters.reject_only and not filters.defer_only and not filters.terminal_committed:
                    out.append(row)
            continue
        key = _state_key(row)
        if filters.state and key != filters.state:
            continue
        if filters.blocked_only and not row.advisory.get("blocked"):
            continue
        decision = _review_decision(row)
        if filters.reject_only and decision != "rejected":
            continue
        if filters.defer_only and decision != "deferred":
            continue
        if filters.terminal_committed and row.advisory.get("terminal") != "handoff_import_committed":
            continue
        out.append(row)
    return out


def aggregate_report(rows: list[CaptureAdvisoryRow]) -> dict[str, Any]:
    by_state: dict[str, int] = {}
    block_reasons: dict[str, int] = {}
    errors: list[str] = []
    for row in rows:
        if row.error:
            errors.append(row.capture_candidate_id)
            by_state["error"] = by_state.get("error", 0) + 1
            continue
        key = _state_key(row)
        by_state[key] = by_state.get(key, 0) + 1
        for reason in row.advisory.get("block_reasons") or []:
            block_reasons[reason] = block_reasons.get(reason, 0) + 1
    return {
        "total": len(rows),
        "by_advisory_state": by_state,
        "block_reason_rollup": block_reasons,
        "derive_errors": errors,
    }


def build_batch_review_document(
    repo_root: Path,
    rows: list[CaptureAdvisoryRow],
    *,
    dry_run: bool = True,
) -> dict[str, Any]:
    """F6 schema — retained for backward compatibility."""
    return {
        "schema": BATCH_REVIEW_SCHEMA,
        "generated_at": _utc_now(),
        "repo_root": str(repo_root.resolve()),
        "dry_run": dry_run,
        "governance_banner": ADVISORY_GOVERNANCE_BANNER,
        "summary": aggregate_report(rows),
        "captures": [r.to_dict(repo_root=repo_root) for r in rows],
    }


def build_grouped_export_indexes(
    capture_dicts: list[dict[str, Any]],
) -> dict[str, dict[str, list[str]]]:
    """Read-only capture id indexes — blocker groups allow multi-membership."""
    by_band: dict[str, list[str]] = {b: [] for b in QUEUE_BAND_ORDER}
    by_blocker: dict[str, list[str]] = {g: [] for g in BLOCKER_GROUP_ORDER}
    by_cohort: dict[str, list[str]] = {}

    for row in capture_dicts:
        cid = row.get("capture_candidate_id")
        if not isinstance(cid, str) or not cid:
            continue
        band = (row.get("queue_priority") or {}).get("band")
        if isinstance(band, str) and band in by_band:
            by_band[band].append(cid)
        cohort = row.get("readiness_cohort")
        if isinstance(cohort, str):
            by_cohort.setdefault(cohort, []).append(cid)
        for g in row.get("blocker_groups") or []:
            if isinstance(g, str) and g in by_blocker:
                by_blocker[g].append(cid)

    return {
        "by_queue_band": {k: v for k, v in by_band.items() if v},
        "by_blocker_group": {k: v for k, v in by_blocker.items() if v},
        "by_readiness_cohort": by_cohort,
    }


def build_standup_section(
    capture_dicts: list[dict[str, Any]],
    summary: dict[str, Any],
) -> dict[str, Any]:
    priority_ids: list[str] = []
    for row in capture_dicts:
        cid = row.get("capture_candidate_id")
        if not isinstance(cid, str):
            continue
        band = (row.get("queue_priority") or {}).get("band")
        if band in PRIORITY_STANDUP_BANDS:
            priority_ids.append(cid)

    blocker_groups = summary.get("blocker_groups") or {}
    top_blockers = sorted(
        blocker_groups.items(),
        key=lambda x: -(x[1].get("count", 0) if isinstance(x[1], dict) else 0),
    )[:5]
    top_blocker_groups = [
        {
            "group_id": gid,
            "count": info.get("count", 0) if isinstance(info, dict) else 0,
            "exemplar_capture_ids": (
                info.get("exemplar_capture_ids", []) if isinstance(info, dict) else []
            ),
        }
        for gid, info in top_blockers
    ]

    notes = [
        "experiment eligibility is warn-only; per-capture advisory is authority",
    ]
    exp = summary.get("experiment_rollup")
    if isinstance(exp, dict) and exp.get("note"):
        notes.append(str(exp["note"]))

    return {
        "priority_capture_ids": priority_ids,
        "top_blocker_groups": top_blocker_groups,
        "cohort_counts": summary.get("readiness_cohorts") or {},
        "experiment_rollup": exp,
        "warn_only_notes": notes,
    }


def validate_advisory_batch_review_v2(doc: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if doc.get("schema") != ADVISORY_BATCH_REVIEW_V2_SCHEMA:
        errors.append(f"schema must be {ADVISORY_BATCH_REVIEW_V2_SCHEMA}")
    if doc.get("dry_run") is not True:
        errors.append("dry_run must be true for v2 export")
    if not doc.get("governance_banner"):
        errors.append("governance_banner required")

    blob = json.dumps(doc).lower()
    for term in FORBIDDEN_EXPORT_LEXICON:
        if term in blob:
            errors.append(f"forbidden lexicon: {term}")

    grouped = doc.get("grouped") or {}
    for band in (grouped.get("by_queue_band") or {}):
        if band not in QUEUE_BAND_ORDER:
            errors.append(f"unknown queue band in grouped: {band}")
    for gid in (grouped.get("by_blocker_group") or {}):
        if gid not in BLOCKER_GROUP_ORDER:
            errors.append(f"unknown blocker group in grouped: {gid}")

    for cap in doc.get("captures") or []:
        if not isinstance(cap, dict):
            continue
        if cap.get("error"):
            continue
        if "queue_priority" not in cap:
            errors.append(f"missing queue_priority on {cap.get('capture_candidate_id')}")
        if "readiness_cohort" not in cap:
            errors.append(f"missing readiness_cohort on {cap.get('capture_candidate_id')}")
        if "blocker_groups" not in cap:
            errors.append(f"missing blocker_groups on {cap.get('capture_candidate_id')}")

    return errors


def build_advisory_batch_review_v2_document(
    repo_root: Path,
    rows: list[CaptureAdvisoryRow],
    *,
    dry_run: bool = True,
    manifest_ref: Path | None = None,
    sort_key: str = "queue",
) -> dict[str, Any]:
    """F7 P2 stand-up / grouped export — superset of rt_advisory_batch_summary_v1."""
    base = build_advisory_batch_summary_document(
        repo_root,
        rows,
        dry_run=dry_run,
        manifest_ref=manifest_ref,
        sort_key=sort_key,
    )
    capture_dicts = base["captures"]
    summary = base["summary"]
    return {
        "schema": ADVISORY_BATCH_REVIEW_V2_SCHEMA,
        "generated_at": base["generated_at"],
        "repo_root": base["repo_root"],
        "dry_run": True,
        "governance_banner": base["governance_banner"],
        "summary": summary,
        "captures": capture_dicts,
        "grouped": build_grouped_export_indexes(capture_dicts),
        "standup": build_standup_section(capture_dicts, summary),
    }


def build_advisory_batch_summary_document(
    repo_root: Path,
    rows: list[CaptureAdvisoryRow],
    *,
    dry_run: bool = True,
    manifest_ref: Path | None = None,
    sort_key: str = "queue",
) -> dict[str, Any]:
    """F7 rt_advisory_batch_summary_v1 — superset of F6 batch review."""
    capture_dicts = [r.to_dict(repo_root=repo_root) for r in rows]
    if manifest_ref is not None:
        rollup = build_experiment_rollup(manifest_ref, capture_dicts, repo_root)
        if rollup:
            warn_set = set(rollup.get("warn_capture_ids") or [])
            if warn_set:
                capture_dicts = [
                    enrich_row_dict(
                        d,
                        repo_root=repo_root,
                        experiment_warn=d.get("capture_candidate_id") in warn_set,
                    )
                    for d in capture_dicts
                ]
    capture_dicts = sort_rows_by_queue(capture_dicts, sort_key=sort_key)
    summary = aggregate_report(rows)
    summary["blocker_groups"] = rollup_blocker_groups(capture_dicts)
    summary["readiness_cohorts"] = rollup_readiness_cohorts(capture_dicts)
    if manifest_ref is not None:
        rollup = build_experiment_rollup(manifest_ref, capture_dicts, repo_root)
        if rollup:
            summary["experiment_rollup"] = rollup
    return {
        "schema": ADVISORY_BATCH_SUMMARY_SCHEMA,
        "generated_at": _utc_now(),
        "repo_root": str(repo_root.resolve()),
        "dry_run": dry_run,
        "governance_banner": ADVISORY_GOVERNANCE_BANNER,
        "summary": summary,
        "captures": capture_dicts,
    }


def build_advisory_batch_summary_v2_document(
    repo_root: Path,
    rows: list[CaptureAdvisoryRow],
    *,
    dry_run: bool = True,
    manifest_ref: Path | None = None,
    sort_key: str = "queue",
    preset_applied: str | None = None,
    focus_capture_ids: list[str] | None = None,
    cohort_index_ref: Path | None = None,
) -> dict[str, Any]:
    """F8 rt_advisory_batch_summary_v2 — additive superset of F7 v1."""
    capture_dicts = [r.to_dict(repo_root=repo_root) for r in rows]
    warn_set: set[str] = set()
    if manifest_ref is not None:
        rollup = build_experiment_rollup(manifest_ref, capture_dicts, repo_root)
        if rollup:
            warn_set = set(rollup.get("warn_capture_ids") or [])
            if warn_set:
                capture_dicts = [
                    enrich_row_dict(
                        d,
                        repo_root=repo_root,
                        experiment_warn=d.get("capture_candidate_id") in warn_set,
                    )
                    for d in capture_dicts
                ]
    capture_dicts = apply_filter_preset(capture_dicts, preset_applied)
    capture_dicts = apply_focus_set(capture_dicts, focus_capture_ids)
    capture_dicts = [
        enrich_row_dict_v2(
            d,
            repo_root=repo_root,
            experiment_warn=d.get("capture_candidate_id") in warn_set,
            experiment_warn_ids=warn_set,
        )
        for d in capture_dicts
    ]
    capture_dicts = sort_rows_by_queue(capture_dicts, sort_key=sort_key)
    summary = aggregate_report(rows)
    by_state: dict[str, int] = {}
    block_reasons: dict[str, int] = {}
    errors: list[str] = []
    for row in capture_dicts:
        cid = row.get("capture_candidate_id", "")
        if row.get("error"):
            errors.append(cid)
            by_state["error"] = by_state.get("error", 0) + 1
            continue
        adv = row.get("advisory") or {}
        if adv.get("terminal"):
            key = "committed"
        elif adv.get("blocked"):
            key = "blocked"
        else:
            key = adv.get("advisory_state") or "not_ready"
        by_state[key] = by_state.get(key, 0) + 1
        for reason in adv.get("block_reasons") or []:
            block_reasons[reason] = block_reasons.get(reason, 0) + 1
    summary["total"] = len(capture_dicts)
    summary["by_advisory_state"] = by_state
    summary["block_reason_rollup"] = block_reasons
    summary["derive_errors"] = errors
    summary["blocker_groups"] = rollup_blocker_groups(capture_dicts)
    summary["readiness_cohorts"] = rollup_readiness_cohorts(capture_dicts)
    summary["readiness_cohorts_v2"] = rollup_readiness_cohorts_v2(capture_dicts)
    summary["multi_capture_cohorts"] = rollup_multi_capture_cohorts(capture_dicts)
    summary["handoff_rollup"] = rollup_handoff_rollup(capture_dicts)
    if manifest_ref is not None:
        summary["experiment_rollup"] = build_experiment_rollup(
            manifest_ref, capture_dicts, repo_root
        )
        summary["experiment_handoff_rollup"] = build_experiment_handoff_rollup(
            manifest_ref,
            capture_dicts,
            repo_root,
            cohort_index_ref=cohort_index_ref,
        )
    return {
        "schema": ADVISORY_BATCH_SUMMARY_V2_SCHEMA,
        "generated_at": _utc_now(),
        "repo_root": str(repo_root.resolve()),
        "dry_run": True if dry_run else dry_run,
        "governance_banner": ADVISORY_GOVERNANCE_BANNER,
        "preset_applied": preset_applied,
        "focus_capture_ids": list(focus_capture_ids) if focus_capture_ids else None,
        "standup_passes": list(STANDUP_PASSES),
        "summary": summary,
        "captures": capture_dicts,
    }


def render_template_pack(
    doc: dict[str, Any],
    pack_id: str,
) -> dict[str, Any]:
    """Render-only stand-up template — no embedded next_cli or --execute."""
    if pack_id not in TEMPLATE_PACK_IDS:
        raise ValueError(f"unknown template pack: {pack_id}")
    summary = doc.get("summary") or {}
    banner = doc.get("governance_banner", ADVISORY_GOVERNANCE_BANNER)
    if pack_id == "standup_json_v2":
        content = json.dumps(
            {
                "governance_banner": banner,
                "schema": doc.get("schema"),
                "dry_run": doc.get("dry_run"),
                "summary": {
                    "total": summary.get("total"),
                    "readiness_cohorts_v2": summary.get("readiness_cohorts_v2"),
                    "multi_capture_cohorts": summary.get("multi_capture_cohorts"),
                    "handoff_rollup": summary.get("handoff_rollup"),
                },
                "standup_passes": doc.get("standup_passes"),
                "priority_capture_ids": [
                    c.get("capture_candidate_id")
                    for c in (doc.get("captures") or [])[:5]
                    if isinstance(c, dict)
                ],
            },
            indent=2,
            sort_keys=True,
        )
        return {
            "pack_id": pack_id,
            "format": "json",
            "content": content,
            "governance_banner": banner,
        }
    if pack_id == "standup_md_minimal":
        total = summary.get("total", 0)
        import_adv = (summary.get("handoff_rollup") or {}).get("by_stage", {}).get(
            "import_advisory", 0
        )
        lines = [
            f"# Stand-up (minimal)\n",
            f"> {banner}\n",
            f"- **Total:** {total}",
            f"- **Import-advisory (manual commit):** {import_adv}",
        ]
        return {
            "pack_id": pack_id,
            "format": "markdown",
            "content": "\n".join(lines) + "\n",
            "governance_banner": banner,
        }
    lines = [
        f"# Stand-up daily\n",
        f"> {banner}\n",
        f"> Preset ≠ CLI invocation. Manual commit required.\n",
    ]
    for p in doc.get("standup_passes") or STANDUP_PASSES:
        if not isinstance(p, dict):
            continue
        lines.append(f"## {p.get('label', p.get('pass_id'))}")
        presets = p.get("preset_ids") or []
        if presets:
            lines.append(f"- Presets: {', '.join(presets)}")
    hr = summary.get("handoff_rollup") or {}
    by_stage = hr.get("by_stage") or {}
    if by_stage:
        lines.append("\n### Handoff stages")
        for stage, count in sorted(by_stage.items()):
            lines.append(f"- {stage}: {count}")
    exp = summary.get("experiment_handoff_rollup") or summary.get("experiment_rollup")
    if isinstance(exp, dict) and exp.get("note"):
        lines.append(f"\n_Warn: {exp['note']}_")
    return {
        "pack_id": pack_id,
        "format": "markdown",
        "content": "\n".join(lines) + "\n",
        "governance_banner": banner,
    }


def filter_rows_by_group(
    rows: list[CaptureAdvisoryRow],
    group_id: str,
    *,
    repo_root: Path,
) -> list[CaptureAdvisoryRow]:
    out: list[CaptureAdvisoryRow] = []
    for row in rows:
        d = row.to_dict(repo_root=repo_root)
        if group_id in (d.get("blocker_groups") or []):
            out.append(row)
    return out


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _corpus_index_paths(repo_root: Path) -> set[str]:
    paths: set[str] = set()
    sa_root = repo_root / "fixtures" / "sa_r0"
    index_path = sa_root / "synthesis" / "replay_corpus_index_v1.json"
    if index_path.is_file():
        data = _load_json(index_path)
        for entry in data.get("entries") or []:
            if isinstance(entry, dict):
                p = entry.get("primary_artifact_path")
                if isinstance(p, str):
                    paths.add(p)
    for demo in sa_root.glob("demo_*"):
        if demo.is_dir():
            paths.add(str(demo.relative_to(repo_root)) + "/index.json")
    return paths


def corpus_preview_for_capture(
    repo_root: Path,
    capture_id: str,
    *,
    corpus_dest: Path | None = None,
) -> dict[str, Any]:
    """Read-only preview vs fixtures/sa_r0 — no corpus writes."""
    staging = capture_staging_dir(repo_root, capture_id)
    out: dict[str, Any] = {
        "schema": CORPUS_PREVIEW_SCHEMA,
        "capture_candidate_id": capture_id,
        "governance_banner": ADVISORY_GOVERNANCE_BANNER,
        "dest_policy": "fixtures_sa_r0_only",
        "dest_valid": True,
        "dest_errors": [],
        "proposed_dest_rel": None,
        "proposed_dest_source": "unknown",
        "bundle_index_present": False,
        "index_state": "not_indexed",
        "would_add": [],
        "would_conflict": [],
        "missing_staging_ref": [],
        "notes": [],
    }
    if not staging.is_dir():
        out["missing_staging_ref"].append("capture_staging_dir")
        return out

    proposed: Path | None = corpus_dest
    if proposed is not None:
        out["proposed_dest_source"] = "arg"
    conversion_path = staging / "conversion.json"
    if proposed is None and conversion_path.is_file():
        conv = _load_json(conversion_path)
        ref = conv.get("corpus_dest") or conv.get("proposed_corpus_dest")
        if isinstance(ref, str):
            proposed = repo_root / ref if not ref.startswith("/") else Path(ref)
            out["proposed_dest_source"] = "conversion"

    handoff_path = repo_root / "runs" / "rt_sandbox" / "sa_handoff" / capture_id
    manifest_path = handoff_path / "handoff_manifest.json"
    if proposed is None and manifest_path.is_file():
        out["notes"].append("no corpus_dest in conversion; pass --corpus-dest")
        out["proposed_dest_source"] = "manifest"

    if proposed is None:
        proposed = repo_root / "fixtures" / "sa_r0" / f"demo_{capture_id}"
        out["proposed_dest_source"] = "default"

    bundle_src = handoff_path / "bundle" / "index.json"
    out["bundle_index_present"] = bundle_src.is_file()
    if not bundle_src.is_file():
        out["missing_staging_ref"].append("handoff_bundle_index")

    repo_resolved = repo_root.resolve()
    sa_root = (repo_root / "fixtures" / "sa_r0").resolve()
    try:
        proposed_resolved = proposed.resolve()
    except OSError:
        proposed_resolved = proposed

    # Enforce fixtures/sa_r0-only policy for corpus preview.
    dest_valid = False
    rel: str | None = None
    try:
        rel = str(proposed_resolved.relative_to(repo_resolved))
        out["proposed_dest_rel"] = rel
        proposed_resolved.relative_to(sa_root)
        dest_valid = True
    except (ValueError, OSError):
        dest_valid = False

    out["dest_valid"] = dest_valid
    if not dest_valid:
        out["dest_errors"].append("outside_fixtures_sa_r0")
        out["notes"].append("preview only; commit requires explicit maintainer CLI")
        out["notes"].append("dest rejected by policy: must be under fixtures/sa_r0")
        # Avoid implying an allowed write path by populating would_add/would_conflict.
        return out

    index_paths = _corpus_index_paths(repo_root)
    dest_exists = proposed_resolved.exists()
    if dest_exists:
        out["would_conflict"].append(rel)
    else:
        out["would_add"].append(rel)
    if rel in index_paths and dest_exists:
        out["index_state"] = "indexed_and_exists"
        out["notes"].append("path already indexed and exists on disk")
    elif rel in index_paths:
        out["index_state"] = "indexed_missing"
        out["notes"].append("path in corpus index but absent on disk")
    else:
        out["index_state"] = "not_indexed"
    return out


@dataclass
class AnnotateReviewResult:
    ok: bool
    message: str
    path: Path | None = None


def annotate_handoff_review_note(
    repo_root: Path,
    capture_id: str,
    *,
    note: str,
    reviewer: str = "batch_advisory_cli",
) -> AnnotateReviewResult:
    """Append maintainer note to handoff_review.json — explicit write path only."""
    from rt_sandbox.sa_handoff import write_handoff_review_v1

    staging = capture_staging_dir(repo_root, capture_id)
    if not staging.is_dir():
        return AnnotateReviewResult(False, "staging not found")
    review_path = staging / "handoff_review.json"
    existing_notes = ""
    decision = "ready_for_approval"
    if review_path.is_file():
        review = _load_json(review_path)
        existing_notes = review.get("notes") or ""
        decision = review.get("decision") or decision
    combined = (
        f"{existing_notes}\n[batch note] {note}".strip() if existing_notes else note
    )
    write_handoff_review_v1(
        staging,
        capture_id=capture_id,
        decision=decision,
        reviewer=reviewer,
        notes=combined,
        repo_root=repo_root,
    )
    return AnnotateReviewResult(True, "review note updated", review_path)


def _dry_run_states_filter() -> frozenset[str]:
    return frozenset({"import_ready", "handoff_ready"})


def run_dry_run_review_for_capture(
    repo_root: Path,
    capture_id: str,
    *,
    write_preview: bool = False,
) -> dict[str, Any]:
    """Import pipeline dry-run for one capture — never commits to corpus."""
    from rt_sandbox.export_boundary import CONVERSION_STEPS
    from rt_sandbox.sa_handoff import sa_handoff_dir

    row = derive_capture_row(repo_root, capture_id)
    if row.error:
        return {
            "capture_candidate_id": capture_id,
            "status": "error",
            "eligible_state": False,
            "advisory_state": None,
            "rc": 1,
            "steps": [],
            "skipped_reason": row.error,
        }

    state = row.advisory.get("advisory_state")
    if state not in _dry_run_states_filter():
        return {
            "capture_candidate_id": capture_id,
            "status": "skipped",
            "eligible_state": False,
            "advisory_state": state if isinstance(state, str) else None,
            "rc": 0,
            "steps": [],
            "skipped_reason": f"advisory_state={state} not in dry-run review set",
        }

    try:
        from rt_sa_import import cmd_prepare, cmd_run_step  # type: ignore[import-untyped]
    except ImportError:
        scripts_rt = repo_root / "scripts" / "rt"
        import sys

        if str(scripts_rt) not in sys.path:
            sys.path.insert(0, str(scripts_rt))
        from rt_sa_import import cmd_prepare, cmd_run_step  # type: ignore[import-untyped]

    steps: list[dict[str, Any]] = []
    rc_prep = cmd_prepare(repo_root, capture_id, dry_run=True)
    steps.append({"phase": "prepare", "rc": rc_prep, "dry_run": True})
    if rc_prep != 0:
        return {
            "capture_candidate_id": capture_id,
            "status": "ran",
            "eligible_state": True,
            "advisory_state": state if isinstance(state, str) else None,
            "rc": rc_prep,
            "steps": steps,
        }

    rc = 0
    for step in CONVERSION_STEPS:
        step_rc = cmd_run_step(repo_root, capture_id, step, dry_run=True)
        steps.append({"phase": step, "rc": step_rc, "dry_run": True})
        if step_rc != 0:
            rc = step_rc
            break

    out: dict[str, Any] = {
        "capture_candidate_id": capture_id,
        "status": "ran",
        "eligible_state": True,
        "advisory_state": state if isinstance(state, str) else None,
        "rc": rc,
        "steps": steps,
    }
    if write_preview:
        handoff_path = sa_handoff_dir(repo_root, capture_id)
        preview_path = handoff_path / "dry_run_preview.json"
        preview_path.parent.mkdir(parents=True, exist_ok=True)
        preview_body = {
            "schema": "rt_sa_import_dry_run_preview_v1",
            "capture_candidate_id": capture_id,
            "dry_run": True,
            "governance_banner": ADVISORY_GOVERNANCE_BANNER,
            "steps": steps,
            "commit": "forbidden — use rt_sa_import commit --corpus-dest explicitly",
        }
        preview_path.write_text(
            json.dumps(preview_body, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        out["preview_path"] = str(preview_path.relative_to(repo_root))
    return out


def build_dry_run_review_document(
    repo_root: Path,
    rows: list[CaptureAdvisoryRow],
    *,
    max_captures: int = 10,
    states: frozenset[str] | None = None,
    write_preview: bool = False,
) -> dict[str, Any]:
    allowed = states or _dry_run_states_filter()
    captures_out: list[dict[str, Any]] = []
    run_count = 0
    for row in rows:
        if row.error:
            captures_out.append(
                {
                    "capture_candidate_id": row.capture_candidate_id,
                    "status": "skipped",
                    "eligible_state": False,
                    "advisory_state": None,
                    "rc": 0,
                    "steps": [],
                    "skipped_reason": row.error,
                }
            )
            continue
        adv_state = row.advisory.get("advisory_state")
        if adv_state not in allowed:
            captures_out.append(
                {
                    "capture_candidate_id": row.capture_candidate_id,
                    "status": "skipped",
                    "eligible_state": False,
                    "advisory_state": adv_state if isinstance(adv_state, str) else None,
                    "rc": 0,
                    "steps": [],
                    "skipped_reason": f"state {adv_state} not in filter",
                }
            )
            continue
        if run_count >= max_captures:
            captures_out.append(
                {
                    "capture_candidate_id": row.capture_candidate_id,
                    "status": "skipped",
                    "eligible_state": True,
                    "advisory_state": adv_state if isinstance(adv_state, str) else None,
                    "rc": 0,
                    "steps": [],
                    "skipped_reason": "max_captures limit reached",
                }
            )
            continue
        result = run_dry_run_review_for_capture(
            repo_root,
            row.capture_candidate_id,
            write_preview=write_preview,
        )
        captures_out.append(result)
        run_count += 1

    return {
        "schema": ADVISORY_DRY_RUN_REVIEW_SCHEMA,
        "generated_at": _utc_now(),
        "repo_root": str(repo_root.resolve()),
        "dry_run": True,
        "governance_banner": ADVISORY_GOVERNANCE_BANNER,
        "max_captures": max_captures,
        "state_filter": sorted(allowed),
        "write_preview": write_preview,
        "captures": captures_out,
    }


def repo_root_or_default(path: Path | None) -> Path:
    return path if path is not None else repo_root_from(Path.cwd())
