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
from rt_sandbox.capture import list_staged_capture_ids
from rt_sandbox.sa_handoff import capture_staging_dir
from rt_sandbox.isolation import repo_root_from

BATCH_REVIEW_SCHEMA = "rt_handoff_batch_review_v1"
CORPUS_PREVIEW_SCHEMA = "rt_handoff_corpus_preview_v1"


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

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "capture_candidate_id": self.capture_candidate_id,
            "advisory": self.advisory,
            "next_cli": self.next_cli,
            "workflow_phase": self.workflow_phase,
        }
        if self.error:
            out["error"] = self.error
        return out


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
    return {
        "schema": BATCH_REVIEW_SCHEMA,
        "generated_at": _utc_now(),
        "repo_root": str(repo_root.resolve()),
        "dry_run": dry_run,
        "governance_banner": ADVISORY_GOVERNANCE_BANNER,
        "summary": aggregate_report(rows),
        "captures": [r.to_dict() for r in rows],
    }


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
        "would_add": [],
        "would_conflict": [],
        "missing_staging_ref": [],
        "notes": [],
    }
    if not staging.is_dir():
        out["missing_staging_ref"].append("capture_staging_dir")
        return out

    proposed: Path | None = corpus_dest
    conversion_path = staging / "conversion.json"
    if proposed is None and conversion_path.is_file():
        conv = _load_json(conversion_path)
        ref = conv.get("corpus_dest") or conv.get("proposed_corpus_dest")
        if isinstance(ref, str):
            proposed = repo_root / ref if not ref.startswith("/") else Path(ref)

    handoff_path = repo_root / "runs" / "rt_sandbox" / "sa_handoff" / capture_id
    manifest_path = handoff_path / "handoff_manifest.json"
    if proposed is None and manifest_path.is_file():
        out["notes"].append("no corpus_dest in conversion; pass --corpus-dest")

    if proposed is None:
        proposed = repo_root / "fixtures" / "sa_r0" / f"demo_{capture_id}"

    rel = str(proposed.resolve().relative_to(repo_root.resolve()))
    index_paths = _corpus_index_paths(repo_root)
    dest_exists = proposed.exists()
    if dest_exists:
        out["would_conflict"].append(rel)
    else:
        out["would_add"].append(rel)
    if rel in index_paths and dest_exists:
        out["notes"].append("path already indexed and exists on disk")
    elif rel in index_paths:
        out["notes"].append("path in corpus index but absent on disk")
    bundle_src = handoff_path / "bundle" / "index.json"
    if not bundle_src.is_file():
        out["missing_staging_ref"].append("handoff_bundle_index")
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


def repo_root_or_default(path: Path | None) -> Path:
    return path if path is not None else repo_root_from(Path.cwd())
