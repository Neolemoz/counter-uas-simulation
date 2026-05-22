#!/usr/bin/env python3
"""Async recovery, reconciliation audits, and batch review artifacts (PLAT-SA-I3)."""

from __future__ import annotations

import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_sa_orchestration_async as orch_async  # noqa: E402
import replay_sa_orchestration_ops as orch_ops  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]
_ORCH = _REPO / "fixtures" / "orchestration"
_RECOVERY = _ORCH / "recovery"
_RECONCILIATION = _ORCH / "reconciliation"
_SYNTHESIS = _ORCH / "synthesis"

RECOVERY_REPORT_TYPE = "orchestration_recovery_report_v1"
RECOVERY_REPORT_VERSION = "1"
BATCH_AUDIT_TYPE = "orchestration_async_batch_audit_v1"
BATCH_AUDIT_VERSION = "1"
LINEAGE_INDEX_TYPE = "orchestration_reconciliation_lineage_index_v1"
LINEAGE_INDEX_VERSION = "1"
RECOVERY_INTEGRITY_TYPE = "orchestration_recovery_integrity_report_v1"
RECOVERY_INTEGRITY_VERSION = "1"

GOVERNANCE_DEFAULT: dict[str, Any] = {
    "notice": "Recovery and reconciliation artifacts for offline reviewer cognition only.",
    "anti_claims": [
        "not live recovery authority",
        "not parser contract",
        "not operational readiness",
    ],
}


def _iso_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _read_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"expected JSON object: {path}")
    return data


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def recovery_report_path(manifest_id: str) -> Path:
    return _RECOVERY / f"{manifest_id}_recovery_report.json"


def build_retry_chain(manifest_id: str) -> list[dict[str, Any]]:
    """Ordered chain: claim → workers → async lineage events."""
    chain: list[dict[str, Any]] = []
    async_m = orch_async.load_async_manifest(manifest_id)
    if not async_m:
        return chain

    cref = async_m.get("queue_claim_ref")
    if cref:
        cp = _REPO / cref
        if cp.is_file():
            claim = _read_json(cp)
            chain.append(
                {
                    "hop_type": "claim",
                    "claim_id": claim.get("claim_id"),
                    "claim_status": claim.get("claim_status"),
                    "snapshot_hash": claim.get("snapshot_hash"),
                    "ref": cref,
                }
            )

    for wref in async_m.get("worker_execution_refs") or []:
        wp = _REPO / wref
        if not wp.is_file():
            chain.append({"hop_type": "worker", "ref": wref, "missing": True})
            continue
        worker = _read_json(wp)
        chain.append(
            {
                "hop_type": "worker",
                "worker_id": worker.get("worker_id"),
                "execution_attempt": worker.get("execution_attempt"),
                "execution_fingerprint": worker.get("execution_fingerprint"),
                "retry_parent_ref": worker.get("retry_parent_ref"),
                "ref": wref,
            }
        )

    for event in async_m.get("async_lineage") or []:
        chain.append(
            {
                "hop_type": "lineage",
                "event_id": event.get("event_id"),
                "from_status": event.get("from_status"),
                "to_status": event.get("to_status"),
                "notes": event.get("notes"),
            }
        )

    return chain


def audit_retry_lineage(manifest_id: str, *, strict: bool) -> list[dict[str, Any]]:
    issues: list[dict[str, Any]] = []
    async_m = orch_async.load_async_manifest(manifest_id)
    if not async_m:
        return issues

    status = async_m.get("async_execution_status")
    lineage = async_m.get("async_lineage") or []
    to_statuses = [e.get("to_status") for e in lineage]

    if status == "retrying" and "failed" not in to_statuses:
        issues.append(
            {
                "kind": "retry_lineage_gap",
                "manifest_id": manifest_id,
                "message": "retrying without prior failed lineage event",
            }
        )

    workers = async_m.get("worker_execution_refs") or []
    for wref in workers:
        wp = _REPO / wref
        if not wp.is_file():
            issues.append(
                {
                    "kind": "retry_lineage_gap",
                    "manifest_id": manifest_id,
                    "message": f"missing worker ref in retry chain: {wref}",
                }
            )

    for i, wref in enumerate(workers):
        wp = _REPO / wref
        if not wp.is_file():
            continue
        worker = _read_json(wp)
        parent = worker.get("retry_parent_ref")
        if worker.get("execution_attempt", 1) > 1 and not parent:
            msg = "execution_attempt > 1 without retry_parent_ref"
            issues.append(
                {"kind": "retry_lineage_gap", "manifest_id": manifest_id, "message": msg}
            )
        if parent and i > 0:
            prev = workers[i - 1]
            if parent != prev:
                issues.append(
                    {
                        "kind": "retry_lineage_gap",
                        "manifest_id": manifest_id,
                        "message": f"retry_parent_ref does not chain to prior worker: {parent}",
                    }
                )

    return issues


def audit_replay_supersession(manifest_id: str, *, strict: bool) -> list[dict[str, Any]]:
    issues: list[dict[str, Any]] = []
    async_m = orch_async.load_async_manifest(manifest_id)
    if not async_m:
        return issues

    if async_m.get("async_execution_status") != "superseded":
        return issues

    cref = async_m.get("queue_claim_ref")
    if cref and (_REPO / cref).is_file():
        claim = _read_json(_REPO / cref)
        if claim.get("claim_status") == orch_async.CLAIM_OPEN:
            issues.append(
                {
                    "kind": "superseded_claim",
                    "manifest_id": manifest_id,
                    "message": "open claim on superseded async snapshot",
                }
            )

    return issues


def audit_fingerprint_reconciliation(manifest_id: str, *, strict: bool) -> list[dict[str, Any]]:
    issues: list[dict[str, Any]] = []
    warnings: list[dict[str, Any]] = []
    async_m = orch_async.load_async_manifest(manifest_id)
    if not async_m:
        return issues

    stored_async_fp = async_m.get("execution_fingerprint")
    for wref in async_m.get("worker_execution_refs") or []:
        wp = _REPO / wref
        if not wp.is_file():
            continue
        worker = _read_json(wp)
        wfp = worker.get("execution_fingerprint")
        if stored_async_fp and wfp and stored_async_fp != wfp:
            issues.append(
                {
                    "kind": "fingerprint_reconciliation",
                    "manifest_id": manifest_id,
                    "message": f"worker {wref} execution_fingerprint differs from async sidecar",
                }
            )

    mf = orch_ops.manifest_path_from_id(manifest_id)
    if mf:
        repro = orch_async.verify_replay_reproducibility(mf, async_m=async_m)
        for msg in repro.get("issues") or []:
            issues.append(
                {"kind": "replay_replacement", "manifest_id": manifest_id, "message": msg}
            )

    status = async_m.get("async_execution_status")
    if status not in ("failed", "retrying", "quarantined", "superseded"):
        ops = orch_ops.load_ops_manifest(manifest_id) or {}
        if ops.get("operations_status") == "replay_generated":
            if not async_m.get("replay_fingerprint"):
                issues.append(
                    {
                        "kind": "recovered_replay_equivalence",
                        "manifest_id": manifest_id,
                        "message": "replay_generated without stored replay_fingerprint",
                    }
                )

    _ = warnings
    return issues


def audit_recovery_continuity(manifest_id: str, *, strict: bool) -> list[dict[str, Any]]:
    issues: list[dict[str, Any]] = []
    mf = orch_ops.manifest_path_from_id(manifest_id)
    if not mf:
        return issues

    continuity = orch_async.queue_execution_continuity(mf)
    computed = continuity.get("computed_execution_fingerprint")
    stored = continuity.get("execution_fingerprint")
    if computed and stored and computed != stored:
        issues.append(
            {
                "kind": "recovery_continuity",
                "manifest_id": manifest_id,
                "message": "execution fingerprint mismatch in continuity report",
            }
        )

    issues.extend(audit_retry_lineage(manifest_id, strict=strict))
    issues.extend(audit_replay_supersession(manifest_id, strict=strict))
    issues.extend(audit_fingerprint_reconciliation(manifest_id, strict=strict))

    async_result = orch_async.audit_async_manifest(manifest_id, strict=strict)
    for item in async_result.get("issues") or []:
        if item.get("kind") in ("partial_execution", "stale_replay", "replay_continuity"):
            issues.append(item)

    return issues


def build_recovery_report(manifest_id: str) -> dict[str, Any]:
    async_m = orch_async.load_async_manifest(manifest_id)
    recovery_issues = audit_recovery_continuity(manifest_id, strict=False)
    replay_recon: dict[str, Any] = {}
    mf = orch_ops.manifest_path_from_id(manifest_id)
    if mf:
        replay_recon = orch_async.verify_replay_reproducibility(mf, async_m=async_m)

    return {
        "artifact_type": RECOVERY_REPORT_TYPE,
        "schema_version": RECOVERY_REPORT_VERSION,
        "manifest_id": manifest_id,
        "generated_at": _iso_now(),
        "async_execution_status": async_m.get("async_execution_status") if async_m else None,
        "retry_chain": build_retry_chain(manifest_id),
        "recovery_issues": recovery_issues,
        "recovery_warnings": [],
        "replay_reconciliation": replay_recon,
        "recovery_continuity_ok": len(recovery_issues) == 0,
        "superseded": (async_m or {}).get("async_execution_status") == "superseded",
        "quarantine_hold": orch_async.is_quarantined(manifest_id),
        "governance": dict(GOVERNANCE_DEFAULT),
        "governance_banner": "RECOVERY REPORT — explanatory; CLI authoritative",
    }


def write_recovery_report(manifest_id: str) -> Path:
    report = build_recovery_report(manifest_id)
    out = recovery_report_path(manifest_id)
    _write_json(out, report)
    return out


def build_reconciliation_lineage_index() -> dict[str, Any]:
    retry_groups: list[dict[str, Any]] = []
    supersede_edges: list[dict[str, Any]] = []
    quarantine_holds: list[dict[str, Any]] = []
    failed_executions: list[dict[str, Any]] = []

    for manifest_id in orch_async._list_async_manifest_ids():
        async_m = orch_async.load_async_manifest(manifest_id)
        if not async_m:
            continue
        status = async_m.get("async_execution_status")
        workers = list(async_m.get("worker_execution_refs") or [])
        if workers:
            retry_groups.append(
                {
                    "manifest_id": manifest_id,
                    "worker_refs": workers,
                    "attempt_count": len(workers),
                    "corpus_group_id": "canonical_r1",
                }
            )
        if status == "quarantined":
            quarantine_holds.append(
                {
                    "manifest_id": manifest_id,
                    "reason": "async_execution_status quarantined",
                    "corpus_group_id": "canonical_r1",
                }
            )
        if status in ("failed", "retrying"):
            failed_executions.append(
                {"manifest_id": manifest_id, "async_execution_status": status}
            )
        if status == "superseded":
            for event in async_m.get("async_lineage") or []:
                notes = str(event.get("notes") or "")
                if "successor" in notes.lower() or "supersede" in notes.lower():
                    supersede_edges.append(
                        {
                            "from_manifest_id": manifest_id,
                            "to_manifest_id": notes,
                            "notes": notes,
                            "corpus_group_id": "canonical_r1",
                        }
                    )
            if not supersede_edges or supersede_edges[-1].get("from_manifest_id") != manifest_id:
                supersede_edges.append(
                    {
                        "from_manifest_id": manifest_id,
                        "to_manifest_id": "",
                        "notes": "superseded async snapshot (read-only)",
                        "corpus_group_id": "canonical_r1",
                    }
                )

    return {
        "artifact_type": LINEAGE_INDEX_TYPE,
        "schema_version": LINEAGE_INDEX_VERSION,
        "generated_at": _iso_now(),
        "retry_groups": retry_groups,
        "supersede_edges": supersede_edges,
        "quarantine_holds": quarantine_holds,
        "failed_executions": failed_executions,
        "governance": dict(GOVERNANCE_DEFAULT),
        "governance_banner": "RECONCILIATION LINEAGE INDEX — read-only navigation",
    }


def write_reconciliation_lineage_index() -> Path:
    index = build_reconciliation_lineage_index()
    out = _RECONCILIATION / "reconciliation_lineage_index_v1.json"
    _write_json(out, index)
    return out


def build_async_batch_audit(recovery_report: dict[str, Any]) -> dict[str, Any]:
    status_counts: dict[str, int] = {}
    failed_ids: list[str] = []
    quarantined_ids: list[str] = []

    for manifest_id in orch_async._list_async_manifest_ids():
        async_m = orch_async.load_async_manifest(manifest_id)
        if not async_m:
            continue
        st = async_m.get("async_execution_status") or "unset"
        status_counts[st] = status_counts.get(st, 0) + 1
        if st in ("failed", "retrying"):
            failed_ids.append(manifest_id)
        if st == "quarantined":
            quarantined_ids.append(manifest_id)

    issues = recovery_report.get("issues") or []
    warnings = recovery_report.get("warnings") or []

    return {
        "artifact_type": BATCH_AUDIT_TYPE,
        "schema_version": BATCH_AUDIT_VERSION,
        "generated_at": recovery_report.get("checked_at") or _iso_now(),
        "ok": recovery_report.get("ok", True),
        "async_manifest_count": len(orch_async._list_async_manifest_ids()),
        "status_counts": status_counts,
        "recovery_issue_count": len(issues),
        "recovery_warning_count": len(warnings),
        "failed_manifest_ids": sorted(set(failed_ids)),
        "quarantined_manifest_ids": sorted(quarantined_ids),
        "governance": dict(GOVERNANCE_DEFAULT),
        "governance_banner": "ASYNC BATCH AUDIT — explanatory export only",
    }


def run_recovery_integrity_audit(*, strict: bool = False) -> dict[str, Any]:
    all_issues: list[dict[str, Any]] = []
    all_warnings: list[dict[str, Any]] = []
    per_manifest: dict[str, Any] = {}

    for manifest_id in orch_async._list_async_manifest_ids():
        issues = audit_recovery_continuity(manifest_id, strict=strict)
        per_manifest[manifest_id] = {
            "manifest_id": manifest_id,
            "issues": issues,
            "retry_chain": build_retry_chain(manifest_id),
        }
        all_issues.extend(issues)

    ok = len(all_issues) == 0
    return {
        "artifact_type": RECOVERY_INTEGRITY_TYPE,
        "schema_version": RECOVERY_INTEGRITY_VERSION,
        "checked_at": _iso_now(),
        "strict": strict,
        "ok": ok,
        "issues": all_issues,
        "warnings": all_warnings,
        "per_manifest": per_manifest,
        "governance": dict(GOVERNANCE_DEFAULT),
        "governance_banner": "RECOVERY INTEGRITY — explanatory audit; CLI authoritative",
    }


def refresh_recovery_artifacts(*, strict: bool = False) -> dict[str, Any]:
    """Write per-manifest recovery reports, lineage index, and batch audit."""
    for manifest_id in orch_async._list_async_manifest_ids():
        write_recovery_report(manifest_id)
    write_reconciliation_lineage_index()
    recovery = run_recovery_integrity_audit(strict=strict)
    batch = build_async_batch_audit(recovery)
    batch_path = _SYNTHESIS / "async_batch_audit_v1.json"
    _write_json(batch_path, batch)
    recovery_path = _RECOVERY / "recovery_integrity_report_v1.json"
    _write_json(recovery_path, recovery)
    return {"recovery": recovery, "batch": batch, "batch_path": str(batch_path.relative_to(_REPO))}
