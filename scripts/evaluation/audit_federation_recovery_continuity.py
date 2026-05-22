#!/usr/bin/env python3
"""Audit federation-scoped async recovery continuity (PLAT-SA-F2A)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_sa_orchestration_recovery as recovery  # noqa: E402
from replay_federation_lineage import (  # noqa: E402
    FEDERATION_ID,
    RECOVERY_CONTINUITY_GOVERNANCE,
    build_federation_manifest,
    federation_revision_fingerprint,
)

_REPO = Path(__file__).resolve().parents[2]
REPORT_PATH = (
    _REPO / "fixtures/sa_r0/federation/audits/orchestration_federation_recovery_continuity_v1.json"
)
LINEAGE_PATH = _REPO / "fixtures/orchestration/reconciliation/reconciliation_lineage_index_v1.json"
BATCH_PATH = _REPO / "fixtures/orchestration/synthesis/async_batch_audit_v1.json"


def audit_federation_recovery_continuity(repo_root: Path) -> dict[str, Any]:
    manifest = build_federation_manifest(repo_root)
    issues: list[dict[str, Any]] = []

    lineage: dict[str, Any] = {}
    if LINEAGE_PATH.is_file():
        lineage = json.loads(LINEAGE_PATH.read_text(encoding="utf-8"))

    batch: dict[str, Any] = {}
    if BATCH_PATH.is_file():
        batch = json.loads(BATCH_PATH.read_text(encoding="utf-8"))

    registered_manifests: set[str] = set()
    for rel in sorted((repo_root / "fixtures/orchestration/manifests").glob("*.json")):
        data = json.loads(rel.read_text(encoding="utf-8"))
        mid = data.get("manifest_id") or data.get("experiment_id")
        if mid:
            registered_manifests.add(mid)

    for edge in lineage.get("supersede_edges") or []:
        mid = edge.get("from_manifest_id")
        if mid and mid not in registered_manifests:
            issues.append(
                {
                    "kind": "recovery_lineage_federation_break",
                    "severity": "error",
                    "message": f"supersede edge references unknown manifest: {mid}",
                    "corpus_group_id": edge.get("corpus_group_id") or "canonical_r1",
                }
            )

    for hold in lineage.get("quarantine_holds") or []:
        mid = hold.get("manifest_id")
        if mid:
            report_path = recovery.recovery_report_path(mid)
            if report_path.is_file():
                report = json.loads(report_path.read_text(encoding="utf-8"))
                if not report.get("quarantine_hold"):
                    issues.append(
                        {
                            "kind": "quarantine_federation_hold",
                            "severity": "warning",
                            "message": f"quarantine hold in lineage but report missing hold: {mid}",
                            "corpus_group_id": hold.get("corpus_group_id") or "canonical_r1",
                        }
                    )

    quarantined = batch.get("quarantined_manifest_ids") or []
    for mid in quarantined:
        report_path = recovery.recovery_report_path(mid)
        if not report_path.is_file():
            issues.append(
                {
                    "kind": "quarantine_federation_hold",
                    "severity": "error",
                    "message": f"batch quarantine without recovery report: {mid}",
                    "corpus_group_id": "canonical_r1",
                }
            )

    for mid in registered_manifests:
        report_path = recovery.recovery_report_path(mid)
        if not report_path.is_file():
            continue
        report = json.loads(report_path.read_text(encoding="utf-8"))
        if report.get("superseded") and report.get("recovery_continuity_ok") is False:
            issues.append(
                {
                    "kind": "reconciliation_continuity_propagation",
                    "severity": "warning",
                    "message": f"superseded manifest recovery_continuity_ok false: {mid}",
                    "corpus_group_id": "canonical_r1",
                }
            )

    errors = [i for i in issues if i.get("severity") == "error"]
    scopes = [
        {
            "corpus_group_id": g["corpus_group_id"],
            "study_label": g.get("study_label"),
            "orchestration_scope": g.get("orchestration_scope")
            or "fixtures/orchestration/",
        }
        for g in manifest.get("corpus_groups") or []
    ]

    return {
        "artifact_type": "orchestration_federation_recovery_continuity_v1",
        "schema_version": "orchestration_federation_recovery_continuity_v1",
        "federation_id": FEDERATION_ID,
        "governance": RECOVERY_CONTINUITY_GOVERNANCE,
        "corpus_group_scopes": scopes,
        "recovery_issues": issues,
        "continuity_ok": not errors,
        "batch_audit_ref": "fixtures/orchestration/synthesis/async_batch_audit_v1.json",
        "lineage_index_ref": "fixtures/orchestration/reconciliation/reconciliation_lineage_index_v1.json",
        "report_revision": federation_revision_fingerprint({"issues": issues}),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="Audit federation recovery continuity")
    ap.add_argument("--strict", action="store_true")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    report = audit_federation_recovery_continuity(_REPO)
    if args.strict:
        for issue in report.get("recovery_issues") or []:
            if issue.get("severity") == "warning":
                issue["severity"] = "error"
        report["continuity_ok"] = not any(
            i.get("severity") == "error" for i in report.get("recovery_issues") or []
        )

    if args.write:
        REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
        REPORT_PATH.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        from replay_federation_lineage import sync_federation_viewer_mirrors  # noqa: E402

        sync_federation_viewer_mirrors(_REPO)
        print(f"wrote {REPORT_PATH}")

    if args.check:
        if not REPORT_PATH.is_file():
            raise SystemExit(f"missing: {REPORT_PATH}")
        committed = json.loads(REPORT_PATH.read_text(encoding="utf-8"))
        expected = audit_federation_recovery_continuity(_REPO)
        if args.strict:
            for issue in expected.get("recovery_issues") or []:
                if issue.get("severity") == "warning":
                    issue["severity"] = "error"
            expected["continuity_ok"] = not any(
                i.get("severity") == "error" for i in expected.get("recovery_issues") or []
            )
        if json.dumps(committed, sort_keys=True) != json.dumps(expected, sort_keys=True):
            raise SystemExit("recovery continuity report stale — run with --write")
        if not committed.get("continuity_ok"):
            raise SystemExit("continuity_ok is false")
        print("audit_federation_recovery_continuity: OK")
        return

    if args.json:
        print(json.dumps(report, indent=2))
    else:
        for issue in report.get("recovery_issues") or []:
            print(f"{issue.get('severity')}: {issue.get('kind')}: {issue.get('message')}")
        if not report.get("continuity_ok"):
            raise SystemExit(1)


if __name__ == "__main__":
    main()
