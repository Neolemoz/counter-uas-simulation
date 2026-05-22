#!/usr/bin/env python3
"""Audit replay federation integrity across corpus groups (PLAT-SA-F2A)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from replay_federation_lineage import (  # noqa: E402
    FEDERATION_ID,
    INTEGRITY_GOVERNANCE,
    build_federation_manifest,
    federation_revision_fingerprint,
    load_corpus_index,
    validate_federation_lineage_dag,
)
from validate_replay_federation import validate_federation_manifest  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]
REPORT_PATH = _REPO / "fixtures/sa_r0/federation/audits/replay_federation_integrity_report_v1.json"


def _entry_artifact_key(entry: dict[str, Any]) -> tuple[str, str] | None:
    path = entry.get("primary_artifact_path")
    sha = entry.get("sha256")
    if path and sha:
        return (path, sha)
    return None


def audit_federation_integrity(
    repo_root: Path,
    *,
    strict: bool = False,
) -> dict[str, Any]:
    manifest = build_federation_manifest(repo_root)
    groups = manifest.get("corpus_groups") or []
    findings: list[dict[str, Any]] = []

    val_errors, val_warnings = validate_federation_manifest(manifest, repo_root=repo_root)
    for msg in val_errors:
        kind = "orphan_corpus_group" if "orphan" in msg else "lineage_continuity_break"
        findings.append({"kind": kind, "severity": "error", "message": msg})
    for msg in val_warnings:
        findings.append({"kind": "corpus_ref_registry_mismatch", "severity": "warning", "message": msg})

    lineage_path = repo_root / "fixtures/sa_r0/federation/replay_federation_lineage_graph_v1.json"
    if lineage_path.is_file():
        graph = json.loads(lineage_path.read_text(encoding="utf-8"))
        for msg in validate_federation_lineage_dag(groups, graph.get("edges") or []):
            findings.append(
                {"kind": "lineage_continuity_break", "severity": "error", "message": msg}
            )

    parent = manifest.get("parent_federation_ref")
    if parent:
        parent_path = repo_root / parent
        if not parent_path.is_file():
            findings.append(
                {
                    "kind": "stale_federation_ref",
                    "severity": "error",
                    "message": f"missing parent_federation_ref: {parent}",
                }
            )

    artifact_map: dict[tuple[str, str], list[str]] = {}
    for group in groups:
        gid = group["corpus_group_id"]
        try:
            index = load_corpus_index(repo_root, group["index_artifact_path"])
        except FileNotFoundError:
            continue
        for entry in index.get("entries") or []:
            key = _entry_artifact_key(entry)
            if key:
                artifact_map.setdefault(key, []).append(gid)

    for key, gids in sorted(artifact_map.items()):
        if len(gids) > 1:
            note = None
            if set(gids) == {"canonical_r1", "release_r1_snapshot"}:
                note = "expected release snapshot duplication"
            findings.append(
                {
                    "kind": "replay_duplication_across_groups",
                    "severity": "warning",
                    "message": f"{key[0]} appears in groups {gids}",
                    "supersession_note": note,
                    "groups": gids,
                }
            )

    pub_path = repo_root / "fixtures/sa_r0/federation/replay_federation_publication_collection_v1.json"
    if pub_path.is_file():
        collection = json.loads(pub_path.read_text(encoding="utf-8"))
        for member in collection.get("members") or []:
            apath = member.get("artifact_path")
            if not apath:
                continue
            full = repo_root / apath
            if not full.is_file():
                findings.append(
                    {
                        "kind": "publication_continuity_break",
                        "severity": "error",
                        "message": f"missing publication member: {apath}",
                    }
                )

    for ref in manifest.get("federation_lineage_refs") or []:
        if lineage_path.is_file():
            graph = json.loads(lineage_path.read_text(encoding="utf-8"))
            edge_ids = {e.get("edge_id") for e in graph.get("edges") or []}
            if ref not in edge_ids:
                findings.append(
                    {
                        "kind": "stale_federation_ref",
                        "severity": "error",
                        "message": f"federation_lineage_ref not in graph: {ref}",
                    }
                )

    if strict:
        for f in findings:
            if f.get("severity") == "warning" and f.get("kind") != "replay_duplication_across_groups":
                f["severity"] = "error"
            if (
                f.get("kind") == "replay_duplication_across_groups"
                and not f.get("supersession_note")
            ):
                f["severity"] = "error"

    errors = [f for f in findings if f.get("severity") == "error"]
    report = {
        "artifact_type": "replay_federation_integrity_report_v1",
        "schema_version": "replay_federation_integrity_report_v1",
        "federation_id": FEDERATION_ID,
        "generation_revision": "f2a_v1",
        "governance": INTEGRITY_GOVERNANCE,
        "findings": findings,
        "integrity_ok": not errors,
        "report_revision": federation_revision_fingerprint({"findings": findings}),
    }
    return report


def main() -> None:
    ap = argparse.ArgumentParser(description="Audit replay federation integrity")
    ap.add_argument("--strict", action="store_true")
    ap.add_argument("--check", action="store_true", help="verify committed report")
    ap.add_argument("--write", action="store_true", help="write integrity report fixture")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    report = audit_federation_integrity(_REPO, strict=args.strict)

    if args.write:
        REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
        REPORT_PATH.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        from replay_federation_lineage import sync_federation_viewer_mirrors  # noqa: E402

        sync_federation_viewer_mirrors(_REPO)
        print(f"wrote {REPORT_PATH}")

    if args.check:
        if not REPORT_PATH.is_file():
            raise SystemExit(f"missing report: {REPORT_PATH}")
        committed = json.loads(REPORT_PATH.read_text(encoding="utf-8"))
        expected = audit_federation_integrity(_REPO, strict=args.strict)
        if json.dumps(committed, sort_keys=True) != json.dumps(expected, sort_keys=True):
            raise SystemExit("federation integrity report stale — run with --write")
        if not committed.get("integrity_ok"):
            raise SystemExit("federation integrity_ok is false")
        print("audit_replay_federation_integrity: OK")
        return

    if args.json:
        print(json.dumps(report, indent=2))
    else:
        for f in report.get("findings") or []:
            print(f"{f.get('severity')}: {f.get('kind')}: {f.get('message')}")
        if not report.get("integrity_ok"):
            raise SystemExit(1)


if __name__ == "__main__":
    main()
