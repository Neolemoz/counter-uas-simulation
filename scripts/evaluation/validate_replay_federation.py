#!/usr/bin/env python3
"""Validate replay federation manifest and index structure (PLAT-SA-F2A)."""

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
    FEDERATION_REF_KINDS,
    load_corpus_index,
    validate_federation_lineage_dag,
)

_REPO = Path(__file__).resolve().parents[2]
DEFAULT_MANIFEST = _REPO / "fixtures/sa_r0/federation/replay_federation_manifest_v1.json"


def validate_federation_manifest(
    manifest: dict[str, Any],
    *,
    repo_root: Path,
) -> tuple[list[str], list[str]]:
    errors: list[str] = []
    warnings: list[str] = []

    if manifest.get("artifact_type") != "replay_federation_manifest_v1":
        errors.append("invalid artifact_type")
    if not manifest.get("federation_id"):
        errors.append("missing federation_id")
    groups = manifest.get("corpus_groups") or []
    if not groups:
        errors.append("corpus_groups empty")

    seen_groups: set[str] = set()
    for g in groups:
        gid = g.get("corpus_group_id")
        if not gid:
            errors.append("corpus_group missing corpus_group_id")
            continue
        if gid in seen_groups:
            errors.append(f"duplicate corpus_group_id: {gid}")
        seen_groups.add(gid)
        idx_path = g.get("index_artifact_path")
        if not idx_path:
            errors.append(f"group {gid} missing index_artifact_path")
            continue
        full = repo_root / idx_path
        if not full.is_file():
            errors.append(f"orphan_corpus_group: {idx_path}")
            continue
        try:
            idx = load_corpus_index(repo_root, idx_path)
            if idx.get("corpus_id") != g.get("corpus_id"):
                warnings.append(
                    f"corpus_id mismatch for group {gid}: manifest={g.get('corpus_id')} index={idx.get('corpus_id')}"
                )
        except Exception as exc:
            errors.append(f"failed to load index for {gid}: {exc}")

    lineage_path = repo_root / "fixtures/sa_r0/federation/replay_federation_lineage_graph_v1.json"
    if lineage_path.is_file():
        graph = json.loads(lineage_path.read_text(encoding="utf-8"))
        errors.extend(validate_federation_lineage_dag(groups, graph.get("edges") or []))
        for edge in graph.get("edges") or []:
            if edge.get("ref_kind") not in FEDERATION_REF_KINDS:
                errors.append(f"unknown ref_kind in graph: {edge.get('ref_kind')}")

    parent = manifest.get("parent_federation_ref")
    if parent:
        warnings.append(f"parent_federation_ref present ({parent}) — verify prior snapshot exists")

    return errors, warnings


def validate_federation_index(
    index: dict[str, Any],
    *,
    repo_root: Path,
) -> tuple[list[str], list[str]]:
    errors: list[str] = []
    warnings: list[str] = []

    if index.get("artifact_type") != "replay_federation_index_v1":
        errors.append("invalid federation index artifact_type")
    if not index.get("federation_revision"):
        errors.append("missing federation_revision")
    manifest_path = repo_root / (index.get("manifest_ref") or "")
    if not manifest_path.is_file():
        errors.append(f"missing manifest_ref: {manifest_path}")
    else:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        me, mw = validate_federation_manifest(manifest, repo_root=repo_root)
        errors.extend(me)
        warnings.extend(mw)

    summaries = index.get("corpus_group_summaries") or []
    if not summaries:
        errors.append("corpus_group_summaries empty")

    return errors, warnings


def main() -> None:
    ap = argparse.ArgumentParser(description="Validate replay federation artifacts")
    ap.add_argument("manifest_path", nargs="?", default=str(DEFAULT_MANIFEST))
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    path = Path(args.manifest_path)
    if not path.is_file():
        raise SystemExit(f"missing manifest: {path}")

    manifest = json.loads(path.read_text(encoding="utf-8"))
    errors, warnings = validate_federation_manifest(manifest, repo_root=_REPO)

    index_path = _REPO / "fixtures/sa_r0/federation/replay_federation_index_v1.json"
    if index_path.is_file():
        index = json.loads(index_path.read_text(encoding="utf-8"))
        ie, iw = validate_federation_index(index, repo_root=_REPO)
        errors.extend(ie)
        warnings.extend(iw)

    ok = not errors
    if args.json:
        print(json.dumps({"ok": ok, "errors": errors, "warnings": warnings}, indent=2))
    else:
        for w in warnings:
            print(f"warning: {w}")
        for e in errors:
            print(e)
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
