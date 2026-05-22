#!/usr/bin/env python3
"""Audit replay corpus provenance integrity (PLAT-SA-F1b)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from replay_corpus_lineage import CORPUS_ID, load_corpus_index, normalize_entry_id  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]
_SA = _REPO / "fixtures/sa_r0"


def audit_corpus_provenance(*, repo_root: Path | None = None) -> list[str]:
    issues: list[str] = []
    root = repo_root or _REPO
    index = load_corpus_index(root)
    if not index:
        return ["missing corpus index"]

    index_rev = index.get("index_revision") or ""
    entry_ids = {e.get("entry_id") for e in index.get("entries") or [] if e.get("entry_id")}

    rb_dir = _SA / "research_bundles" / CORPUS_ID
    manifest_path = rb_dir / "manifest.json"
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        ref = manifest.get("corpus_ref")
        if ref:
            if ref.get("entry_id") not in entry_ids:
                issues.append(f"research bundle corpus_ref unknown entry_id: {ref.get('entry_id')}")
            if ref.get("index_revision") and ref.get("index_revision") != index_rev:
                issues.append("research bundle corpus_ref index_revision stale")
        paths = {e["path"] for e in manifest.get("included_files") or []}
        if "synthesis/replay_corpus_index_v1.json" not in paths:
            issues.append("research bundle missing synthesis/replay_corpus_index_v1.json in included_files")

    release_manifest = _SA / "corpus_releases" / f"{CORPUS_ID}_r1" / "replay_corpus_release_manifest_v1.json"
    if release_manifest.is_file():
        rel = json.loads(release_manifest.read_text(encoding="utf-8"))
        indexed = set(rel.get("indexed_entry_ids") or [])
        if indexed != entry_ids:
            missing = entry_ids - indexed
            extra = indexed - entry_ids
            if missing:
                issues.append(f"release manifest missing entry ids: {sorted(missing)[:3]}…")
            if extra:
                issues.append(f"release manifest extra entry ids: {sorted(extra)[:3]}…")

    synth = _SA / "synthesis/cross_sweep_synthesis_v1.json"
    if synth.is_file():
        data = json.loads(synth.read_text(encoding="utf-8"))
        ref = data.get("corpus_ref")
        eid = normalize_entry_id("synthesis_report", "cross_sweep_synthesis_v1")
        if ref and ref.get("entry_id") != eid:
            issues.append("synthesis corpus_ref entry_id mismatch")

    prov_path = rb_dir / "provenance/generator_versions.json"
    if prov_path.is_file():
        gens = json.loads(prov_path.read_text(encoding="utf-8"))
        for key in ("build_replay_corpus_index.py", "export_research_bundle.py"):
            if key not in gens:
                issues.append(f"generator_versions missing {key}")

    return issues


def main() -> None:
    ap = argparse.ArgumentParser(description="Audit replay corpus provenance")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    issues = audit_corpus_provenance()
    if args.json:
        print(json.dumps({"ok": not issues, "issues": issues}, indent=2))
    else:
        for issue in issues:
            print(issue)

    if issues:
        raise SystemExit(1)
    print("audit replay corpus provenance OK")


if __name__ == "__main__":
    main()
