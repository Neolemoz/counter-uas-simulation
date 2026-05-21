#!/usr/bin/env python3
"""Build replay corpus evolution manifest and summary (PLAT-SA-F1d)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from build_cross_sweep_synthesis import SWEEP_IDS  # noqa: E402
from build_replay_corpus_index import build_corpus_index  # noqa: E402
from replay_corpus_lineage import (  # noqa: E402
    CORPUS_ID,
    EVOLUTION_GENERATION_REVISION,
    EVOLUTION_GOVERNANCE,
    build_chronology_structure,
    build_cross_release_diff_chain,
    discover_corpus_releases,
    rollup_sweep_evolution,
    write_viewer_mirror,
)

_REPO = Path(__file__).resolve().parents[2]
_SYNTH = _REPO / "fixtures/sa_r0/synthesis"
_MANIFEST_PATH = _SYNTH / "replay_corpus_evolution_manifest_v1.json"
_SUMMARY_PATH = _SYNTH / "replay_corpus_evolution_summary_v1.json"
_DIFF_FIXTURE = _REPO / "fixtures/sa_r0/corpus_audits/replay_corpus_release_diff_v1.json"


def _build_family_narratives(index: dict[str, Any]) -> list[dict[str, Any]]:
    by_family: dict[str, list[str]] = {}
    for entry in index.get("entries") or []:
        fam = entry.get("replay_family") or entry.get("entry_kind") or "unknown"
        by_family.setdefault(fam, []).append(entry["entry_id"])

    narratives: list[dict[str, Any]] = []
    for fam in sorted(by_family.keys()):
        eids = sorted(by_family[fam])
        kinds = sorted({e.split("__", 1)[0] for e in eids if "__" in e})
        narratives.append(
            {
                "replay_family": fam,
                "entry_ids": eids,
                "entry_kinds": kinds,
                "summary": (
                    f"Replay family `{fam}` spans {len(eids)} corpus entries "
                    f"({', '.join(kinds[:4])}{'…' if len(kinds) > 4 else ''}) — "
                    "explanatory lineage grouping only."
                ),
                "caveat": "Family grouping does not imply tactical equivalence across experiments.",
            }
        )
    return narratives


def _build_divergence_chronology(index: dict[str, Any], sweep_rollups: dict[str, Any]) -> list[str]:
    bullets: list[str] = []
    for tier in build_chronology_structure(index):
        bullets.append(
            f"[{tier['tier_id']}] {tier['entry_count']} entries — {tier['descriptor']}."
        )
    for sid in SWEEP_IDS:
        notes = None
        for row in sweep_rollups.get("per_sweep") or []:
            if row.get("sweep_id") == sid:
                notes = row.get("pattern_notes")
                break
        if notes:
            bullets.append(f"Sweep `{sid}`: pattern rollup present in cross-sweep synthesis.")
        else:
            bullets.append(f"Sweep `{sid}`: sweep family indexed; see workstation for member replay detail.")
    bullets.append(
        "Divergence chronology is replay-local ordering — not causal inference or operational timeline."
    )
    return bullets


def _ambiguity_trend_rollup(index: dict[str, Any], sweep_rollups: dict[str, Any]) -> dict[str, Any]:
    tiers = build_chronology_structure(index)
    by_tier: dict[str, Any] = {}
    for tier in tiers:
        by_tier[tier["tier_id"]] = {
            "entry_count": tier["entry_count"],
            "release_generation_id": tier["release_generation_id"],
        }
    return {
        "chronology_tiers": by_tier,
        "shared_hotspot_cells": sweep_rollups.get("shared_ambiguity_hotspots") or [],
        "top_cells_by_sweep": sweep_rollups.get("top_cells_by_sweep") or {},
        "note": "Ambiguity concentration compared across sweep families — explanatory spatial analytics only.",
    }


def _pattern_evolution_rollup(index: dict[str, Any]) -> dict[str, Any]:
    by_family: dict[str, dict[str, Any]] = {}
    for entry in index.get("entries") or []:
        fam = entry.get("replay_family") or "unknown"
        slot = by_family.setdefault(
            fam,
            {"entry_count": 0, "entry_kinds": set(), "chronology_groups": set()},
        )
        slot["entry_count"] += 1
        slot["entry_kinds"].add(entry.get("entry_kind"))
        slot["chronology_groups"].add(entry.get("chronology_group"))
    return {
        fam: {
            "entry_count": data["entry_count"],
            "entry_kinds": sorted(data["entry_kinds"]),
            "chronology_groups": sorted(data["chronology_groups"]),
        }
        for fam, data in sorted(by_family.items())
    }


def build_evolution_manifest(index: dict[str, Any]) -> dict[str, Any]:
    releases = discover_corpus_releases(_REPO)
    release_rows: list[dict[str, Any]] = []
    for rel in releases:
        manifest = rel["manifest"]
        release_rows.append(
            {
                "release_id": rel["release_id"],
                "manifest_path": rel["manifest_path"],
                "index_path": rel["index_path"],
                "index_revision": (rel.get("index") or {}).get("index_revision"),
                "parent_release_ids": manifest.get("parent_release_ids") or [],
                "indexed_entry_count": len(manifest.get("indexed_entry_ids") or []),
            }
        )
    release_rows.append(
        {
            "release_id": "canonical_index",
            "manifest_path": None,
            "index_path": "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json",
            "index_revision": index.get("index_revision"),
            "parent_release_ids": [r["release_id"] for r in release_rows],
            "indexed_entry_count": len(index.get("entries") or []),
        }
    )

    cross_diffs = build_cross_release_diff_chain(releases, canonical_index=index)
    if _DIFF_FIXTURE.is_file():
        committed_diff_doc = json.loads(_DIFF_FIXTURE.read_text(encoding="utf-8"))
        cross_diffs.append(
            {
                "diff_id": "committed_release_diff",
                "baseline_id": committed_diff_doc.get("baseline_id"),
                "target_id": committed_diff_doc.get("target_id"),
                "artifact_path": _DIFF_FIXTURE.relative_to(_REPO).as_posix(),
                "release_behind_canonical": committed_diff_doc.get("release_behind_canonical"),
            }
        )

    return {
        "artifact_type": "replay_corpus_evolution_manifest_v1",
        "schema_version": "replay_corpus_evolution_manifest_v1",
        "corpus_id": CORPUS_ID,
        "generation_revision": EVOLUTION_GENERATION_REVISION,
        "governance": EVOLUTION_GOVERNANCE,
        "releases": release_rows,
        "chronology_tiers": build_chronology_structure(index),
        "cross_release_diffs": cross_diffs,
    }


def build_evolution_summary(index: dict[str, Any]) -> dict[str, Any]:
    sweep_rollups = rollup_sweep_evolution(_REPO, SWEEP_IDS)
    return {
        "artifact_type": "replay_corpus_evolution_summary_v1",
        "schema_version": "replay_corpus_evolution_summary_v1",
        "corpus_id": CORPUS_ID,
        "generation_revision": EVOLUTION_GENERATION_REVISION,
        "governance": EVOLUTION_GOVERNANCE,
        "topology_sensitivity_evolution": sweep_rollups,
        "ambiguity_trend_rollup": _ambiguity_trend_rollup(index, sweep_rollups),
        "pattern_evolution_rollup": _pattern_evolution_rollup(index),
        "divergence_chronology": _build_divergence_chronology(index, sweep_rollups),
        "long_horizon_family_narratives": _build_family_narratives(index),
    }


def write_evolution_artifacts(manifest: dict[str, Any], summary: dict[str, Any]) -> None:
    _SYNTH.mkdir(parents=True, exist_ok=True)
    manifest_text = json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    summary_text = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    _MANIFEST_PATH.write_text(manifest_text, encoding="utf-8")
    _SUMMARY_PATH.write_text(summary_text, encoding="utf-8")
    mirror = _REPO / "platform/sa-r0-viewer/public/demo/synthesis"
    mirror.mkdir(parents=True, exist_ok=True)
    write_viewer_mirror(_MANIFEST_PATH, mirror / "replay_corpus_evolution_manifest_v1.json")
    write_viewer_mirror(_SUMMARY_PATH, mirror / "replay_corpus_evolution_summary_v1.json")


def check_evolution_artifacts() -> None:
    index = build_corpus_index()
    expected_manifest = build_evolution_manifest(index)
    expected_summary = build_evolution_summary(index)
    if not _MANIFEST_PATH.is_file() or not _SUMMARY_PATH.is_file():
        raise SystemExit("missing evolution artifacts — run build_replay_corpus_evolution.py")
    actual_m = json.loads(_MANIFEST_PATH.read_text(encoding="utf-8"))
    actual_s = json.loads(_SUMMARY_PATH.read_text(encoding="utf-8"))
    if json.dumps(actual_m, sort_keys=True) != json.dumps(expected_manifest, sort_keys=True):
        raise SystemExit("replay_corpus_evolution_manifest_v1.json is stale")
    if json.dumps(actual_s, sort_keys=True) != json.dumps(expected_summary, sort_keys=True):
        raise SystemExit("replay_corpus_evolution_summary_v1.json is stale")


def main() -> None:
    ap = argparse.ArgumentParser(description="Build replay corpus evolution artifacts")
    ap.add_argument("--check", action="store_true", help="verify committed fixtures")
    args = ap.parse_args()

    if args.check:
        check_evolution_artifacts()
        print("corpus evolution check OK")
        return

    index = build_corpus_index()
    write_evolution_artifacts(build_evolution_manifest(index), build_evolution_summary(index))
    print("corpus evolution OK")


if __name__ == "__main__":
    main()
