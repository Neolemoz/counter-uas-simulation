#!/usr/bin/env python3
"""Build deterministic replay corpus index (replay_corpus_index_v1)."""

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
from replay_corpus_lineage import (  # noqa: E402
    CHRONOLOGY_DESCRIPTORS,
    CORPUS_ID,
    GOVERNANCE,
    INDEX_GENERATION_REVISION,
    RELEASE_GENERATION_IDS,
    build_derivation_ref,
    build_lineage_edge,
    entry_content_revision,
    normalize_entry_id,
    normalize_lineage_parents,
    sha256_bytes,
    sha256_file,
)

_REPO = Path(__file__).resolve().parents[2]
_SA = _REPO / "fixtures/sa_r0"
_SCENARIOS = _REPO / "fixtures/scenarios"

_SCENARIO_SKIP = frozenset(
    {
        "index.json",
        "sweeps_index_v1.json",
        "compare_pairs_v1.json",
        "README.md",
    }
)

_SYNTHESIS_JSON = (
    "cross_sweep_synthesis_v1.json",
    "replay_linkage_index_v1.json",
)

_SYNTHESIS_MD = (
    "cognition_rollup_summary.md",
    "cross_sweep_summary.md",
    "linkage_summary.md",
)

_EXPORT_REPORTS = (
    "replay_compare_report_v1.json",
    "replay_presentation_report_v1.json",
    "replay_review_report_v1.json",
)


def _slug_to_hint(slug: str) -> str:
    return slug.replace("_", " ").strip()


def _apply_navigation_metadata(entry: dict[str, Any]) -> None:
    """Deterministic F1c navigation fields (additive, replay-local)."""
    kind = entry.get("entry_kind") or ""
    eid = entry.get("entry_id") or ""
    slug = eid.split("__", 1)[-1] if "__" in eid else eid
    replay = entry.get("replay_scope") or {}
    sweep = entry.get("sweep_scope") or {}
    pres = entry.get("presentation_scope") or {}

    category = "topology_lab"
    family = slug
    chronology = "topology_baseline"
    tags: list[str] = []

    if kind == "topology_experiment":
        category = "topology_lab"
        family = replay.get("pack_id") or slug
        chronology = "topology_baseline"
        tags = ["topology"]
    elif kind == "demo_bundle":
        category = "topology_lab"
        family = replay.get("pack_id") or slug
        chronology = "topology_demo"
        tags = ["demo_bundle", "topology"]
    elif kind == "sweep_family":
        category = "sweep_experiment"
        family = sweep.get("sweep_id") or slug
        chronology = "sweep_wave_d2"
        tags = ["mc_sweep", "sweep"]
    elif kind == "presentation_deck":
        category = "presentation"
        family = pres.get("storyboard_id") or slug
        chronology = "presentation_e1"
        tags = ["presentation", "storyboard"]
    elif kind == "synthesis_report":
        category = "synthesis"
        family = "cross_sweep" if "cross_sweep" in slug or "linkage" in slug else slug
        chronology = "synthesis_e2"
        tags = ["synthesis"]
    elif kind == "publication_packet":
        category = "export"
        family = (sweep.get("sweep_id") or slug).replace("_publication", "")
        chronology = "export_e1"
        tags = ["publication", "export"]
    elif kind == "replay_export":
        category = "export"
        sid = sweep.get("sweep_id") or ""
        family = sid or slug.split("_")[0]
        chronology = "export_e1"
        tags = ["export", "replay_export"]
    elif kind == "research_bundle":
        category = "corpus_ops"
        family = entry.get("corpus_id") or CORPUS_ID
        chronology = "corpus_release"
        tags = ["research_bundle", "corpus_ops"]
    elif kind == "corpus_release":
        category = "corpus_ops"
        family = slug
        chronology = "corpus_release"
        tags = ["corpus_release", "corpus_ops"]

    hint = _slug_to_hint(family if kind in ("sweep_family", "topology_experiment") else slug)
    if kind == "sweep_family":
        hint = f"{_slug_to_hint(family)} sweep family"
    elif kind == "synthesis_report":
        hint = f"Synthesis: {_slug_to_hint(slug)}"
    elif kind == "research_bundle":
        hint = f"Research bundle {family}"

    entry["reviewer_category"] = category
    entry["replay_family"] = family
    entry["chronology_group"] = chronology
    entry["navigation_tags"] = sorted(set(tags))
    entry["navigation_hint"] = hint


def _apply_evolution_metadata(entry: dict[str, Any], entries_by_id: dict[str, dict[str, Any]]) -> None:
    """Deterministic F1d evolution fields (additive, replay-local)."""
    chronology = entry.get("chronology_group") or "topology_baseline"
    kind = entry.get("entry_kind") or ""

    entry["release_generation_id"] = RELEASE_GENERATION_IDS.get(chronology, "gen_unknown")
    entry["replay_chronology_descriptor"] = CHRONOLOGY_DESCRIPTORS.get(
        chronology, chronology.replace("_", " ")
    )

    tags: list[str] = []
    if kind in ("topology_experiment", "demo_bundle"):
        tags.append("first_generation")
    elif kind == "replay_export":
        tags.append("regenerated_export")
    elif kind == "synthesis_report":
        tags.append("synthesis_rollup")
    elif kind == "publication_packet":
        tags.append("publication_derived")
    elif kind == "presentation_deck":
        tags.append("presentation_derived")
    elif kind == "research_bundle":
        tags.append("corpus_aggregate")
    entry["evolution_tags"] = sorted(set(tags))

    parent_revs: list[str] = []
    for pid in entry.get("lineage_parent_ids") or []:
        parent = entries_by_id.get(pid)
        if parent and parent.get("content_revision"):
            parent_revs.append(str(parent["content_revision"]))
    entry["publication_revision_lineage"] = sorted(set(parent_revs))


def _make_entry(
    *,
    entry_kind: str,
    slug: str,
    primary_path: Path,
    generation_tool: str,
    lineage_parent_ids: list[str] | None = None,
    derived_from: list[dict[str, Any]] | None = None,
    source_artifacts: list[str] | None = None,
    replay_scope: dict[str, Any] | None = None,
    sweep_scope: dict[str, Any] | None = None,
    presentation_scope: dict[str, Any] | None = None,
    export_scope: dict[str, Any] | None = None,
    sha256_manifest: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    rel = primary_path.relative_to(_REPO).as_posix()
    digest = sha256_file(primary_path)
    entry_id = normalize_entry_id(entry_kind, slug)
    entry: dict[str, Any] = {
        "entry_id": entry_id,
        "corpus_id": CORPUS_ID,
        "entry_kind": entry_kind,
        "lineage_parent_ids": normalize_lineage_parents(lineage_parent_ids),
        "derived_from": derived_from or [],
        "source_artifacts": source_artifacts or [rel],
        "generation_tool": generation_tool,
        "generation_revision": digest,
        "primary_artifact_path": rel,
        "sha256": digest,
        "content_revision": "",
    }
    if replay_scope:
        entry["replay_scope"] = replay_scope
    if sweep_scope:
        entry["sweep_scope"] = sweep_scope
    if presentation_scope:
        entry["presentation_scope"] = presentation_scope
    if export_scope:
        entry["export_scope"] = export_scope
    if sha256_manifest:
        entry["sha256_manifest"] = sha256_manifest
    entry["content_revision"] = entry_content_revision(entry)
    return entry


def _topology_entries() -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for pack_dir in sorted(_SCENARIOS.iterdir()):
        if not pack_dir.is_dir():
            continue
        meta = pack_dir / "metadata.json"
        if not meta.is_file():
            continue
        pack_id = pack_dir.name
        entries.append(
            _make_entry(
                entry_kind="topology_experiment",
                slug=pack_id,
                primary_path=meta,
                generation_tool="validate_scenario.py",
                derived_from=[
                    build_derivation_ref(
                        "bundle_packed_from_log",
                        pack_id,
                        str(pack_dir.relative_to(_REPO)),
                    )
                ],
                replay_scope={"pack_id": pack_id},
            )
        )
    return entries


def _demo_entries(topology_ids: dict[str, str]) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for demo_dir in sorted(_SA.glob("demo_*")):
        index_path = demo_dir / "index.json"
        if not index_path.is_file():
            continue
        slug = demo_dir.name.replace("demo_", "", 1)
        bundle = json.loads(index_path.read_text(encoding="utf-8"))
        pack_id = (bundle.get("scenario") or {}).get("catalog_pack_id") or slug
        parent = topology_ids.get(pack_id)
        parents = [parent] if parent else []
        derived = []
        if parent:
            derived.append(
                build_derivation_ref(
                    "regenerated_from",
                    parent,
                    f"fixtures/scenarios/{pack_id}/metadata.json",
                )
            )
        log_path = (bundle.get("lineage") or {}).get("log_path")
        if log_path:
            derived.append(
                build_derivation_ref("bundle_packed_from_log", slug, log_path)
            )
        entries.append(
            _make_entry(
                entry_kind="demo_bundle",
                slug=slug,
                primary_path=index_path,
                generation_tool="replay_sa_bundle.py",
                lineage_parent_ids=parents,
                derived_from=derived,
                source_artifacts=[index_path.relative_to(_REPO).as_posix()],
                replay_scope={
                    "pack_id": pack_id,
                    "seed": (bundle.get("lineage") or {}).get("seed"),
                },
            )
        )
    return entries


def _sweep_entries(demo_by_pack: dict[str, list[str]]) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for sweep_id in sorted(SWEEP_IDS):
        sweep_path = _SA / "sweeps" / sweep_id / "sweep.json"
        if not sweep_path.is_file():
            continue
        manifest = json.loads(sweep_path.read_text(encoding="utf-8"))
        baseline = manifest.get("baseline_topology_key") or ""
        pack_ids = sorted({m.get("pack_id") for m in manifest.get("members") or [] if m.get("pack_id")})
        parents: list[str] = []
        derived: list[dict[str, Any]] = []
        for pid in pack_ids:
            tid = normalize_entry_id("topology_experiment", pid)
            parents.append(tid)
            derived.append(
                build_derivation_ref(
                    "sweep_derived",
                    sweep_id,
                    f"fixtures/scenarios/{pid}/metadata.json",
                )
            )
        for did in demo_by_pack.get(baseline, []):
            if did not in parents:
                parents.append(did)
        parents = normalize_lineage_parents(parents)
        member_ids = [m.get("member_id") for m in manifest.get("members") or [] if m.get("member_id")]
        entries.append(
            _make_entry(
                entry_kind="sweep_family",
                slug=sweep_id,
                primary_path=sweep_path,
                generation_tool="gen_d2_sweep_fixtures.py",
                lineage_parent_ids=parents,
                derived_from=derived,
                sweep_scope={
                    "sweep_id": sweep_id,
                    "member_ids": member_ids,
                    "baseline_topology_key": baseline,
                },
            )
        )
    return entries


def _presentation_entries(sweep_ids: dict[str, str]) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    pres_dir = _SA / "presentations"
    for path in sorted(pres_dir.glob("*.json")):
        if path.name == "index.json":
            continue
        storyboard_id = path.stem
        parents = list(sweep_ids.values())
        entries.append(
            _make_entry(
                entry_kind="presentation_deck",
                slug=storyboard_id,
                primary_path=path,
                generation_tool="gen_e1_presentation_fixtures.py",
                lineage_parent_ids=parents[:4],
                derived_from=[
                    build_derivation_ref(
                        "presentation_derived",
                        storyboard_id,
                        path.relative_to(_REPO).as_posix(),
                    )
                ],
                presentation_scope={"storyboard_id": storyboard_id},
            )
        )
    return entries


def _synthesis_entries(sweep_ids: dict[str, str]) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    synth_dir = _SA / "synthesis"
    parents = list(sweep_ids.values())
    tools = {
        "cross_sweep_synthesis_v1.json": "build_cross_sweep_synthesis.py",
        "replay_linkage_index_v1.json": "build_replay_linkage.py",
    }
    for name in _SYNTHESIS_JSON:
        path = synth_dir / name
        if not path.is_file():
            continue
        slug = name.replace(".json", "")
        entries.append(
            _make_entry(
                entry_kind="synthesis_report",
                slug=slug,
                primary_path=path,
                generation_tool=tools.get(name, "build_replay_corpus_index.py"),
                lineage_parent_ids=parents,
                derived_from=[
                    build_derivation_ref("synthesis_derived", slug, path.relative_to(_REPO).as_posix())
                ],
                sweep_scope={"sweep_ids": list(SWEEP_IDS)},
            )
        )
    assets_index = synth_dir / "assets" / "index.json"
    if assets_index.is_file():
        entries.append(
            _make_entry(
                entry_kind="synthesis_report",
                slug="presentation_assets_index_v1",
                primary_path=assets_index,
                generation_tool="gen_presentation_assets.py",
                lineage_parent_ids=parents,
                derived_from=[
                    build_derivation_ref(
                        "presentation_derived",
                        "presentation_assets_index_v1",
                        assets_index.relative_to(_REPO).as_posix(),
                    )
                ],
                presentation_scope={"asset_kinds": ["thumbnail", "chapter", "pattern"]},
            )
        )
    for name in _SYNTHESIS_MD:
        path = synth_dir / name
        if path.is_file():
            slug = name.replace(".md", "")
            entries.append(
                _make_entry(
                    entry_kind="synthesis_report",
                    slug=slug,
                    primary_path=path,
                    generation_tool="build_cross_sweep_synthesis.py",
                    lineage_parent_ids=parents,
                    derived_from=[
                        build_derivation_ref("synthesis_derived", slug, path.relative_to(_REPO).as_posix())
                    ],
                )
            )
    return entries


def _publication_and_export_entries(sweep_ids: dict[str, str]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    pubs: list[dict[str, Any]] = []
    exports: list[dict[str, Any]] = []
    for sweep_id in sorted(SWEEP_IDS):
        reports = _SA / "sweeps" / sweep_id / "reports"
        if not reports.is_dir():
            continue
        parent = sweep_ids[sweep_id]
        pub_path = reports / "replay_publication_report_v1.json"
        if pub_path.is_file():
            pubs.append(
                _make_entry(
                    entry_kind="publication_packet",
                    slug=f"{sweep_id}_publication",
                    primary_path=pub_path,
                    generation_tool="export_presentation_pack.py",
                    lineage_parent_ids=[parent],
                    derived_from=[
                        build_derivation_ref(
                            "export_derived",
                            sweep_id,
                            pub_path.relative_to(_REPO).as_posix(),
                        )
                    ],
                    sweep_scope={"sweep_id": sweep_id},
                    export_scope={"export_kinds": ["publication"]},
                )
            )
        for report_name in _EXPORT_REPORTS:
            path = reports / report_name
            if not path.is_file():
                continue
            kind = report_name.replace("replay_", "").replace("_report_v1.json", "")
            exports.append(
                _make_entry(
                    entry_kind="replay_export",
                    slug=f"{sweep_id}_{kind}",
                    primary_path=path,
                    generation_tool="export_replay_analytics_report.py",
                    lineage_parent_ids=[parent],
                    derived_from=[
                        build_derivation_ref(
                            "export_derived",
                            f"{sweep_id}_{kind}",
                            path.relative_to(_REPO).as_posix(),
                        )
                    ],
                    sweep_scope={"sweep_id": sweep_id},
                    export_scope={"export_kinds": [kind]},
                )
            )
    return pubs, exports


def _research_bundle_entry(synthesis_ids: list[str], sweep_ids: dict[str, str]) -> dict[str, Any]:
    manifest_path = _SA / "research_bundles" / CORPUS_ID / "manifest.json"
    parents = synthesis_ids + list(sweep_ids.values())
    parents.append(normalize_entry_id("publication_packet", f"{SWEEP_IDS[0]}_publication"))
    return _make_entry(
        entry_kind="research_bundle",
        slug=CORPUS_ID,
        primary_path=manifest_path,
        generation_tool="export_research_bundle.py",
        lineage_parent_ids=normalize_lineage_parents(parents)[:8],
        derived_from=[
            build_derivation_ref(
                "research_bundle_aggregated",
                CORPUS_ID,
                manifest_path.relative_to(_REPO).as_posix(),
            )
        ],
        source_artifacts=[manifest_path.relative_to(_REPO).as_posix()],
    )


def _build_lineage_edges(entries: list[dict[str, Any]]) -> list[dict[str, Any]]:
    edges: list[dict[str, Any]] = []
    for entry in entries:
        child = entry["entry_id"]
        for parent in entry.get("lineage_parent_ids") or []:
            ref_kind = "sweep_derived"
            if entry["entry_kind"] == "synthesis_report":
                ref_kind = "synthesis_derived"
            elif entry["entry_kind"] in ("replay_export", "publication_packet"):
                ref_kind = "export_derived"
            elif entry["entry_kind"] == "research_bundle":
                ref_kind = "research_bundle_aggregated"
            elif entry["entry_kind"] == "demo_bundle":
                ref_kind = "regenerated_from"
            edges.append(
                build_lineage_edge(
                    parent,
                    child,
                    ref_kind,
                    entry.get("source_artifacts") or [],
                )
            )
    seen: set[str] = set()
    unique: list[dict[str, Any]] = []
    for e in edges:
        eid = e["edge_id"]
        if eid not in seen:
            seen.add(eid)
            unique.append(e)
    return sorted(unique, key=lambda x: x["edge_id"])


def build_corpus_index() -> dict[str, Any]:
    topology = _topology_entries()
    topology_ids = {e["replay_scope"]["pack_id"]: e["entry_id"] for e in topology if e.get("replay_scope")}

    entries: list[dict[str, Any]] = []
    entries.extend(topology)
    entries.extend(_demo_entries(topology_ids))

    demo_by_pack: dict[str, list[str]] = {}
    for e in entries:
        if e["entry_kind"] != "demo_bundle":
            continue
        pid = (e.get("replay_scope") or {}).get("pack_id")
        if pid:
            demo_by_pack.setdefault(pid, []).append(e["entry_id"])

    sweeps = _sweep_entries(demo_by_pack)
    entries.extend(sweeps)
    sweep_ids = {e["sweep_scope"]["sweep_id"]: e["entry_id"] for e in sweeps if e.get("sweep_scope")}

    entries.extend(_presentation_entries(sweep_ids))
    synthesis = _synthesis_entries(sweep_ids)
    entries.extend(synthesis)
    synthesis_ids = [e["entry_id"] for e in synthesis if "cross_sweep" in e["entry_id"] or "linkage" in e["entry_id"]]

    pubs, exports = _publication_and_export_entries(sweep_ids)
    entries.extend(pubs)
    entries.extend(exports)

    if (_SA / "research_bundles" / CORPUS_ID / "manifest.json").is_file():
        entries.append(_research_bundle_entry(synthesis_ids, sweep_ids))

    for entry in entries:
        _apply_navigation_metadata(entry)

    entries = sorted(entries, key=lambda x: x["entry_id"])
    entries_by_id = {e["entry_id"]: e for e in entries}
    for entry in entries:
        _apply_evolution_metadata(entry, entries_by_id)
    edges = _build_lineage_edges(entries)

    rev_payload = json.dumps([e["entry_id"] for e in entries], sort_keys=True)
    index_revision = sha256_bytes(rev_payload.encode("utf-8"))

    return {
        "artifact_type": "replay_corpus_index_v1",
        "schema_version": "replay_corpus_index_v1",
        "corpus_id": CORPUS_ID,
        "generation_revision": INDEX_GENERATION_REVISION,
        "index_revision": index_revision,
        "governance": GOVERNANCE,
        "entries": entries,
        "lineage_edges": edges,
    }


def write_corpus_index(index: dict[str, Any]) -> None:
    fixture = _REPO / "fixtures/sa_r0/synthesis"
    pub = _REPO / "platform/sa-r0-viewer/public/demo/synthesis"
    fixture.mkdir(parents=True, exist_ok=True)
    pub.mkdir(parents=True, exist_ok=True)
    text = json.dumps(index, indent=2, sort_keys=True) + "\n"
    for d in (fixture, pub):
        (d / "replay_corpus_index_v1.json").write_text(text, encoding="utf-8")


def check_corpus_index() -> None:
    expected = build_corpus_index()
    path = _REPO / "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json"
    if not path.is_file():
        raise SystemExit(f"missing corpus index fixture: {path}")
    actual = json.loads(path.read_text(encoding="utf-8"))
    if json.dumps(actual, sort_keys=True) != json.dumps(expected, sort_keys=True):
        raise SystemExit(
            "replay_corpus_index_v1.json is stale — run build_replay_corpus_index.py"
        )


def corpus_ref_for_entry(entry: dict[str, Any], index_revision: str) -> dict[str, Any]:
    from replay_corpus_lineage import build_corpus_ref  # noqa: E402

    return build_corpus_ref(
        entry["entry_id"],
        entry.get("lineage_parent_ids"),
        index_revision,
    )


def main() -> None:
    ap = argparse.ArgumentParser(description="Build replay corpus index")
    ap.add_argument("--check", action="store_true", help="verify committed fixtures match")
    args = ap.parse_args()

    if args.check:
        check_corpus_index()
        print("replay corpus index check OK")
        return

    index = build_corpus_index()
    write_corpus_index(index)
    print(f"replay corpus index OK ({len(index['entries'])} entries)")


if __name__ == "__main__":
    main()
