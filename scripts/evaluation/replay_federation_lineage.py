"""Deterministic multi-corpus federation lineage and indexing (PLAT-SA-F2A)."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from replay_corpus_lineage import (
    CORPUS_ID,
    sha256_bytes,
    sha256_file,
    write_viewer_mirror,
)

FEDERATION_ID = "sa_replay_federation_r0_v1"
FEDERATION_GENERATION_REVISION = "f2a_v1"
RELEASE_ID = "sa_r0_corpus_r1_r1"

CANONICAL_INDEX_PATH = "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json"
RELEASE_INDEX_PATH = f"fixtures/sa_r0/corpus_releases/{RELEASE_ID}/replay_corpus_index_v1.json"

FEDERATION_REF_KINDS = frozenset(
    {
        "federation_release_derived",
        "federation_publication_chain",
        "federation_recovery_continuity",
        "federation_study_lineage",
    }
)

FEDERATION_GOVERNANCE = {
    "notice": (
        "Replay federation index for offline multi-corpus research operations only — "
        "deterministic registry and cross-study lineage, not cloud sync or operational authority."
    ),
    "anti_claims": [
        "Federation groups partition offline fixture indexes; they do not live-sync corpora.",
        "Cross-corpus lineage documents structural partitioning, not causal inference.",
        "Federation integrity does not certify replay effectiveness or readiness.",
        "Recovery continuity summaries are explanatory; CLI remains authoritative for promote.",
    ],
}

INTEGRITY_GOVERNANCE = {
    "notice": (
        "Federation integrity report for maintainer review only — "
        "cross-corpus drift inventory, not operational failure."
    ),
    "anti_claims": [
        "Orphan group detection indicates registry drift, not runtime faults.",
        "Cross-group duplication may be expected for release snapshots.",
    ],
}

RECOVERY_CONTINUITY_GOVERNANCE = {
    "notice": (
        "Federation recovery continuity rollup for reviewer cognition only — "
        "not live recovery authority."
    ),
    "anti_claims": [
        "Quarantine and supersession invariants from I3 remain unchanged.",
        "Browser does not clear holds or reconcile history.",
    ],
}


def _read_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"expected JSON object: {path}")
    return data


def federation_revision_fingerprint(payload: dict[str, Any]) -> str:
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return sha256_bytes(text.encode("utf-8"))


def default_corpus_groups(repo_root: Path) -> list[dict[str, Any]]:
    """Two-group MVP: canonical index + frozen release snapshot."""
    return [
        {
            "corpus_group_id": "canonical_r1",
            "corpus_id": CORPUS_ID,
            "index_artifact_path": CANONICAL_INDEX_PATH,
            "study_label": "Canonical replay corpus index",
        },
        {
            "corpus_group_id": "release_r1_snapshot",
            "corpus_id": CORPUS_ID,
            "index_artifact_path": RELEASE_INDEX_PATH,
            "release_id": RELEASE_ID,
            "study_label": f"Frozen release snapshot ({RELEASE_ID})",
        },
    ]


def load_corpus_index(repo_root: Path, index_path: str) -> dict[str, Any]:
    path = repo_root / index_path
    if not path.is_file():
        raise FileNotFoundError(f"missing corpus index: {path}")
    return _read_json(path)


def build_federation_manifest(
    repo_root: Path,
    *,
    federation_id: str = FEDERATION_ID,
    parent_federation_ref: str | None = None,
) -> dict[str, Any]:
    groups = default_corpus_groups(repo_root)
    lineage_graph = build_federation_lineage_graph(groups)
    edge_ids = [e["edge_id"] for e in lineage_graph.get("edges") or []]

    pub_collection_path = "fixtures/sa_r0/federation/replay_federation_publication_collection_v1.json"
    manifest: dict[str, Any] = {
        "artifact_type": "replay_federation_manifest_v1",
        "schema_version": "replay_federation_manifest_v1",
        "federation_id": federation_id,
        "generation_revision": FEDERATION_GENERATION_REVISION,
        "governance": FEDERATION_GOVERNANCE,
        "corpus_groups": groups,
        "publication_collection_refs": [pub_collection_path],
        "federation_lineage_refs": edge_ids,
    }
    if parent_federation_ref:
        manifest["parent_federation_ref"] = parent_federation_ref
    return manifest


def build_federation_lineage_graph(
    corpus_groups: list[dict[str, Any]],
) -> dict[str, Any]:
    edges: list[dict[str, Any]] = []
    by_group = {g["corpus_group_id"]: g for g in corpus_groups}

    if "canonical_r1" in by_group and "release_r1_snapshot" in by_group:
        edges.append(
            {
                "edge_id": "canonical_r1__release_r1_snapshot__federation_release_derived",
                "from_corpus_group_id": "canonical_r1",
                "to_corpus_group_id": "release_r1_snapshot",
                "ref_kind": "federation_release_derived",
                "evidence": {
                    "source_paths": sorted(
                        [
                            CANONICAL_INDEX_PATH,
                            RELEASE_INDEX_PATH,
                        ]
                    ),
                    "note": "Release snapshot frozen from canonical corpus index at sa_r0_corpus_r1_r1",
                },
            }
        )

    edges.append(
        {
            "edge_id": "canonical_r1__orchestration_recovery__federation_recovery_continuity",
            "from_corpus_group_id": "canonical_r1",
            "to_corpus_group_id": "canonical_r1",
            "ref_kind": "federation_recovery_continuity",
            "evidence": {
                "source_paths": [
                    "fixtures/orchestration/reconciliation/reconciliation_lineage_index_v1.json",
                    "fixtures/orchestration/synthesis/async_batch_audit_v1.json",
                ],
                "note": "I3 async recovery plane scoped to canonical orchestration fixtures",
            },
        }
    )

    return {
        "artifact_type": "replay_federation_lineage_graph_v1",
        "schema_version": "replay_federation_lineage_graph_v1",
        "federation_id": FEDERATION_ID,
        "generation_revision": FEDERATION_GENERATION_REVISION,
        "governance": FEDERATION_GOVERNANCE,
        "edges": edges,
    }


def validate_federation_lineage_dag(
    corpus_groups: list[dict[str, Any]],
    edges: list[dict[str, Any]],
) -> list[str]:
    errors: list[str] = []
    group_ids = {g.get("corpus_group_id") for g in corpus_groups if g.get("corpus_group_id")}

    for edge in edges or []:
        fg = edge.get("from_corpus_group_id")
        tg = edge.get("to_corpus_group_id")
        if fg not in group_ids:
            errors.append(f"lineage edge missing from group: {fg}")
        if tg not in group_ids:
            errors.append(f"lineage edge missing to group: {tg}")
        rk = edge.get("ref_kind")
        if rk not in FEDERATION_REF_KINDS:
            errors.append(f"unknown federation ref_kind: {rk}")

    # Cycle detection on group ids (ignore self-loops for recovery continuity)
    adj: dict[str, list[str]] = {g: [] for g in group_ids}
    for edge in edges or []:
        fg = edge.get("from_corpus_group_id")
        tg = edge.get("to_corpus_group_id")
        if fg and tg and fg != tg:
            adj.setdefault(fg, []).append(tg)

    visiting: set[str] = set()
    visited: set[str] = set()

    def dfs(node: str) -> bool:
        if node in visiting:
            return True
        if node in visited:
            return False
        visiting.add(node)
        for nxt in adj.get(node, []):
            if dfs(nxt):
                return True
        visiting.remove(node)
        visited.add(node)
        return False

    for g in group_ids:
        if g and dfs(g):
            errors.append(f"federation lineage cycle detected at {g}")
            break

    return errors


def _entry_kind_counts(entries: list[dict[str, Any]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for e in entries:
        kind = e.get("entry_kind") or "unknown"
        counts[kind] = counts.get(kind, 0) + 1
    return dict(sorted(counts.items()))


def build_corpus_group_summary(
    repo_root: Path,
    group: dict[str, Any],
) -> dict[str, Any]:
    index = load_corpus_index(repo_root, group["index_artifact_path"])
    entries = index.get("entries") or []
    return {
        "corpus_group_id": group["corpus_group_id"],
        "corpus_id": group.get("corpus_id"),
        "release_id": group.get("release_id"),
        "study_label": group.get("study_label"),
        "index_artifact_path": group["index_artifact_path"],
        "index_revision": index.get("index_revision"),
        "entry_count": len(entries),
        "entry_kind_counts": _entry_kind_counts(entries),
    }


def build_federation_continuity_index(
    repo_root: Path,
    corpus_groups: list[dict[str, Any]],
) -> dict[str, Any]:
    canonical = next((g for g in corpus_groups if g["corpus_group_id"] == "canonical_r1"), None)
    release = next((g for g in corpus_groups if g["corpus_group_id"] == "release_r1_snapshot"), None)
    pairs: list[dict[str, Any]] = []

    if canonical and release:
        c_index = load_corpus_index(repo_root, canonical["index_artifact_path"])
        r_index = load_corpus_index(repo_root, release["index_artifact_path"])
        c_by_id = {e["entry_id"]: e for e in c_index.get("entries") or [] if e.get("entry_id")}
        r_by_id = {e["entry_id"]: e for e in r_index.get("entries") or [] if e.get("entry_id")}
        shared = sorted(set(c_by_id) & set(r_by_id))
        for eid in shared:
            ce = c_by_id[eid]
            re = r_by_id[eid]
            pairs.append(
                {
                    "entry_id": eid,
                    "canonical_sha256": ce.get("sha256"),
                    "release_sha256": re.get("sha256"),
                    "bytes_match": ce.get("sha256") == re.get("sha256"),
                }
            )

    pub_path = repo_root / "fixtures/sa_r0/synthesis/replay_corpus_publication_packet_v1.json"
    pub_ref = None
    if pub_path.is_file():
        pub = _read_json(pub_path)
        pub_ref = {
            "artifact_path": pub_path.relative_to(repo_root).as_posix(),
            "sha256": sha256_file(pub_path),
            "release_id": pub.get("release_id"),
        }

    return {
        "artifact_type": "replay_federation_continuity_index_v1",
        "schema_version": "replay_federation_continuity_index_v1",
        "federation_id": FEDERATION_ID,
        "generation_revision": FEDERATION_GENERATION_REVISION,
        "governance": FEDERATION_GOVERNANCE,
        "canonical_release_pairs": pairs,
        "shared_entry_count": len(pairs),
        "publication_chain_head": pub_ref,
    }


def build_publication_collection(
    repo_root: Path,
    corpus_groups: list[dict[str, Any]],
) -> dict[str, Any]:
    members: list[dict[str, Any]] = []
    for group in corpus_groups:
        gid = group["corpus_group_id"]
        if gid == "canonical_r1":
            for rel, kind in (
                ("fixtures/sa_r0/synthesis/replay_corpus_publication_packet_v1.json", "publication_packet"),
                ("fixtures/sa_r0/research_bundles/sa_r0_corpus_r1/manifest.json", "research_bundle"),
            ):
                path = repo_root / rel
                if path.is_file():
                    members.append(
                        {
                            "corpus_group_id": gid,
                            "artifact_path": rel,
                            "artifact_kind": kind,
                            "sha256": sha256_file(path),
                        }
                    )
        if group.get("release_id"):
            rel = f"fixtures/sa_r0/corpus_releases/{group['release_id']}/replay_corpus_release_manifest_v1.json"
            path = repo_root / rel
            if path.is_file():
                members.append(
                    {
                        "corpus_group_id": gid,
                        "artifact_path": rel,
                        "artifact_kind": "corpus_release",
                        "sha256": sha256_file(path),
                    }
                )

    return {
        "artifact_type": "replay_federation_publication_collection_v1",
        "schema_version": "replay_federation_publication_collection_v1",
        "federation_id": FEDERATION_ID,
        "collection_id": "sa_r0_primary_publications",
        "generation_revision": FEDERATION_GENERATION_REVISION,
        "governance": FEDERATION_GOVERNANCE,
        "members": members,
    }


def build_federation_replay_summary(
    repo_root: Path,
    corpus_groups: list[dict[str, Any]],
) -> dict[str, Any]:
    summaries = [build_corpus_group_summary(repo_root, g) for g in corpus_groups]
    total_entries = sum(s.get("entry_count", 0) for s in summaries)
    return {
        "artifact_type": "replay_federation_replay_summary_v1",
        "schema_version": "replay_federation_replay_summary_v1",
        "federation_id": FEDERATION_ID,
        "generation_revision": FEDERATION_GENERATION_REVISION,
        "governance": FEDERATION_GOVERNANCE,
        "corpus_group_summaries": summaries,
        "total_entry_count": total_entries,
    }


def build_federation_index(repo_root: Path) -> dict[str, Any]:
    manifest = build_federation_manifest(repo_root)
    groups = manifest["corpus_groups"]
    summaries = [build_corpus_group_summary(repo_root, g) for g in groups]
    rev_payload = {
        "manifest": manifest,
        "summaries": summaries,
    }
    return {
        "artifact_type": "replay_federation_index_v1",
        "schema_version": "replay_federation_index_v1",
        "federation_id": FEDERATION_ID,
        "generation_revision": FEDERATION_GENERATION_REVISION,
        "governance": FEDERATION_GOVERNANCE,
        "manifest_ref": "fixtures/sa_r0/federation/replay_federation_manifest_v1.json",
        "corpus_group_summaries": summaries,
        "federation_revision": federation_revision_fingerprint(rev_payload),
        "continuity_index_ref": "fixtures/sa_r0/federation/replay_federation_continuity_index_v1.json",
        "lineage_graph_ref": "fixtures/sa_r0/federation/replay_federation_lineage_graph_v1.json",
    }


def federation_fixture_paths(repo_root: Path) -> dict[str, Path]:
    base = repo_root / "fixtures/sa_r0/federation"
    return {
        "manifest": base / "replay_federation_manifest_v1.json",
        "index": base / "replay_federation_index_v1.json",
        "lineage_graph": base / "replay_federation_lineage_graph_v1.json",
        "continuity_index": base / "replay_federation_continuity_index_v1.json",
        "publication_collection": base / "replay_federation_publication_collection_v1.json",
        "replay_summary": base / "replay_federation_replay_summary_v1.json",
        "integrity_report": base / "audits/replay_federation_integrity_report_v1.json",
        "reproducibility": base / "replay_federation_reproducibility_v1.json",
        "snapshot": base / "replay_federation_snapshot_v1.json",
        "recovery_continuity": base / "audits/orchestration_federation_recovery_continuity_v1.json",
    }


def write_federation_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def sync_federation_viewer_mirrors(repo_root: Path) -> None:
    paths = federation_fixture_paths(repo_root)
    demo = repo_root / "platform/sa-r0-viewer/public/demo/federation"
    for key in (
        "manifest",
        "index",
        "lineage_graph",
        "continuity_index",
        "publication_collection",
        "replay_summary",
        "reproducibility",
        "snapshot",
    ):
        canonical = paths[key]
        if canonical.is_file():
            mirror = demo / canonical.name
            write_viewer_mirror(canonical, mirror)
    audits_demo = demo / "audits"
    for key in ("integrity_report", "recovery_continuity"):
        canonical = paths[key]
        if canonical.is_file():
            mirror = audits_demo / canonical.name
            write_viewer_mirror(canonical, mirror)


def build_federation_reproducibility(repo_root: Path, index: dict[str, Any]) -> dict[str, Any]:
    manifest = build_federation_manifest(repo_root)
    pub = build_publication_collection(repo_root, manifest["corpus_groups"])
    pub_rev = federation_revision_fingerprint({"members": pub.get("members") or []})
    group_revs = [
        {
            "corpus_group_id": s["corpus_group_id"],
            "index_revision": s.get("index_revision"),
        }
        for s in index.get("corpus_group_summaries") or []
    ]
    chain_payload = {
        "federation_revision": index.get("federation_revision"),
        "group_revisions": group_revs,
        "publication_revision": pub_rev,
    }
    return {
        "artifact_type": "replay_federation_reproducibility_v1",
        "schema_version": "replay_federation_reproducibility_v1",
        "federation_id": FEDERATION_ID,
        "generation_revision": FEDERATION_GENERATION_REVISION,
        "governance": FEDERATION_GOVERNANCE,
        "federation_revision": index.get("federation_revision"),
        "corpus_group_index_revisions": group_revs,
        "publication_revision": pub_rev,
        "reproducibility_fingerprint": federation_revision_fingerprint(chain_payload),
        "parent_federation_ref": manifest.get("parent_federation_ref"),
    }


def build_federation_snapshot(
    repo_root: Path,
    index: dict[str, Any],
    *,
    parent_snapshot_ref: str | None = None,
) -> dict[str, Any]:
    paths = federation_fixture_paths(repo_root)
    files: list[dict[str, Any]] = []
    for key in (
        "manifest",
        "index",
        "lineage_graph",
        "continuity_index",
        "publication_collection",
        "replay_summary",
        "reproducibility",
    ):
        p = paths[key]
        if p.is_file():
            files.append(
                {
                    "path": p.relative_to(repo_root).as_posix(),
                    "sha256": sha256_file(p),
                    "size_bytes": p.stat().st_size,
                }
            )
    snap = {
        "artifact_type": "replay_federation_snapshot_v1",
        "schema_version": "replay_federation_snapshot_v1",
        "federation_id": FEDERATION_ID,
        "snapshot_id": f"{FEDERATION_ID}_s1",
        "generation_revision": FEDERATION_GENERATION_REVISION,
        "governance": FEDERATION_GOVERNANCE,
        "federation_revision": index.get("federation_revision"),
        "files": files,
    }
    if parent_snapshot_ref:
        snap["parent_snapshot_ref"] = parent_snapshot_ref
    return snap


def write_all_federation_artifacts(repo_root: Path) -> dict[str, Path]:
    paths = federation_fixture_paths(repo_root)
    manifest = build_federation_manifest(repo_root)
    groups = manifest["corpus_groups"]
    lineage = build_federation_lineage_graph(groups)
    continuity = build_federation_continuity_index(repo_root, groups)
    publication = build_publication_collection(repo_root, groups)
    summary = build_federation_replay_summary(repo_root, groups)
    index = build_federation_index(repo_root)
    reproducibility = build_federation_reproducibility(repo_root, index)
    snapshot = build_federation_snapshot(repo_root, index)

    write_federation_json(paths["manifest"], manifest)
    write_federation_json(paths["lineage_graph"], lineage)
    write_federation_json(paths["continuity_index"], continuity)
    write_federation_json(paths["publication_collection"], publication)
    write_federation_json(paths["replay_summary"], summary)
    write_federation_json(paths["index"], index)
    write_federation_json(paths["reproducibility"], reproducibility)
    write_federation_json(paths["snapshot"], snapshot)
    sync_federation_viewer_mirrors(repo_root)
    return paths
