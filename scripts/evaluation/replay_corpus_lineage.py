"""Deterministic corpus lineage normalization and DAG validation (PLAT-SA-F1a)."""

from __future__ import annotations

import hashlib
import json
import re
import shutil
from pathlib import Path
from typing import Any

CORPUS_ID = "sa_r0_corpus_r1"
INDEX_GENERATION_REVISION = "f1a_v1"
DRIFT_GENERATION_REVISION = "f1b_v1"
EVOLUTION_GENERATION_REVISION = "f1d_v1"
PUBLICATION_GENERATION_REVISION = "f1d_v1"

CHRONOLOGY_ORDER = (
    "topology_baseline",
    "topology_demo",
    "sweep_wave_d2",
    "presentation_e1",
    "export_e1",
    "synthesis_e2",
    "corpus_release",
)

CHRONOLOGY_DESCRIPTORS = {
    "topology_baseline": "Topology experiment baseline",
    "topology_demo": "Demo bundle replay wave",
    "sweep_wave_d2": "Monte Carlo sweep generation (D2)",
    "presentation_e1": "Presentation and storyboard wave (E1)",
    "export_e1": "Static export and publication packets (E1)",
    "synthesis_e2": "Cross-sweep synthesis wave (E2)",
    "corpus_release": "Corpus release and research bundle",
}

RELEASE_GENERATION_IDS = {
    "topology_baseline": "gen_topology",
    "topology_demo": "gen_topology_demo",
    "sweep_wave_d2": "gen_sweep_d2",
    "presentation_e1": "gen_presentation",
    "export_e1": "gen_export",
    "synthesis_e2": "gen_synthesis",
    "corpus_release": "gen_corpus_ops",
}

RELEASE_GENERATION_ID_VALUES = frozenset(RELEASE_GENERATION_IDS.values())

EVOLUTION_TAG_VALUES = frozenset(
    {
        "first_generation",
        "regenerated_export",
        "synthesis_rollup",
        "publication_derived",
        "corpus_aggregate",
        "presentation_derived",
    }
)

EVOLUTION_GOVERNANCE = {
    "notice": (
        "Replay corpus evolution manifest for long-horizon research review only — "
        "descriptive chronology and release comparison, not operational lifecycle management."
    ),
    "anti_claims": [
        "Evolution rollups document artifact derivation waves, not causal tactical doctrine.",
        "Chronology tiers do not certify comparability across experiments.",
        "Cross-release diffs are structural integrity comparisons only.",
    ],
}

PUBLICATION_GOVERNANCE = {
    "notice": (
        "Replay corpus publication packet for offline research archive only — "
        "not deployment packaging or operational release certification."
    ),
    "anti_claims": [
        "Publication inventory lists replay-local artifacts only.",
        "Archive export does not imply readiness or authority.",
    ],
}

ROOT_ENTRY_KINDS = frozenset({"topology_experiment", "presentation_deck"})

DRIFT_GOVERNANCE = {
    "notice": (
        "Corpus drift report for maintainer integrity review only — "
        "explanatory fixture drift inventory, not operational failure or readiness."
    ),
    "anti_claims": [
        "Drift findings indicate bytes or references out of sync with the corpus index.",
        "Regeneration restores deterministic artifacts; drift does not imply invalid doctrine.",
    ],
}

DIFF_GOVERNANCE = {
    "notice": (
        "Corpus release diff for offline reproducibility review — "
        "structural index comparison only, not deployment certification."
    ),
    "anti_claims": [
        "Non-empty diff between canonical index and frozen release is expected until release rebuild.",
    ],
}

_UNINDEXED_ALLOW_SUFFIXES = frozenset(
    {".png", ".log", ".meta.json", ".zip", ".html", ".md", ".sha256"}
)
_UNINDEXED_ALLOW_NAMES = frozenset(
    {
        "README.md",
        "MANIFEST.sha256",
        "generator_versions.json",
        "publication_packet.html",
        "storyline_linkage_overlay_v1.json",
    }
)

REVIEWER_CATEGORIES = frozenset(
    {
        "topology_lab",
        "sweep_experiment",
        "presentation",
        "synthesis",
        "export",
        "corpus_ops",
    }
)

ENTRY_KINDS = frozenset(
    {
        "demo_bundle",
        "sweep_family",
        "topology_experiment",
        "presentation_deck",
        "publication_packet",
        "synthesis_report",
        "replay_export",
        "research_bundle",
        "corpus_release",
    }
)

REF_KINDS = frozenset(
    {
        "bundle_packed_from_log",
        "sweep_derived",
        "synthesis_derived",
        "presentation_derived",
        "export_derived",
        "regenerated_from",
        "research_bundle_aggregated",
    }
)

GOVERNANCE = {
    "notice": (
        "Replay corpus index for offline inventory and structural lineage only — "
        "explanatory artifact graph, not operational deployment or validated doctrine."
    ),
    "anti_claims": [
        "Corpus lineage documents derivation paths, not causal inference.",
        "Index entries are not parser authority or tactical state.",
        "SHA256 integrity does not certify replay effectiveness or readiness.",
    ],
}

_SLUG_RE = re.compile(r"[^a-z0-9_]+")


def normalize_slug(raw: str) -> str:
    s = raw.strip().lower().replace("-", "_")
    s = _SLUG_RE.sub("_", s)
    s = re.sub(r"_+", "_", s).strip("_")
    return s or "unknown"


def normalize_entry_id(entry_kind: str, slug: str) -> str:
    return f"{entry_kind}__{normalize_slug(slug)}"


def normalize_lineage_parents(parents: list[str] | None) -> list[str]:
    if not parents:
        return []
    out = sorted({p for p in parents if p})
    return out


def build_derivation_ref(
    ref_kind: str,
    ref_id: str,
    artifact_path: str | None = None,
) -> dict[str, Any]:
    if ref_kind not in REF_KINDS:
        raise ValueError(f"unknown ref_kind: {ref_kind}")
    ref: dict[str, Any] = {"ref_kind": ref_kind, "ref_id": ref_id}
    if artifact_path:
        ref["artifact_path"] = artifact_path
    return ref


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Any) -> str:
    return sha256_bytes(path.read_bytes())


def entry_content_revision(entry: dict[str, Any]) -> str:
    payload = {k: v for k, v in entry.items() if k not in ("content_revision",)}
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return sha256_bytes(text.encode("utf-8"))


def build_lineage_edge(
    parent_entry_id: str,
    child_entry_id: str,
    ref_kind: str,
    source_paths: list[str],
    note: str | None = None,
) -> dict[str, Any]:
    edge_id = f"{parent_entry_id}__{child_entry_id}__{ref_kind}"
    evidence: dict[str, Any] = {"source_paths": sorted(source_paths)}
    if note:
        evidence["note"] = note
    return {
        "edge_id": edge_id,
        "parent_entry_id": parent_entry_id,
        "child_entry_id": child_entry_id,
        "ref_kind": ref_kind,
        "evidence": evidence,
    }


def validate_lineage_dag(
    entries: list[dict[str, Any]],
    edges: list[dict[str, Any]] | None = None,
) -> list[str]:
    errors: list[str] = []
    by_id: dict[str, dict[str, Any]] = {}
    for e in entries:
        eid = e.get("entry_id")
        if not eid:
            errors.append("entry missing entry_id")
            continue
        if eid in by_id:
            errors.append(f"duplicate entry_id: {eid}")
        by_id[eid] = e

    for eid, e in by_id.items():
        parents = e.get("lineage_parent_ids") or []
        if eid in parents:
            errors.append(f"self-reference in lineage_parent_ids: {eid}")
        for p in parents:
            if p not in by_id:
                errors.append(f"missing lineage parent {p} for entry {eid}")

    # Cycle detection via DFS
    visiting: set[str] = set()
    visited: set[str] = set()

    def dfs(node: str) -> bool:
        if node in visiting:
            return True
        if node in visited:
            return False
        visiting.add(node)
        entry = by_id.get(node)
        if entry:
            for p in entry.get("lineage_parent_ids") or []:
                if dfs(p):
                    return True
        visiting.remove(node)
        visited.add(node)
        return False

    for eid in by_id:
        if dfs(eid):
            errors.append(f"lineage cycle detected involving {eid}")

    if edges:
        for edge in edges:
            pe = edge.get("parent_entry_id")
            ce = edge.get("child_entry_id")
            if pe and pe not in by_id:
                errors.append(f"lineage edge missing parent entry: {pe}")
            if ce and ce not in by_id:
                errors.append(f"lineage edge missing child entry: {ce}")

    return errors


def build_corpus_ref(
    entry_id: str,
    lineage_parent_ids: list[str] | None,
    index_revision: str,
    corpus_id: str = CORPUS_ID,
) -> dict[str, Any]:
    return {
        "corpus_id": corpus_id,
        "entry_id": entry_id,
        "lineage_parent_ids": normalize_lineage_parents(lineage_parent_ids),
        "index_revision": index_revision,
    }


def load_corpus_index(repo_root: Any | None = None) -> dict[str, Any] | None:
    from pathlib import Path

    root = Path(repo_root) if repo_root else Path(__file__).resolve().parents[2]
    path = root / "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json"
    if not path.is_file():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def lookup_corpus_ref(
    index: dict[str, Any],
    entry_kind: str,
    slug: str,
) -> dict[str, Any] | None:
    eid = normalize_entry_id(entry_kind, slug)
    for entry in index.get("entries") or []:
        if entry.get("entry_id") == eid:
            return build_corpus_ref(
                eid,
                entry.get("lineage_parent_ids"),
                index.get("index_revision") or "",
            )
    return None


def attach_corpus_ref(
    artifact: dict[str, Any],
    entry_kind: str,
    slug: str,
    repo_root: Any | None = None,
) -> dict[str, Any]:
    index = load_corpus_index(repo_root)
    if not index:
        return artifact
    ref = lookup_corpus_ref(index, entry_kind, slug)
    if ref:
        artifact = dict(artifact)
        artifact["corpus_ref"] = ref
    return artifact


def _finding(
    *,
    kind: str,
    severity: str,
    message: str,
    seq: int,
    entry_id: str | None = None,
    path: str | None = None,
    evidence: dict[str, Any] | None = None,
) -> dict[str, Any]:
    fid = f"{kind}__{seq:04d}"
    out: dict[str, Any] = {
        "finding_id": fid,
        "kind": kind,
        "severity": severity,
        "message": message,
    }
    if entry_id:
        out["entry_id"] = entry_id
    if path:
        out["path"] = path
    if evidence:
        out["evidence"] = evidence
    return out


def _indexed_paths(index: dict[str, Any]) -> set[str]:
    paths: set[str] = set()
    for entry in index.get("entries") or []:
        p = entry.get("primary_artifact_path")
        if p:
            paths.add(str(p))
        for sp in entry.get("source_artifacts") or []:
            paths.add(str(sp))
        for item in entry.get("sha256_manifest") or []:
            if item.get("path"):
                paths.add(str(item["path"]))
    return paths


def _discover_candidate_files(sa_root: Any) -> list[str]:
    from pathlib import Path

    sa = Path(sa_root)
    repo = sa.parent.parent
    candidates: list[str] = []
    patterns = [
        "demo_*/index.json",
        "sweeps/*/sweep.json",
        "sweeps/*/reports/replay_*_report_v1.json",
        "synthesis/*.json",
        "synthesis/assets/index.json",
        "presentations/*.json",
        "research_bundles/*/manifest.json",
    ]
    for pat in patterns:
        for path in sorted(sa.glob(pat)):
            if path.is_file():
                candidates.append(path.relative_to(repo).as_posix())
    return candidates


def _should_index_file(rel: str) -> bool:
    name = rel.rsplit("/", 1)[-1]
    if name in _UNINDEXED_ALLOW_NAMES:
        return False
    for suf in _UNINDEXED_ALLOW_SUFFIXES:
        if rel.endswith(suf):
            return False
    if "/members/" in rel and name in ("demo.log", "demo.meta.json"):
        return False
    if "/figures/" in rel or "/assets/" in rel and name.endswith(".png"):
        return False
    return True


def collect_drift_findings(
    index: dict[str, Any],
    *,
    repo_root: Any | None = None,
    expected_index: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    from pathlib import Path

    root = Path(repo_root) if repo_root else Path(__file__).resolve().parents[2]
    findings: list[dict[str, Any]] = []
    seq = 0

    entries = index.get("entries") or []
    for entry in entries:
        eid = entry.get("entry_id", "")
        path_str = entry.get("primary_artifact_path")
        if path_str:
            full = root / path_str
            if not full.is_file():
                seq += 1
                findings.append(
                    _finding(
                        kind="missing_artifact",
                        severity="error",
                        message=f"primary artifact missing for {eid}",
                        seq=seq,
                        entry_id=eid,
                        path=path_str,
                    )
                )
            else:
                digest = sha256_file(full)
                if digest != entry.get("sha256"):
                    seq += 1
                    findings.append(
                        _finding(
                            kind="stale_sha256",
                            severity="warning",
                            message=f"sha256 mismatch for {eid}",
                            seq=seq,
                            entry_id=eid,
                            path=path_str,
                            evidence={"expected": entry.get("sha256"), "actual": digest},
                        )
                    )

    if expected_index is not None:
        if json.dumps(expected_index, sort_keys=True) != json.dumps(index, sort_keys=True):
            seq += 1
            findings.append(
                _finding(
                    kind="index_stale",
                    severity="warning",
                    message="committed corpus index differs from rebuild",
                    seq=seq,
                )
            )

    child_refs: set[str] = set()
    for entry in entries:
        child_refs.update(entry.get("lineage_parent_ids") or [])
    for entry in entries:
        eid = entry.get("entry_id", "")
        kind = entry.get("entry_kind", "")
        parents = entry.get("lineage_parent_ids") or []
        if not parents and kind not in ROOT_ENTRY_KINDS and eid not in child_refs:
            seq += 1
            findings.append(
                _finding(
                    kind="orphan_entry",
                    severity="info",
                    message=f"entry {eid} has no lineage parents (non-root kind)",
                    seq=seq,
                    entry_id=eid,
                )
            )

    indexed = _indexed_paths(index)
    sa_root = root / "fixtures/sa_r0"
    for rel in _discover_candidate_files(sa_root):
        if not _should_index_file(rel):
            continue
        if rel not in indexed:
            seq += 1
            findings.append(
                _finding(
                    kind="unindexed_file",
                    severity="info",
                    message=f"discoverable artifact not in corpus index: {rel}",
                    seq=seq,
                    path=rel,
                )
            )

    index_rev = index.get("index_revision") or ""
    by_id = {e["entry_id"]: e for e in entries if e.get("entry_id")}

    def _check_corpus_ref(path: Path, entry_kind: str, slug: str) -> None:
        nonlocal seq
        if not path.is_file():
            return
        data = json.loads(path.read_text(encoding="utf-8"))
        eid = normalize_entry_id(entry_kind, slug)
        ref = data.get("corpus_ref")
        if not ref:
            seq += 1
            findings.append(
                _finding(
                    kind="corpus_ref_missing",
                    severity="info",
                    message=f"optional corpus_ref absent on {path.relative_to(root).as_posix()}",
                    seq=seq,
                    path=path.relative_to(root).as_posix(),
                    entry_id=eid,
                )
            )
            return
        if ref.get("entry_id") != eid:
            seq += 1
            findings.append(
                _finding(
                    kind="corpus_ref_mismatch",
                    severity="warning",
                    message=f"corpus_ref entry_id mismatch on {path.name}",
                    seq=seq,
                    path=path.relative_to(root).as_posix(),
                    evidence={"expected_entry_id": eid, "actual": ref.get("entry_id")},
                )
            )
        if ref.get("index_revision") and ref.get("index_revision") != index_rev:
            seq += 1
            findings.append(
                _finding(
                    kind="corpus_ref_mismatch",
                    severity="info",
                    message=f"corpus_ref index_revision stale on {path.name}",
                    seq=seq,
                    path=path.relative_to(root).as_posix(),
                    evidence={"index_revision": index_rev, "ref_revision": ref.get("index_revision")},
                )
            )

    for sweep_dir in sorted((sa_root / "sweeps").glob("*/")):
        sid = sweep_dir.name
        sweep_path = sweep_dir / "sweep.json"
        if sweep_path.is_file():
            _check_corpus_ref(sweep_path, "sweep_family", sid)
        pub = sweep_dir / "reports/replay_publication_report_v1.json"
        if pub.is_file():
            _check_corpus_ref(pub, "publication_packet", f"{sid}_publication")

    synth = sa_root / "synthesis/cross_sweep_synthesis_v1.json"
    if synth.is_file():
        _check_corpus_ref(synth, "synthesis_report", "cross_sweep_synthesis_v1")

    rb = sa_root / "research_bundles" / CORPUS_ID / "manifest.json"
    if rb.is_file():
        _check_corpus_ref(rb, "research_bundle", CORPUS_ID)

    canonical = sa_root / "synthesis/replay_corpus_index_v1.json"
    mirror = root / "platform/sa-r0-viewer/public/demo/synthesis/replay_corpus_index_v1.json"
    if canonical.is_file() and mirror.is_file():
        if canonical.read_bytes() != mirror.read_bytes():
            seq += 1
            findings.append(
                _finding(
                    kind="viewer_mirror_stale",
                    severity="warning",
                    message="viewer demo mirror differs from canonical corpus index",
                    seq=seq,
                    path="platform/sa-r0-viewer/public/demo/synthesis/replay_corpus_index_v1.json",
                )
            )

    drift_canonical = sa_root / "corpus_audits/replay_corpus_drift_report_v1.json"
    drift_mirror = root / "platform/sa-r0-viewer/public/demo/corpus_audits/replay_corpus_drift_report_v1.json"
    if drift_canonical.is_file() and drift_mirror.is_file():
        if drift_canonical.read_bytes() != drift_mirror.read_bytes():
            seq += 1
            findings.append(
                _finding(
                    kind="viewer_mirror_stale",
                    severity="warning",
                    message="viewer drift report mirror differs from canonical",
                    seq=seq,
                    path="platform/sa-r0-viewer/public/demo/corpus_audits/replay_corpus_drift_report_v1.json",
                )
            )

    findings.sort(key=lambda f: f["finding_id"])
    return findings


def write_viewer_mirror(canonical: Path, mirror: Path) -> None:
    """Dual-write canonical fixture to sa-r0-viewer public demo mirror."""
    mirror.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(canonical, mirror)


def discover_corpus_releases(repo_root: Path) -> list[dict[str, Any]]:
    """Load release manifests sorted by release_id."""
    releases_dir = repo_root / "fixtures/sa_r0/corpus_releases"
    out: list[dict[str, Any]] = []
    if not releases_dir.is_dir():
        return out
    for manifest_path in sorted(releases_dir.glob("*/replay_corpus_release_manifest_v1.json")):
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        index_path = manifest_path.parent / "replay_corpus_index_v1.json"
        index: dict[str, Any] | None = None
        if index_path.is_file():
            index = json.loads(index_path.read_text(encoding="utf-8"))
        out.append(
            {
                "release_id": manifest.get("release_id") or manifest_path.parent.name,
                "manifest_path": manifest_path.relative_to(repo_root).as_posix(),
                "index_path": index_path.relative_to(repo_root).as_posix()
                if index_path.is_file()
                else None,
                "manifest": manifest,
                "index": index,
            }
        )
    return out


def build_chronology_structure(index: dict[str, Any]) -> list[dict[str, Any]]:
    """Group index entries into ordered chronology tiers."""
    by_group: dict[str, list[str]] = {}
    for entry in index.get("entries") or []:
        cg = entry.get("chronology_group") or "unknown"
        by_group.setdefault(cg, []).append(entry["entry_id"])
    tiers: list[dict[str, Any]] = []
    order = {g: i for i, g in enumerate(CHRONOLOGY_ORDER)}
    for cg in sorted(by_group.keys(), key=lambda g: (order.get(g, 999), g)):
        eids = sorted(by_group[cg])
        tiers.append(
            {
                "tier_id": cg,
                "descriptor": CHRONOLOGY_DESCRIPTORS.get(cg, cg.replace("_", " ")),
                "release_generation_id": RELEASE_GENERATION_IDS.get(cg, "gen_unknown"),
                "entry_ids": eids,
                "entry_count": len(eids),
            }
        )
    return tiers


def build_cross_release_diff_chain(
    releases: list[dict[str, Any]],
    *,
    canonical_index: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    """Pairwise diffs: each release index vs canonical, plus parent chain when present."""
    diffs: list[dict[str, Any]] = []
    if not canonical_index:
        return diffs
    for rel in releases:
        idx = rel.get("index")
        if not idx:
            continue
        rid = rel["release_id"]
        diffs.append(
            {
                "diff_id": f"{rid}__vs__canonical_index",
                "baseline_id": rid,
                "target_id": "canonical_index",
                "diff": diff_corpus_indexes(idx, canonical_index, baseline_id=rid, target_id="canonical_index"),
            }
        )
    return diffs


def rollup_sweep_evolution(
    repo_root: Path,
    sweep_ids: tuple[str, ...] | list[str],
) -> dict[str, Any]:
    """Pull cross-sweep synthesis rollups for evolution summary."""
    synth_path = repo_root / "fixtures/sa_r0/synthesis/cross_sweep_synthesis_v1.json"
    if not synth_path.is_file():
        return {}
    synthesis = json.loads(synth_path.read_text(encoding="utf-8"))
    pattern_rollup = synthesis.get("pattern_frequency_rollup") or {}
    topology_rollup = synthesis.get("topology_sensitivity_rollup") or {}
    ambiguity = synthesis.get("ambiguity_concentration_comparison") or {}
    los = synthesis.get("los_instability_rollup") or {}
    divergence = synthesis.get("divergence_rollup") or {}

    per_sweep: list[dict[str, Any]] = []
    for sid in sweep_ids:
        sweep_path = repo_root / "fixtures/sa_r0/sweeps" / sid / "sweep.json"
        baseline = ""
        if sweep_path.is_file():
            baseline = (json.loads(sweep_path.read_text(encoding="utf-8")) or {}).get(
                "baseline_topology_key"
            ) or ""
        per_sweep.append(
            {
                "sweep_id": sid,
                "baseline_topology_key": baseline,
                "pattern_notes": pattern_rollup.get(sid) if isinstance(pattern_rollup, dict) else None,
                "topology_notes": topology_rollup.get(sid) if isinstance(topology_rollup, dict) else None,
            }
        )

    return {
        "sweep_ids": list(sweep_ids),
        "per_sweep": per_sweep,
        "shared_ambiguity_hotspots": ambiguity.get("shared_hotspot_cells") or [],
        "top_cells_by_sweep": ambiguity.get("top_cells_by_sweep") or {},
        "los_instability_summary": los,
        "divergence_rollup": divergence,
    }


def publication_revision_fingerprint(paths: list[str]) -> str:
    payload = json.dumps(sorted(paths), sort_keys=True)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def summarize_findings(findings: list[dict[str, Any]]) -> dict[str, Any]:
    by_kind: dict[str, int] = {}
    by_severity: dict[str, int] = {}
    for f in findings:
        by_kind[f["kind"]] = by_kind.get(f["kind"], 0) + 1
        by_severity[f["severity"]] = by_severity.get(f["severity"], 0) + 1
    return {
        "total": len(findings),
        "by_kind": dict(sorted(by_kind.items())),
        "by_severity": dict(sorted(by_severity.items())),
    }


def diff_corpus_indexes(
    baseline: dict[str, Any],
    target: dict[str, Any],
    *,
    baseline_id: str,
    target_id: str,
) -> dict[str, Any]:
    b_entries = {e["entry_id"]: e for e in baseline.get("entries") or [] if e.get("entry_id")}
    t_entries = {e["entry_id"]: e for e in target.get("entries") or [] if e.get("entry_id")}

    added = sorted(set(t_entries) - set(b_entries))
    removed = sorted(set(b_entries) - set(t_entries))
    changed: list[dict[str, Any]] = []

    for eid in sorted(set(b_entries) & set(t_entries)):
        be, te = b_entries[eid], t_entries[eid]
        deltas: dict[str, Any] = {}
        for field in ("sha256", "content_revision", "entry_kind", "lineage_parent_ids"):
            if be.get(field) != te.get(field):
                deltas[field] = {"baseline": be.get(field), "target": te.get(field)}
        if deltas:
            changed.append({"entry_id": eid, "deltas": deltas})

    b_edges = {e["edge_id"]: e for e in baseline.get("lineage_edges") or [] if e.get("edge_id")}
    t_edges = {e["edge_id"]: e for e in target.get("lineage_edges") or [] if e.get("edge_id")}
    edges_added = sorted(set(t_edges) - set(b_edges))
    edges_removed = sorted(set(b_edges) - set(t_edges))

    b_rev = baseline.get("index_revision") or ""
    t_rev = target.get("index_revision") or ""

    return {
        "artifact_type": "replay_corpus_release_diff_v1",
        "schema_version": "replay_corpus_release_diff_v1",
        "corpus_id": baseline.get("corpus_id") or target.get("corpus_id") or CORPUS_ID,
        "baseline_id": baseline_id,
        "target_id": target_id,
        "index_revision_delta": {"baseline": b_rev, "target": t_rev},
        "entries_added": added,
        "entries_removed": removed,
        "entries_changed": changed,
        "lineage_edges_added": edges_added,
        "lineage_edges_removed": edges_removed,
        "release_behind_canonical": bool(added or removed or changed or edges_added or edges_removed),
        "governance": DIFF_GOVERNANCE,
    }
