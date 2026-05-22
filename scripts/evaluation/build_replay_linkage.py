#!/usr/bin/env python3
"""Rule-based replay linkage index (replay_linkage_index_v1)."""

from __future__ import annotations

import json
import statistics
import sys
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from build_cross_sweep_synthesis import (  # noqa: E402
    SWEEP_IDS,
    _load_member_bundles,
    _load_sweep,
)
from classify_replay_pattern import PATTERN_PRIORITY, primary_pattern  # noqa: E402
from export_replay_analytics_report import _lint_markdown  # noqa: E402
from replay_narrative_intelligence import _member_metrics  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]
METRIC_RATIO_LO = 0.85
METRIC_RATIO_HI = 1.15
GOVERNANCE = {
    "notice": (
        "Replay linkage index for reviewer exploration only — "
        "rule-based relationships between sweep families, not causal inference."
    ),
    "anti_claims": [
        "Linkage edges are replay-derived heuristics, not validated doctrine.",
        "Shared patterns indicate descriptive similarity in fixture replays only.",
    ],
}


def _load_sweeps_index() -> dict[str, Any]:
    path = _REPO / "fixtures/scenarios/sweeps_index_v1.json"
    return json.loads(path.read_text(encoding="utf-8"))


def _experiment_tags(sweep_id: str, index: dict[str, Any]) -> list[str]:
    for s in index.get("sweeps") or []:
        if s.get("sweep_id") == sweep_id:
            return list(s.get("experiment_tags") or [])
    return []


def _dominant_patterns(manifest: dict[str, Any]) -> list[str]:
    counts: dict[str, int] = {}
    for m in manifest.get("members") or []:
        pid = primary_pattern(m.get("replay_pattern_tags") or [])
        if pid:
            counts[pid] = counts.get(pid, 0) + 1
    return sorted(counts.keys(), key=lambda p: (-counts[p], PATTERN_PRIORITY.index(p) if p in PATTERN_PRIORITY else 99))


def build_nodes(manifests: list[dict[str, Any]], index: dict[str, Any]) -> list[dict[str, Any]]:
    nodes: list[dict[str, Any]] = []
    for manifest in manifests:
        sid = manifest["sweep_id"]
        linkage = manifest.get("topology_linkage") or {}
        nodes.append(
            {
                "sweep_id": sid,
                "baseline_topology_key": manifest.get("baseline_topology_key"),
                "topology_keys": list(linkage.get("topology_keys") or []),
                "dominant_patterns": _dominant_patterns(manifest),
                "experiment_tags": _experiment_tags(sid, index),
            }
        )
    return nodes


def _sweep_medians(manifest: dict[str, Any]) -> dict[str, float | None]:
    bundles = _load_member_bundles(manifest)
    metrics = [_member_metrics(b) for b in bundles]
    fds = [m["first_detection_t"] for m in metrics if m["first_detection_t"] is not None]
    ambs = [m["ambiguity_window_count"] for m in metrics]
    return {
        "median_first_detection": float(statistics.median(fds)) if fds else None,
        "median_ambiguity": float(statistics.median(ambs)) if ambs else None,
    }


def _ratio_similar(a: float | None, b: float | None) -> bool:
    if a is None or b is None or a == 0 or b == 0:
        return False
    ratio = a / b if a >= b else b / a
    return METRIC_RATIO_LO <= ratio <= METRIC_RATIO_HI


def _edge_copy(kind: str, source: str, target: str, detail: str) -> str:
    templates = {
        "shared_pattern": (
            f"`{source}` and `{target}` share replay pattern `{detail}` "
            "in reviewer exploration — explanatory similarity only."
        ),
        "shared_topology": (
            f"`{source}` and `{target}` share topology key `{detail}` "
            "in fixture replay families — not operational equivalence."
        ),
        "metric_similarity": (
            f"`{source}` and `{target}` show similar {detail} medians "
            "across replay variants — replay-local concentration only."
        ),
        "storyline_reference": (
            f"Curated storyboard references relate `{source}` and `{target}` — {detail}"
        ),
    }
    return templates.get(kind, f"`{source}` linked to `{target}` — {detail}")


def build_edges(
    nodes: list[dict[str, Any]],
    manifests: list[dict[str, Any]],
    *,
    storyline_overlay: list[dict[str, Any]] | None = None,
) -> list[dict[str, Any]]:
    edges: list[dict[str, Any]] = []
    seen: set[str] = set()
    manifest_by_id = {m["sweep_id"]: m for m in manifests}
    medians = {m["sweep_id"]: _sweep_medians(m) for m in manifests}

    def add_edge(source: str, target: str, kind: str, evidence: dict[str, Any], detail: str) -> None:
        if source == target:
            return
        a, b = sorted([source, target])
        edge_id = f"{a}__{b}__{kind}"
        if edge_id in seen:
            return
        seen.add(edge_id)
        edges.append(
            {
                "edge_id": edge_id,
                "source": a,
                "target": b,
                "link_kind": kind,
                "evidence": evidence,
                "copy": _edge_copy(kind, a, b, detail),
            }
        )

    for i, n1 in enumerate(nodes):
        for n2 in nodes[i + 1 :]:
            s1, s2 = n1["sweep_id"], n2["sweep_id"]
            shared_patterns = set(n1["dominant_patterns"]) & set(n2["dominant_patterns"])
            for p in sorted(shared_patterns):
                add_edge(s1, s2, "shared_pattern", {"pattern_id": p}, p)

            shared_topo = set(n1["topology_keys"]) & set(n2["topology_keys"])
            if n1["baseline_topology_key"] == n2["baseline_topology_key"]:
                shared_topo.add(str(n1["baseline_topology_key"]))
            for k in sorted(shared_topo):
                add_edge(s1, s2, "shared_topology", {"topology_key": k}, k)

            m1, m2 = medians[s1], medians[s2]
            if _ratio_similar(m1.get("median_first_detection"), m2.get("median_first_detection")):
                add_edge(
                    s1,
                    s2,
                    "metric_similarity",
                    {"metric": "median_first_detection", "values": [m1["median_first_detection"], m2["median_first_detection"]]},
                    "first-detection timing",
                )
            if _ratio_similar(m1.get("median_ambiguity"), m2.get("median_ambiguity")):
                add_edge(
                    s1,
                    s2,
                    "metric_similarity",
                    {"metric": "median_ambiguity", "values": [m1["median_ambiguity"], m2["median_ambiguity"]]},
                    "ambiguity window count",
                )

    for item in storyline_overlay or []:
        add_edge(
            str(item["source"]),
            str(item["target"]),
            "storyline_reference",
            item.get("evidence") or {},
            str(item.get("detail", "storyboard reference")),
        )

    edges.sort(key=lambda e: e["edge_id"])
    return edges


def _default_storyline_overlay() -> list[dict[str, Any]]:
    return [
        {
            "source": "ridge_overlap_sweep",
            "target": "delayed_detection_sweep",
            "detail": "both sweeps appear in topology-divergence showcase decks",
            "evidence": {"storyboard_id": "showcase_topology_divergence"},
        },
        {
            "source": "delayed_detection_sweep",
            "target": "saturation_assignment_sweep",
            "detail": "ingress-timing variation themes in assignment instability walkthrough",
            "evidence": {"storyboard_id": "walkthrough_assignment_instability"},
        },
    ]


def build_linkage_index(
    sweep_ids: tuple[str, ...] | None = None,
    *,
    storyline_overlay: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    ids = sweep_ids or SWEEP_IDS
    manifests = [_load_sweep(sid) for sid in ids]
    index = _load_sweeps_index()
    nodes = build_nodes(manifests, index)
    overlay_path = _REPO / "fixtures/sa_r0/synthesis/storyline_linkage_overlay_v1.json"
    overlay = storyline_overlay
    if overlay is None and overlay_path.is_file():
        data = json.loads(overlay_path.read_text(encoding="utf-8"))
        overlay = data.get("edges") or _default_storyline_overlay()
    elif overlay is None:
        overlay = _default_storyline_overlay()

    return {
        "artifact_type": "replay_linkage_index_v1",
        "schema_version": "replay_linkage_index_v1",
        "governance": GOVERNANCE,
        "nodes": nodes,
        "edges": build_edges(nodes, manifests, storyline_overlay=overlay),
    }


def render_linkage_summary(linkage: dict[str, Any]) -> str:
    lines = [
        "# Replay linkage summary",
        "",
        linkage.get("governance", {}).get("notice", ""),
        "",
        "## Nodes",
        "",
        "| sweep_id | baseline | dominant patterns |",
        "|----------|----------|-------------------|",
    ]
    for n in linkage.get("nodes") or []:
        patterns = ", ".join(n.get("dominant_patterns") or []) or "—"
        lines.append(f"| {n.get('sweep_id')} | {n.get('baseline_topology_key')} | {patterns} |")

    lines.extend(["", "## Edges", ""])
    for e in linkage.get("edges") or []:
        lines.append(
            f"- **{e.get('link_kind')}** `{e.get('source')}` ↔ `{e.get('target')}`: {e.get('copy')}"
        )
    lines.append("")
    return "\n".join(lines)


def write_linkage(linkage: dict[str, Any]) -> None:
    fixture = _REPO / "fixtures/sa_r0/synthesis"
    pub = _REPO / "platform/sa-r0-viewer/public/demo/synthesis"
    fixture.mkdir(parents=True, exist_ok=True)
    pub.mkdir(parents=True, exist_ok=True)

    json_text = json.dumps(linkage, indent=2, sort_keys=True) + "\n"
    md_text = render_linkage_summary(linkage)
    lint = _lint_markdown(md_text)
    if lint:
        raise SystemExit(f"linkage_summary lint: {lint}")

    for d in (fixture, pub):
        (d / "replay_linkage_index_v1.json").write_text(json_text, encoding="utf-8")
        (d / "linkage_summary.md").write_text(md_text, encoding="utf-8")

    overlay_src = fixture / "storyline_linkage_overlay_v1.json"
    if overlay_src.is_file():
        overlay_text = overlay_src.read_text(encoding="utf-8")
        (pub / "storyline_linkage_overlay_v1.json").write_text(overlay_text, encoding="utf-8")


def check_linkage() -> None:
    expected = build_linkage_index()
    path = _REPO / "fixtures/sa_r0/synthesis/replay_linkage_index_v1.json"
    if not path.is_file():
        raise SystemExit(f"missing linkage fixture: {path}")
    actual = json.loads(path.read_text(encoding="utf-8"))
    if json.dumps(actual, sort_keys=True) != json.dumps(expected, sort_keys=True):
        raise SystemExit(
            "replay_linkage_index_v1.json is stale — run build_replay_linkage.py"
        )


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description="Build replay linkage index")
    ap.add_argument("--check", action="store_true", help="verify committed fixtures match")
    args = ap.parse_args()

    if args.check:
        check_linkage()
        print("replay linkage check OK")
        return

    linkage = build_linkage_index()
    write_linkage(linkage)
    print("replay linkage index OK")


if __name__ == "__main__":
    main()
