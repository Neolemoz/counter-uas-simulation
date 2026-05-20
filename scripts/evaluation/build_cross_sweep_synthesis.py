#!/usr/bin/env python3
"""Deterministic cross-sweep replay synthesis (cross_sweep_synthesis_v1)."""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from aggregate_spatial_analytics import load_bundle  # noqa: E402
from classify_replay_pattern import PATTERN_PRIORITY, primary_pattern  # noqa: E402
from export_replay_analytics_report import _lint_markdown  # noqa: E402
from replay_mc_sweep import validate_sweep  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]
SWEEP_IDS = (
    "valley_sensor_sweep",
    "ridge_overlap_sweep",
    "delayed_detection_sweep",
    "saturation_assignment_sweep",
)
TOP_N_CELLS = 5
GOVERNANCE = {
    "notice": (
        "Cross-sweep replay synthesis for mentor review and research documentation only — "
        "explanatory replay rollups, not operational planning or validated doctrine."
    ),
    "anti_claims": [
        "Synthesis aggregates are not parser authority.",
        "Cross-sweep comparisons are replay-local concentration summaries only.",
        "No deployment confidence or tactical superiority implied.",
    ],
}


def _load_sweep(sweep_id: str) -> dict[str, Any]:
    path = _REPO / "fixtures/sa_r0/sweeps" / sweep_id / "sweep.json"
    return json.loads(path.read_text(encoding="utf-8"))


def _load_member_bundles(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    sid = manifest["sweep_id"]
    bundles: list[dict[str, Any]] = []
    for m in manifest.get("members") or []:
        mid = m["member_id"]
        path = _REPO / "fixtures/sa_r0/sweeps" / sid / "members" / mid / "index.json"
        bundles.append(load_bundle(path))
    return bundles


def _top_cell_indices(counts: list[int], n: int = TOP_N_CELLS) -> list[tuple[int, int]]:
    indexed = [(i, c) for i, c in enumerate(counts) if c > 0]
    indexed.sort(key=lambda x: (-x[1], x[0]))
    return indexed[:n]


def _member_los_counts(bundles: list[dict[str, Any]]) -> list[int]:
    return [
        sum(
            1
            for s in b.get("los_segments") or []
            if str(s.get("status")) in ("terrain_blocked", "partially_occluded")
        )
        for b in bundles
    ]


def build_pattern_frequency_rollup(
    manifests: list[dict[str, Any]],
) -> dict[str, Any]:
    by_pattern: dict[str, dict[str, Any]] = {}
    for manifest in manifests:
        sid = manifest["sweep_id"]
        for m in manifest.get("members") or []:
            tags = m.get("replay_pattern_tags") or []
            pid = primary_pattern(tags) or "ungrouped_replay"
            entry = by_pattern.setdefault(
                pid,
                {"count": 0, "sweep_ids": [], "member_count": 0},
            )
            entry["count"] += 1
            entry["member_count"] += 1
            if sid not in entry["sweep_ids"]:
                entry["sweep_ids"].append(sid)

    for pid in PATTERN_PRIORITY:
        if pid in by_pattern:
            by_pattern[pid]["sweep_ids"] = sorted(by_pattern[pid]["sweep_ids"])

    dominant = sorted(
        by_pattern.keys(),
        key=lambda p: (-by_pattern[p]["member_count"], PATTERN_PRIORITY.index(p) if p in PATTERN_PRIORITY else 99),
    )
    return {"by_pattern": by_pattern, "dominant_across_corpus": dominant}


def build_ambiguity_concentration_comparison(
    manifests: list[dict[str, Any]],
) -> dict[str, Any]:
    top_by_sweep: dict[str, list[dict[str, Any]]] = {}
    all_tops: dict[str, set[int]] = {}

    for manifest in manifests:
        sid = manifest["sweep_id"]
        layer = (manifest.get("spatial_aggregate") or {}).get("layers", {}).get(
            "ambiguity_density", {}
        )
        counts = layer.get("counts") or []
        tops = _top_cell_indices(counts)
        top_by_sweep[sid] = [{"cell_index": i, "count": c} for i, c in tops]
        all_tops[sid] = {i for i, _ in tops}

    cell_sweep_count: dict[int, int] = {}
    for tops in all_tops.values():
        for cell in tops:
            cell_sweep_count[cell] = cell_sweep_count.get(cell, 0) + 1
    shared = sorted(i for i, n in cell_sweep_count.items() if n >= 2)

    return {
        "top_cells_by_sweep": top_by_sweep,
        "shared_hotspot_cells": shared,
    }


def build_topology_sensitivity_rollup(
    manifests: list[dict[str, Any]],
) -> dict[str, Any]:
    by_sweep: dict[str, dict[str, Any]] = {}
    all_keys: dict[str, set[str]] = {}

    for manifest in manifests:
        sid = manifest["sweep_id"]
        layer = (manifest.get("spatial_aggregate") or {}).get("layers", {}).get(
            "topology_sensitivity", {}
        )
        counts = layer.get("counts") or []
        nonzero = [c for c in counts if c > 0]
        linkage = manifest.get("topology_linkage") or {}
        keys = list(linkage.get("topology_keys") or [])
        by_sweep[sid] = {
            "max_count": max(counts) if counts else 0,
            "median_count": float(statistics.median(nonzero)) if nonzero else 0.0,
            "topology_keys": keys,
        }
        all_keys[sid] = set(keys)

    shared_keys: set[str] = set()
    sweep_list = list(all_keys.keys())
    for i, s1 in enumerate(sweep_list):
        for s2 in sweep_list[i + 1 :]:
            shared_keys |= all_keys[s1] & all_keys[s2]

    baselines = {m.get("baseline_topology_key") for m in manifests}
    shared_keys |= {b for b in baselines if sum(1 for m in manifests if m.get("baseline_topology_key") == b) >= 2}

    return {
        "by_sweep": by_sweep,
        "shared_topology_keys": sorted(shared_keys),
    }


def build_los_instability_rollup(
    manifests: list[dict[str, Any]],
    bundles_by_sweep: dict[str, list[dict[str, Any]]],
) -> dict[str, Any]:
    by_sweep: dict[str, dict[str, Any]] = {}
    for manifest in manifests:
        sid = manifest["sweep_id"]
        layer = (manifest.get("spatial_aggregate") or {}).get("layers", {}).get(
            "los_degraded", {}
        )
        counts = layer.get("counts") or []
        member_los = _member_los_counts(bundles_by_sweep.get(sid, []))
        by_sweep[sid] = {
            "max_los_count": max(counts) if counts else 0,
            "member_los_totals": member_los,
        }

    ranked = sorted(
        by_sweep.keys(),
        key=lambda s: (-by_sweep[s]["max_los_count"], s),
    )
    return {"by_sweep": by_sweep, "ranked_sweep_ids": ranked}


def build_divergence_rollup(manifests: list[dict[str, Any]]) -> dict[str, Any]:
    sweeps_with: list[str] = []
    member_count = 0
    div_tag = "topology_sensitive_divergence"

    for manifest in manifests:
        sid = manifest["sweep_id"]
        has_div = False
        for m in manifest.get("members") or []:
            tags = m.get("replay_pattern_tags") or []
            if div_tag in tags or "topology_sensitive_divergence" in str(m.get("replay_pattern_summary", "")):
                member_count += 1
                has_div = True
        if has_div:
            sweeps_with.append(sid)

    return {
        "sweeps_with_divergence": sweeps_with,
        "member_divergence_count": member_count,
    }


def build_cross_sweep_synthesis(
    sweep_ids: tuple[str, ...] | None = None,
) -> dict[str, Any]:
    ids = sweep_ids or SWEEP_IDS
    manifests = [_load_sweep(sid) for sid in ids]
    bundles_by_sweep = {m["sweep_id"]: _load_member_bundles(m) for m in manifests}

    return {
        "artifact_type": "cross_sweep_synthesis_v1",
        "schema_version": "cross_sweep_synthesis_v1",
        "governance": GOVERNANCE,
        "sweep_ids": list(ids),
        "pattern_frequency_rollup": build_pattern_frequency_rollup(manifests),
        "ambiguity_concentration_comparison": build_ambiguity_concentration_comparison(manifests),
        "topology_sensitivity_rollup": build_topology_sensitivity_rollup(manifests),
        "los_instability_rollup": build_los_instability_rollup(manifests, bundles_by_sweep),
        "divergence_rollup": build_divergence_rollup(manifests),
        "interpretation_caveats": [
            "Cross-sweep synthesis is replay-local and explanatory only.",
            "Shared spatial cells indicate concentration overlap in reviewer exploration — not causal proof.",
            "Pattern frequency counts reflect taxonomy tags on fixture members, not operational doctrine.",
        ],
    }


def render_cross_sweep_summary(synthesis: dict[str, Any]) -> str:
    lines = [
        "# Cross-sweep replay synthesis summary",
        "",
        synthesis.get("governance", {}).get("notice", ""),
        "",
        f"Sweeps included: `{', '.join(synthesis.get('sweep_ids') or [])}`",
        "",
        "## Pattern frequency rollup",
        "",
    ]
    rollup = synthesis.get("pattern_frequency_rollup") or {}
    for pid in rollup.get("dominant_across_corpus") or []:
        entry = (rollup.get("by_pattern") or {}).get(pid, {})
        sweeps = ", ".join(entry.get("sweep_ids") or [])
        lines.append(
            f"- **{pid}**: {entry.get('member_count', 0)} member(s) across [{sweeps}]"
        )

    lines.extend(["", "## Ambiguity concentration comparison", ""])
    amb = synthesis.get("ambiguity_concentration_comparison") or {}
    shared = amb.get("shared_hotspot_cells") or []
    if shared:
        lines.append(f"- Shared hotspot cells across sweeps: `{shared[:10]}`")
    else:
        lines.append("- No shared top-N ambiguity hotspot cells across sweeps.")

    lines.extend(["", "## Topology sensitivity rollup", ""])
    topo = synthesis.get("topology_sensitivity_rollup") or {}
    for key in topo.get("shared_topology_keys") or []:
        lines.append(f"- Shared topology key: `{key}`")

    lines.extend(["", "## LOS instability rollup", ""])
    los = synthesis.get("los_instability_rollup") or {}
    for sid in los.get("ranked_sweep_ids") or []:
        entry = (los.get("by_sweep") or {}).get(sid, {})
        lines.append(f"- `{sid}`: max LOS layer count {entry.get('max_los_count', 0)}")

    lines.extend(["", "## Divergence rollup", ""])
    div = synthesis.get("divergence_rollup") or {}
    lines.append(
        f"- Sweeps with topology-sensitive divergence: `{div.get('sweeps_with_divergence', [])}`"
    )
    lines.append(f"- Member divergence count: {div.get('member_divergence_count', 0)}")

    lines.extend(["", "## Interpretation caveats", ""])
    for c in synthesis.get("interpretation_caveats") or []:
        lines.append(f"- {c}")
    lines.append("")
    return "\n".join(lines)


def _out_paths() -> tuple[Path, Path]:
    fixture = _REPO / "fixtures/sa_r0/synthesis"
    pub = _REPO / "platform/sa-r0-viewer/public/demo/synthesis"
    fixture.mkdir(parents=True, exist_ok=True)
    pub.mkdir(parents=True, exist_ok=True)
    return fixture, pub


def write_synthesis(synthesis: dict[str, Any]) -> None:
    fixture_dir, pub_dir = _out_paths()
    json_text = json.dumps(synthesis, indent=2, sort_keys=True) + "\n"
    md_text = render_cross_sweep_summary(synthesis)
    lint = _lint_markdown(md_text)
    if lint:
        raise SystemExit(f"cross_sweep_summary lint: {lint}")

    for d in (fixture_dir, pub_dir):
        (d / "cross_sweep_synthesis_v1.json").write_text(json_text, encoding="utf-8")
        (d / "cross_sweep_summary.md").write_text(md_text, encoding="utf-8")


def check_synthesis() -> None:
    from build_replay_cognition_rollup import enrich_synthesis_with_cognition  # noqa: WPS433

    expected = enrich_synthesis_with_cognition(build_cross_sweep_synthesis())
    path = _REPO / "fixtures/sa_r0/synthesis/cross_sweep_synthesis_v1.json"
    if not path.is_file():
        raise SystemExit(f"missing synthesis fixture: {path}")
    actual = json.loads(path.read_text(encoding="utf-8"))
    if json.dumps(actual, sort_keys=True) != json.dumps(expected, sort_keys=True):
        raise SystemExit("cross_sweep_synthesis_v1.json is stale — run build_cross_sweep_synthesis.py --all")


def main() -> None:
    ap = argparse.ArgumentParser(description="Build cross-sweep replay synthesis")
    ap.add_argument("--all", action="store_true", help="build and write synthesis artifacts")
    ap.add_argument("--check", action="store_true", help="verify committed fixtures match")
    args = ap.parse_args()

    if args.check:
        check_synthesis()
        print("cross-sweep synthesis check OK")
        return

    if args.all:
        for sid in SWEEP_IDS:
            manifest = _load_sweep(sid)
            errs = validate_sweep(manifest)
            if errs:
                raise SystemExit(f"invalid sweep {sid}: {errs}")
        synthesis = build_cross_sweep_synthesis()
        write_synthesis(synthesis)
        print("cross-sweep synthesis OK")
        return

    raise SystemExit("specify --all or --check")


if __name__ == "__main__":
    main()
