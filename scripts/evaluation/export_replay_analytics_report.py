#!/usr/bin/env python3
"""Export deterministic replay analytics reports for MC sweeps."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from replay_mc_sweep import validate_sweep  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]
FORBIDDEN_PHRASES = re.compile(
    r"(?<!not )deployment\s+readiness|(?<!not )validated\s+effectiveness|(?<!not )validated\s+probability|"
    r"P\s*\(\s*kill\s*\)|tactical\s+superiority",
    re.I,
)


def _lint_markdown(text: str) -> list[str]:
    issues = []
    if FORBIDDEN_PHRASES.search(text):
        issues.append("forbidden operational phrasing in report")
    return issues


def _load_sweep(sweep_id: str) -> dict[str, Any]:
    path = _REPO / "fixtures/sa_r0/sweeps" / sweep_id / "sweep.json"
    if not path.is_file():
        raise SystemExit(f"missing sweep: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def render_analytics_summary(manifest: dict[str, Any]) -> str:
    sid = manifest.get("sweep_id")
    agg = manifest.get("replay_aggregation") or {}
    patterns = agg.get("dominant_patterns") or []
    hist = agg.get("outcome_histogram") or {}
    lines = [
        f"# Replay analytics summary — `{sid}`",
        "",
        manifest.get("governance", {}).get("notice", ""),
        "",
        "## Dominant replay patterns (explanatory)",
        "",
    ]
    for p in patterns:
        lines.append(f"- {p}")
    lines.extend(["", "## Outcome histogram (replay-local)", ""])
    for key, vals in sorted(hist.items()):
        lines.append(f"- **{key}**: `{vals}`")
    lines.extend(
        [
            "",
            "## Spatial layers",
            "",
            "Precomputed concentration grids in `sweep.json` → `spatial_aggregate.layers`.",
            "Explanatory replay concentration only — not operational prediction.",
            "",
        ]
    )
    return "\n".join(lines) + "\n"


def render_topology_summary(manifest: dict[str, Any]) -> str:
    sid = manifest.get("sweep_id")
    members = manifest.get("members") or []
    lines = [
        f"# Topology sweep summary — `{sid}`",
        "",
        "| member_id | pack_id | seed |",
        "|-----------|---------|------|",
    ]
    for m in members:
        lines.append(
            f"| {m.get('member_id')} | {m.get('pack_id')} | {m.get('seed', '—')} |"
        )
    lines.append("")
    lines.append(f"Baseline topology: `{manifest.get('baseline_topology_key')}`")
    return "\n".join(lines) + "\n"


def render_sweep_narrative_summary(manifest: dict[str, Any]) -> str:
    sid = manifest.get("sweep_id")
    narrative = manifest.get("replay_narrative_summary") or {}
    lines = [
        f"# Sweep narrative summary — `{sid}`",
        "",
        manifest.get("governance", {}).get("notice", ""),
        "",
        f"## {narrative.get('headline', 'Replay narrative summary')}",
        "",
    ]
    for b in narrative.get("bullets") or []:
        lines.append(f"- {b}")
    lines.extend(["", "## Cohorts", ""])
    for c in manifest.get("replay_cohorts") or []:
        idxs = ", ".join(str(i) for i in c.get("member_indices") or [])
        lines.append(f"- **{c.get('label')}** (members {idxs}): {c.get('dominant_summary')}")
    lines.append("")
    return "\n".join(lines) + "\n"


def render_replay_cluster_report(manifest: dict[str, Any]) -> str:
    sid = manifest.get("sweep_id")
    lines = [
        f"# Replay cluster report — `{sid}`",
        "",
        "| cohort_id | pattern_tags | members | anomalies |",
        "|-----------|--------------|---------|-----------|",
    ]
    for c in manifest.get("replay_cohorts") or []:
        tags = ", ".join(c.get("pattern_tags") or [])
        idxs = ", ".join(str(i) for i in c.get("member_indices") or [])
        anom = ", ".join(str(i) for i in c.get("anomaly_member_indices") or []) or "—"
        lines.append(
            f"| {c.get('cohort_id')} | {tags} | {idxs} | {anom} |"
        )
    lines.extend(["", "## Member pattern tags", ""])
    for m in manifest.get("members") or []:
        tags = ", ".join(m.get("replay_pattern_tags") or []) or "—"
        lines.append(f"- `{m.get('member_id')}` ({m.get('pack_id')}): {tags}")
    lines.append("")
    return "\n".join(lines) + "\n"


def render_topology_divergence_report(manifest: dict[str, Any]) -> str:
    sid = manifest.get("sweep_id")
    baseline = manifest.get("baseline_topology_key")
    members = manifest.get("members") or []
    lines = [
        f"# Topology divergence report — `{sid}`",
        "",
        f"Baseline topology: `{baseline}`",
        "",
    ]
    for m in members:
        pack = m.get("pack_id")
        if pack == baseline:
            lines.append(f"- `{m.get('member_id')}`: baseline pack (no divergence tag required)")
        else:
            lines.append(
                f"- `{m.get('member_id')}`: pack `{pack}` differs from baseline — "
                "replay-local topology comparison only."
            )
    linkage = manifest.get("topology_linkage") or {}
    if linkage.get("shared_log_ref"):
        lines.append(f"\nShared log ref: `{linkage['shared_log_ref']}`")
    lines.append("")
    return "\n".join(lines) + "\n"


def render_ambiguity_hotspot_summary(manifest: dict[str, Any]) -> str:
    sid = manifest.get("sweep_id")
    layer = (manifest.get("spatial_aggregate") or {}).get("layers", {}).get(
        "ambiguity_density", {}
    )
    counts = layer.get("counts") or []
    grid = (manifest.get("spatial_aggregate") or {}).get("grid") or {}
    lines = [
        f"# Ambiguity hotspot summary — `{sid}`",
        "",
        "Top replay ambiguity concentration cells (explanatory only):",
        "",
    ]
    indexed = [(i, c) for i, c in enumerate(counts) if c > 0]
    indexed.sort(key=lambda x: -x[1])
    cols = int((grid.get("size") or [40, 30])[0])
    ox, oy = (grid.get("origin_enu_m") or [-2500, -500])[:2]
    spacing = float(grid.get("spacing_m") or 100)
    for i, count in indexed[:12]:
        row, col = divmod(i, cols)
        cx = ox + (col + 0.5) * spacing
        cy = oy + (row + 0.5) * spacing
        lines.append(f"- cell ({col},{row}) count={count} ENU≈[{cx:.0f},{cy:.0f}]")
    if not indexed:
        lines.append("- No ambiguity density cells recorded.")
    lines.append("")
    return "\n".join(lines) + "\n"


def render_review_report_v1(manifest: dict[str, Any]) -> dict[str, Any]:
    return {
        "artifact_type": "replay_review_report_v1",
        "schema_version": "replay_review_report_v1",
        "sweep_id": manifest.get("sweep_id"),
        "governance": manifest.get("governance"),
        "replay_narrative_summary": manifest.get("replay_narrative_summary"),
        "replay_cohorts": manifest.get("replay_cohorts"),
        "members": [
            {
                "member_id": m.get("member_id"),
                "pack_id": m.get("pack_id"),
                "replay_pattern_tags": m.get("replay_pattern_tags"),
                "replay_pattern_summary": m.get("replay_pattern_summary"),
            }
            for m in manifest.get("members") or []
        ],
        "replay_aggregation": manifest.get("replay_aggregation"),
        "interpretation_caveats": [
            "Replay review artifact — explanatory only, not parser authority.",
            "Pattern tags are replay-local taxonomy, not operational recommendations.",
        ],
    }


def render_compare_report_v1(manifest: dict[str, Any]) -> dict[str, Any]:
    return {
        "artifact_type": "replay_compare_report_v1",
        "schema_version": "replay_compare_report_v1",
        "sweep_id": manifest.get("sweep_id"),
        "governance": manifest.get("governance"),
        "members": [
            {
                "member_id": m.get("member_id"),
                "pack_id": m.get("pack_id"),
                "seed": m.get("seed"),
                "demo_bundle_url": m.get("demo_bundle_url"),
            }
            for m in manifest.get("members") or []
        ],
        "replay_aggregation": manifest.get("replay_aggregation"),
        "spatial_aggregate_summary": {
            "grid": (manifest.get("spatial_aggregate") or {}).get("grid"),
            "layer_keys": list(
                ((manifest.get("spatial_aggregate") or {}).get("layers") or {}).keys()
            ),
        },
        "interpretation_caveats": [
            "Explanatory replay compare report — not parser authority.",
            "Spatial aggregates are replay concentration, not probability.",
        ],
    }


def export_sweep(sweep_id: str, *, formats: str, check_only: bool = False) -> None:
    manifest = _load_sweep(sweep_id)
    errs = validate_sweep(manifest)
    if errs:
        raise SystemExit(f"invalid sweep: {errs}")
    out_dir = _REPO / "fixtures/sa_r0/sweeps" / sweep_id / "reports"
    fmt_set = {f.strip() for f in formats.split(",") if f.strip()}

    md_reports = (
        ("analytics_summary.md", render_analytics_summary),
        ("topology_sweep_summary.md", render_topology_summary),
        ("sweep_narrative_summary.md", render_sweep_narrative_summary),
        ("replay_cluster_report.md", render_replay_cluster_report),
        ("topology_divergence_report.md", render_topology_divergence_report),
        ("ambiguity_hotspot_summary.md", render_ambiguity_hotspot_summary),
    )

    if "md" in fmt_set:
        for filename, renderer in md_reports:
            text = renderer(manifest)
            lint = _lint_markdown(text)
            if lint:
                raise SystemExit(f"{filename}: {lint}")
            if not check_only:
                out_dir.mkdir(parents=True, exist_ok=True)
                (out_dir / filename).write_text(text, encoding="utf-8")

    if "json" in fmt_set:
        compare_report = render_compare_report_v1(manifest)
        review_report = render_review_report_v1(manifest)
        if not check_only:
            out_dir.mkdir(parents=True, exist_ok=True)
            (out_dir / "replay_compare_report_v1.json").write_text(
                json.dumps(compare_report, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            (out_dir / "replay_review_report_v1.json").write_text(
                json.dumps(review_report, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )

    print(f"export OK: {sweep_id} ({formats})")


def main() -> None:
    ap = argparse.ArgumentParser(description="Export replay analytics reports for sweeps")
    ap.add_argument("--sweep", required=True, help="sweep_id")
    ap.add_argument("--format", default="md,json", help="comma-separated: md,json")
    ap.add_argument("--check", action="store_true", help="validate only, do not write")
    args = ap.parse_args()
    export_sweep(args.sweep, formats=args.format, check_only=args.check)


if __name__ == "__main__":
    main()
