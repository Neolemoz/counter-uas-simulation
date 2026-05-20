#!/usr/bin/env python3
"""Reviewer cognition rollups (replay_cognition_rollup_v1)."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from build_cross_sweep_synthesis import build_cross_sweep_synthesis  # noqa: E402
from build_replay_storytelling import FORBIDDEN_SUBSTRINGS, lint_storytelling_sections  # noqa: E402
from export_replay_analytics_report import _lint_markdown  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]

OPERATIONAL_DENYLIST = FORBIDDEN_SUBSTRINGS + (
    "recommend intercept",
    "should deploy",
    "tactical doctrine",
)


def _lint_bullet(observation: str, caveat: str) -> list[str]:
    combined = f"{observation} {caveat}"
    issues: list[str] = []
    lower = combined.lower()
    for phrase in OPERATIONAL_DENYLIST:
        if phrase.lower() in lower:
            issues.append(f"forbidden phrase: {phrase}")
    return issues


def build_cognition_rollup(synthesis: dict[str, Any]) -> dict[str, Any]:
    bullets: list[dict[str, Any]] = []
    pattern = synthesis.get("pattern_frequency_rollup") or {}
    dominant = pattern.get("dominant_across_corpus") or []
    los = synthesis.get("los_instability_rollup") or {}
    amb = synthesis.get("ambiguity_concentration_comparison") or {}
    div = synthesis.get("divergence_rollup") or {}
    topo = synthesis.get("topology_sensitivity_rollup") or {}

    if "los_fragmented_replay" in dominant:
        ranked = los.get("ranked_sweep_ids") or []
        top = ranked[0] if ranked else "ridge_overlap_sweep"
        bullets.append(
            {
                "bullet_id": "los_instability_concentration",
                "observation": (
                    f"LOS-fragmented replay concentration appears most strongly in `{top}` "
                    "and related ridge-transition fixture variants"
                ),
                "caveat": "derived from replay pattern tags and spatial LOS layers only",
                "related_sweep_ids": ranked[:2],
                "topology_group": "ridge_transition",
            }
        )

    shared_cells = amb.get("shared_hotspot_cells") or []
    if shared_cells:
        sweep_ids = synthesis.get("sweep_ids") or []
        bullets.append(
            {
                "bullet_id": "ambiguity_hotspot_overlap",
                "observation": (
                    f"Ambiguity-heavy sweeps share {len(shared_cells)} spatial hotspot cell(s) "
                    "in overlapping ingress windows across fixture replays"
                ),
                "caveat": "spatial grid overlap is explanatory concentration, not sensor physics proof",
                "related_sweep_ids": sweep_ids[:3],
                "topology_group": "ingress_overlap",
            }
        )

    div_sweeps = div.get("sweeps_with_divergence") or []
    if div_sweeps:
        bullets.append(
            {
                "bullet_id": "topology_sensitive_divergence_clusters",
                "observation": (
                    "Topology-sensitive replay instability clusters in "
                    f"{', '.join(f'`{s}`' for s in div_sweeps)} cohorts"
                ),
                "caveat": "divergence tags reflect replay taxonomy on fixture members only",
                "related_sweep_ids": div_sweeps,
                "topology_group": "delayed_detection_cohort",
            }
        )

    shared_keys = topo.get("shared_topology_keys") or []
    if shared_keys:
        bullets.append(
            {
                "bullet_id": "shared_topology_families",
                "observation": (
                    f"Multiple sweep families reference shared topology keys "
                    f"({', '.join(f'`{k}`' for k in shared_keys[:3])}) in linkage exploration"
                ),
                "caveat": "shared keys indicate fixture lineage overlap, not operational equivalence",
                "related_sweep_ids": synthesis.get("sweep_ids") or [],
                "topology_group": "cross_topology",
            }
        )

    if not bullets:
        bullets.append(
            {
                "bullet_id": "corpus_baseline",
                "observation": "Cross-sweep corpus shows localized replay concentration patterns across fixture families",
                "caveat": "synthesis is descriptive replay review guidance only",
                "related_sweep_ids": synthesis.get("sweep_ids") or [],
                "topology_group": "corpus",
            }
        )

    groupings = sorted({b.get("topology_group") for b in bullets if b.get("topology_group")})

    return {
        "artifact_type": "replay_cognition_rollup_v1",
        "schema_version": "replay_cognition_rollup_v1",
        "bullets": bullets,
        "groupings": groupings,
    }


def render_cognition_rollup_summary(rollup: dict[str, Any]) -> str:
    lines = [
        "# Cognition rollup summary",
        "",
        "Deterministic replay cognition summaries for large corpus review — explanatory only.",
        "",
    ]
    for b in rollup.get("bullets") or []:
        text = f"{b.get('observation')} — replay-local; {b.get('caveat')}"
        lines.append(f"- {text}")
    lines.append("")
    return "\n".join(lines)


def enrich_synthesis_with_cognition(synthesis: dict[str, Any]) -> dict[str, Any]:
    rollup = build_cognition_rollup(synthesis)
    for b in rollup.get("bullets") or []:
        issues = _lint_bullet(b.get("observation", ""), b.get("caveat", ""))
        if issues:
            raise ValueError(f"cognition bullet lint: {issues}")
    enriched = dict(synthesis)
    enriched["cognition_rollup"] = rollup
    return enriched


def write_cognition_rollup(synthesis: dict[str, Any]) -> dict[str, Any]:
    enriched = enrich_synthesis_with_cognition(synthesis)
    rollup = enriched["cognition_rollup"]
    md = render_cognition_rollup_summary(rollup)
    lint = _lint_markdown(md)
    if lint:
        raise SystemExit(f"cognition_rollup_summary lint: {lint}")

    sections = {b["bullet_id"]: f"{b['observation']} — replay-local; {b['caveat']}" for b in rollup["bullets"]}
    section_lint = lint_storytelling_sections(sections)
    if section_lint:
        raise SystemExit(f"storytelling lint: {section_lint}")

    fixture = _REPO / "fixtures/sa_r0/synthesis"
    pub = _REPO / "platform/sa-r0-viewer/public/demo/synthesis"
    for d in (fixture, pub):
        d.mkdir(parents=True, exist_ok=True)
        (d / "cross_sweep_synthesis_v1.json").write_text(
            json.dumps(enriched, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        (d / "cognition_rollup_summary.md").write_text(md, encoding="utf-8")
    return enriched


def main() -> None:
    synthesis = build_cross_sweep_synthesis()
    write_cognition_rollup(synthesis)
    print("cognition rollup OK")


if __name__ == "__main__":
    main()
