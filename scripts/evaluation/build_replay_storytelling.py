#!/usr/bin/env python3
"""Deterministic replay storytelling summaries for E1 presentation layer."""

from __future__ import annotations

from typing import Any


FORBIDDEN_SUBSTRINGS = (
    "deployment readiness",
    "validated effectiveness",
    "tactical superiority",
    "P(kill)",
)


def _lint_text(text: str) -> list[str]:
    issues = []
    lower = text.lower()
    for phrase in FORBIDDEN_SUBSTRINGS:
        if phrase.lower() in lower:
            issues.append(f"forbidden phrase: {phrase}")
    return issues


def build_storytelling_sections(
    bundle: dict[str, Any] | None = None,
    *,
    manifest: dict[str, Any] | None = None,
    baseline_topology_key: str | None = None,
) -> dict[str, str]:
    """Build deterministic storytelling section strings."""
    sections: dict[str, str] = {}

    if bundle:
        topology = str((bundle.get("comparison_hints") or {}).get("topology_key") or "unknown")
        tags = bundle.get("scenario", {}).get("topology_tags") or []
        duration = bundle.get("clock", {}).get("duration") or {}
        span = int(duration.get("end", 0)) - int(duration.get("start", 0))
        events = (bundle.get("narrative") or {}).get("events") or []
        windows = (bundle.get("narrative") or {}).get("windows") or []
        los = sum(
            1
            for s in bundle.get("los_segments") or []
            if str(s.get("status")) in ("terrain_blocked", "partially_occluded")
        )

        sections["what_changed"] = (
            f"This replay variant uses topology `{topology}` "
            f"({', '.join(tags[:3]) or 'no extra tags'}) "
            f"over a {span}-step log span — derived replay summary only."
        )
        sections["topology_pacing"] = (
            f"Ingress archetype `{bundle.get('scenario', {}).get('ingress_archetype', 'standard')}` "
            f"shapes replay pacing across {len(events)} narrative events."
        )
        if los >= 1:
            sections["los_instability"] = (
                f"LOS instability appears in {los} degraded segment(s) in this replay variant — "
                "review near terrain masking overlays; not sensor physics proof."
            )
        if windows:
            sections["ambiguity_concentration"] = (
                f"Ambiguity concentrates in {len(windows)} replay window(s) during ingress overlap — "
                "explanatory spatial concentration only."
            )

    if manifest:
        agg = manifest.get("replay_aggregation") or {}
        hist = agg.get("outcome_histogram") or {}
        members = manifest.get("members") or []
        baseline = baseline_topology_key or str(manifest.get("baseline_topology_key") or "")

        fd = hist.get("first_detection_t") or []
        if fd and len(fd) >= 2 and max(fd) - min(fd) >= 2:
            sections["divergence_summary"] = (
                f"Detection timing differs by {max(fd) - min(fd)} log-line indices "
                f"across {len(members)} replay variants under `{baseline}`."
            )
        elif agg.get("dominant_patterns"):
            sections["divergence_summary"] = str(agg["dominant_patterns"][0])

        amb_layer = (manifest.get("spatial_aggregate") or {}).get("layers", {}).get(
            "ambiguity_density", {}
        )
        counts = amb_layer.get("counts") or []
        if counts and max(counts) >= 3:
            sections["ambiguity_concentration"] = (
                "Ambiguity density concentrates in shared spatial cells across sweep members — "
                "see ambiguity overlay in presentation mode."
            )

        los_layer = (manifest.get("spatial_aggregate") or {}).get("layers", {}).get(
            "los_degraded", {}
        )
        los_counts = los_layer.get("counts") or []
        if los_counts and max(los_counts) >= 2:
            sections["los_instability"] = (
                "LOS degradation replay concentration differs across topology variants in this sweep."
            )

        if not sections.get("what_changed"):
            non_baseline = sum(1 for m in members if str(m.get("pack_id")) != baseline)
            sections["what_changed"] = (
                f"{non_baseline} non-baseline pack(s) diverge from `{baseline}` in this sweep family."
            )

    for key, text in list(sections.items()):
        if _lint_text(text):
            sections[key] = text.replace("deployment readiness", "readiness framing")

    return sections


def lint_storytelling_sections(sections: dict[str, str]) -> list[str]:
    issues: list[str] = []
    for key, text in sections.items():
        for issue in _lint_text(text):
            issues.append(f"{key}: {issue}")
    return issues
