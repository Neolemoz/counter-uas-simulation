#!/usr/bin/env python3
"""Deterministic replay presentation chapters (replay_presentation_v1)."""

from __future__ import annotations

from typing import Any


def _topology_highlight(bundle: dict[str, Any]) -> str:
    key = str((bundle.get("comparison_hints") or {}).get("topology_key") or "")
    tags = bundle.get("scenario", {}).get("topology_tags") or []
    combined = f"{key} {' '.join(tags)}".lower()
    if "valley" in combined:
        return "valley"
    if "corridor" in combined:
        return "corridor"
    if "ridge" in combined or "multi_ridge" in combined:
        return "ridge"
    if "saturation" in combined or "assignment" in combined:
        return "assignment"
    return "none"


def _los_degraded_count(bundle: dict[str, Any]) -> int:
    return sum(
        1
        for s in bundle.get("los_segments") or []
        if str(s.get("status")) in ("terrain_blocked", "partially_occluded")
    )


def _events_by_category(bundle: dict[str, Any]) -> dict[str, list[dict[str, Any]]]:
    out: dict[str, list[dict[str, Any]]] = {}
    for ev in (bundle.get("narrative") or {}).get("events") or []:
        cat = str(ev.get("category") or "other")
        out.setdefault(cat, []).append(ev)
    return out


def _chapter_id(slug: str) -> str:
    return slug.replace(" ", "_").lower()


def build_bundle_presentation(bundle: dict[str, Any]) -> dict[str, Any]:
    """Build presentation.chapters from bundle narrative and clock."""
    clock = bundle.get("clock") or {}
    dur = clock.get("duration") or {}
    t_start = int(dur.get("start") or 0)
    t_end = int(dur.get("end") or t_start)
    scenario_id = str((bundle.get("scenario") or {}).get("scenario_id") or "replay")
    walkthrough_id = f"{scenario_id}_walkthrough"

    markers = clock.get("markers") or []
    events = (bundle.get("narrative") or {}).get("events") or []
    annotations = (bundle.get("narrative") or {}).get("annotations") or []
    by_cat = _events_by_category(bundle)

    chapters: list[dict[str, Any]] = []

    detection = by_cat.get("detection") or []
    if detection or markers:
        first_t = detection[0].get("t") if detection else markers[0].get("t", t_start)
        first_t = int(first_t or t_start)
        ingress_end = min(t_end, first_t + max(3, (t_end - t_start) // 4))
        focus_ids = [
            str(ev.get("event_id"))
            for ev in detection[:2]
            if ev.get("event_id")
        ]
        chapters.append(
            {
                "chapter_id": "ingress",
                "title": "Ingress",
                "t_start": t_start,
                "t_end": ingress_end,
                "summary": (
                    "Review ingress geometry and initial threat track appearance in this replay variant — "
                    "explanatory orientation only."
                ),
                "focus_event_ids": focus_ids or None,
                "topology_highlight": _topology_highlight(bundle),
                "narrative_emphasis": "topology",
                "spatial_declutter": "top_k",
            }
        )

    los_count = _los_degraded_count(bundle)
    los_events = by_cat.get("los") or []
    if los_count >= 1 or los_events:
        los_markers = [m for m in markers if m.get("category") in ("los", "window", "ambiguity")]
        los_t = int(los_markers[0].get("t", t_start + (t_end - t_start) // 3)) if los_markers else t_start + 2
        chapters.append(
            {
                "chapter_id": "los_degradation",
                "title": "LOS degradation",
                "t_start": max(t_start, los_t - 2),
                "t_end": min(t_end, los_t + max(4, (t_end - t_start) // 3)),
                "summary": (
                    "LOS instability concentrates near terrain masking in this replay — "
                    "review degraded segments as explanatory evidence only."
                ),
                "topology_highlight": _topology_highlight(bundle),
                "narrative_emphasis": "los",
                "los_scope": "selected_track",
                "spatial_declutter": "top_k",
            }
        )

    amb_events = by_cat.get("ambiguity") or []
    windows = (bundle.get("narrative") or {}).get("windows") or []
    if amb_events or len(windows) >= 1:
        amb_t = int(amb_events[0].get("t", t_start + (t_end - t_start) // 2)) if amb_events else t_start + 3
        chapters.append(
            {
                "chapter_id": "ambiguity",
                "title": "Ambiguity concentration",
                "t_start": max(t_start, amb_t - 1),
                "t_end": min(t_end, amb_t + max(5, (t_end - t_start) // 3)),
                "summary": (
                    "Ambiguity windows cluster in overlapping ingress phases — "
                    "not evidence of validated operational performance."
                ),
                "narrative_emphasis": "ambiguity",
                "spatial_declutter": "threshold",
            }
        )

    sel_events = by_cat.get("selection") or []
    div_events = by_cat.get("divergence") or []
    if sel_events or div_events:
        focus_t = int(
            (div_events[0] if div_events else sel_events[0]).get("t", t_start + (t_end - t_start) * 2 // 3)
        )
        focus_ids = [
            str(ev.get("event_id"))
            for ev in (div_events + sel_events)[:3]
            if ev.get("event_id")
        ]
        ann_ids = [
            str(a.get("annotation_id"))
            for a in annotations[:5]
            if a.get("annotation_id") and str(a.get("category", "")) in ("selection", "divergence", "assignment")
        ][:3]
        chapters.append(
            {
                "chapter_id": "assignment_instability",
                "title": "Assignment instability",
                "t_start": max(t_start, focus_t - 2),
                "t_end": min(t_end, focus_t + 4),
                "summary": (
                    "Selection and divergence events appear in replay-local sequence — "
                    "localized mismatch language only; not causal proof."
                ),
                "focus_event_ids": focus_ids or None,
                "spotlight_annotation_ids": ann_ids or None,
                "narrative_emphasis": "assignment",
            }
        )

    intercept = by_cat.get("intercept") or by_cat.get("outcome") or []
    summary_end = t_end
    chapters.append(
        {
            "chapter_id": "replay_outcome",
            "title": "Replay outcome summary",
            "t_start": max(t_start, summary_end - max(6, (t_end - t_start) // 4)),
            "t_end": t_end,
            "summary": (
                "Review replay outcome markers and final track states — "
                "derived summary only; not deployment readiness."
            ),
            "focus_event_ids": [
                str(ev.get("event_id")) for ev in intercept[:2] if ev.get("event_id")
            ]
            or None,
            "narrative_emphasis": "pacing",
        }
    )

    if not chapters:
        chapters.append(
            {
                "chapter_id": "full_replay",
                "title": "Full replay",
                "t_start": t_start,
                "t_end": t_end,
                "summary": "Full replay span for explanatory review.",
                "topology_highlight": _topology_highlight(bundle),
            }
        )

    cleaned: list[dict[str, Any]] = []
    for ch in chapters:
        row = {k: v for k, v in ch.items() if v is not None}
        cleaned.append(row)

    return {"walkthrough_id": walkthrough_id, "chapters": cleaned}


def build_sweep_presentation_walkthrough(
    manifest: dict[str, Any],
    bundles: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Build presentation_walkthrough for a sweep manifest."""
    sweep_id = str(manifest.get("sweep_id") or "sweep")
    walkthrough_id = f"{sweep_id}_walkthrough"
    narrative = manifest.get("replay_narrative_summary") or {}
    headline = narrative.get("headline") or (
        f"Guided walkthrough for replay variants in `{sweep_id}` — explanatory only."
    )

    steps: list[dict[str, Any]] = []
    steps.append(
        {
            "step_id": "overview",
            "label": "Sweep overview",
            "kind": "analytics_panel",
            "copy": str(narrative.get("bullets", ["Review sweep narrative summary."])[0]),
        }
    )

    cohorts = manifest.get("replay_cohorts") or []
    for cohort in cohorts[:3]:
        idxs = cohort.get("member_indices") or []
        if len(idxs) >= 2:
            steps.append(
                {
                    "step_id": f"cohort_{cohort.get('cohort_id', 'group')}",
                    "label": str(cohort.get("label") or "Cohort review"),
                    "kind": "cohort_filmstrip",
                    "cohort_id": str(cohort.get("cohort_id")),
                    "filmstrip_indices": idxs[:4],
                    "copy": str(
                        cohort.get("dominant_summary")
                        or "Compare cohort members in sync-clock filmstrip mode."
                    ),
                }
            )

    members = manifest.get("members") or []
    baseline_idx = 0
    for i, m in enumerate(members):
        if str(m.get("pack_id")) == str(manifest.get("baseline_topology_key")):
            baseline_idx = i
            break

    if len(members) >= 2 and baseline_idx < len(members):
        other = 1 if baseline_idx == 0 else 0
        steps.append(
            {
                "step_id": "baseline_compare",
                "label": "Baseline comparison",
                "kind": "compare_pair",
                "member_index": other,
                "copy": (
                    "Compare baseline and variant replay outcomes side-by-side — "
                    "topology divergence is explanatory only."
                ),
            }
        )

    if bundles:
        for i, bundle in enumerate(bundles[:3]):
            pres = build_bundle_presentation(bundle)
            ch0 = pres.get("chapters") or []
            if ch0:
                steps.append(
                    {
                        "step_id": f"member_{i}_chapter",
                        "label": f"Member {i + 1}: {ch0[0].get('title', 'Chapter')}",
                        "kind": "chapter",
                        "member_index": i,
                        "chapter_index": 0,
                        "copy": str(ch0[0].get("summary") or "Review member chapter."),
                    }
                )

    if len(steps) == 1:
        steps.append(
            {
                "step_id": "spatial_review",
                "label": "Spatial analytics",
                "kind": "analytics_panel",
                "copy": "Review spatial aggregate overlays for ambiguity and LOS concentration.",
            }
        )

    return {
        "walkthrough_id": walkthrough_id,
        "headline": headline,
        "steps": steps,
    }


def enrich_bundle_presentation(bundle: dict[str, Any]) -> dict[str, Any]:
    """Add optional presentation block to bundle."""
    out = dict(bundle)
    out["presentation"] = build_bundle_presentation(bundle)
    return out


def enrich_sweep_presentation(
    manifest: dict[str, Any],
    bundle_paths: list[Any] | None = None,
) -> dict[str, Any]:
    """Add presentation_walkthrough to sweep manifest."""
    from pathlib import Path

    from aggregate_spatial_analytics import load_bundle

    bundles = None
    if bundle_paths:
        bundles = [load_bundle(Path(p)) for p in bundle_paths]
    out = dict(manifest)
    out["presentation_walkthrough"] = build_sweep_presentation_walkthrough(manifest, bundles)
    return out
