#!/usr/bin/env python3
"""Deterministic sweep-level replay narrative intelligence (replay_narrative_intelligence_v1)."""

from __future__ import annotations

import statistics
from typing import Any

from aggregate_spatial_analytics import _first_detection_t, load_bundle
from classify_replay_pattern import (
    PATTERN_LABELS,
    classify_member_patterns,
    pattern_summary,
    primary_pattern,
)


def _member_metrics(bundle: dict[str, Any]) -> dict[str, Any]:
    los = sum(
        1
        for s in bundle.get("los_segments") or []
        if str(s.get("status")) in ("terrain_blocked", "partially_occluded")
    )
    amb = len((bundle.get("narrative") or {}).get("windows") or [])
    sel = sum(
        1
        for ev in (bundle.get("narrative") or {}).get("events") or []
        if str(ev.get("category")) == "selection"
    )
    div = any(
        str(ev.get("category")) == "divergence"
        for ev in (bundle.get("narrative") or {}).get("events") or []
    )
    clock = bundle.get("clock") or {}
    dur = clock.get("duration") or {}
    span = int(dur.get("end") or 0) - int(dur.get("start") or 0)
    return {
        "first_detection_t": _first_detection_t(bundle),
        "los_degraded_count": los,
        "ambiguity_window_count": amb,
        "selection_count": sel,
        "has_divergence": div,
        "duration_span": span,
    }


def _outlier_indices(values: list[int | None], indices: list[int]) -> list[int]:
    nums = [v for v in values if v is not None]
    if len(nums) < 3:
        return []
    try:
        q1, _, q3 = statistics.quantiles(nums, n=4)
        iqr = q3 - q1
        lo, hi = q1 - 1.5 * iqr, q3 + 1.5 * iqr
    except statistics.StatisticsError:
        return []
    out: list[int] = []
    for i, v in zip(indices, values):
        if v is None:
            continue
        if v < lo or v > hi:
            out.append(i)
    return out


def build_replay_cohorts(
    members: list[dict[str, Any]],
    bundles: list[dict[str, Any]],
    *,
    baseline_topology_key: str,
    member_tags: list[list[str]],
) -> list[dict[str, Any]]:
    """Group members by primary pattern tag."""
    by_primary: dict[str, list[int]] = {}
    for idx, tags in enumerate(member_tags):
        pid = primary_pattern(tags) or "ungrouped_replay"
        by_primary.setdefault(pid, []).append(idx)

    metrics = [_member_metrics(b) for b in bundles]
    fd_vals = [m["first_detection_t"] for m in metrics]
    los_vals = [m["los_degraded_count"] for m in metrics]
    fd_anomalies = set(_outlier_indices(fd_vals, list(range(len(members)))))
    los_anomalies = set(_outlier_indices(los_vals, list(range(len(members)))))

    cohorts: list[dict[str, Any]] = []
    for pid, idxs in sorted(by_primary.items()):
        label = PATTERN_LABELS.get(pid, "Replay variant cohort")
        if len(idxs) == 1:
            label = f"{label} (single member)"
        anomalies = sorted(fd_anomalies | los_anomalies & set(idxs))
        cohorts.append(
            {
                "cohort_id": f"{pid}_cohort",
                "label": label,
                "pattern_tags": [pid] if pid != "ungrouped_replay" else [],
                "member_indices": idxs,
                "dominant_summary": (
                    f"{len(idxs)} replay variant(s) share {label.lower()} in this sweep family."
                ),
                "anomaly_member_indices": anomalies,
            }
        )
    return cohorts


def build_sweep_narrative_summary(
    manifest: dict[str, Any],
    bundles: list[dict[str, Any]],
    *,
    member_tags: list[list[str]] | None = None,
) -> dict[str, Any]:
    """Build replay_narrative_summary from sweep manifest + member bundles."""
    baseline = str(manifest.get("baseline_topology_key") or "")
    members = manifest.get("members") or []
    metrics = [_member_metrics(b) for b in bundles]
    fds = [m["first_detection_t"] for m in metrics if m["first_detection_t"] is not None]
    los = [m["los_degraded_count"] for m in metrics]
    ambs = [m["ambiguity_window_count"] for m in metrics]
    spans = [m["duration_span"] for m in metrics]

    bullets: list[str] = []
    weights: dict[str, float] = {
        "ambiguity": 0.5,
        "los": 0.5,
        "topology": 0.5,
        "assignment": 0.5,
        "pacing": 0.5,
    }

    agg = manifest.get("replay_aggregation") or {}
    bullets.extend(agg.get("dominant_patterns") or [])

    if fds and max(fds) - min(fds) >= 3:
        bullets.append(
            "Detection timing variance clusters across replay variants — "
            "see first_detection markers in member timelines."
        )
        weights["pacing"] = 0.85

    if los and max(los) - min(los) >= 2:
        bullets.append(
            "Most sweep members show LOS degradation concentration near ridge or masking overlays."
        )
        weights["los"] = 0.9

    if ambs and max(ambs) >= 2:
        bullets.append(
            "Ambiguity escalation appears in overlapping ingress windows across replay variants."
        )
        weights["ambiguity"] = 0.88

    if any(m["has_divergence"] or m["selection_count"] >= 2 for m in metrics):
        bullets.append(
            "Assignment instability increases during selection-heavy replay segments in this sweep."
        )
        weights["assignment"] = 0.82

    non_baseline = sum(
        1 for m in members if str(m.get("pack_id")) != baseline
    )
    if non_baseline >= 2:
        bullets.append(
            "Topology-sensitive replay divergence appears across non-baseline scenario packs."
        )
        weights["topology"] = 0.87

    if spans and max(spans) - min(spans) >= 5:
        bullets.append(
            "Replay pacing differs across members — compare clock duration spans in filmstrip mode."
        )
        weights["pacing"] = max(weights["pacing"], 0.75)

    clusters = (manifest.get("spatial_aggregate") or {}).get("layers", {}).get(
        "replay_event_clusters", {}
    )
    if clusters.get("labels"):
        bullets.append(str(clusters["labels"][0]))

    seen: set[str] = set()
    unique_bullets: list[str] = []
    for b in bullets:
        if b not in seen:
            unique_bullets.append(b)
            seen.add(b)

    if not unique_bullets:
        unique_bullets.append(
            f"Replay variants under {baseline} form an explanatory sweep family for structured review."
        )

    headline = (
        f"Across {len(members)} replay variants in `{manifest.get('sweep_id')}`, "
        "deterministic sweep summaries highlight spatial concentration and timing divergence — "
        "explanatory replay review only."
    )

    return {
        "headline": headline,
        "bullets": unique_bullets[:8],
        "importance_weights": weights,
    }


def enrich_sweep_manifest(
    manifest: dict[str, Any],
    bundle_paths: list[Any],
) -> dict[str, Any]:
    """Add narrative summary, cohorts, and per-member pattern tags."""
    from pathlib import Path

    bundles = [load_bundle(Path(p)) for p in bundle_paths]
    members = manifest.get("members") or []
    baseline = str(manifest.get("baseline_topology_key") or "")

    fds = [_first_detection_t(b) for b in bundles]
    valid_fds = [f for f in fds if f is not None]
    median_fd = float(statistics.median(valid_fds)) if valid_fds else None

    sens = (manifest.get("spatial_aggregate") or {}).get("layers", {}).get(
        "topology_sensitivity", {}
    )
    has_sensitivity = bool(sens.get("counts") and max(sens["counts"]) > 0)

    member_tags: list[list[str]] = []
    enriched_members: list[dict[str, Any]] = []
    for m, b in zip(members, bundles):
        tags = classify_member_patterns(
            b,
            member=m,
            baseline_topology_key=baseline,
            sweep_median_fd=median_fd,
            has_sensitivity=has_sensitivity,
        )
        member_tags.append(tags)
        em = dict(m)
        em["replay_pattern_tags"] = tags
        em["replay_pattern_summary"] = pattern_summary(tags)
        enriched_members.append(em)

    manifest = dict(manifest)
    manifest["members"] = enriched_members
    manifest["replay_narrative_summary"] = build_sweep_narrative_summary(
        manifest, bundles, member_tags=member_tags
    )
    manifest["replay_cohorts"] = build_replay_cohorts(
        enriched_members,
        bundles,
        baseline_topology_key=baseline,
        member_tags=member_tags,
    )
    try:
        from build_replay_presentation import enrich_sweep_presentation  # noqa: WPS433

        manifest = enrich_sweep_presentation(manifest, bundle_paths)
    except Exception:
        pass
    return manifest
