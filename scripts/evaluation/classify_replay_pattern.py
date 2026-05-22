#!/usr/bin/env python3
"""Deterministic replay-local pattern taxonomy (replay_pattern_taxonomy_v1)."""

from __future__ import annotations

from typing import Any

PATTERN_PRIORITY = (
    "los_fragmented_replay",
    "assignment_instability_replay",
    "delayed_detection_replay",
    "corridor_pressure_replay",
    "saturation_driven_ambiguity",
    "topology_sensitive_divergence",
)

PATTERN_LABELS: dict[str, str] = {
    "los_fragmented_replay": "LOS-fragmented replay concentration",
    "assignment_instability_replay": "Assignment instability across replay windows",
    "delayed_detection_replay": "Delayed detection timing in replay sequence",
    "corridor_pressure_replay": "Corridor-pressure replay archetype",
    "saturation_driven_ambiguity": "Saturation-driven ambiguity concentration",
    "topology_sensitive_divergence": "Topology-sensitive replay divergence",
}


def _first_detection_t(bundle: dict[str, Any]) -> int | None:
    for m in (bundle.get("clock") or {}).get("markers") or []:
        if str(m.get("category")) == "detection":
            return int(m.get("t") or 0)
    for ev in (bundle.get("narrative") or {}).get("events") or []:
        if str(ev.get("category")) == "detection":
            return int(ev.get("line_index") or 0)
    return None


def _los_degraded_count(bundle: dict[str, Any]) -> int:
    return sum(
        1
        for s in bundle.get("los_segments") or []
        if str(s.get("status")) in ("terrain_blocked", "partially_occluded")
    )


def _ambiguity_window_count(bundle: dict[str, Any]) -> int:
    return len((bundle.get("narrative") or {}).get("windows") or [])


def _selection_event_count(bundle: dict[str, Any]) -> int:
    return sum(
        1
        for ev in (bundle.get("narrative") or {}).get("events") or []
        if str(ev.get("category")) == "selection"
    )


def _has_divergence_event(bundle: dict[str, Any]) -> bool:
    return any(
        str(ev.get("category")) == "divergence"
        for ev in (bundle.get("narrative") or {}).get("events") or []
    )


def _pack_id(bundle: dict[str, Any], member: dict[str, Any] | None) -> str:
    hints = bundle.get("comparison_hints") or {}
    if hints.get("topology_key"):
        return str(hints["topology_key"])
    if member and member.get("pack_id"):
        return str(member["pack_id"])
    return str(bundle.get("scenario", {}).get("topology_key") or "")


def classify_member_patterns(
    bundle: dict[str, Any],
    *,
    member: dict[str, Any] | None = None,
    baseline_topology_key: str = "",
    sweep_median_fd: float | None = None,
    has_sensitivity: bool = False,
) -> list[str]:
    """Return pattern_id tags for one member bundle (stable priority order)."""
    tags: list[str] = []
    pack = _pack_id(bundle, member)
    fd = _first_detection_t(bundle)
    los = _los_degraded_count(bundle)
    amb = _ambiguity_window_count(bundle)
    sel = _selection_event_count(bundle)
    div = _has_divergence_event(bundle)

    if los >= 2:
        tags.append("los_fragmented_replay")

    if sel >= 2 or div:
        tags.append("assignment_instability_replay")

    if sweep_median_fd is not None and fd is not None and fd > sweep_median_fd + 2:
        tags.append("delayed_detection_replay")

    if "corridor" in pack.lower():
        tags.append("corridor_pressure_replay")

    if "saturation" in pack.lower() and amb >= 2:
        tags.append("saturation_driven_ambiguity")

    if baseline_topology_key and pack and pack != baseline_topology_key:
        tags.append("topology_sensitive_divergence")

    seen: set[str] = set()
    ordered: list[str] = []
    for pid in PATTERN_PRIORITY:
        if pid in tags and pid not in seen:
            ordered.append(pid)
            seen.add(pid)
    return ordered


def primary_pattern(tags: list[str]) -> str | None:
    for pid in PATTERN_PRIORITY:
        if pid in tags:
            return pid
    return None


def pattern_summary(tags: list[str]) -> str:
    if not tags:
        return "Replay-local pattern tags not assigned for this member."
    labels = [PATTERN_LABELS.get(t, t) for t in tags[:3]]
    return "; ".join(labels) + " — explanatory replay characterization only."
