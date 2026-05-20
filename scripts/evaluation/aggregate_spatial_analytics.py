#!/usr/bin/env python3
"""Deterministic spatial analytics aggregation from replay_sa_bundle_v1 dicts."""

from __future__ import annotations

import json
import math
import statistics
from pathlib import Path
from typing import Any

SPATIAL_CAVEAT = (
    "Replay-side spatial concentration for reviewer exploration only — "
    "not operational prediction, validated probability, or deployment confidence."
)

DEFAULT_GRID = {
    "origin_enu_m": [-2500.0, -500.0],
    "spacing_m": 100.0,
    "size": [40, 30],
}


def _grid_dims(grid: dict[str, Any]) -> tuple[int, int, float, float, float]:
    cols, rows = int(grid["size"][0]), int(grid["size"][1])
    ox, oy = float(grid["origin_enu_m"][0]), float(grid["origin_enu_m"][1])
    spacing = float(grid["spacing_m"])
    return cols, rows, ox, oy, spacing


def _cell_index(grid: dict[str, Any], x: float, y: float) -> int | None:
    cols, rows, ox, oy, spacing = _grid_dims(grid)
    col = int(math.floor((x - ox) / spacing))
    row = int(math.floor((y - oy) / spacing))
    if col < 0 or row < 0 or col >= cols or row >= rows:
        return None
    return row * cols + col


def _empty_counts(grid: dict[str, Any]) -> list[int]:
    cols, rows, _, _, _ = _grid_dims(grid)
    return [0] * (cols * rows)


def _midpoint_enu(polyline: list[Any]) -> tuple[float, float] | None:
    if not polyline:
        return None
    mid = polyline[len(polyline) // 2]
    if len(mid) < 2:
        return None
    return float(mid[0]), float(mid[1])


def _track_sample_at_t(bundle: dict[str, Any], track_id: str, t: int) -> tuple[float, float] | None:
    for tr in bundle.get("tracks") or []:
        if str(tr.get("track_id")) != track_id:
            continue
        best = None
        best_dt = 10**9
        for s in tr.get("samples") or []:
            st = int(s.get("t") or 0)
            dt = abs(st - t)
            if dt < best_dt:
                best_dt = dt
                best = s
        if best:
            return float(best.get("x_m") or 0), float(best.get("y_m") or 0)
    return None


def _primary_threat_id(bundle: dict[str, Any]) -> str:
    for tr in bundle.get("tracks") or []:
        if str(tr.get("role")) == "threat":
            return str(tr.get("track_id"))
    tracks = bundle.get("tracks") or []
    if tracks:
        return str(tracks[0].get("track_id"))
    return "threat_uav_0"


def _first_detection_t(bundle: dict[str, Any]) -> int | None:
    for m in (bundle.get("clock") or {}).get("markers") or []:
        if str(m.get("category")) == "detection":
            return int(m.get("t") or 0)
    for ev in (bundle.get("narrative") or {}).get("events") or []:
        if str(ev.get("category")) == "detection":
            return int(ev.get("line_index") or 0)
    return None


def _outcome_label(bundle: dict[str, Any]) -> str:
    glance = (bundle.get("comprehension") or {}).get("at_a_glance") or {}
    cards = glance.get("cards") if isinstance(glance, dict) else []
    if isinstance(cards, list):
        for c in cards:
            if not isinstance(c, dict):
                continue
            lab = str(c.get("label") or "").lower()
            if "outcome" in lab or "intercept" in lab or "result" in lab:
                return str(c.get("value") or "ambiguous")
    summary = glance.get("summary") if isinstance(glance, dict) else {}
    if isinstance(summary, dict):
        for key in ("outcome", "intercept_outcome", "replay_outcome"):
            if summary.get(key):
                return str(summary[key])
    return "ambiguous"


def accumulate_member_layers(bundle: dict[str, Any], grid: dict[str, Any]) -> dict[str, Any]:
    """Per-member layer contributions before sweep merge."""
    cols, rows, _, _, _ = _grid_dims(grid)
    ambiguity = _empty_counts(grid)
    los_deg = _empty_counts(grid)
    first_det_points: list[list[float]] = []
    intercept = _empty_counts(grid)
    event_points: list[tuple[float, float]] = []

    threat_id = _primary_threat_id(bundle)
    fd_t = _first_detection_t(bundle)
    if fd_t is not None:
        pt = _track_sample_at_t(bundle, threat_id, fd_t)
        if pt:
            first_det_points.append([pt[0], pt[1]])
            idx = _cell_index(grid, pt[0], pt[1])
            if idx is not None:
                ambiguity[idx] = ambiguity[idx]  # no-op; use first_det layer

    for win in (bundle.get("narrative") or {}).get("windows") or []:
        start = int(win.get("start_line_index") or win.get("start_t") or 0)
        end = int(win.get("end_line_index") or win.get("end_t") or start)
        mid_t = (start + end) // 2
        pt = _track_sample_at_t(bundle, threat_id, mid_t)
        if pt:
            idx = _cell_index(grid, pt[0], pt[1])
            if idx is not None:
                ambiguity[idx] += 1

    for ev in (bundle.get("narrative") or {}).get("events") or []:
        if str(ev.get("category")) == "ambiguity":
            t = int(ev.get("line_index") or 0)
            pt = _track_sample_at_t(bundle, threat_id, t)
            if pt:
                idx = _cell_index(grid, pt[0], pt[1])
                if idx is not None:
                    ambiguity[idx] += 1
        t = int(ev.get("line_index") or 0)
        pt = _track_sample_at_t(bundle, threat_id, t)
        if pt:
            event_points.append(pt)

    for seg in bundle.get("los_segments") or []:
        st = str(seg.get("status") or "")
        if st not in ("terrain_blocked", "partially_occluded"):
            continue
        mid = _midpoint_enu(seg.get("polyline_enu_m") or [])
        if mid:
            idx = _cell_index(grid, mid[0], mid[1])
            if idx is not None:
                los_deg[idx] += 1

    outcome = _outcome_label(bundle)
    for tr in bundle.get("tracks") or []:
        if str(tr.get("role")) not in ("interceptor", "friendly"):
            continue
        samples = tr.get("samples") or []
        if not samples:
            continue
        last = samples[-1]
        x, y = float(last.get("x_m") or 0), float(last.get("y_m") or 0)
        idx = _cell_index(grid, x, y)
        if idx is not None and outcome.lower() not in ("n/a", ""):
            intercept[idx] += 1

    return {
        "ambiguity_density": ambiguity,
        "los_degraded": los_deg,
        "first_detection": first_det_points,
        "intercept_outcome": intercept,
        "event_points": event_points,
        "first_detection_t": fd_t,
        "outcome_label": outcome,
    }


def merge_member_layers(
    grid: dict[str, Any],
    member_layers: list[dict[str, Any]],
    *,
    baseline_index: int = 0,
) -> dict[str, Any]:
    cols, rows, _, _, _ = _grid_dims(grid)
    ambiguity = _empty_counts(grid)
    los_deg = _empty_counts(grid)
    sensitivity = _empty_counts(grid)
    intercept = _empty_counts(grid)
    all_first: list[list[float]] = []
    all_events: list[tuple[float, float]] = []

    baseline_fd = member_layers[baseline_index].get("first_detection_t") if member_layers else None

    for ml in member_layers:
        for i, v in enumerate(ml.get("ambiguity_density") or []):
            ambiguity[i] += int(v)
        for i, v in enumerate(ml.get("los_degraded") or []):
            los_deg[i] += int(v)
        for i, v in enumerate(ml.get("intercept_outcome") or []):
            intercept[i] += int(v)
        all_first.extend(ml.get("first_detection") or [])
        all_events.extend(ml.get("event_points") or [])

        fd = ml.get("first_detection_t")
        if baseline_fd is not None and fd is not None and abs(int(fd) - int(baseline_fd)) >= 2:
            for pt in ml.get("first_detection") or []:
                idx = _cell_index(grid, float(pt[0]), float(pt[1]))
                if idx is not None:
                    sensitivity[idx] += 1

    nonzero = [c for c in ambiguity if c > 0]
    threshold = 0
    if nonzero:
        try:
            threshold = int(statistics.quantiles(nonzero, n=4)[2])
        except statistics.StatisticsError:
            threshold = max(nonzero)

    clusters: list[dict[str, Any]] = []
    for idx, count in enumerate(ambiguity):
        if count < threshold or count == 0:
            continue
        row, col = divmod(idx, cols)
        ox, oy = float(grid["origin_enu_m"][0]), float(grid["origin_enu_m"][1])
        spacing = float(grid["spacing_m"])
        cx = ox + (col + 0.5) * spacing
        cy = oy + (row + 0.5) * spacing
        clusters.append(
            {
                "centroid_enu_m": [round(cx, 1), round(cy, 1)],
                "count": count,
                "label": f"Replay event concentration (cell count {count})",
            }
        )

    return {
        "grid": grid,
        "layers": {
            "ambiguity_density": {"counts": ambiguity, "caveat": SPATIAL_CAVEAT},
            "los_degraded": {"counts": los_deg, "caveat": SPATIAL_CAVEAT},
            "first_detection": {"points_enu_m": all_first, "caveat": SPATIAL_CAVEAT},
            "intercept_outcome": {"counts": intercept, "caveat": SPATIAL_CAVEAT},
            "topology_sensitivity": {"counts": sensitivity, "caveat": SPATIAL_CAVEAT},
            "replay_event_clusters": {
                "centroids_enu_m": [c["centroid_enu_m"] for c in clusters[:8]],
                "labels": [c["label"] for c in clusters[:8]],
                "caveat": SPATIAL_CAVEAT,
            },
        },
    }


def build_replay_aggregation(
    bundles: list[dict[str, Any]],
    *,
    baseline_topology_key: str,
) -> dict[str, Any]:
    first_ts: list[int] = []
    los_degraded: list[int] = []
    ambiguity_windows: list[int] = []

    for b in bundles:
        fd = _first_detection_t(b)
        if fd is not None:
            first_ts.append(fd)
        los_degraded.append(
            sum(
                1
                for s in b.get("los_segments") or []
                if str(s.get("status")) in ("terrain_blocked", "partially_occluded")
            )
        )
        ambiguity_windows.append(len((b.get("narrative") or {}).get("windows") or []))

    patterns: list[str] = []
    if first_ts and max(first_ts) - min(first_ts) >= 3:
        patterns.append(
            "Detection timing variability increases across replay variants in this sweep family."
        )
    if los_degraded and max(los_degraded) > min(los_degraded) + 2:
        patterns.append(
            "Most replay variants show LOS degradation concentration near ridge or masking overlays."
        )
    if not patterns:
        patterns.append(
            f"Replay variants under baseline {baseline_topology_key} show localized spatial concentration — explanatory only."
        )

    return {
        "outcome_histogram": {
            "first_detection_t": first_ts,
            "los_degraded_count": los_degraded,
            "ambiguity_window_count": ambiguity_windows,
        },
        "dominant_patterns": patterns,
    }


def aggregate_from_bundles(
    bundles: list[dict[str, Any]],
    *,
    grid: dict[str, Any] | None = None,
    baseline_index: int = 0,
    baseline_topology_key: str = "",
) -> dict[str, Any]:
    g = dict(grid or DEFAULT_GRID)
    member_layers = [accumulate_member_layers(b, g) for b in bundles]
    spatial = merge_member_layers(g, member_layers, baseline_index=baseline_index)
    replay_agg = build_replay_aggregation(
        bundles, baseline_topology_key=baseline_topology_key or "unknown"
    )
    return {"spatial_aggregate": spatial, "replay_aggregation": replay_agg}


def load_bundle(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("artifact_type") != "replay_sa_bundle":
        raise ValueError(f"expected replay_sa_bundle: {path}")
    return data
