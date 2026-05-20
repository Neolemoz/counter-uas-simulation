"""Scenario geometry helpers for replay SA bundles (evaluation-side only)."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any, Iterable, Sequence

_LAUNCH_ARG_RE = re.compile(r"(\w+):=(-?\d+(?:\.\d+)?)")

DEFAULT_DOME_RADII_M = {
    "outer_detection": 720.0,
    "tracking": 480.0,
    "engagement": 240.0,
    "protected": 120.0,
}

ZONE_DISPLAY_LABELS = {
    "outer_detection": "Outer Detection",
    "tracking": "Tracking",
    "engagement": "Engagement",
    "protected": "Critical / Protected Area",
}

LOS_SEGMENT_CAVEAT = (
    "Replay-local LOS association from fixture topology — not sensor truth or causal proof."
)

SENSOR_KINDS = frozenset({"radar", "eoir"})
OVERLAY_KIND_BLOCKED = "los_blocked"
OVERLAY_KIND_RIDGE = "ridge_mask"
OVERLAY_KIND_DEGRADED = "degraded_visibility"
OVERLAY_KIND_INGRESS = "ingress_corridor"

MAX_LOS_SEGMENTS = 12


def parse_launch_args_from_meta(meta_path: Path | None) -> dict[str, float]:
    """Extract numeric launch overrides from meta cmd list."""
    if meta_path is None or not meta_path.is_file():
        return {}
    try:
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    cmd = meta.get("cmd")
    text = " ".join(str(c) for c in cmd) if isinstance(cmd, list) else str(cmd or "")
    return {k: float(v) for k, v in _LAUNCH_ARG_RE.findall(text)}


def build_dome_zones(
    center_enu_m: tuple[float, float, float] = (0.0, 0.0, 0.0),
    *,
    radii_m: dict[str, float] | None = None,
) -> list[dict[str, Any]]:
    """Build concentric threat/risk zone circles (explanatory overlays)."""
    radii = radii_m or DEFAULT_DOME_RADII_M
    kind_map = {
        "outer_detection": "threat_risk",
        "tracking": "threat_risk",
        "engagement": "threat_risk",
        "protected": "protected",
    }
    zones: list[dict[str, Any]] = []
    for zone_id, radius in radii.items():
        zones.append(
            {
                "zone_id": zone_id,
                "kind": kind_map.get(zone_id, "threat_risk"),
                "display_label": ZONE_DISPLAY_LABELS.get(zone_id, zone_id),
                "geometry": {
                    "type": "circle",
                    "center_enu_m": list(center_enu_m),
                    "radius_m": float(radius),
                },
                "caveat": "Scenario policy overlay — not validated doctrine.",
            }
        )
    return zones


def load_scenario_overlay(path: Path | None) -> dict[str, Any]:
    """Load optional fixture overlay (static sites, overlays, annotations)."""
    if path is None or not path.is_file():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return data if isinstance(data, dict) else {}


def load_scenario_topology(path: Path | None) -> dict[str, Any]:
    """Load scenario_topology_v1 pack (directory) or legacy scenario_overlay.json (file)."""
    if path is None:
        return {}
    resolved = path.resolve()
    if resolved.is_dir():
        from replay_sa_scenario import load_scenario_pack  # noqa: PLC0415

        return load_scenario_pack(resolved)
    if resolved.is_file():
        return load_scenario_overlay(resolved)
    return {}


def _point_in_polygon_2d(px: float, py: float, vertices: Sequence[Sequence[float]]) -> bool:
    """Ray-casting point-in-polygon (ENU x/y)."""
    if len(vertices) < 3:
        return False
    inside = False
    n = len(vertices)
    j = n - 1
    for i in range(n):
        xi, yi = float(vertices[i][0]), float(vertices[i][1])
        xj, yj = float(vertices[j][0]), float(vertices[j][1])
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def _overlay_vertices(overlay: dict[str, Any]) -> list[list[float]]:
    geom = overlay.get("geometry") or {}
    verts = geom.get("vertices_enu_m") or []
    return [list(v) for v in verts if isinstance(v, (list, tuple)) and len(v) >= 2]


def classify_point_visibility(
    point: tuple[float, float, float],
    overlays: Iterable[dict[str, Any]],
) -> str:
    """Classify a point against explanatory overlay regions."""
    px, py = point[0], point[1]
    in_blocked = False
    in_ridge = False
    in_degraded = False
    for ov in overlays:
        verts = _overlay_vertices(ov)
        if not verts:
            continue
        kind = str(ov.get("kind") or "")
        if not _point_in_polygon_2d(px, py, verts):
            continue
        if kind == OVERLAY_KIND_BLOCKED:
            in_blocked = True
        elif kind == OVERLAY_KIND_RIDGE:
            in_ridge = True
        elif kind in (OVERLAY_KIND_DEGRADED, OVERLAY_KIND_INGRESS):
            in_degraded = True
    if in_blocked:
        return "blocked"
    if in_ridge:
        return "occluded"
    if in_degraded:
        return "degraded"
    return "clear"


def sample_segment_enu(
    origin: tuple[float, float, float],
    target: tuple[float, float, float],
    n: int = 32,
) -> list[list[float]]:
    """Sample polyline from origin to target in ENU."""
    if n < 2:
        n = 2
    ox, oy, oz = origin
    tx, ty, tz = target
    pts: list[list[float]] = []
    for i in range(n):
        u = i / (n - 1)
        pts.append(
            [
                round(ox + (tx - ox) * u, 3),
                round(oy + (ty - oy) * u, 3),
                round(oz + (tz - oz) * u, 3),
            ]
        )
    return pts


def approximate_los_status(
    origin: tuple[float, float, float],
    target: tuple[float, float, float],
    overlays: list[dict[str, Any]],
) -> str:
    """Aggregate visibility along a segment (explanatory 2.5D proxy)."""
    polyline = sample_segment_enu(origin, target, n=24)
    classes = [classify_point_visibility((p[0], p[1], p[2]), overlays) for p in polyline]
    has_clear = any(c == "clear" for c in classes)
    has_bad = any(c in ("blocked", "occluded") for c in classes)
    has_degraded = any(c == "degraded" for c in classes)
    if has_clear and not has_bad and not has_degraded:
        return "visible"
    if has_clear and (has_bad or has_degraded):
        return "partially_occluded"
    if has_bad and not has_clear:
        return "terrain_blocked"
    if has_degraded and not has_clear and not has_bad:
        return "partially_occluded"
    return "visible"


def _interpolate_track_at_t(
    samples: list[dict[str, Any]],
    t: int,
) -> tuple[float, float, float] | None:
    at = [s for s in samples if isinstance(s.get("t"), int) and int(s["t"]) == t]
    if at:
        s = at[0]
        return (float(s["x_m"]), float(s["y_m"]), float(s.get("z_m") or 0.0))
    before = [s for s in samples if isinstance(s.get("t"), int) and int(s["t"]) < t]
    if not before:
        return None
    s = before[-1]
    return (float(s["x_m"]), float(s["y_m"]), float(s.get("z_m") or 0.0))


def _pick_keyframe_times(threat_samples: list[dict[str, Any]], max_frames: int = 4) -> list[int]:
    times = sorted({int(s["t"]) for s in threat_samples if isinstance(s.get("t"), int)})
    if not times:
        return []
    if len(times) <= max_frames:
        return times
    picks = [times[0], times[len(times) // 3], times[(2 * len(times)) // 3], times[-1]]
    out: list[int] = []
    for t in picks:
        if t not in out:
            out.append(t)
    return out[:max_frames]


def build_los_segments(
    entities_static: list[dict[str, Any]],
    tracks: list[dict[str, Any]],
    overlays: list[dict[str, Any]],
    *,
    max_segments: int = MAX_LOS_SEGMENTS,
) -> list[dict[str, Any]]:
    """Build explanatory LOS segments from sites/tracks to each threat at keyframe times."""
    threats = [t for t in tracks if t.get("role") == "threat"]
    if not threats:
        return []

    interceptors = [t for t in tracks if t.get("role") == "interceptor"]
    segments: list[dict[str, Any]] = []
    frames_per_threat = max(2, min(4, max_segments // max(len(threats) * 2, 1)))

    def _add_segment(
        segment_id: str,
        from_id: str,
        to_track_id: str,
        origin: tuple[float, float, float],
        target: tuple[float, float, float],
        t: int,
        from_kind: str = "site",
    ) -> None:
        if len(segments) >= max_segments:
            return
        status = approximate_los_status(origin, target, overlays)
        segments.append(
            {
                "segment_id": segment_id,
                "from_entity_id": from_id,
                "from_kind": from_kind,
                "to_track_id": to_track_id,
                "t": t,
                "status": status,
                "polyline_enu_m": sample_segment_enu(origin, target, n=16),
                "caveat": LOS_SEGMENT_CAVEAT,
                "linked_event_ids": [],
            }
        )

    for threat in threats:
        if len(segments) >= max_segments:
            break
        threat_samples = list(threat.get("samples") or [])
        key_times = _pick_keyframe_times(threat_samples, max_frames=frames_per_threat)
        if not key_times:
            continue
        threat_id = str(threat.get("track_id") or "threat_uav_0")

        for t in key_times:
            if len(segments) >= max_segments:
                break
            target = _interpolate_track_at_t(threat_samples, t)
            if target is None:
                continue
            for ent in entities_static:
                if len(segments) >= max_segments:
                    break
                kind = str(ent.get("kind") or "")
                if kind not in SENSOR_KINDS:
                    continue
                pos = ent.get("position_enu_m")
                if not isinstance(pos, (list, tuple)) or len(pos) < 3:
                    continue
                origin = (float(pos[0]), float(pos[1]), float(pos[2]))
                eid = str(ent.get("entity_id") or kind)
                _add_segment(
                    f"los_{eid}_{threat_id}_t{t}",
                    eid,
                    threat_id,
                    origin,
                    target,
                    t,
                    "site",
                )

            for interceptor in interceptors:
                if len(segments) >= max_segments:
                    break
                int_samples = list(interceptor.get("samples") or [])
                int_pos = _interpolate_track_at_t(int_samples, t)
                if int_pos:
                    iid = str(interceptor.get("track_id") or "interceptor_0")
                    _add_segment(
                        f"los_{iid}_{threat_id}_t{t}",
                        iid,
                        threat_id,
                        int_pos,
                        target,
                        t,
                        "track",
                    )

    return segments[:max_segments]


def build_fictional_heightmap(
    *,
    origin_enu_m: tuple[float, float] = (-2000.0, -2000.0),
    spacing_m: float = 200.0,
    size: int = 16,
    ridge_peak_m: float = 120.0,
) -> dict[str, Any]:
    """Small fictional height grid for replay terrain emphasis (not survey data)."""
    heights: list[list[float]] = []
    for row in range(size):
        row_heights: list[float] = []
        for col in range(size):
            cx = origin_enu_m[0] + col * spacing_m
            cy = origin_enu_m[1] + row * spacing_m
            # Ridge proxy along negative X corridor
            ridge = ridge_peak_m * max(0.0, 1.0 - abs(cx + 700.0) / 500.0) * max(0.0, 1.0 - abs(cy) / 400.0)
            row_heights.append(round(ridge, 1))
        heights.append(row_heights)
    return {
        "type": "fictional_heightmap",
        "grid_enu_m": {
            "origin": list(origin_enu_m),
            "spacing_m": spacing_m,
            "size": size,
            "heights_m": heights,
        },
        "caveat": "Exaggerated replay terrain — not deployed geography.",
    }


def sample_fictional_height(
    terrain_model: dict[str, Any] | None,
    x_m: float,
    y_m: float,
) -> float:
    """Bilinear sample of fictional heightmap at ENU x/y."""
    if not terrain_model or terrain_model.get("type") != "fictional_heightmap":
        return 0.0
    grid = terrain_model.get("grid_enu_m")
    if not isinstance(grid, dict):
        return 0.0
    origin = grid.get("origin") or [0.0, 0.0]
    spacing = float(grid.get("spacing_m") or 200.0)
    heights = grid.get("heights_m")
    if not isinstance(heights, list) or not heights:
        return 0.0
    size = int(grid.get("size") or len(heights))
    ox, oy = float(origin[0]), float(origin[1])
    fx = (x_m - ox) / spacing
    fy = (y_m - oy) / spacing
    if fx < 0 or fy < 0 or fx >= size - 1 or fy >= len(heights) - 1:
        return 0.0
    ix, iy = int(fx), int(fy)
    tx, ty = fx - ix, fy - iy
    h00 = float(heights[iy][ix])
    h10 = float(heights[iy][min(ix + 1, size - 1)])
    h01 = float(heights[min(iy + 1, len(heights) - 1)][ix])
    h11 = float(heights[min(iy + 1, len(heights) - 1)][min(ix + 1, size - 1)])
    return (1 - tx) * (1 - ty) * h00 + tx * (1 - ty) * h10 + (1 - tx) * ty * h01 + tx * ty * h11
