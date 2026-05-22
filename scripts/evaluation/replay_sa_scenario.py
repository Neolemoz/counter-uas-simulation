#!/usr/bin/env python3
"""Load and validate scenario_topology_v1 packs for SA replay."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

SCENARIO_ARTIFACT_TYPE = "scenario_topology_v1"
SCENARIO_SCHEMA_VERSION = "scenario_topology_v1"

ENTITY_KINDS = frozenset({"radar", "eoir", "passive_rf", "interceptor_base", "fusion_center"})
OVERLAY_KINDS = frozenset({"ridge_mask", "los_blocked", "degraded_visibility", "ingress_corridor"})
ZONE_KINDS = frozenset({"protected", "threat_risk"})
PROHIBITED_PACK_KEYS = frozenset(
    {
        "command",
        "engage",
        "websocket_url",
        "ros_topic_live",
        "readiness_score",
        "authority_state",
        "live_mode",
        "los_segments",
    }
)

REQUIRED_PACK_FILES = ("metadata.json", "topology.json", "overlays.json", "annotations.json")

INGRESS_ARCHETYPES = frozenset({"open", "valley", "corridor", "ridge"})
AMBIGUITY_LEVELS = frozenset({"low", "moderate", "high", "saturation"})
REPLAY_DURATION_CLASSES = frozenset({"short", "medium", "long"})
TERRAIN_PROFILES = frozenset({"none", "flat", "ridged", "valley", "urban_clutter"})
NARRATIVE_RANKS = frozenset({1, 2, 3})

KNOWN_TOPOLOGY_TAGS = frozenset(
    {
        "ridge_defense",
        "protected_area",
        "terrain_aware",
        "valley_ingress",
        "terrain_masking",
        "delayed_track_confirmation",
        "reacquisition",
        "multi_ridge",
        "chained_masking",
        "intermittent_visibility",
        "corridor_defense",
        "narrow_corridor",
        "overlapping_radar",
        "timing_pressure",
        "saturation_ingress",
        "multi_threat",
        "assignment_ambiguity",
        "urban_masking",
        "fragmented_tracking",
        "cluttered_replay",
        "delayed_detection",
        "late_acquisition",
        "urgency",
        "long_range_ingress",
        "launch_sequence",
        "terrain_following",
        "sensor_experiment",
        "topology_experiment",
    }
)
KNOWN_REPLAY_TAGS = frozenset(
    {
        "terrain_masking",
        "protected_area",
        "reacquisition",
        "delayed_track_confirmation",
        "chained_masking",
        "intermittent_visibility",
        "multi_track",
        "narrow_corridor",
        "timing_pressure",
        "overlapping_radar",
        "multi_threat",
        "assignment_ambiguity",
        "compressed_intercept_window",
        "fragmented_tracking",
        "cluttered_replay",
        "urban_clutter",
        "late_acquisition",
        "urgency",
        "launch_sequence",
        "terrain_following",
        "long_range",
        "sensor_placement",
    }
)
KNOWN_NARRATIVE_FOCUS = frozenset(
    {
        "ridge_masking",
        "protected_area",
        "valley_masking",
        "reacquisition",
        "chained_masking",
        "corridor_ingress",
        "timing_pressure",
        "assignment_ambiguity",
        "prioritization_pressure",
        "urban_clutter",
        "fragmented_tracking",
        "late_acquisition",
        "urgency",
        "launch_sequence",
        "terrain_following",
        "sensor_placement",
    }
)

CATALOG_CATEGORIES = frozenset(
    {
        "terrain_masking",
        "ingress_geometry",
        "multi_threat",
        "detection_timing",
        "corridor_defense",
        "topology_experiment",
    }
)


def _read_json_object(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"expected JSON object: {path}")
    return data


def _check_prohibited(obj: dict[str, Any], prefix: str, issues: list[str], warnings: list[str]) -> None:
    for key in PROHIBITED_PACK_KEYS:
        if key in obj:
            issues.append(f"{prefix}: prohibited field {key!r}")


def load_scenario_pack(pack_dir: Path) -> dict[str, Any]:
    """Load split-file pack and return merged overlay dict for bundle builder."""
    pack_dir = pack_dir.resolve()
    if not pack_dir.is_dir():
        raise ValueError(f"scenario pack directory not found: {pack_dir}")
    return merge_scenario_pack_to_overlay(_load_pack_parts(pack_dir))


def _load_pack_parts(pack_dir: Path) -> dict[str, Any]:
    missing = [name for name in REQUIRED_PACK_FILES if not (pack_dir / name).is_file()]
    if missing:
        raise ValueError(f"scenario pack missing required files {missing}: {pack_dir}")

    metadata = _read_json_object(pack_dir / "metadata.json")
    topology = _read_json_object(pack_dir / "topology.json")
    overlays_file = _read_json_object(pack_dir / "overlays.json")
    annotations_file = _read_json_object(pack_dir / "annotations.json")

    terrain: dict[str, Any] = {}
    terrain_path = pack_dir / "terrain.json"
    if terrain_path.is_file():
        terrain = _read_json_object(terrain_path)

    return {
        "pack_dir": str(pack_dir),
        "metadata": metadata,
        "topology": topology,
        "overlays_file": overlays_file,
        "annotations_file": annotations_file,
        "terrain": terrain,
    }


def derive_overlay_descriptors(overlays: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Summarize overlay kinds for metadata/catalog (deterministic order)."""
    counts: dict[str, int] = {}
    for ov in overlays:
        kind = str(ov.get("kind") or "")
        if kind:
            counts[kind] = counts.get(kind, 0) + 1
    return [{"kind": kind, "count": counts[kind]} for kind in sorted(counts)]


def compute_sensor_layout_id(topology: dict[str, Any]) -> str:
    """Stable hash of static entity positions for comparison pairing."""
    entities = list(topology.get("entities_static") or [])
    rows: list[str] = []
    for ent in sorted(entities, key=lambda e: str(e.get("entity_id") or "")):
        eid = str(ent.get("entity_id") or "")
        kind = str(ent.get("kind") or "")
        pos = ent.get("position_enu_m") or [0, 0, 0]
        pos_s = ",".join(f"{float(p):.3f}" for p in pos[:3])
        rows.append(f"{eid}|{kind}|{pos_s}")
    payload = "\n".join(rows).encode("utf-8")
    digest = hashlib.sha256(payload).hexdigest()
    return f"sha256:{digest[:16]}"


def diff_topology_entities(
    entities_a: list[dict[str, Any]],
    entities_b: list[dict[str, Any]],
) -> dict[str, Any]:
    """Deterministic entity diff for tests and optional pack lint."""
    by_a = {str(e.get("entity_id")): e for e in entities_a if e.get("entity_id")}
    by_b = {str(e.get("entity_id")): e for e in entities_b if e.get("entity_id")}
    added = sorted(set(by_b) - set(by_a))
    removed = sorted(set(by_a) - set(by_b))
    moved: list[dict[str, Any]] = []
    for eid in sorted(set(by_a) & set(by_b)):
        pa = by_a[eid].get("position_enu_m") or [0, 0, 0]
        pb = by_b[eid].get("position_enu_m") or [0, 0, 0]
        dx = float(pb[0]) - float(pa[0])
        dy = float(pb[1]) - float(pa[1])
        dz = float((pb[2] if len(pb) > 2 else 0)) - float((pa[2] if len(pa) > 2 else 0))
        if abs(dx) > 1e-6 or abs(dy) > 1e-6 or abs(dz) > 1e-6:
            moved.append({"entity_id": eid, "delta_enu_m": [dx, dy, dz]})
    return {"added": added, "removed": removed, "moved": moved}


def catalog_category_for_metadata(metadata: dict[str, Any]) -> str:
    """Deterministic catalog grouping from topology tags and ingress archetype."""
    tags = set(metadata.get("topology_tags") or [])
    if "topology_experiment" in tags or "sensor_experiment" in tags:
        return "topology_experiment"
    if "saturation_ingress" in tags or "multi_threat" in tags:
        return "multi_threat"
    if "corridor_defense" in tags:
        return "corridor_defense"
    if "delayed_detection" in tags or "late_acquisition" in tags:
        return "detection_timing"
    if metadata.get("ingress_archetype") in {"valley", "ridge"} or tags & {
        "valley_ingress",
        "long_range_ingress",
        "ridge_defense",
        "multi_ridge",
    }:
        if tags & {"urban_masking", "fragmented_tracking"}:
            return "terrain_masking"
        if metadata.get("ingress_archetype") == "valley" or "valley_ingress" in tags:
            return "ingress_geometry"
        if "ridge_defense" in tags or "multi_ridge" in tags:
            return "terrain_masking"
        return "ingress_geometry"
    if tags & {"urban_masking", "terrain_masking", "fragmented_tracking"}:
        return "terrain_masking"
    return "ingress_geometry"


def replay_duration_class_from_span(span: int) -> str:
    if span < 120:
        return "short"
    if span <= 250:
        return "medium"
    return "long"


def merge_scenario_pack_to_overlay(parts: dict[str, Any]) -> dict[str, Any]:
    """Merge pack parts into legacy monolithic overlay shape."""
    metadata = parts["metadata"]
    topology = parts["topology"]
    terrain = parts.get("terrain") or {}
    overlays = list((parts.get("overlays_file") or {}).get("overlays") or [])

    merged: dict[str, Any] = {
        "scenario_id": metadata.get("scenario_id"),
        "title": metadata.get("title"),
        "topology_tags": list(metadata.get("topology_tags") or []),
        "georef_anchor": dict(topology.get("georef_anchor") or {}),
        "entities_static": list(topology.get("entities_static") or []),
        "zones": list(topology.get("zones") or []),
        "overlays": overlays,
        "annotations": list((parts.get("annotations_file") or {}).get("annotations") or []),
        "scenario_pack_id": metadata.get("scenario_id"),
        "_scenario_pack_metadata": metadata,
        "_pack_dir_name": Path(str(parts.get("pack_dir") or "")).name,
    }
    for key in (
        "replay_tags",
        "ingress_archetype",
        "overlay_descriptors",
        "ambiguity_profile",
        "replay_duration_class",
        "terrain_profile",
        "narrative_focus",
        "provenance",
    ):
        if metadata.get(key) is not None:
            merged[key] = metadata[key]
    if terrain.get("terrain_model") is not None:
        merged["terrain_model"] = terrain["terrain_model"]
    if terrain.get("include_fictional_terrain"):
        merged["include_fictional_terrain"] = True
    return merged


def lint_scenario_pack(
    pack_dir: Path,
    *,
    strict: bool = False,
) -> dict[str, Any]:
    """Validate pack structure; return {ok, issues, warnings}."""
    issues: list[str] = []
    warnings: list[str] = []

    pack_dir = pack_dir.resolve()
    if not pack_dir.is_dir():
        return {"ok": False, "issues": [f"not a directory: {pack_dir}"], "warnings": []}

    for name in REQUIRED_PACK_FILES:
        if not (pack_dir / name).is_file():
            issues.append(f"missing required file: {name}")

    if issues:
        return {"ok": False, "issues": issues, "warnings": warnings}

    try:
        parts = _load_pack_parts(pack_dir)
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        return {"ok": False, "issues": [str(exc)], "warnings": warnings}

    metadata = parts["metadata"]
    topology = parts["topology"]
    overlays_file = parts["overlays_file"]
    annotations_file = parts["annotations_file"]
    terrain = parts.get("terrain") or {}

    for blob, label in (
        (metadata, "metadata.json"),
        (topology, "topology.json"),
        (overlays_file, "overlays.json"),
        (annotations_file, "annotations.json"),
        (terrain, "terrain.json"),
    ):
        if blob:
            _check_prohibited(blob, label, issues, warnings)

    if metadata.get("artifact_type") != SCENARIO_ARTIFACT_TYPE:
        issues.append("metadata.artifact_type must be scenario_topology_v1")
    if metadata.get("schema_version") != SCENARIO_SCHEMA_VERSION:
        issues.append("metadata.schema_version must be scenario_topology_v1")
    if not metadata.get("scenario_id"):
        issues.append("metadata.scenario_id is required")
    if not metadata.get("title"):
        issues.append("metadata.title is required")
    if not isinstance(metadata.get("topology_tags"), list):
        issues.append("metadata.topology_tags must be a list")
    governance = metadata.get("governance")
    if not isinstance(governance, dict) or not governance.get("notice"):
        issues.append("metadata.governance.notice is required")

    provenance = metadata.get("provenance")
    if not isinstance(provenance, dict):
        issues.append("metadata.provenance is required")
    else:
        if not provenance.get("fixture_source"):
            issues.append("metadata.provenance.fixture_source is required")
        if not provenance.get("fictional_disclaimer"):
            issues.append("metadata.provenance.fictional_disclaimer is required")

    ingress = metadata.get("ingress_archetype")
    if not ingress:
        issues.append("metadata.ingress_archetype is required")
    elif ingress not in INGRESS_ARCHETYPES:
        issues.append(f"metadata.ingress_archetype invalid: {ingress!r}")

    for tag_list, label, known in (
        (metadata.get("topology_tags"), "topology_tags", KNOWN_TOPOLOGY_TAGS),
        (metadata.get("replay_tags"), "replay_tags", KNOWN_REPLAY_TAGS),
        (metadata.get("narrative_focus"), "narrative_focus", KNOWN_NARRATIVE_FOCUS),
    ):
        if tag_list is None:
            if label == "replay_tags":
                issues.append("metadata.replay_tags is required")
            continue
        if not isinstance(tag_list, list):
            issues.append(f"metadata.{label} must be a list")
            continue
        for tag in tag_list:
            if not isinstance(tag, str) or not tag.strip():
                issues.append(f"metadata.{label} contains invalid entry")
            elif tag not in known:
                msg = f"metadata.{label} unknown token: {tag!r}"
                if strict:
                    issues.append(msg)
                else:
                    warnings.append(msg)

    terrain_profile = metadata.get("terrain_profile")
    if not terrain_profile:
        issues.append("metadata.terrain_profile is required")
    elif terrain_profile not in TERRAIN_PROFILES:
        issues.append(f"metadata.terrain_profile invalid: {terrain_profile!r}")

    duration_class = metadata.get("replay_duration_class")
    if duration_class is not None and duration_class not in REPLAY_DURATION_CLASSES:
        issues.append(f"metadata.replay_duration_class invalid: {duration_class!r}")

    ambiguity = metadata.get("ambiguity_profile")
    if ambiguity is not None:
        if not isinstance(ambiguity, dict):
            issues.append("metadata.ambiguity_profile must be an object")
        else:
            level = ambiguity.get("level")
            if level not in AMBIGUITY_LEVELS:
                issues.append(f"metadata.ambiguity_profile.level invalid: {level!r}")
            focus = ambiguity.get("focus_tags")
            if focus is not None and not isinstance(focus, list):
                issues.append("metadata.ambiguity_profile.focus_tags must be a list")

    overlay_desc = metadata.get("overlay_descriptors")
    if overlay_desc is not None:
        if not isinstance(overlay_desc, list):
            issues.append("metadata.overlay_descriptors must be a list")
        else:
            for desc in overlay_desc:
                if not isinstance(desc, dict):
                    issues.append("metadata.overlay_descriptors entries must be objects")
                elif desc.get("kind") not in OVERLAY_KINDS:
                    issues.append(f"metadata.overlay_descriptors unknown kind: {desc.get('kind')!r}")

    anchor = topology.get("georef_anchor")
    if not isinstance(anchor, dict):
        issues.append("topology.georef_anchor is required")
    else:
        for key in ("lat_deg", "lon_deg", "h_m"):
            if key not in anchor:
                issues.append(f"topology.georef_anchor missing {key}")

    entity_ids: set[str] = set()
    for idx, ent in enumerate(topology.get("entities_static") or []):
        if not isinstance(ent, dict):
            issues.append(f"entities_static[{idx}] must be an object")
            continue
        eid = ent.get("entity_id")
        if not eid:
            issues.append(f"entities_static[{idx}] missing entity_id")
        elif eid in entity_ids:
            issues.append(f"duplicate entity_id: {eid}")
        else:
            entity_ids.add(str(eid))
        kind = ent.get("kind")
        if kind not in ENTITY_KINDS:
            issues.append(f"entities_static[{idx}] unknown kind: {kind!r}")
        pos = ent.get("position_enu_m")
        if not isinstance(pos, (list, tuple)) or len(pos) < 3:
            issues.append(f"entities_static[{idx}] invalid position_enu_m")

    zone_ids: set[str] = set()
    zones = topology.get("zones") or []
    if not isinstance(zones, list):
        issues.append("topology.zones must be a list")
        zones = []
    circle_zones: list[dict[str, Any]] = []
    for idx, zone in enumerate(zones):
        if not isinstance(zone, dict):
            issues.append(f"zones[{idx}] must be an object")
            continue
        zid = zone.get("zone_id")
        if not zid:
            issues.append(f"zones[{idx}] missing zone_id")
        elif zid in zone_ids:
            issues.append(f"duplicate zone_id: {zid}")
        else:
            zone_ids.add(str(zid))
        if zone.get("kind") not in ZONE_KINDS:
            issues.append(f"zones[{idx}] unknown kind: {zone.get('kind')!r}")
        geom = zone.get("geometry") or {}
        if geom.get("type") != "circle":
            issues.append(f"zones[{idx}] geometry.type must be circle")
        else:
            center = geom.get("center_enu_m")
            radius = geom.get("radius_m")
            if not isinstance(center, (list, tuple)) or len(center) < 2:
                issues.append(f"zones[{idx}] circle missing center_enu_m")
            if not isinstance(radius, (int, float)) or radius <= 0:
                issues.append(f"zones[{idx}] circle missing positive radius_m")
            else:
                circle_zones.append(zone)

    overlay_ids: set[str] = set()
    overlays = overlays_file.get("overlays")
    if not isinstance(overlays, list):
        issues.append("overlays.json must contain overlays array")
        overlays = []
    overlay_polys: list[dict[str, Any]] = []
    for idx, ov in enumerate(overlays):
        if not isinstance(ov, dict):
            issues.append(f"overlays[{idx}] must be an object")
            continue
        oid = ov.get("overlay_id")
        if not oid:
            issues.append(f"overlays[{idx}] missing overlay_id")
        elif oid in overlay_ids:
            issues.append(f"duplicate overlay_id: {oid}")
        else:
            overlay_ids.add(str(oid))
        if ov.get("kind") not in OVERLAY_KINDS:
            issues.append(f"overlays[{idx}] unknown kind: {ov.get('kind')!r}")
        geom = ov.get("geometry") or {}
        verts = geom.get("vertices_enu_m") or []
        if geom.get("type") != "polygon" or len(verts) < 3:
            issues.append(f"overlays[{idx}] polygon requires >=3 vertices")
        else:
            overlay_polys.append(ov)
        tr = ov.get("active_t_range")
        if tr is not None:
            if not isinstance(tr, (list, tuple)) or len(tr) != 2:
                issues.append(f"overlays[{idx}] active_t_range must be [start, end]")
            elif tr[0] > tr[1]:
                issues.append(f"overlays[{idx}] active_t_range start > end")
        for eid in ov.get("linked_event_ids") or []:
            if isinstance(eid, str) and not eid.strip():
                warnings.append(f"overlays[{idx}] has empty linked_event_id")

    if overlay_desc is not None and isinstance(overlay_desc, list):
        derived_desc = derive_overlay_descriptors(overlays)
        meta_kinds = {str(d.get("kind")) for d in overlay_desc if isinstance(d, dict)}
        overlay_kinds = {str(d.get("kind")) for d in derived_desc}
        if meta_kinds - overlay_kinds:
            issues.append(
                f"metadata.overlay_descriptors kinds not in overlays.json: {sorted(meta_kinds - overlay_kinds)}"
            )

    ann_ids: set[str] = set()
    annotations = annotations_file.get("annotations")
    if not isinstance(annotations, list):
        issues.append("annotations.json must contain annotations array")
        annotations = []
    for idx, ann in enumerate(annotations):
        if not isinstance(ann, dict):
            issues.append(f"annotations[{idx}] must be an object")
            continue
        aid = ann.get("annotation_id")
        if not aid:
            issues.append(f"annotations[{idx}] missing annotation_id")
        elif aid in ann_ids:
            issues.append(f"duplicate annotation_id: {aid}")
        else:
            ann_ids.add(str(aid))
        rank = ann.get("narrative_rank")
        if rank is not None and rank not in NARRATIVE_RANKS:
            issues.append(f"annotations[{idx}] narrative_rank must be 1, 2, or 3")

    _warn_zone_overlaps(circle_zones, warnings)
    _warn_overlay_overlaps(overlay_polys, warnings)

    ok = len(issues) == 0 and (not strict or len(warnings) == 0)
    return {"ok": ok, "issues": issues, "warnings": warnings}


def _circle_overlap(z1: dict[str, Any], z2: dict[str, Any]) -> bool:
    g1, g2 = z1.get("geometry") or {}, z2.get("geometry") or {}
    c1, c2 = g1.get("center_enu_m") or [0, 0], g2.get("center_enu_m") or [0, 0]
    r1, r2 = float(g1.get("radius_m") or 0), float(g2.get("radius_m") or 0)
    dx = float(c1[0]) - float(c2[0])
    dy = float(c1[1]) - float(c2[1])
    dist = math.hypot(dx, dy)
    return dist < (r1 + r2)


def _warn_zone_overlaps(zones: list[dict[str, Any]], warnings: list[str]) -> None:
    for i, z1 in enumerate(zones):
        for z2 in zones[i + 1 :]:
            if z1.get("kind") == z2.get("kind") and _circle_overlap(z1, z2):
                warnings.append(
                    f"zone overlap ({z1.get('zone_id')} / {z2.get('zone_id')}) same kind={z1.get('kind')}"
                )


def _bbox(verts: list[list[float]]) -> tuple[float, float, float, float]:
    xs = [float(v[0]) for v in verts]
    ys = [float(v[1]) for v in verts]
    return min(xs), min(ys), max(xs), max(ys)


def _bbox_overlap_area(v1: list[list[float]], v2: list[list[float]]) -> float:
    min_x1, min_y1, max_x1, max_y1 = _bbox(v1)
    min_x2, min_y2, max_x2, max_y2 = _bbox(v2)
    ix0 = max(min_x1, min_x2)
    iy0 = max(min_y1, min_y2)
    ix1 = min(max_x1, max_x2)
    iy1 = min(max_y1, max_y2)
    if ix1 <= ix0 or iy1 <= iy0:
        return 0.0
    return (ix1 - ix0) * (iy1 - iy0)


def _warn_overlay_overlaps(overlays: list[dict[str, Any]], warnings: list[str]) -> None:
    threshold = 1e6
    for i, o1 in enumerate(overlays):
        v1 = (o1.get("geometry") or {}).get("vertices_enu_m") or []
        for o2 in overlays[i + 1 :]:
            v2 = (o2.get("geometry") or {}).get("vertices_enu_m") or []
            if len(v1) >= 3 and len(v2) >= 3 and _bbox_overlap_area(v1, v2) > threshold:
                warnings.append(
                    f"overlay bbox overlap > threshold ({o1.get('overlay_id')} / {o2.get('overlay_id')})"
                )
