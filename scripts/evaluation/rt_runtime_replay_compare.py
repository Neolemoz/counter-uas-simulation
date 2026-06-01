#!/usr/bin/env python3
"""Compare two D1 runtime replay bundles (evaluation-only, read-only)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

SCHEMA = "rt_runtime_replay_compare_v1"


def _read_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"expected JSON object: {path}")
    return data


def _duration(bundle: dict[str, Any]) -> dict[str, int]:
    dur = ((bundle.get("clock") or {}).get("duration") or {})
    start = int(dur.get("start") or 0)
    end = int(dur.get("end") or start)
    return {"start": start, "end": end, "span": end - start + 1}


def _count(bundle: dict[str, Any], key: str) -> int:
    value = bundle.get(key)
    return len(value) if isinstance(value, list) else 0


def _lifecycle_markers(bundle: dict[str, Any]) -> list[dict[str, Any]]:
    markers = ((bundle.get("clock") or {}).get("markers") or [])
    out: list[dict[str, Any]] = []
    for marker in markers:
        if not isinstance(marker, dict):
            continue
        if str(marker.get("category") or "") != "lifecycle":
            continue
        out.append(
            {
                "t": marker.get("t"),
                "event_id": marker.get("event_id"),
                "label": marker.get("label"),
            }
        )
    return out


def _pos_key(entity: dict[str, Any]) -> tuple[float, float, float]:
    pos = entity.get("position_enu_m") if isinstance(entity.get("position_enu_m"), list) else []
    vals = [float(pos[i]) if i < len(pos) and isinstance(pos[i], (int, float)) else 0.0 for i in range(3)]
    return vals[0], vals[1], vals[2]


def _entity_map(bundle: dict[str, Any]) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for entity in bundle.get("entities_static") or []:
        if not isinstance(entity, dict):
            continue
        entity_id = str(entity.get("entity_id") or "")
        if entity_id:
            out[entity_id] = entity
    return out


def _topology_diff(a: dict[str, Any], b: dict[str, Any]) -> dict[str, Any]:
    ent_a = _entity_map(a)
    ent_b = _entity_map(b)
    added = sorted(set(ent_b) - set(ent_a))
    removed = sorted(set(ent_a) - set(ent_b))
    shifted: list[dict[str, Any]] = []
    for entity_id in sorted(set(ent_a) & set(ent_b)):
        ax, ay, az = _pos_key(ent_a[entity_id])
        bx, by, bz = _pos_key(ent_b[entity_id])
        dx, dy, dz = bx - ax, by - ay, bz - az
        if abs(dx) > 0.5 or abs(dy) > 0.5 or abs(dz) > 0.5:
            shifted.append(
                {
                    "entity_id": entity_id,
                    "delta_enu_m": [round(dx, 3), round(dy, 3), round(dz, 3)],
                }
            )

    zones_a = {str(z.get("zone_id")) for z in a.get("zones") or [] if isinstance(z, dict)}
    zones_b = {str(z.get("zone_id")) for z in b.get("zones") or [] if isinstance(z, dict)}
    overlays_a = {str(o.get("overlay_id")) for o in a.get("overlays") or [] if isinstance(o, dict)}
    overlays_b = {str(o.get("overlay_id")) for o in b.get("overlays") or [] if isinstance(o, dict)}

    los_a = len(a.get("los_segments") or [])
    los_b = len(b.get("los_segments") or [])
    bullets: list[str] = []
    for row in shifted:
        bullets.append(f"Entity {row['entity_id']} shifted by {row['delta_enu_m']} ENU meters.")
    for entity_id in added:
        bullets.append(f"Bundle B adds entity {entity_id}.")
    for entity_id in removed:
        bullets.append(f"Bundle B removes entity {entity_id}.")
    if los_a != los_b:
        bullets.append(f"LOS segment count differs: {los_a} -> {los_b}.")
    if not bullets:
        bullets.append("No topology diff detected in static entities, zones, overlays, or LOS counts.")

    return {
        "entity_added": added,
        "entity_removed": removed,
        "entity_shifted": shifted,
        "zone_added": sorted(zones_b - zones_a),
        "zone_removed": sorted(zones_a - zones_b),
        "overlay_added": sorted(overlays_b - overlays_a),
        "overlay_removed": sorted(overlays_a - overlays_b),
        "los_segment_count_a": los_a,
        "los_segment_count_b": los_b,
        "bullets": bullets,
    }


def _warnings(a: dict[str, Any], b: dict[str, Any]) -> list[str]:
    warnings: list[str] = []
    for label, bundle in (("A", a), ("B", b)):
        if bundle.get("artifact_type") != "replay_sa_bundle":
            warnings.append(f"bundle {label} artifact_type is not replay_sa_bundle")
        if bundle.get("bundle_schema_version") != "replay_sa_bundle_v1":
            warnings.append(f"bundle {label} schema is not replay_sa_bundle_v1")
        if not isinstance(bundle.get("rt_runtime_import"), dict):
            warnings.append(f"bundle {label} has no rt_runtime_import block")
        markers = ((bundle.get("clock") or {}).get("markers") or [])
        if not any(isinstance(m, dict) and m.get("category") == "detection" for m in markers):
            warnings.append(f"bundle {label} has no detection marker; detection deltas unavailable")
        if not (bundle.get("los_segments") or []):
            warnings.append(f"bundle {label} has no LOS segments; LOS deltas are weak/empty")
        narrative = bundle.get("narrative") if isinstance(bundle.get("narrative"), dict) else {}
        if not (narrative.get("events") or []):
            warnings.append(f"bundle {label} has no narrative events; event deltas are weak/empty")
    return warnings


def build_compare(a: dict[str, Any], b: dict[str, Any], *, path_a: str, path_b: str) -> dict[str, Any]:
    dur_a = _duration(a)
    dur_b = _duration(b)
    entity_a = _count(a, "entities_static")
    entity_b = _count(b, "entities_static")
    track_a = _count(a, "tracks")
    track_b = _count(b, "tracks")
    return {
        "artifact_type": SCHEMA,
        "schema_version": SCHEMA,
        "governance": {
            "notice": "Runtime replay compare is explanatory only; not parser authority or operational evidence.",
            "anti_claims": [
                "not operational readiness",
                "not validated effectiveness",
                "not tactical superiority",
            ],
        },
        "inputs": {"bundle_a": path_a, "bundle_b": path_b},
        "duration_delta": {"a": dur_a, "b": dur_b, "delta": dur_b["span"] - dur_a["span"]},
        "entity_count_delta": {"a": entity_a, "b": entity_b, "delta": entity_b - entity_a},
        "track_count_delta": {"a": track_a, "b": track_b, "delta": track_b - track_a},
        "lifecycle_markers": {"a": _lifecycle_markers(a), "b": _lifecycle_markers(b)},
        "topology_diff_summary": _topology_diff(a, b),
        "warnings": _warnings(a, b),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare two runtime replay bundles.")
    parser.add_argument("bundle_a", type=Path)
    parser.add_argument("bundle_b", type=Path)
    parser.add_argument("--json", action="store_true", help="Emit JSON to stdout")
    parser.add_argument("--out", type=Path, default=None, help="Optional output JSON path")
    args = parser.parse_args()

    try:
        a = _read_json(args.bundle_a)
        b = _read_json(args.bundle_b)
        report = build_compare(a, b, path_a=args.bundle_a.as_posix(), path_b=args.bundle_b.as_posix())
    except Exception as exc:
        print(f"rt_runtime_replay_compare: {exc}", file=sys.stderr)
        return 1

    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
    if args.json or not args.out:
        print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
