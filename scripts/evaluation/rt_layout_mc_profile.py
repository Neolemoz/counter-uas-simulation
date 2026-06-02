#!/usr/bin/env python3
"""Translate offline RT layout artifacts into Monte Carlo launch profile previews.

The translator is intentionally dry-run only: it does not launch Gazebo, mutate RT
sessions, write SA scenario packs, or promote replay artifacts.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

LAYOUT_SCHEMA_VERSION = "rt_layout_scenario_v1"
PROFILE_SCHEMA_VERSION = "rt_layout_mc_profile_v1"
ENTITY_TYPES = frozenset({"radar", "interceptor", "drone", "waypoint_marker"})
REQUIRED_LAYOUT_KEYS = ("schema_version", "layout_id", "terrain_preset", "entities", "source")
POSE_KEYS = ("x", "y", "z")


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _read_json_object(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"expected JSON object: {path}")
    return data


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _floatish(value: Any) -> bool:
    try:
        float(value)
    except (TypeError, ValueError):
        return False
    return True


def validate_layout(data: dict[str, Any]) -> dict[str, Any]:
    """Validate an rt_layout_scenario_v1 object without touching runtime state."""
    issues: list[str] = []
    warnings: list[str] = []

    for key in REQUIRED_LAYOUT_KEYS:
        if key not in data:
            issues.append(f"missing required field: {key}")

    if data.get("schema_version") != LAYOUT_SCHEMA_VERSION:
        issues.append(f"schema_version must be {LAYOUT_SCHEMA_VERSION!r}")
    if not isinstance(data.get("layout_id"), str) or not str(data.get("layout_id")).strip():
        issues.append("layout_id must be a non-empty string")
    if not isinstance(data.get("terrain_preset"), str) or not str(data.get("terrain_preset")).strip():
        issues.append("terrain_preset must be a non-empty string")
    if not isinstance(data.get("source"), dict):
        issues.append("source must be an object")

    entities = data.get("entities")
    if not isinstance(entities, list):
        issues.append("entities must be a list")
        entities = []
    if len(entities) == 0:
        warnings.append("layout has no entities")

    for idx, entity in enumerate(entities):
        if not isinstance(entity, dict):
            issues.append(f"entities[{idx}] must be an object")
            continue
        entity_type = entity.get("entity_type")
        if entity_type not in ENTITY_TYPES:
            issues.append(f"entities[{idx}].entity_type unsupported: {entity_type!r}")
        pose = entity.get("pose")
        if not isinstance(pose, dict):
            issues.append(f"entities[{idx}].pose must be an object")
            continue
        for key in POSE_KEYS:
            if key not in pose:
                issues.append(f"entities[{idx}].pose missing {key}")
            elif not _floatish(pose[key]):
                issues.append(f"entities[{idx}].pose.{key} must be numeric")
        if "yaw_deg" in pose and not _floatish(pose["yaw_deg"]):
            issues.append(f"entities[{idx}].pose.yaw_deg must be numeric when present")

    return {"ok": len(issues) == 0, "issues": issues, "warnings": warnings}


def _normalized_pose(pose: dict[str, Any]) -> dict[str, float]:
    out = {key: float(pose[key]) for key in POSE_KEYS}
    if "yaw_deg" in pose:
        out["yaw_deg"] = float(pose["yaw_deg"])
    return out


def _normalized_entities(data: dict[str, Any]) -> list[dict[str, Any]]:
    entities: list[dict[str, Any]] = []
    for entity in data.get("entities") or []:
        if not isinstance(entity, dict):
            continue
        pose = entity.get("pose")
        if not isinstance(pose, dict):
            continue
        entities.append(
            {
                "entity_type": str(entity.get("entity_type") or ""),
                "pose": _normalized_pose(pose),
            }
        )
    return entities


def geometry_fingerprint(data: dict[str, Any]) -> str:
    """Return a stable geometry-only fingerprint for matched-seed MC pairing."""
    canonical = {
        "schema_version": LAYOUT_SCHEMA_VERSION,
        "terrain_preset": str(data.get("terrain_preset") or ""),
        "entities": _normalized_entities(data),
    }
    payload = json.dumps(canonical, sort_keys=True, separators=(",", ":")).encode("utf-8")
    digest = hashlib.sha256(payload).hexdigest()
    return f"rt_layout:sha256:{digest[:16]}"


def _fmt_float(value: float) -> str:
    return f"{value:.6g}"


def _count_entities(entities: list[dict[str, Any]]) -> dict[str, int]:
    counts = {key: 0 for key in sorted(ENTITY_TYPES)}
    for entity in entities:
        entity_type = str(entity.get("entity_type") or "")
        if entity_type in counts:
            counts[entity_type] += 1
    return counts


def translate_layout_to_profile(data: dict[str, Any]) -> dict[str, Any]:
    validation = validate_layout(data)
    if not validation["ok"]:
        raise ValueError("; ".join(validation["issues"]))

    entities = _normalized_entities(data)
    counts = _count_entities(entities)
    drones = [e for e in entities if e["entity_type"] == "drone"]
    interceptors = [e for e in entities if e["entity_type"] == "interceptor"]
    radars = [e for e in entities if e["entity_type"] == "radar"]
    waypoints = [e for e in entities if e["entity_type"] == "waypoint_marker"]

    warnings = list(validation["warnings"])
    unsupported_fields: list[dict[str, str]] = []
    launch_fields: dict[str, str] = {}

    if len(drones) == 1:
        pose = drones[0]["pose"]
        launch_fields["target_start_x_m"] = _fmt_float(pose["x"])
        launch_fields["target_start_y_m"] = _fmt_float(pose["y"])
        launch_fields["target_start_z_m"] = _fmt_float(pose["z"])
    elif len(drones) == 0:
        warnings.append("no drone entity: target_start_* launch args were not generated")
    else:
        warnings.append("multiple drone entities: MC target_start_* mapping supports exactly one drone today")
        unsupported_fields.append(
            {
                "field": "entities[drone]",
                "reason": "multiple drone layout is not launch-arg compatible today",
            }
        )

    if len(interceptors) == 3:
        coords: list[str] = []
        for entity in interceptors:
            pose = entity["pose"]
            coords.extend([_fmt_float(pose["x"]), _fmt_float(pose["y"])])
        launch_fields["interceptor_ic_layout"] = "custom:" + ",".join(coords)
    elif len(interceptors) == 0:
        warnings.append("no interceptor entities: interceptor_ic_layout was not generated")
    else:
        warnings.append("interceptor_ic_layout custom mapping requires exactly three interceptors today")
        unsupported_fields.append(
            {
                "field": "entities[interceptor]",
                "reason": f"found {len(interceptors)} interceptor entities; expected 3",
            }
        )

    for entity_type, rows in (("radar", radars), ("waypoint_marker", waypoints)):
        if rows:
            warnings.append(f"{entity_type} entities are retained as metadata only for MC preview")
            unsupported_fields.append(
                {
                    "field": f"entities[{entity_type}]",
                    "reason": "no current Monte Carlo launch-arg mapping",
                }
            )

    scenario_suggestion = "multi" if len(drones) > 1 else "single"
    launch_args = " ".join(f"{key}:={value}" for key, value in launch_fields.items())

    return {
        "schema_version": PROFILE_SCHEMA_VERSION,
        "source_layout_id": str(data["layout_id"]),
        "geometry_id": geometry_fingerprint(data),
        "scenario_suggestion": scenario_suggestion,
        "launch_args": launch_args,
        "launch_args_fields": launch_fields,
        "entity_counts": counts,
        "warnings": warnings,
        "unsupported_fields": unsupported_fields,
        "source": {
            "artifact_schema": LAYOUT_SCHEMA_VERSION,
            "layout_source": data.get("source"),
        },
        "generated_utc": _utc_now(),
    }


def profile_csv_row(profile: dict[str, Any]) -> dict[str, str]:
    return {
        "source_layout_id": str(profile.get("source_layout_id") or ""),
        "geometry_id": str(profile.get("geometry_id") or ""),
        "scenario_suggestion": str(profile.get("scenario_suggestion") or ""),
        "launch_args": str(profile.get("launch_args") or ""),
        "warning_count": str(len(profile.get("warnings") or [])),
        "unsupported_count": str(len(profile.get("unsupported_fields") or [])),
        "warnings": " | ".join(str(w) for w in profile.get("warnings") or []),
    }


def write_profile_csv(path: Path, profile: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "source_layout_id",
        "geometry_id",
        "scenario_suggestion",
        "launch_args",
        "warning_count",
        "unsupported_count",
        "warnings",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerow(profile_csv_row(profile))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Preview MC launch args from an offline rt_layout_scenario_v1 artifact."
    )
    parser.add_argument("layout_json", type=Path)
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--out-csv", type=Path, default=None)
    args = parser.parse_args()

    try:
        layout = _read_json_object(args.layout_json)
        profile = translate_layout_to_profile(layout)
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        print(f"rt_layout_mc_profile: {exc}", file=sys.stderr)
        return 2

    if args.out_json:
        _write_json(args.out_json, profile)
    if args.out_csv:
        write_profile_csv(args.out_csv, profile)
    if not args.out_json:
        print(json.dumps(profile, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
