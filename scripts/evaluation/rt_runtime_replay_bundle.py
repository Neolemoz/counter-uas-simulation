#!/usr/bin/env python3
"""Map rt_runtime_run_capture_v1 (runtime_run.json) → replay_sa_bundle_v1 (evaluation-only).

Read-only transformation helpers. Does not modify runtime bridge, ROS, or viewer code.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

RUNTIME_CAPTURE_SCHEMA = "rt_runtime_run_capture_v1"
BUNDLE_SCHEMA_VERSION = "replay_sa_bundle_v1"
DRY_RUN_PREVIEW_SCHEMA = "rt_runtime_replay_import_dry_run_v1"

_RUNTIME_REQUIRED = (
    "session_id",
    "capture_id",
    "started_utc",
    "stopped_utc",
    "timestamp",
    "entities",
    "telemetry_frames",
    "assignments",
    "lifecycle_transitions",
)

_ENTITY_KIND = {
    "waypoint_marker": "waypoint",
    "radar": "radar",
    "interceptor": "interceptor",
    "drone": "threat",
}

_TRACK_ROLES = {
    "drone": "threat",
    "interceptor": "interceptor",
}

_GOVERNANCE_NOTICE = (
    "Derived evaluation artifact from RT runtime capture. Does not replace parser-visible "
    "summaries, runtime topics, tactical authority, or replay log contracts."
)

_INTERPRETATION_CAVEATS = [
    "Mapped from rt_runtime_run_capture_v1 — explanatory RT mirror, not replay log truth.",
    "Clock domain uses runtime capture frame indices, not ROS log line numbers.",
    "Track samples reflect entity_pose_mirror frames only; sparse gaps are expected.",
    "Static entities and assignments mirror command-authoritative RT registry at capture stop.",
]


def load_runtime_run(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"expected JSON object: {path}")
    return data


def validate_runtime_run(artifact: dict[str, Any]) -> dict[str, Any]:
    missing = [field for field in _RUNTIME_REQUIRED if field not in artifact]
    type_errors: list[str] = []
    if artifact.get("schema") not in (None, RUNTIME_CAPTURE_SCHEMA):
        type_errors.append(f"unexpected schema: {artifact.get('schema')!r}")
    for key in ("entities", "telemetry_frames", "lifecycle_transitions"):
        if key in artifact and not isinstance(artifact[key], list):
            type_errors.append(f"{key} must be a list")
    if "assignments" in artifact and not isinstance(artifact["assignments"], dict):
        type_errors.append("assignments must be an object")
    return {
        "valid": not missing and not type_errors,
        "missing": missing,
        "type_errors": type_errors,
    }


def collect_mapping_warnings(artifact: dict[str, Any]) -> list[str]:
    warnings: list[str] = []
    validation = validate_runtime_run(artifact)
    if validation["missing"]:
        warnings.append(f"missing required fields: {', '.join(validation['missing'])}")
    if validation["type_errors"]:
        warnings.extend(validation["type_errors"])
    if artifact.get("schema") != RUNTIME_CAPTURE_SCHEMA:
        warnings.append(f"schema is not {RUNTIME_CAPTURE_SCHEMA}")

    frames = artifact.get("telemetry_frames") or []
    pose_frames = [f for f in frames if isinstance(f, dict) and f.get("channel") == "entity_pose_mirror"]
    if not pose_frames:
        warnings.append("no entity_pose_mirror telemetry frames — tracks will be pose-only at t=0")

    entities = artifact.get("entities") or []
    if not entities:
        warnings.append("entities list is empty")

    for entity in entities:
        if not isinstance(entity, dict):
            warnings.append("entities contains non-object entry")
            continue
        pose = entity.get("pose")
        if not isinstance(pose, dict):
            warnings.append(f"entity {entity.get('entity_id')} missing pose")
        elif not all(k in pose for k in ("x", "y", "z")):
            warnings.append(f"entity {entity.get('entity_id')} pose missing x/y/z")

    if not artifact.get("world_revision"):
        warnings.append("world_revision absent — revision lineage omitted in bundle")

    if not artifact.get("governance_banner"):
        warnings.append("governance_banner absent on runtime capture")

    return warnings


def _pose_xyz(pose: dict[str, Any]) -> tuple[float, float, float]:
    return (float(pose.get("x", 0.0)), float(pose.get("y", 0.0)), float(pose.get("z", 0.0)))


def _entity_type(entity: dict[str, Any]) -> str:
    return str(entity.get("entity_type") or "")


def _entity_id(entity: dict[str, Any]) -> str:
    return str(entity.get("entity_id") or "")


def _build_entities_static(entities: list[Any]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for raw in entities:
        if not isinstance(raw, dict):
            continue
        entity_id = _entity_id(raw)
        entity_type = _entity_type(raw)
        pose = raw.get("pose") if isinstance(raw.get("pose"), dict) else {}
        x, y, z = _pose_xyz(pose)
        kind = _ENTITY_KIND.get(entity_type, entity_type or "unknown")
        out.append(
            {
                "entity_id": entity_id,
                "kind": kind,
                "position_enu_m": [x, y, z],
                "label": entity_id or entity_type,
                "authoritative": False,
            }
        )
    return out


def _build_tracks(artifact: dict[str, Any]) -> list[dict[str, Any]]:
    entities = [e for e in (artifact.get("entities") or []) if isinstance(e, dict)]
    track_roles: dict[str, str] = {}
    for entity in entities:
        entity_type = _entity_type(entity)
        role = _TRACK_ROLES.get(entity_type)
        if role:
            track_roles[_entity_id(entity)] = role

    samples_by_track: dict[str, list[dict[str, Any]]] = {tid: [] for tid in track_roles}

    for entity in entities:
        entity_id = _entity_id(entity)
        if entity_id not in track_roles:
            continue
        pose = entity.get("pose") if isinstance(entity.get("pose"), dict) else {}
        x, y, z = _pose_xyz(pose)
        samples_by_track[entity_id].append(
            {
                "t": 0,
                "x_m": x,
                "y_m": y,
                "z_m": z,
                "source": "runtime_capture_entities",
            }
        )

    frames = artifact.get("telemetry_frames") or []
    for frame_index, frame in enumerate(frames):
        if not isinstance(frame, dict):
            continue
        if frame.get("channel") != "entity_pose_mirror":
            continue
        payload = frame.get("payload") if isinstance(frame.get("payload"), dict) else {}
        mirror_entities = payload.get("entities")
        if not isinstance(mirror_entities, list):
            continue
        for mirror in mirror_entities:
            if not isinstance(mirror, dict):
                continue
            entity_id = _entity_id(mirror)
            if entity_id not in track_roles:
                continue
            pose = mirror.get("pose") if isinstance(mirror.get("pose"), dict) else {}
            if pose:
                x, y, z = _pose_xyz(pose)
            else:
                x = float(mirror.get("x", 0.0))
                y = float(mirror.get("y", 0.0))
                z = float(mirror.get("z", 0.0))
            sample: dict[str, Any] = {
                "t": frame_index,
                "x_m": x,
                "y_m": y,
                "z_m": z,
                "source": "entity_pose_mirror",
            }
            if mirror.get("speed_mps") is not None:
                sample["speed_mps"] = float(mirror["speed_mps"])
            if mirror.get("heading_deg") is not None:
                sample["heading_deg"] = float(mirror["heading_deg"])
            samples_by_track[entity_id].append(sample)

    tracks: list[dict[str, Any]] = []
    for track_id, role in sorted(track_roles.items()):
        samples = samples_by_track.get(track_id) or []
        tracks.append(
            {
                "track_id": track_id,
                "role": role,
                "samples": samples,
                "interpretation_caveat": "RT runtime mirror track — not parser /tracks/state replay.",
            }
        )
    return tracks


def _build_clock(artifact: dict[str, Any], tracks: list[dict[str, Any]]) -> dict[str, Any]:
    frame_count = len(artifact.get("telemetry_frames") or [])
    max_sample_t = 0
    for track in tracks:
        for sample in track.get("samples") or []:
            if isinstance(sample, dict) and isinstance(sample.get("t"), int):
                max_sample_t = max(max_sample_t, int(sample["t"]))
    end_t = max(frame_count - 1, max_sample_t, 0)
    markers: list[dict[str, Any]] = []
    for index, transition in enumerate(artifact.get("lifecycle_transitions") or []):
        if not isinstance(transition, dict):
            continue
        payload = transition.get("payload") if isinstance(transition.get("payload"), dict) else {}
        command_type = str(payload.get("command_type") or payload.get("state") or "lifecycle")
        markers.append(
            {
                "t": min(index, end_t),
                "event_id": f"rt_lifecycle_{index:03d}",
                "label": command_type,
                "category": "lifecycle",
            }
        )
    return {
        "domain": "runtime_capture_frame_index",
        "duration": {"start": 0, "end": end_t, "step": 1},
        "markers": markers,
    }


def _build_telemetry_series(
    artifact: dict[str, Any],
    tracks: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    series: list[dict[str, Any]] = []
    assignments = artifact.get("assignments") if isinstance(artifact.get("assignments"), dict) else {}
    track_by_id = {str(t.get("track_id")): t for t in tracks if isinstance(t, dict)}

    for frame_index, frame in enumerate(artifact.get("telemetry_frames") or []):
        if not isinstance(frame, dict) or frame.get("channel") != "entity_pose_mirror":
            continue
        payload = frame.get("payload") if isinstance(frame.get("payload"), dict) else {}
        for mirror in payload.get("entities") or []:
            if not isinstance(mirror, dict):
                continue
            entity_id = _entity_id(mirror)
            if str(mirror.get("entity_type") or "") != "interceptor":
                continue
            target_id = mirror.get("active_target_id") or assignments.get(entity_id)
            row: dict[str, Any] = {
                "t": frame_index,
                "interceptor_id": entity_id,
                "source": "runtime_capture_mirror",
            }
            if target_id:
                row["target_id"] = str(target_id)
            if mirror.get("assignment_state"):
                row["assignment_state"] = str(mirror["assignment_state"])
            if mirror.get("speed_mps") is not None:
                row["speed_mps"] = float(mirror["speed_mps"])
            threat = track_by_id.get(str(target_id)) if target_id else None
            if threat and threat.get("samples"):
                last = threat["samples"][-1]
                if isinstance(last, dict):
                    ix = float(last.get("x_m", 0.0))
                    iy = float(last.get("y_m", 0.0))
                    iz = float(last.get("z_m", 0.0))
                    mirror_pose = mirror.get("pose") if isinstance(mirror.get("pose"), dict) else {}
                    ex = float(mirror.get("x", mirror_pose.get("x", 0.0)))
                    ey = float(mirror.get("y", mirror_pose.get("y", 0.0)))
                    ez = float(mirror.get("z", mirror_pose.get("z", 0.0)))
                    if ex == 0.0 and ey == 0.0 and ez == 0.0:
                        for ent in artifact.get("entities") or []:
                            if isinstance(ent, dict) and _entity_id(ent) == entity_id:
                                pose = ent.get("pose") if isinstance(ent.get("pose"), dict) else {}
                                ex, ey, ez = _pose_xyz(pose)
                                break
                    row["dist_m"] = round(((ix - ex) ** 2 + (iy - ey) ** 2 + (iz - ez) ** 2) ** 0.5, 3)
            series.append(row)
    return series


def _lifecycle_summary(artifact: dict[str, Any]) -> dict[str, Any]:
    commands: list[str] = []
    states: list[str] = []
    for transition in artifact.get("lifecycle_transitions") or []:
        if not isinstance(transition, dict):
            continue
        payload = transition.get("payload") if isinstance(transition.get("payload"), dict) else {}
        if payload.get("command_type"):
            commands.append(str(payload["command_type"]))
        if payload.get("state"):
            states.append(str(payload["state"]))
    return {
        "transition_count": len(artifact.get("lifecycle_transitions") or []),
        "commands_seen": commands,
        "states_seen": states,
    }


def _assignment_summary(artifact: dict[str, Any]) -> dict[str, Any]:
    assignments = artifact.get("assignments") if isinstance(artifact.get("assignments"), dict) else {}
    pairs = [
        {"defender_id": str(defender_id), "target_id": str(target_id)}
        for defender_id, target_id in sorted(assignments.items())
    ]
    return {"assignment_count": len(pairs), "pairs": pairs}


def build_dry_run_preview(
    artifact: dict[str, Any],
    *,
    source_path: str | None = None,
) -> dict[str, Any]:
    warnings = collect_mapping_warnings(artifact)
    tracks = _build_tracks(artifact)
    entities_static = _build_entities_static(artifact.get("entities") or [])
    clock = _build_clock(artifact, tracks)
    preview: dict[str, Any] = {
        "schema": DRY_RUN_PREVIEW_SCHEMA,
        "governance_banner": "RT runtime replay import dry-run — read-only preview; not replay truth",
        "source": {
            "runtime_capture_schema": artifact.get("schema"),
            "capture_id": artifact.get("capture_id"),
            "session_id": artifact.get("session_id"),
            "artifact_path": source_path,
        },
        "replay_clock": clock,
        "tracks_count": len(tracks),
        "entities_count": len(entities_static),
        "markers": list(clock.get("markers") or []),
        "lifecycle_summary": _lifecycle_summary(artifact),
        "assignment_summary": _assignment_summary(artifact),
        "missing_fields_warnings": warnings,
        "mapping_ok": not any(
            w.startswith("missing required") or w.startswith("schema is not") for w in warnings
        ),
    }
    return preview


def build_replay_sa_bundle_from_runtime_run(
    artifact: dict[str, Any],
    *,
    source_path: str | None = None,
) -> dict[str, Any]:
    warnings = collect_mapping_warnings(artifact)
    tracks = _build_tracks(artifact)
    entities_static = _build_entities_static(artifact.get("entities") or [])
    clock = _build_clock(artifact, tracks)
    telemetry_series = _build_telemetry_series(artifact, tracks)

    capture_id = str(artifact.get("capture_id") or "unknown_capture")
    session_id = str(artifact.get("session_id") or "unknown_session")

    bundle: dict[str, Any] = {
        "artifact_type": "replay_sa_bundle",
        "bundle_schema_version": BUNDLE_SCHEMA_VERSION,
        "mode": "replay_static",
        "governance": {
            "schema_version": BUNDLE_SCHEMA_VERSION,
            "notice": _GOVERNANCE_NOTICE,
            "constraints": [
                "evaluation-side only",
                "additive-only",
                "parser-safe",
                "non-authoritative",
                "read_only_viewer",
                "rt_runtime_capture_import",
            ],
            "anti_claims": [
                "not operational readiness evidence",
                "not a tactical authority surface",
                "not live RT bridge state",
            ],
        },
        "lineage": {
            "import_kind": "rt_runtime_run_capture_v1",
            "capture_id": capture_id,
            "session_id": session_id,
            "started_utc": artifact.get("started_utc"),
            "stopped_utc": artifact.get("stopped_utc"),
            "runtime_capture_path": source_path,
            "world_revision": artifact.get("world_revision"),
        },
        "scenario": {
            "scenario_id": f"rt_runtime_{capture_id}",
            "title": f"RT runtime capture: {capture_id}",
            "topology_tags": ["rt_sandbox", "runtime_capture"],
            "replay_duration_class": "short" if int(clock["duration"]["end"]) <= 30 else "medium",
            "provenance": {
                "fixture_source": source_path or "runtime_run.json",
                "fictional_disclaimer": "Mapped from RT sandbox capture — not deployed scenario pack.",
            },
        },
        "comparison_hints": {
            "topology_key": "rt_runtime_capture",
            "scenario_id": f"rt_runtime_{capture_id}",
            "catalog_entry_id": "rt_runtime_capture",
            "comparison_ready": False,
            "duration_class": "short" if int(clock["duration"]["end"]) <= 30 else "medium",
        },
        "georef_display": {
            "frame": "scenario_enu",
            "origin_enu_m": [0.0, 0.0, 0.0],
            "anchor": {"lat_deg": 35.0, "lon_deg": -116.0, "h_m": 0.0},
            "caveat": "Fictional georef from RT sandbox — not deployed geography.",
        },
        "clock": clock,
        "tracks": tracks,
        "entities_static": entities_static,
        "zones": [],
        "overlays": [],
        "los_segments": [],
        "narrative": {
            "events": [],
            "windows": [],
            "bookmarks": [],
            "annotations": [],
        },
        "comprehension": {
            "headline": f"RT runtime import preview ({capture_id})",
            "scan_guide": [
                "Review static entities from capture stop snapshot.",
                "Scrub mirror tracks from entity_pose_mirror frames.",
                "Read assignment rows in telemetry panel as explanatory only.",
            ],
            "at_a_glance": {
                "cards": [
                    {"label": "Capture", "value": capture_id},
                    {"label": "Session", "value": session_id},
                    {"label": "Tracks", "value": str(len(tracks))},
                    {"label": "Entities", "value": str(len(entities_static))},
                ],
                "summary": {
                    "mapping_warnings": len(warnings),
                    "assignment_count": _assignment_summary(artifact)["assignment_count"],
                },
            },
        },
        "source_artifacts": {
            "embedded": True,
            "runtime_capture_schema": RUNTIME_CAPTURE_SCHEMA,
            "runtime_capture_path": source_path,
            "import_tool": "rt_runtime_replay_bundle.py",
        },
        "views": {
            "radar_mock": {"sweep_sector_deg": 120, "linked_track_ids": [t["track_id"] for t in tracks]},
            "eoir_mock": {"fov_deg": 45, "thumbnail_frames": []},
            "onboard_mock": {
                "camera_id": "rt_capture",
                "frames": [],
                "caveat": "No video frames in runtime capture — mirror poses only.",
            },
        },
        "panels": {
            "telemetry_series": telemetry_series,
            "threat_assessment": [
                {
                    "t": int(clock["duration"]["end"]),
                    "label": "rt_runtime_capture",
                    "caveat": "Import-local label — not threat score or readiness.",
                }
            ],
        },
        "interpretation_caveats": list(_INTERPRETATION_CAVEATS),
        "rt_runtime_import": {
            "schema": "rt_runtime_replay_import_v1",
            "mapping_warnings": warnings,
            "dry_run_preview_schema": DRY_RUN_PREVIEW_SCHEMA,
        },
    }
    return bundle


def lint_bundle_candidate(bundle: dict[str, Any]) -> dict[str, Any]:
    """Reuse replay_sa_bundle governance lint when available."""
    try:
        from replay_sa_bundle import lint_replay_sa_bundle  # noqa: WPS433

        return lint_replay_sa_bundle(bundle)
    except ImportError:
        issues: list[str] = []
        if bundle.get("artifact_type") != "replay_sa_bundle":
            issues.append("artifact_type must be replay_sa_bundle")
        return {"ok": not issues, "issues": issues, "warnings": []}

def write_replay_bundle(
    runtime_run_path: Path,
    out_path: Path,
    *,
    source_path: str | None = None,
) -> dict[str, Any]:
    artifact = load_runtime_run(runtime_run_path)
    bundle = build_replay_sa_bundle_from_runtime_run(
        artifact,
        source_path=source_path or runtime_run_path.as_posix(),
    )
    lint = lint_bundle_candidate(bundle)
    if not lint.get("ok"):
        raise ValueError(f"bundle lint failed: {lint.get('issues')}")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(bundle, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return bundle


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Export rt_runtime_run_capture_v1 as replay_sa_bundle_v1 JSON."
    )
    parser.add_argument("runtime_run_json", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--source-path",
        default=None,
        help="Optional source path string to embed in bundle lineage/source_artifacts.",
    )
    args = parser.parse_args()

    try:
        write_replay_bundle(
            args.runtime_run_json,
            args.out,
            source_path=args.source_path or args.runtime_run_json.as_posix(),
        )
    except Exception as exc:
        print(f"rt_runtime_replay_bundle: {exc}", file=sys.stderr)
        return 1
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
