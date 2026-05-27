#!/usr/bin/env python3
"""Pack frozen replay evaluation artifacts into replay_sa_bundle_v1 for SA-R0 viewer.

All outputs are derived evaluation artifacts. They do not modify runtime behavior,
parser contracts, or tactical authority surfaces.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import sys
import zipfile
from pathlib import Path
from typing import Any

_EVAL_DIR = Path(__file__).resolve().parent
if str(_EVAL_DIR) not in sys.path:
    sys.path.insert(0, str(_EVAL_DIR))

from replay_sa_geometry import build_dome_zones  # noqa: E402
from replay_sa_geometry import build_fictional_heightmap  # noqa: E402
from replay_sa_geometry import build_los_segments  # noqa: E402
from replay_sa_geometry import load_scenario_topology  # noqa: E402
from replay_sa_geometry import parse_launch_args_from_meta  # noqa: E402
from replay_sa_scenario import compute_sensor_layout_id  # noqa: E402
from replay_sa_scenario import replay_duration_class_from_span  # noqa: E402
from replay_viz_comprehension import parse_log_outcome_metrics  # noqa: E402
from replay_viz_figures import _GUIDANCE_POS_RE  # noqa: E402
from replay_viz_figures import _P_HEATMAP_POS_RE  # noqa: E402
from replay_viz_figures import _parse_launch_geometry  # noqa: E402
from replay_viz_figures import _strip_ros_prefix  # noqa: E402
from rt_tactical_replay_continuity import (  # noqa: E402
    attach_rt_tactical_replay_continuity,
    lint_rt_tactical_replay_continuity,
)

BUNDLE_SCHEMA_VERSION = "replay_sa_bundle_v1"
_REPO_ROOT = Path(__file__).resolve().parents[2]
NON_AUTHORITATIVE_NOTICE = (
    "Derived evaluation artifact only. Does not replace parser-visible summaries, "
    "runtime topics, tactical authority, lifecycle semantics, or replay contracts."
)

_METRICS_ROW_RE = re.compile(
    r"id=(?P<iid>\S+)"
    r".*?dist=(?P<dist>[0-9]+(?:\.[0-9]+)?)\s*m"
    r".*?t_go=(?P<tgo>[0-9]+(?:\.[0-9]+)?|n/a)\s*"
    r".*?vel=(?P<vel>[0-9]+(?:\.[0-9]+)?)\s*m/s"
)
_P_HEATMAP_THREAT_RE = re.compile(
    r"\[P_HEATMAP\]\s+"
    r"(?:threat_id=(?P<threat_id>\S+)\s+)?"
    r"pos=\(\s*(?P<x>[-\d.]+)\s*,\s*(?P<y>[-\d.]+)\s*,\s*(?P<z>[-\d.]+)\s*\)"
)
_GUIDANCE_TRACKS_RE = re.compile(
    r"(?:interceptor_id=(?P<interceptor_id>\S+)\s+)?"
    r"interceptor_pos=\(\s*(?P<ix>[-\d.]+)\s*,\s*(?P<iy>[-\d.]+)\s*,\s*(?P<iz>[-\d.]+)\s*\)"
    r".*target_pos=\(\s*(?P<tx>[-\d.]+)\s*,\s*(?P<ty>[-\d.]+)\s*,\s*(?P<tz>[-\d.]+)\s*\)"
    r"(?:\s+threat_id=(?P<threat_id>\S+))?"
)
_DEFAULT_THREAT_ID = "threat_uav_0"
_DEFAULT_INTERCEPTOR_ID = "interceptor_0"

# Fictional display anchor (not deployed geography).
DEFAULT_GEOREF_ANCHOR = {"lat_deg": 35.0, "lon_deg": -116.0, "h_m": 0.0}


def _normalize_at_a_glance(raw: Any, summary: dict[str, Any] | None = None) -> dict[str, Any]:
    """Normalize comprehension at_a_glance to bundle object shape: {cards, summary?}."""

    cards: list[dict[str, str]] = []
    summary_out: dict[str, Any] = dict(summary) if isinstance(summary, dict) else {}

    if isinstance(raw, list):
        for item in raw:
            if isinstance(item, dict) and item.get("label") is not None:
                cards.append({"label": str(item["label"]), "value": str(item.get("value", ""))})
    elif isinstance(raw, dict):
        if isinstance(raw.get("cards"), list):
            for item in raw["cards"]:
                if isinstance(item, dict) and item.get("label") is not None:
                    cards.append({"label": str(item["label"]), "value": str(item.get("value", ""))})
        elif raw.get("cards") is None and raw.get("summary") is None:
            summary_out = {**summary_out, **{k: v for k, v in raw.items() if k != "cards"}}
        if isinstance(raw.get("summary"), dict):
            summary_out = {**summary_out, **raw["summary"]}

    return {"cards": cards, "summary": summary_out}


def _read_json(path: Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"unreadable JSON: {path}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"expected JSON object: {path}")
    return data


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8")


def _governance_block() -> dict[str, Any]:
    return {
        "schema_version": BUNDLE_SCHEMA_VERSION,
        "notice": NON_AUTHORITATIVE_NOTICE,
        "constraints": [
            "evaluation-side only",
            "additive-only",
            "parser-safe",
            "replay-safe",
            "non-authoritative",
            "read_only_viewer",
            "static-rendered",
        ],
        "anti_claims": [
            "not operational readiness evidence",
            "not a tactical authority surface",
            "not a live dashboard",
            "not deployed geography",
            "not governance approval",
        ],
    }


def _relativize_repo_path(path: Path) -> str:
    """Store lineage paths relative to repo root when possible (portable fixtures)."""
    try:
        return path.resolve().relative_to(_REPO_ROOT).as_posix()
    except ValueError:
        return str(path.resolve())


def _resolve_lineage_path(raw: str) -> Path:
    p = Path(raw)
    if p.is_absolute():
        return p
    return (_REPO_ROOT / p).resolve()


def _normalize_lineage_paths(lineage: dict[str, Any]) -> dict[str, Any]:
    out = dict(lineage)
    for key in ("log_path", "meta_path"):
        raw = out.get(key)
        if not raw:
            continue
        path = Path(str(raw))
        if path.is_file():
            out[key] = _relativize_repo_path(path)
        elif not path.is_absolute():
            resolved = _resolve_lineage_path(str(raw))
            if resolved.is_file():
                out[key] = _relativize_repo_path(resolved)
    return out


def _copy_lineage(narrative: dict[str, Any], observability: dict[str, Any] | None) -> dict[str, Any]:
    lineage = dict(narrative.get("lineage") or {})
    if observability and isinstance(observability.get("bundle"), dict):
        bundle_lineage = observability["bundle"].get("lineage") or {}
        if isinstance(bundle_lineage, dict):
            for key in ("log_path", "meta_path", "run_id", "seed", "seed_source", "cohort", "git_commit", "git_dirty"):
                if key not in lineage and key in bundle_lineage:
                    lineage[key] = bundle_lineage[key]
    return _normalize_lineage_paths(lineage)


def _append_sample(bucket: dict[str, list[dict[str, Any]]], track_id: str, sample: dict[str, Any]) -> None:
    bucket.setdefault(track_id, []).append(sample)


def _parse_tracks_from_log(log_path: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Build time-indexed track samples and sparse telemetry from log."""
    if not log_path.is_file():
        return [], []

    threat_by_id: dict[str, list[dict[str, Any]]] = {}
    interceptor_by_id: dict[str, list[dict[str, Any]]] = {}
    telemetry: list[dict[str, Any]] = []
    last_metrics_interceptor_id = _DEFAULT_INTERCEPTOR_ID
    line_index = 0

    for raw_line in log_path.read_text(encoding="utf-8", errors="replace").splitlines():
        line_index += 1
        s = _strip_ros_prefix(raw_line)

        heat = _P_HEATMAP_THREAT_RE.search(s)
        if heat:
            tid = heat.group("threat_id") or _DEFAULT_THREAT_ID
            _append_sample(
                threat_by_id,
                str(tid),
                {
                    "t": line_index,
                    "x_m": round(float(heat.group("x")), 3),
                    "y_m": round(float(heat.group("y")), 3),
                    "z_m": round(float(heat.group("z")), 3),
                    "source": "log_evidence",
                },
            )
            continue
        heat_legacy = _P_HEATMAP_POS_RE.search(s)
        if heat_legacy:
            _append_sample(
                threat_by_id,
                _DEFAULT_THREAT_ID,
                {
                    "t": line_index,
                    "x_m": round(float(heat_legacy.group(1)), 3),
                    "y_m": round(float(heat_legacy.group(2)), 3),
                    "z_m": round(float(heat_legacy.group(3)), 3),
                    "source": "log_evidence",
                },
            )
            continue

        guide = _GUIDANCE_TRACKS_RE.search(s)
        if guide:
            iid = guide.group("interceptor_id") or last_metrics_interceptor_id
            tid = guide.group("threat_id") or _DEFAULT_THREAT_ID
            _append_sample(
                interceptor_by_id,
                str(iid),
                {
                    "t": line_index,
                    "x_m": round(float(guide.group("ix")), 3),
                    "y_m": round(float(guide.group("iy")), 3),
                    "z_m": round(float(guide.group("iz")), 3),
                    "source": "log_evidence",
                },
            )
            _append_sample(
                threat_by_id,
                str(tid),
                {
                    "t": line_index,
                    "x_m": round(float(guide.group("tx")), 3),
                    "y_m": round(float(guide.group("ty")), 3),
                    "z_m": round(float(guide.group("tz")), 3),
                    "source": "log_evidence",
                },
            )
            continue
        guide_legacy = _GUIDANCE_POS_RE.search(s)
        if guide_legacy:
            _append_sample(
                interceptor_by_id,
                last_metrics_interceptor_id,
                {
                    "t": line_index,
                    "x_m": round(float(guide_legacy.group(1)), 3),
                    "y_m": round(float(guide_legacy.group(2)), 3),
                    "z_m": round(float(guide_legacy.group(3)), 3),
                    "source": "log_evidence",
                },
            )
            _append_sample(
                threat_by_id,
                _DEFAULT_THREAT_ID,
                {
                    "t": line_index,
                    "x_m": round(float(guide_legacy.group(4)), 3),
                    "y_m": round(float(guide_legacy.group(5)), 3),
                    "z_m": round(float(guide_legacy.group(6)), 3),
                    "source": "log_evidence",
                },
            )
            continue

        metrics = _METRICS_ROW_RE.search(s)
        if metrics:
            last_metrics_interceptor_id = str(metrics.group("iid"))
            t_go_raw = metrics.group("tgo")
            telemetry.append(
                {
                    "t": line_index,
                    "interceptor_id": last_metrics_interceptor_id,
                    "dist_m": float(metrics.group("dist")),
                    "t_go_s": None if t_go_raw == "n/a" else float(t_go_raw),
                    "vel_m_s": float(metrics.group("vel")),
                    "source": "log_evidence",
                }
            )

    tracks: list[dict[str, Any]] = []
    caveat = "Log-evidenced samples only; not continuous path truth."
    for tid in sorted(threat_by_id):
        tracks.append(
            {
                "track_id": tid,
                "role": "threat",
                "samples": threat_by_id[tid],
                "style": {"polyline": "dashed", "authoritative": False},
                "interpretation_caveat": caveat,
            }
        )
    for iid in sorted(interceptor_by_id):
        tracks.append(
            {
                "track_id": iid,
                "role": "interceptor",
                "samples": interceptor_by_id[iid],
                "style": {"polyline": "dashed", "authoritative": False},
                "interpretation_caveat": caveat,
            }
        )
    return tracks, telemetry


def _clock_from_narrative(narrative: dict[str, Any], tracks: list[dict[str, Any]]) -> dict[str, Any]:
    line_indices: list[int] = []
    for event in narrative.get("events") or []:
        if isinstance(event.get("line_index"), int):
            line_indices.append(int(event["line_index"]))
    for track in tracks:
        for sample in track.get("samples") or []:
            if isinstance(sample.get("t"), int):
                line_indices.append(int(sample["t"]))
    for window in narrative.get("windows") or []:
        for key in ("start_line_index", "end_line_index"):
            if isinstance(window.get(key), int):
                line_indices.append(int(window[key]))

    if line_indices:
        start, end = min(line_indices), max(line_indices)
    else:
        start, end = 0, 0

    markers: list[dict[str, Any]] = []
    for event in narrative.get("events") or []:
        if event.get("line_index") is None:
            continue
        markers.append(
            {
                "t": int(event["line_index"]),
                "event_id": event.get("event_id"),
                "label": event.get("label"),
                "category": event.get("category"),
            }
        )
    for window in narrative.get("windows") or []:
        wtype = window.get("window_type")
        if isinstance(window.get("start_line_index"), int):
            markers.append(
                {
                    "t": int(window["start_line_index"]),
                    "event_id": window.get("start_event_id"),
                    "label": str(wtype),
                    "category": "window",
                }
            )

    markers.sort(key=lambda m: (m.get("t", 0), str(m.get("event_id") or "")))
    return {
        "domain": "log_line_index",
        "duration": {"start": start, "end": end, "step": 1},
        "markers": markers,
    }


def _default_static_entities() -> list[dict[str, Any]]:
    return [
        {
            "entity_id": "radar_01",
            "kind": "radar",
            "position_enu_m": [0.0, 0.0, 10.0],
            "label": "Radar site (replay fixture)",
            "authoritative": False,
        },
        {
            "entity_id": "eoir_01",
            "kind": "eoir",
            "position_enu_m": [200.0, 100.0, 15.0],
            "label": "EO/IR site (replay fixture)",
            "authoritative": False,
        },
        {
            "entity_id": "passive_rf_01",
            "kind": "passive_rf",
            "position_enu_m": [-150.0, 200.0, 8.0],
            "label": "Passive RF sensor (replay fixture)",
            "authoritative": False,
        },
        {
            "entity_id": "fusion_center",
            "kind": "fusion_center",
            "position_enu_m": [0.0, 0.0, 0.0],
            "label": "Fusion center (replay fixture)",
            "authoritative": False,
        },
        {
            "entity_id": "int_base_01",
            "kind": "interceptor_base",
            "position_enu_m": [0.0, 0.0, 0.0],
            "label": "Interceptor base INT-01 (replay fixture)",
            "authoritative": False,
        },
        {
            "entity_id": "int_base_02",
            "kind": "interceptor_base",
            "position_enu_m": [300.0, -100.0, 0.0],
            "label": "Interceptor base INT-02 (replay fixture)",
            "authoritative": False,
        },
    ]


def _default_annotations() -> list[dict[str, Any]]:
    return [
        {
            "annotation_id": "anno_ridge_occlusion",
            "kind": "los_blockage",
            "title": "Ridge occlusion (explanatory)",
            "body": "Track confirmation delayed by ridge occlusion — replay-local association, not causal proof.",
            "linked_event_ids": [],
        },
        {
            "annotation_id": "anno_tti_selection",
            "kind": "tti_explanation",
            "title": "TTI selection (explanatory)",
            "body": "INT-02 selected due to lowest feasible TTI in replay selection evidence — not tactical authority.",
            "linked_event_ids": [],
        },
        {
            "annotation_id": "anno_terrain_lost",
            "kind": "los_blockage",
            "title": "Terrain loss (explanatory)",
            "body": "Target temporarily lost behind terrain — localized near fragmented-gap window; not readiness status.",
            "linked_event_ids": [],
        },
    ]


def _bookmarks_from_windows(narrative: dict[str, Any]) -> list[dict[str, Any]]:
    bookmarks: list[dict[str, Any]] = []
    for idx, window in enumerate(narrative.get("windows") or []):
        if not isinstance(window.get("start_line_index"), int):
            continue
        bookmarks.append(
            {
                "bookmark_id": f"bookmark_{idx:03d}",
                "t": int(window["start_line_index"]),
                "label": str(window.get("window_type") or "window"),
                "event_id": window.get("start_event_id"),
            }
        )
    return bookmarks


def _scenario_pack_rel_path(pack_dir: Path | None) -> str | None:
    if pack_dir is None:
        return None
    try:
        return pack_dir.resolve().relative_to(_REPO_ROOT).as_posix()
    except ValueError:
        return str(pack_dir.resolve())


def build_replay_sa_bundle(
    narrative: dict[str, Any],
    *,
    observability: dict[str, Any] | None = None,
    viz_manifest: dict[str, Any] | None = None,
    scenario_overlay_path: Path | None = None,
    scenario_pack_path: Path | None = None,
    scenario_id: str = "gazebo_single_hostile_km",
    scenario_title: str | None = None,
    sweep_context: dict[str, Any] | None = None,
) -> dict[str, Any]:
    if narrative.get("artifact_type") != "replay_narrative_report":
        raise ValueError("expected artifact_type replay_narrative_report")
    if narrative.get("narrative_schema_version") != "replay_narrative_v1":
        raise ValueError("expected narrative_schema_version replay_narrative_v1")

    lineage = _copy_lineage(narrative, observability)
    log_raw = lineage.get("log_path")
    log_path = _resolve_lineage_path(str(log_raw)) if log_raw else Path("")
    meta_raw = lineage.get("meta_path")
    meta_path = _resolve_lineage_path(str(meta_raw)) if meta_raw else None

    tracks, telemetry = _parse_tracks_from_log(log_path)
    clock = _clock_from_narrative(narrative, tracks)
    launch_geom = parse_launch_args_from_meta(meta_path if meta_path and meta_path.is_file() else None)
    launch_xy = _parse_launch_geometry(meta_path, None)
    if launch_xy and "target_start_x_m" not in launch_geom:
        launch_geom["target_start_x_m"] = launch_xy[0]
        launch_geom["target_start_y_m"] = launch_xy[1]

    topology_path = scenario_pack_path or scenario_overlay_path
    overlay = load_scenario_topology(topology_path)
    scenario_pack_rel = _scenario_pack_rel_path(scenario_pack_path)
    entities = list(overlay.get("entities_static") or _default_static_entities())
    zones = list(overlay.get("zones") or build_dome_zones((0.0, 0.0, 0.0)))
    overlays = list(overlay.get("overlays") or [])
    annotations = list(overlay.get("annotations") or _default_annotations())
    los_segments = build_los_segments(entities, tracks, overlays)
    terrain_model = overlay.get("terrain_model")
    if terrain_model is None and overlay.get("include_fictional_terrain"):
        terrain_model = build_fictional_heightmap()

    summary = narrative.get("summary") if isinstance(narrative.get("summary"), dict) else {}
    comprehension = {}
    if viz_manifest and isinstance(viz_manifest.get("comprehension"), dict):
        comprehension = viz_manifest["comprehension"]
    elif isinstance(narrative.get("summary"), dict):
        comprehension = {
            "headline": summary.get("run_id"),
            "scan_guide": [],
            "at_a_glance": _normalize_at_a_glance(None, summary),
        }

    run_id = summary.get("run_id") or lineage.get("run_id") or "unknown_run"
    title = scenario_title or overlay.get("title") or f"Replay: {run_id}"

    linked_tracks = [t["track_id"] for t in tracks]
    views = {
        "radar_mock": {"sweep_sector_deg": 120, "linked_track_ids": linked_tracks},
        "eoir_mock": {"fov_deg": 45, "thumbnail_frames": []},
        "onboard_mock": {
            "camera_id": "int_cam_0",
            "frames": [],
            "caveat": "Recorded replay frame mock — not live video.",
        },
    }

    scenario_block: dict[str, Any] = {
        "scenario_id": overlay.get("scenario_id") or scenario_id,
        "title": title,
        "topology_tags": list(overlay.get("topology_tags") or ["ridge_defense", "protected_area"]),
        "launch_geometry": launch_geom,
    }
    for key in (
        "replay_tags",
        "ingress_archetype",
        "overlay_descriptors",
        "ambiguity_profile",
        "terrain_profile",
        "narrative_focus",
        "provenance",
    ):
        if overlay.get(key) is not None:
            scenario_block[key] = overlay[key]
    clock_span = int(clock["duration"]["end"]) - int(clock["duration"]["start"]) + 1
    duration_class = overlay.get("replay_duration_class") or replay_duration_class_from_span(clock_span)
    scenario_block["replay_duration_class"] = duration_class
    if terrain_model:
        scenario_block["terrain_model"] = terrain_model
    pack_dir_name = str(overlay.get("_pack_dir_name") or "")
    pack_id = overlay.get("scenario_pack_id") or overlay.get("scenario_id")
    if scenario_pack_rel and pack_id:
        scenario_block["scenario_pack_id"] = str(pack_id)
    if pack_dir_name:
        scenario_block["catalog_pack_id"] = pack_dir_name

    source_artifacts: dict[str, Any] = {
        "observability_artifact_type": observability.get("artifact_type") if observability else None,
        "narrative_schema_version": narrative.get("narrative_schema_version"),
        "visualization_schema_version": (
            viz_manifest.get("visualization_schema_version") if viz_manifest else None
        ),
        "embedded": True,
    }
    if scenario_pack_rel:
        source_artifacts["scenario_pack"] = scenario_pack_rel

    pack_meta = overlay.get("_scenario_pack_metadata")
    if isinstance(pack_meta, dict):
        source_artifacts["scenario_pack_metadata"] = {
            "provenance": pack_meta.get("provenance"),
            "governance": pack_meta.get("governance"),
        }

    catalog_key = pack_dir_name or (Path(scenario_pack_rel).name if scenario_pack_rel else "")
    comparison_hints: dict[str, Any] = {
        "topology_key": catalog_key or str(scenario_block.get("scenario_id") or ""),
        "scenario_id": str(scenario_block.get("scenario_id") or ""),
        "catalog_entry_id": catalog_key or str(scenario_block.get("scenario_id") or ""),
        "duration_class": duration_class,
        "comparison_ready": bool(catalog_key),
    }
    if entities:
        comparison_hints["sensor_layout_id"] = compute_sensor_layout_id(
            {"entities_static": entities}
        )
    if isinstance(pack_meta, dict):
        prov = pack_meta.get("provenance") or {}
        baseline = prov.get("baseline_pack_id")
        if baseline:
            comparison_hints["compare_mode"] = "sensor_study"
            comparison_hints["baseline_topology_key"] = str(baseline)
            comparison_hints["paired_topology_key"] = catalog_key or str(baseline)
    if lineage.get("seed") is not None:
        comparison_hints["seed"] = lineage.get("seed")
    if sweep_context:
        for key in ("sweep_id", "member_index", "sweep_variant_id"):
            if sweep_context.get(key) is not None:
                comparison_hints[key] = sweep_context[key]

    bundle_body: dict[str, Any] = {
        "artifact_type": "replay_sa_bundle",
        "bundle_schema_version": BUNDLE_SCHEMA_VERSION,
        "mode": "replay_static",
        "governance": _governance_block(),
        "lineage": lineage,
        "scenario": scenario_block,
        "comparison_hints": comparison_hints,
        "georef_display": {
            "frame": "scenario_enu",
            "origin_enu_m": [0.0, 0.0, 0.0],
            "anchor": dict(overlay.get("georef_anchor") or DEFAULT_GEOREF_ANCHOR),
            "caveat": "Fictional georef for visualization only — not deployed geography.",
        },
        "clock": clock,
        "tracks": tracks,
        "entities_static": entities,
        "zones": zones,
        "overlays": overlays,
        "los_segments": los_segments,
        "narrative": {
            "events": list(narrative.get("events") or []),
            "windows": list(narrative.get("windows") or []),
            "bookmarks": _bookmarks_from_windows(narrative),
            "annotations": annotations,
        },
        "comprehension": {
            "scan_guide": list(comprehension.get("scan_guide") or []),
            "headline": comprehension.get("headline"),
            "at_a_glance": _normalize_at_a_glance(
                comprehension.get("at_a_glance"),
                summary if isinstance(summary, dict) else None,
            ),
        },
        "source_artifacts": source_artifacts,
        "views": views,
        "panels": {
            "telemetry_series": telemetry,
            "threat_assessment": [
                {
                    "t": clock["duration"]["end"],
                    "label": str(summary.get("selection_oracle_divergence_class") or "unknown"),
                    "caveat": "Replay-local taxonomy label only — not threat score or WEZ validity.",
                }
            ],
        },
        "interpretation_caveats": [
            "SA replay bundle is an explanatory visualization layer, not a unified authoritative replay state.",
            "Map geometry uses scenario-local ENU with fictional georef display anchor.",
            "Dashed trajectories connect log-evidenced samples only.",
            "LOS segments and masking overlays are replay-local topology proxies — not sensor physics.",
        ],
    }
    if sweep_context:
        try:
            from aggregate_spatial_analytics import accumulate_member_layers  # noqa: WPS433

            grid = {
                "origin_enu_m": [-2500.0, -500.0],
                "spacing_m": 100.0,
                "size": [40, 30],
            }
            layers = accumulate_member_layers(bundle_body, grid)
            bundle_body["spatial_analytics"] = {
                "grid": grid,
                "layers": {
                    "ambiguity_density": {
                        "counts": layers["ambiguity_density"],
                        "caveat": (
                            "Replay-side spatial concentration for reviewer exploration only — "
                            "not operational prediction or validated probability."
                        ),
                    },
                    "los_degraded": {
                        "counts": layers["los_degraded"],
                        "caveat": (
                            "Replay-side spatial concentration for reviewer exploration only — "
                            "not operational prediction or validated probability."
                        ),
                    },
                },
            }
        except Exception:
            pass
    try:
        from build_replay_presentation import enrich_bundle_presentation  # noqa: WPS433

        bundle_body = enrich_bundle_presentation(bundle_body)
    except Exception:
        pass
    return bundle_body


def lint_replay_sa_bundle(payload: dict[str, Any]) -> dict[str, Any]:
    issues: list[str] = []
    warnings: list[str] = []
    if payload.get("artifact_type") != "replay_sa_bundle":
        issues.append("missing or unexpected artifact_type")
    if payload.get("bundle_schema_version") != BUNDLE_SCHEMA_VERSION:
        issues.append("missing or unexpected bundle_schema_version")
    if payload.get("mode") != "replay_static":
        issues.append("mode must be replay_static")
    governance = payload.get("governance")
    if not isinstance(governance, dict):
        issues.append("missing governance block")
    else:
        notice = str(governance.get("notice") or "")
        if "Derived evaluation artifact" not in notice:
            issues.append("governance notice must identify derived evaluation artifact status")
    prohibited = {"command", "engage", "websocket_url", "ros_topic_live", "readiness_score", "authority_state"}
    found = prohibited.intersection(payload.keys())
    if found:
        issues.append(f"prohibited top-level fields: {sorted(found)}")
    georef = payload.get("georef_display")
    if isinstance(georef, dict) and "caveat" not in georef:
        issues.append("georef_display must include caveat")
    comprehension = payload.get("comprehension")
    if isinstance(comprehension, dict):
        glance = comprehension.get("at_a_glance")
        if glance is not None and not isinstance(glance, dict):
            issues.append("comprehension.at_a_glance must be an object with cards and summary")
        elif isinstance(glance, dict):
            if not isinstance(glance.get("cards"), list):
                issues.append("comprehension.at_a_glance.cards must be a list")
            if "summary" in glance and not isinstance(glance.get("summary"), dict):
                issues.append("comprehension.at_a_glance.summary must be an object when present")
    for idx, seg in enumerate(payload.get("los_segments") or []):
        if not isinstance(seg, dict):
            issues.append(f"los_segments[{idx}] must be an object")
            continue
        if not seg.get("caveat"):
            issues.append(f"los_segments[{idx}] must include caveat")
        status = seg.get("status")
        if status not in ("visible", "partially_occluded", "terrain_blocked"):
            issues.append(f"los_segments[{idx}] has invalid status: {status}")
    clock = payload.get("clock") or {}
    duration = clock.get("duration") or {}
    log_start = duration.get("start")
    log_end = duration.get("end")
    if log_start is not None and log_end is not None:
        ls, le = int(log_start), int(log_end)
        for idx, ov in enumerate(payload.get("overlays") or []):
            if not isinstance(ov, dict):
                continue
            tr = ov.get("active_t_range")
            if not tr or len(tr) != 2:
                continue
            a, b = int(tr[0]), int(tr[1])
            if a < ls or b > le:
                warnings.append(
                    f"overlays[{idx}] active_t_range [{a},{b}] exceeds log span [{ls},{le}]"
                )
    issues.extend(lint_rt_tactical_replay_continuity(payload))
    return {"ok": len(issues) == 0, "issues": issues, "warnings": warnings}


def pack_bundle(
    *,
    narrative_json: Path,
    observability_json: Path | None = None,
    viz_manifest_json: Path | None = None,
    scenario_overlay_json: Path | None = None,
    scenario_pack_dir: Path | None = None,
    rt_capture_staging_dir: Path | None = None,
    out_dir: Path,
) -> dict[str, Any]:
    if scenario_overlay_json and scenario_pack_dir:
        raise ValueError("--scenario-overlay and --scenario-pack are mutually exclusive")
    narrative = _read_json(narrative_json)
    observability = _read_json(observability_json) if observability_json else None
    viz_manifest = _read_json(viz_manifest_json) if viz_manifest_json else None

    bundle = build_replay_sa_bundle(
        narrative,
        observability=observability,
        viz_manifest=viz_manifest,
        scenario_overlay_path=scenario_overlay_json,
        scenario_pack_path=scenario_pack_dir,
    )
    pack_id = (bundle.get("scenario") or {}).get("catalog_pack_id")
    if pack_id:
        from replay_corpus_lineage import attach_corpus_ref  # noqa: E402

        bundle = attach_corpus_ref(bundle, "demo_bundle", str(pack_id))
    lint = lint_replay_sa_bundle(bundle)
    if not lint["ok"]:
        raise ValueError(f"bundle governance lint failed: {lint['issues']}")

    out_dir.mkdir(parents=True, exist_ok=True)
    if rt_capture_staging_dir is not None:
        attach_rt_tactical_replay_continuity(
            bundle,
            capture_staging_dir=rt_capture_staging_dir,
            repo_root=_REPO_ROOT,
            out_dir=out_dir,
        )
    _write_json(out_dir / "index.json", bundle)

    if viz_manifest:
        figures = viz_manifest.get("figures") or []
        assets_dir = out_dir / "assets"
        assets_dir.mkdir(exist_ok=True)
        for fig in figures:
            if not isinstance(fig, dict):
                continue
            src = fig.get("path")
            if not src:
                continue
            src_path = Path(str(src))
            if src_path.is_file():
                dest = assets_dir / src_path.name
                if not dest.exists():
                    shutil.copy2(src_path, dest)
    return bundle


def export_portable_zip(bundle_dir: Path, out_zip: Path) -> None:
    index = bundle_dir / "index.json"
    if not index.is_file():
        raise ValueError(f"missing bundle index: {index}")
    out_zip.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(out_zip, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for path in sorted(bundle_dir.rglob("*")):
            if path.is_file():
                zf.write(path, path.relative_to(bundle_dir).as_posix())


def _cmd_pack(args: argparse.Namespace) -> None:
    if args.scenario_overlay and args.scenario_pack:
        raise SystemExit("error: --scenario-overlay and --scenario-pack are mutually exclusive")
    pack_bundle(
        narrative_json=Path(args.narrative_json),
        observability_json=Path(args.observability_json) if args.observability_json else None,
        viz_manifest_json=Path(args.viz_manifest_json) if args.viz_manifest_json else None,
        scenario_overlay_json=Path(args.scenario_overlay) if args.scenario_overlay else None,
        scenario_pack_dir=Path(args.scenario_pack) if args.scenario_pack else None,
        rt_capture_staging_dir=Path(args.rt_capture_staging) if args.rt_capture_staging else None,
        out_dir=Path(args.out_dir),
    )
    print(f"wrote {args.out_dir}/index.json")


def _cmd_lint(args: argparse.Namespace) -> None:
    payload = _read_json(Path(args.bundle_json))
    result = lint_replay_sa_bundle(payload)
    print(json.dumps(result, indent=2))
    if not result["ok"]:
        raise SystemExit(1)


def _cmd_export_portable(args: argparse.Namespace) -> None:
    export_portable_zip(Path(args.bundle_dir), Path(args.out_zip))
    print(f"wrote {args.out_zip}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Pack replay artifacts into replay_sa_bundle_v1.")
    sub = parser.add_subparsers(dest="command", required=True)

    pack = sub.add_parser("pack", help="Build index.json bundle directory.")
    pack.add_argument("--narrative-json", required=True)
    pack.add_argument("--observability-json", default=None)
    pack.add_argument("--viz-manifest-json", default=None)
    pack.add_argument("--scenario-overlay", default=None, help="Legacy monolithic scenario_overlay.json")
    pack.add_argument("--scenario-pack", default=None, help="scenario_topology_v1 pack directory")
    pack.add_argument("--out-dir", required=True)
    pack.add_argument(
        "--rt-capture-staging",
        default=None,
        help="RT capture staging dir (candidate.json + tactical_annex) for PLAT-RT-SA3 embed",
    )
    pack.set_defaults(func=_cmd_pack)

    lint = sub.add_parser("lint", help="Lint a bundle index.json.")
    lint.add_argument("bundle_json")
    lint.set_defaults(func=_cmd_lint)

    export = sub.add_parser("export-portable", help="Zip a bundle directory.")
    export.add_argument("--bundle-dir", required=True)
    export.add_argument("--out-zip", required=True)
    export.set_defaults(func=_cmd_export_portable)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
