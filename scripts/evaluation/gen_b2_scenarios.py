#!/usr/bin/env python3
"""One-shot generator for B2 scenario packs, demo logs, narratives, and SA bundles."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

_REPO = Path(__file__).resolve().parents[2]
_EVAL = _REPO / "scripts" / "evaluation"
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_observability as obs  # noqa: E402
import replay_sa_bundle as sa  # noqa: E402
import replay_static_visualization as viz  # noqa: E402

_GOVERNANCE = {
    "notice": "Fictional scenario topology for replay visualization only.",
    "constraints": ["replay-safe", "explanatory-only", "non-authoritative", "parser-safe"],
    "anti_claims": [
        "not deployed geography",
        "not operational effectiveness",
        "not tactical authority",
    ],
}

_MINIMAL_NARRATIVE = json.loads(
    (_REPO / "src/counter_uas/test/fixtures/replay_narrative_minimal.json").read_text()
)


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8")


def _meta(run_id: str, start_x: float, start_y: float, seed: int = 101) -> dict[str, Any]:
    return {
        "run_id": run_id,
        "cohort": "sa_b2_demo",
        "cmd": [
            "ros2",
            "launch",
            "counter_uas",
            "bringup.launch.py",
            f"target_start_x_m:={start_x}",
            f"target_start_y_m:={start_y}",
            "target_start_z_m:=120.0",
            f"noise_seed:={seed}",
        ],
    }


def _narrative_for(run_id: str, log_rel: str, meta_rel: str, line_map: dict[str, int]) -> dict[str, Any]:
    n = json.loads(json.dumps(_MINIMAL_NARRATIVE))
    n["lineage"]["run_id"] = run_id
    n["lineage"]["log_path"] = log_rel
    n["lineage"]["meta_path"] = meta_rel
    n["lineage"]["cohort"] = "sa_b2_demo"
    n["summary"]["run_id"] = run_id
    n["summary"]["canonical_parser_visible_summary"]["run_id"] = run_id
    for ev in n["events"]:
        eid = ev.get("event_id", "")
        for key, line in line_map.items():
            if key in eid and line is not None:
                ev["line_index"] = line
                ev["time_label"] = f"line {line}"
                if ev.get("source_ref"):
                    ev["source_ref"]["line_index"] = line
    for win in n.get("windows") or []:
        if win.get("window_type") == "ambiguity_fragmented_gap_open":
            win["start_line_index"] = line_map.get("gap_start", 8)
    return n


def _pack_bundle(pack_id: str, scenario_id: str, title: str, tags: list[str], log_lines: list[str], run_id: str,
                  line_map: dict[str, int], start_x: float, start_y: float) -> None:
    pack_dir = _REPO / "fixtures" / "scenarios" / pack_id
    demo_dir = _REPO / "fixtures" / "sa_r0" / f"demo_{pack_id}"
    public_dir = _REPO / "platform" / "sa-r0-viewer" / "public" / "demo" / pack_id

    demo_dir.mkdir(parents=True, exist_ok=True)
    log_path = demo_dir / "demo.log"
    meta_path = demo_dir / "demo.meta.json"
    header = f"=== run_id: {run_id} ==="
    log_path.write_text(header + "\n" + "\n".join(log_lines) + "\n", encoding="utf-8")
    _write_json(meta_path, _meta(run_id, start_x, start_y))

    log_rel = str(log_path.relative_to(_REPO))
    meta_rel = str(meta_path.relative_to(_REPO))
    narr_path = _REPO / "src/counter_uas/test/fixtures" / f"replay_narrative_{pack_id}.json"
    narrative = _narrative_for(run_id, log_rel, meta_rel, line_map)
    _write_json(narr_path, narrative)

    single = obs.build_single_run_report(log_path, meta_path=meta_path)
    manifest = viz.build_visualization_manifest(narrative, observability=single)
    bundle = sa.build_replay_sa_bundle(
        narrative,
        observability=single,
        viz_manifest=manifest,
        scenario_pack_path=pack_dir,
        scenario_title=title,
    )
    _write_json(demo_dir / "index.json", bundle)
    public_dir.mkdir(parents=True, exist_ok=True)
    _write_json(public_dir / "index.json", bundle)

    readme = demo_dir / "README.md"
    readme.write_text(
        f"# Demo {pack_id}\n\nTopology: `fixtures/scenarios/{pack_id}/`.\n\n"
        f"Regenerate: `python3 scripts/evaluation/gen_b2_scenarios.py`\n",
        encoding="utf-8",
    )


# --- Pack file writers called before bundle gen ---

def write_multi_ridge_pack() -> None:
    d = _REPO / "fixtures/scenarios/multi_ridge"
    _write_json(d / "metadata.json", {
        "artifact_type": "scenario_topology_v1",
        "schema_version": "scenario_topology_v1",
        "scenario_id": "multi_ridge_demo",
        "title": "Multi-ridge chained masking replay",
        "topology_tags": ["multi_ridge", "chained_masking", "reacquisition", "intermittent_visibility"],
        "ingress_archetype": "open",
        "governance": _GOVERNANCE,
        "provenance": {"fixture_source": "fixtures/scenarios/multi_ridge", "fictional_disclaimer": "Fictional replay fixture."},
    })
    _write_json(d / "topology.json", {
        "georef_anchor": {"lat_deg": 35.2, "lon_deg": -116.2, "h_m": 0.0},
        "entities_static": [
            {"entity_id": "radar_01", "kind": "radar", "position_enu_m": [0.0, 0.0, 15.0], "label": "Radar west", "authoritative": False},
            {"entity_id": "radar_02", "kind": "radar", "position_enu_m": [400.0, 200.0, 15.0], "label": "Radar east", "authoritative": False},
            {"entity_id": "fusion_center", "kind": "fusion_center", "position_enu_m": [200.0, 100.0, 0.0], "label": "Fusion center", "authoritative": False},
            {"entity_id": "int_base_01", "kind": "interceptor_base", "position_enu_m": [200.0, 100.0, 0.0], "label": "INT-01", "authoritative": False},
        ],
        "zones": [
            {"zone_id": "protected_core", "kind": "protected", "display_label": "Protected core", "geometry": {"type": "circle", "center_enu_m": [200.0, 100.0, 0.0], "radius_m": 550.0}, "caveat": "Scenario policy overlay only."},
            {"zone_id": "threat_risk_east", "kind": "threat_risk", "display_label": "Threat risk (east approach)", "geometry": {"type": "circle", "center_enu_m": [-300.0, 150.0, 0.0], "radius_m": 400.0}, "caveat": "Explanatory risk localization only."},
        ],
    })
    _write_json(d / "overlays.json", {
        "overlays": [
            {"overlay_id": "ridge_mask_west", "kind": "ridge_mask", "geometry": {"type": "polygon", "vertices_enu_m": [[-900, -300, 0], [-500, -250, 0], [-450, 150, 0], [-850, 200, 0]], "ridge_outline_enu_m": [[-800, -100, 45], [-600, 50, 55]]}, "caveat": "West ridge — explanatory only.", "active_t_range": [4, 14]},
            {"overlay_id": "ridge_mask_mid", "kind": "ridge_mask", "geometry": {"type": "polygon", "vertices_enu_m": [[-550, -180, 0], [-250, -140, 0], [-200, 200, 0], [-500, 240, 0]], "ridge_outline_enu_m": [[-450, 20, 50], [-280, 120, 48]]}, "caveat": "Mid ridge chain — not terrain truth.", "active_t_range": [10, 22]},
            {"overlay_id": "ridge_mask_east", "kind": "ridge_mask", "geometry": {"type": "polygon", "vertices_enu_m": [[-350, -120, 0], [50, -80, 0], [80, 260, 0], [-320, 300, 0]], "ridge_outline_enu_m": [[-280, 40, 42], [-80, 150, 52]]}, "caveat": "East ridge — replay localization.", "active_t_range": [16, 32]},
            {"overlay_id": "los_blocked_west", "kind": "los_blocked", "geometry": {"type": "polygon", "vertices_enu_m": [[-750, -80, 0], [-520, -60, 0], [-500, 120, 0], [-730, 100, 0]]}, "caveat": "LOS blocked west pocket.", "active_t_range": [6, 12], "linked_event_ids": ["narrative_event_0003_ambiguity_realism_fragmented_gap_start"]},
            {"overlay_id": "los_blocked_east", "kind": "los_blocked", "geometry": {"type": "polygon", "vertices_enu_m": [[-320, 40, 0], [-120, 60, 0], [-100, 220, 0], [-300, 200, 0]]}, "caveat": "LOS blocked east pocket.", "active_t_range": [18, 26], "linked_event_ids": ["narrative_event_0003_ambiguity_realism_fragmented_gap_start"]},
            {"overlay_id": "degraded_reacquire", "kind": "degraded_visibility", "geometry": {"type": "polygon", "vertices_enu_m": [[-600, 80, 0], [-400, 100, 0], [-380, 280, 0], [-580, 260, 0]]}, "caveat": "Staggered visibility degradation.", "active_t_range": [12, 20]},
        ],
    })
    _write_json(d / "annotations.json", {
        "annotations": [
            {"annotation_id": "anno_west_mask", "kind": "los_blockage", "title": "West ridge mask", "body": "First ridge masks threat track — replay-local association only.", "linked_event_ids": ["narrative_event_0003_ambiguity_realism_fragmented_gap_start"]},
            {"annotation_id": "anno_mid_chain", "kind": "los_blockage", "title": "Chained ridge transition", "body": "Visibility fragments as threat crosses mid ridge — not causal proof.", "linked_event_ids": ["narrative_event_0003_ambiguity_realism_fragmented_gap_start"]},
            {"annotation_id": "anno_delayed_reacq", "kind": "los_blockage", "title": "Delayed reacquisition", "body": "Track reacquired after east ridge gap — explanatory evidence only.", "linked_event_ids": ["narrative_event_0003_ambiguity_realism_fragmented_gap_start"]},
            {"annotation_id": "anno_ambiguity_esc", "kind": "visibility_degraded", "title": "Spatial ambiguity escalation", "body": "Overlapping mask windows increase replay interpretation difficulty.", "linked_event_ids": ["narrative_event_0003_ambiguity_realism_fragmented_gap_start"]},
            {"annotation_id": "anno_selection_post_gap", "kind": "tti_explanation", "title": "Post-gap selection", "body": "Selection evidence after reacquisition — not tactical authority.", "linked_event_ids": ["narrative_event_0002_selection_selection_block"]},
        ],
    })
    _write_json(d / "terrain.json", {"include_fictional_terrain": True})
    (d / "README.md").write_text("# multi_ridge\n\nChained ridge masking and reacquisition stress fixture (B2).\n", encoding="utf-8")

    lines = []
    for i, x in enumerate(range(-1400, -400, 35)):
        lines.append(f"[interception_logic_node-1] [P_HEATMAP] pos=({float(x):.3f}, {80 + (i % 5) * 8:.3f}, {90 + (i % 3) * 4:.3f})")
    lines.insert(8, "[tracking_node-1] Candidate detected: track_id=9")
    lines.insert(9, "[noisy_measurement_node-1] [REALISM_EVENT] fragmented_gap_start fragmentation_index=1 gap_ticks=4")
    lines.insert(18, "[noisy_measurement_node-1] [REALISM_EVENT] fragmented_gap_end fragmentation_index=1")
    lines.insert(19, "[noisy_measurement_node-1] [REALISM_EVENT] fragmented_gap_start fragmentation_index=2 gap_ticks=3")
    lines.insert(28, "[noisy_measurement_node-1] [REALISM_EVENT] fragmented_gap_end fragmentation_index=2")
    lines.append("[interception_logic_node-1] interceptor_pos=(-60.000, 8.000, 42.000) target_pos=(-450.000, 120.000, 85.000)")
    lines.append("[interception_logic_node-1] [METRICS] id=interceptor_0  | dist=22.000 m | t_go=5.0 s | vel=17.0 m/s | mode=predict")
    _pack_bundle("multi_ridge", "multi_ridge_demo", "Multi-ridge chained masking replay", ["multi_ridge"],
                 lines, "replay_multi_ridge_demo", {"gap_start": 10, "detection": 9, "selection": 38}, -1400.0, 80.0)


def write_corridor_defense_pack() -> None:
    d = _REPO / "fixtures/scenarios/corridor_defense"
    _write_json(d / "metadata.json", {
        "artifact_type": "scenario_topology_v1",
        "schema_version": "scenario_topology_v1",
        "scenario_id": "corridor_defense_demo",
        "title": "Corridor defense narrow ingress replay",
        "topology_tags": ["corridor_defense", "narrow_corridor", "overlapping_radar", "timing_pressure"],
        "ingress_archetype": "corridor",
        "governance": _GOVERNANCE,
        "provenance": {"fixture_source": "fixtures/scenarios/corridor_defense", "fictional_disclaimer": "Fictional replay fixture."},
    })
    _write_json(d / "topology.json", {
        "georef_anchor": {"lat_deg": 35.05, "lon_deg": -116.05, "h_m": 0.0},
        "entities_static": [
            {"entity_id": "radar_north", "kind": "radar", "position_enu_m": [0.0, 300.0, 14.0], "label": "Radar north", "authoritative": False},
            {"entity_id": "radar_south", "kind": "radar", "position_enu_m": [0.0, -300.0, 14.0], "label": "Radar south", "authoritative": False},
            {"entity_id": "eoir_01", "kind": "eoir", "position_enu_m": [150.0, 0.0, 18.0], "label": "EO/IR corridor watch", "authoritative": False},
            {"entity_id": "int_base_01", "kind": "interceptor_base", "position_enu_m": [0.0, 0.0, 0.0], "label": "INT-01", "authoritative": False},
        ],
        "zones": [
            {"zone_id": "protected_corridor", "kind": "protected", "display_label": "Protected corridor core", "geometry": {"type": "circle", "center_enu_m": [0.0, 0.0, 0.0], "radius_m": 280.0}, "caveat": "Narrow protected corridor — policy overlay only."},
        ],
    })
    _write_json(d / "overlays.json", {
        "overlays": [
            {"overlay_id": "ingress_corridor_narrow", "kind": "ingress_corridor", "geometry": {"type": "polygon", "vertices_enu_m": [[-1600, -60, 0], [-200, -80, 0], [-180, 80, 0], [-1580, 60, 0]]}, "caveat": "Constrained ingress corridor.", "active_t_range": [0, 22]},
            {"overlay_id": "los_blocked_corridor_flank", "kind": "los_blocked", "geometry": {"type": "polygon", "vertices_enu_m": [[-1200, 100, 0], [-600, 120, 0], [-580, 380, 0], [-1180, 360, 0]]}, "caveat": "Flank LOS blockage.", "active_t_range": [4, 18]},
            {"overlay_id": "los_blocked_corridor_flank_s", "kind": "los_blocked", "geometry": {"type": "polygon", "vertices_enu_m": [[-1200, -380, 0], [-600, -360, 0], [-580, -120, 0], [-1180, -100, 0]]}, "caveat": "South flank blockage.", "active_t_range": [4, 18]},
            {"overlay_id": "degraded_corridor_mid", "kind": "degraded_visibility", "geometry": {"type": "polygon", "vertices_enu_m": [[-900, -40, 0], [-500, -50, 0], [-480, 50, 0], [-880, 40, 0]]}, "caveat": "Corridor mid degradation.", "active_t_range": [8, 16]},
        ],
    })
    _write_json(d / "annotations.json", {
        "annotations": [
            {"annotation_id": "anno_corridor_pressure", "kind": "visibility_degraded", "title": "Corridor pressure", "body": "Ingress compressed into narrow corridor — replay timing stress only.", "linked_event_ids": ["narrative_event_0001_detection_candidate_spawn"]},
            {"annotation_id": "anno_overlapping_los", "kind": "los_blockage", "title": "Overlapping LOS reasoning", "body": "North and south radars see partial flank blockage — not sensor truth.", "linked_event_ids": ["narrative_event_0003_ambiguity_realism_fragmented_gap_start"]},
            {"annotation_id": "anno_compressed_tti", "kind": "tti_explanation", "title": "Compressed intercept timing", "body": "TTI falls rapidly as threat enters protected corridor — not readiness.", "linked_event_ids": ["narrative_event_0002_selection_selection_block"]},
            {"annotation_id": "anno_timing_escalation", "kind": "tti_explanation", "title": "Timing escalation", "body": "Short decision window localized near corridor entry.", "linked_event_ids": ["narrative_event_0002_selection_selection_block"]},
        ],
    })
    lines = []
    for i, x in enumerate(range(-1500, -350, 55)):
        lines.append(f"[interception_logic_node-1] [P_HEATMAP] pos=({float(x):.3f}, {(i % 3 - 1) * 15:.3f}, {100.0:.3f})")
    lines.insert(4, "[tracking_node-1] Candidate detected: track_id=12")
    lines.append("[interception_logic_node-1] [METRICS] id=interceptor_0  | dist=35.000 m | t_go=6.5 s | vel=18.0 m/s | mode=predict")
    lines.append("[interception_logic_node-1] interceptor_pos=(-40.000, 5.000, 40.000) target_pos=(-380.000, 10.000, 95.000)")
    lines.append("[interception_logic_node-1] [METRICS] id=interceptor_0  | dist=18.000 m | t_go=2.8 s | vel=19.0 m/s | mode=predict")
    lines.append("[interception_logic_node-1] [METRICS] id=interceptor_0  | dist=9.500 m | t_go=1.2 s | vel=20.0 m/s | mode=predict")
    _pack_bundle("corridor_defense", "corridor_defense_demo", "Corridor defense narrow ingress replay", ["corridor_defense"],
                 lines, "replay_corridor_defense_demo", {"detection": 5, "gap_start": 8, "selection": 20}, -1500.0, 0.0)


def _urban_blocks() -> list[dict[str, Any]]:
    blocks = []
    specs = [
        ("mask_nw", -400, 320, 120), ("mask_n1", -200, 380, 100), ("mask_ne", 80, 350, 110),
        ("mask_c", -120, 120, 130), ("mask_sw", -350, -80, 90), ("mask_s", -80, -200, 100),
        ("mask_se", 200, -120, 85), ("mask_e", 280, 180, 95),
    ]
    for oid, x0, y0, sz in specs:
        blocks.append({
            "overlay_id": oid,
            "kind": "degraded_visibility",
            "geometry": {"type": "polygon", "vertices_enu_m": [[x0, y0, 0], [x0 + sz, y0, 0], [x0 + sz, y0 + sz, 0], [x0, y0 + sz, 0]]},
            "caveat": "Fictional obstruction block — not urban simulation.",
            "active_t_range": [2, 28],
        })
    return blocks


def write_saturation_ingress_pack() -> None:
    d = _REPO / "fixtures/scenarios/saturation_ingress"
    _write_json(d / "metadata.json", {
        "artifact_type": "scenario_topology_v1",
        "schema_version": "scenario_topology_v1",
        "scenario_id": "saturation_ingress_demo",
        "title": "Saturation ingress multi-threat replay",
        "topology_tags": ["saturation_ingress", "multi_threat", "assignment_ambiguity"],
        "ingress_archetype": "open",
        "governance": _GOVERNANCE,
        "provenance": {"fixture_source": "fixtures/scenarios/saturation_ingress", "fictional_disclaimer": "Fictional replay fixture."},
    })
    _write_json(d / "topology.json", {
        "georef_anchor": {"lat_deg": 35.15, "lon_deg": -116.15, "h_m": 0.0},
        "entities_static": [
            {"entity_id": "radar_01", "kind": "radar", "position_enu_m": [0.0, 0.0, 14.0], "label": "Radar 01", "authoritative": False},
            {"entity_id": "radar_02", "kind": "radar", "position_enu_m": [280.0, -120.0, 14.0], "label": "Radar 02", "authoritative": False},
            {"entity_id": "int_base_01", "kind": "interceptor_base", "position_enu_m": [0.0, 0.0, 0.0], "label": "INT-01", "authoritative": False},
            {"entity_id": "int_base_02", "kind": "interceptor_base", "position_enu_m": [220.0, 60.0, 0.0], "label": "INT-02", "authoritative": False},
            {"entity_id": "int_base_03", "kind": "interceptor_base", "position_enu_m": [-80.0, 180.0, 0.0], "label": "INT-03", "authoritative": False},
        ],
        "zones": [
            {"zone_id": "protected_core", "kind": "protected", "display_label": "Protected core", "geometry": {"type": "circle", "center_enu_m": [100.0, 50.0, 0.0], "radius_m": 450.0}, "caveat": "Policy overlay only."},
        ],
    })
    _write_json(d / "overlays.json", {
        "overlays": [
            {"overlay_id": "ingress_a", "kind": "ingress_corridor", "geometry": {"type": "polygon", "vertices_enu_m": [[-1900, 60, 0], [-1100, 40, 0], [-1080, 200, 0], [-1880, 220, 0]]}, "caveat": "Threat A ingress.", "active_t_range": [0, 30]},
            {"overlay_id": "ingress_b", "kind": "ingress_corridor", "geometry": {"type": "polygon", "vertices_enu_m": [[-1850, -220, 0], [-1050, -200, 0], [-1030, -40, 0], [-1830, -20, 0]]}, "caveat": "Threat B ingress.", "active_t_range": [6, 32]},
            {"overlay_id": "overlap_ambiguity", "kind": "degraded_visibility", "geometry": {"type": "polygon", "vertices_enu_m": [[-1200, -80, 0], [-900, -90, 0], [-880, 120, 0], [-1180, 130, 0]]}, "caveat": "Trajectory overlap pocket.", "active_t_range": [14, 28]},
        ],
    })
    _write_json(d / "annotations.json", {
        "annotations": [
            {"annotation_id": "anno_prioritization", "kind": "tti_explanation", "title": "Prioritization pressure", "body": "Multiple threats create replay prioritization pressure — not assignment authority.", "linked_event_ids": ["narrative_event_0002_selection_selection_block"]},
            {"annotation_id": "anno_assignment_switch", "kind": "tti_explanation", "title": "Assignment switching", "body": "Selection evidence shifts between interceptors — explanatory only.", "linked_event_ids": ["narrative_event_0005_selection_selection_block"]},
            {"annotation_id": "anno_overlap_tracks", "kind": "visibility_degraded", "title": "Overlapping trajectories", "body": "Overlapping ingress paths increase narrative complexity in replay.", "linked_event_ids": ["narrative_event_0003_ambiguity_realism_fragmented_gap_start"]},
            {"annotation_id": "anno_ambiguity_heavy", "kind": "visibility_degraded", "title": "Ambiguity-heavy window", "body": "Concurrent tracks during gap window — not fusion truth.", "linked_event_ids": ["narrative_event_0003_ambiguity_realism_fragmented_gap_start"]},
            {"annotation_id": "anno_oracle_mismatch", "kind": "tti_explanation", "title": "Oracle disagreement", "body": "Replay-local oracle mismatch under saturation — not tactical doctrine.", "linked_event_ids": ["narrative_event_0006_divergence_selection_oracle_mismatch"]},
        ],
    })
    lines = [
        "[interception_logic_node-1] [P_HEATMAP] threat_id=threat_uav_0 pos=(-1800.000, 80.000, 110.000)",
        "[interception_logic_node-1] [P_HEATMAP] threat_id=threat_uav_1 pos=(-1750.000, -120.000, 105.000)",
        "[interception_logic_node-1] [P_HEATMAP] threat_id=threat_uav_0 pos=(-1700.000, 85.000, 108.000)",
        "[interception_logic_node-1] [P_HEATMAP] threat_id=threat_uav_1 pos=(-1650.000, -115.000, 102.000)",
        "[tracking_node-1] Candidate detected: track_id=21",
        "[tracking_node-1] Candidate detected: track_id=22",
        "[noisy_measurement_node-1] [REALISM_EVENT] fragmented_gap_start fragmentation_index=1 gap_ticks=2",
        "[interception_logic_node-1] [METRICS] id=interceptor_0  | dist=42.000 m | t_go=7.5 s | vel=16.0 m/s | mode=predict",
        "[interception_logic_node-1] interceptor_id=interceptor_0 interceptor_pos=(-50.000, 5.000, 40.000) target_pos=(-1600.000, 82.000, 105.000) threat_id=threat_uav_0",
        "[interception_logic_node-1] [METRICS] id=interceptor_1  | dist=48.000 m | t_go=8.2 s | vel=15.5 m/s | mode=predict",
        "[interception_logic_node-1] interceptor_id=interceptor_1 interceptor_pos=(90.000, -15.000, 42.000) target_pos=(-1580.000, -110.000, 100.000) threat_id=threat_uav_1",
        "[interception_logic_node-1] [P_HEATMAP] threat_id=threat_uav_0 pos=(-1500.000, 90.000, 100.000)",
        "[interception_logic_node-1] [P_HEATMAP] threat_id=threat_uav_1 pos=(-1480.000, -105.000, 98.000)",
        "[interception_logic_node-1] [METRICS] id=interceptor_0  | dist=28.000 m | t_go=4.5 s | vel=17.0 m/s | mode=predict",
        "[interception_logic_node-1] [METRICS] id=interceptor_1  | dist=32.000 m | t_go=5.1 s | vel=16.5 m/s | mode=predict",
    ]
    _pack_bundle("saturation_ingress", "saturation_ingress_demo", "Saturation ingress multi-threat replay", ["saturation_ingress"],
                 lines, "replay_saturation_ingress_demo", {"detection": 5, "gap_start": 7, "selection": 9}, -1800.0, 80.0)


def write_urban_masking_pack() -> None:
    d = _REPO / "fixtures/scenarios/urban_masking"
    _write_json(d / "metadata.json", {
        "artifact_type": "scenario_topology_v1",
        "schema_version": "scenario_topology_v1",
        "scenario_id": "urban_masking_demo",
        "title": "Urban masking fictional obstruction replay",
        "topology_tags": ["urban_masking", "fragmented_tracking", "cluttered_replay"],
        "ingress_archetype": "open",
        "governance": _GOVERNANCE,
        "provenance": {"fixture_source": "fixtures/scenarios/urban_masking", "fictional_disclaimer": "Lightweight fictional blocks only — not photorealistic urban simulation."},
    })
    _write_json(d / "topology.json", {
        "georef_anchor": {"lat_deg": 35.25, "lon_deg": -116.25, "h_m": 0.0},
        "entities_static": [
            {"entity_id": "radar_01", "kind": "radar", "position_enu_m": [0.0, 0.0, 16.0], "label": "Radar urban fixture", "authoritative": False},
            {"entity_id": "eoir_01", "kind": "eoir", "position_enu_m": [120.0, 80.0, 20.0], "label": "EO/IR fixture", "authoritative": False},
            {"entity_id": "int_base_01", "kind": "interceptor_base", "position_enu_m": [0.0, 0.0, 0.0], "label": "INT-01", "authoritative": False},
        ],
        "zones": [
            {"zone_id": "threat_risk_clutter", "kind": "threat_risk", "display_label": "Cluttered approach risk", "geometry": {"type": "circle", "center_enu_m": [-200.0, 80.0, 0.0], "radius_m": 350.0}, "caveat": "Explanatory only."},
        ],
    })
    _write_json(d / "overlays.json", {"overlays": _urban_blocks()})
    _write_json(d / "annotations.json", {
        "annotations": [
            {"annotation_id": "anno_unstable_continuity", "kind": "visibility_degraded", "title": "Unstable track continuity", "body": "Fragmented samples through fictional mask blocks — not track robustness proof.", "linked_event_ids": ["narrative_event_0003_ambiguity_realism_fragmented_gap_start"]},
            {"annotation_id": "anno_confidence_deg", "kind": "visibility_degraded", "title": "Confidence degradation", "body": "Replay-local degraded visibility semantics only.", "linked_event_ids": ["narrative_event_0003_ambiguity_realism_fragmented_gap_start"]},
            {"annotation_id": "anno_clutter_interp", "kind": "los_blockage", "title": "Cluttered interpretation", "body": "Dense overlays stress replay readability — not urban geography.", "linked_event_ids": []},
            {"annotation_id": "anno_stale_evidence", "kind": "visibility_degraded", "title": "Stale detection evidence", "body": "Stale detection marker localized — explanatory only.", "linked_event_ids": ["narrative_event_0003_ambiguity_realism_fragmented_gap_start"]},
        ],
    })
    lines = []
    x = -1100.0
    for i in range(22):
        jitter = ((i * 17) % 40) - 20
        lines.append(f"[interception_logic_node-1] [P_HEATMAP] pos=({x + i * 32:.3f}, {60 + jitter:.3f}, {85 + (i % 4):.3f})")
    lines.insert(6, "[tracking_node-1] Candidate detected: track_id=31")
    lines.insert(10, "[noisy_measurement_node-1] [REALISM_EVENT] fragmented_gap_start fragmentation_index=1 gap_ticks=5")
    lines.insert(14, "[noisy_measurement_node-1] [REALISM_EVENT] stale_detection count=1 stale_s=0.200")
    lines.append("[interception_logic_node-1] interceptor_pos=(-55.000, 6.000, 41.000) target_pos=(-350.000, 55.000, 82.000)")
    lines.append("[interception_logic_node-1] [METRICS] id=interceptor_0  | dist=24.000 m | t_go=4.0 s | vel=16.0 m/s | mode=predict")
    _pack_bundle("urban_masking", "urban_masking_demo", "Urban masking fictional obstruction replay", ["urban_masking"],
                 lines, "replay_urban_masking_demo", {"detection": 7, "gap_start": 11}, -1100.0, 60.0)


def write_delayed_detection_pack() -> None:
    d = _REPO / "fixtures/scenarios/delayed_detection"
    _write_json(d / "metadata.json", {
        "artifact_type": "scenario_topology_v1",
        "schema_version": "scenario_topology_v1",
        "scenario_id": "delayed_detection_demo",
        "title": "Delayed detection compressed window replay",
        "topology_tags": ["delayed_detection", "late_acquisition", "urgency"],
        "ingress_archetype": "open",
        "governance": _GOVERNANCE,
        "provenance": {"fixture_source": "fixtures/scenarios/delayed_detection", "fictional_disclaimer": "Fictional replay fixture."},
    })
    _write_json(d / "topology.json", {
        "georef_anchor": {"lat_deg": 35.3, "lon_deg": -116.3, "h_m": 0.0},
        "entities_static": [
            {"entity_id": "radar_01", "kind": "radar", "position_enu_m": [0.0, 0.0, 15.0], "label": "Radar sole site", "authoritative": False},
            {"entity_id": "int_base_01", "kind": "interceptor_base", "position_enu_m": [0.0, 0.0, 0.0], "label": "INT-01", "authoritative": False},
        ],
        "zones": [
            {"zone_id": "protected_tight", "kind": "protected", "display_label": "Tight protected zone", "geometry": {"type": "circle", "center_enu_m": [0.0, 0.0, 0.0], "radius_m": 220.0}, "caveat": "Compressed decision geometry."},
            {"zone_id": "threat_risk_close", "kind": "threat_risk", "display_label": "Close threat risk", "geometry": {"type": "circle", "center_enu_m": [-250.0, 30.0, 0.0], "radius_m": 180.0}, "caveat": "Escalation localization only."},
        ],
    })
    _write_json(d / "overlays.json", {
        "overlays": [
            {"overlay_id": "late_acquisition_zone", "kind": "degraded_visibility", "geometry": {"type": "polygon", "vertices_enu_m": [[-900, -100, 0], [-500, -120, 0], [-480, 80, 0], [-880, 100, 0]]}, "caveat": "Late acquisition pocket.", "active_t_range": [0, 14]},
            {"overlay_id": "protected_escalation", "kind": "ingress_corridor", "geometry": {"type": "polygon", "vertices_enu_m": [[-700, -40, 0], [-150, -50, 0], [-130, 60, 0], [-680, 70, 0]]}, "caveat": "Rapid ingress into protected volume.", "active_t_range": [15, 28]},
        ],
    })
    _write_json(d / "annotations.json", {
        "annotations": [
            {"annotation_id": "anno_late_radar", "kind": "visibility_degraded", "title": "Late radar acquisition", "body": "First heatmap evidence appears late in replay — not sensor specification.", "linked_event_ids": ["narrative_event_0001_detection_candidate_spawn"]},
            {"annotation_id": "anno_short_window", "kind": "tti_explanation", "title": "Short decision window", "body": "Compressed intercept window after late detection — explanatory urgency only.", "linked_event_ids": ["narrative_event_0002_selection_selection_block"]},
            {"annotation_id": "anno_rapid_tti", "kind": "tti_explanation", "title": "Rapid TTI reduction", "body": "TTI falls quickly in METRICS evidence — not operational effectiveness.", "linked_event_ids": ["narrative_event_0002_selection_selection_block"]},
            {"annotation_id": "anno_zone_escalation", "kind": "los_blockage", "title": "Protected-zone escalation", "body": "Threat enters tight protected overlay — policy visualization only.", "linked_event_ids": ["narrative_event_0001_detection_candidate_spawn"]},
        ],
    })
    lines = [f"[interception_logic_node-1] noop warmup line {i}" for i in range(1, 16)]
    lines[14] = "[interception_logic_node-1] [P_HEATMAP] pos=(-820.000, 25.000, 95.000)"
    lines.append("[noisy_measurement_node-1] [REALISM_EVENT] delayed_detection count=1 delay_s=0.250")
    lines.append("[tracking_node-1] Candidate detected: track_id=41")
    for x in range(-780, -200, 40):
        lines.append(f"[interception_logic_node-1] [P_HEATMAP] pos=({float(x):.3f}, 30.000, 90.000)")
    lines.append("[interception_logic_node-1] [METRICS] id=interceptor_0  | dist=45.000 m | t_go=6.0 s | vel=17.0 m/s | mode=predict")
    lines.append("[interception_logic_node-1] interceptor_pos=(-35.000, 4.000, 40.000) target_pos=(-220.000, 32.000, 88.000)")
    lines.append("[interception_logic_node-1] [METRICS] id=interceptor_0  | dist=14.000 m | t_go=1.5 s | vel=19.5 m/s | mode=predict")
    lines.append("[interception_logic_node-1] [METRICS] id=interceptor_0  | dist=6.000 m | t_go=0.8 s | vel=20.0 m/s | mode=predict")
    _pack_bundle("delayed_detection", "delayed_detection_demo", "Delayed detection compressed window replay", ["delayed_detection"],
                 lines, "replay_delayed_detection_demo", {"detection": 17, "selection": 24}, -820.0, 25.0)


def write_long_range_ingress_pack() -> None:
    d = _REPO / "fixtures/scenarios/long_range_ingress"
    _write_json(d / "metadata.json", {
        "artifact_type": "scenario_topology_v1",
        "schema_version": "scenario_topology_v1",
        "scenario_id": "long_range_ingress_demo",
        "title": "Long-range ingress cinematic replay",
        "topology_tags": ["long_range_ingress", "launch_sequence", "terrain_following"],
        "ingress_archetype": "valley",
        "governance": _GOVERNANCE,
        "provenance": {"fixture_source": "fixtures/scenarios/long_range_ingress", "fictional_disclaimer": "Fictional replay fixture."},
    })
    _write_json(d / "topology.json", {
        "georef_anchor": {"lat_deg": 35.35, "lon_deg": -116.35, "h_m": 0.0},
        "entities_static": [
            {"entity_id": "radar_01", "kind": "radar", "position_enu_m": [0.0, 0.0, 14.0], "label": "Radar", "authoritative": False},
            {"entity_id": "int_base_01", "kind": "interceptor_base", "position_enu_m": [0.0, 0.0, 0.0], "label": "INT-01", "authoritative": False},
        ],
        "zones": [
            {"zone_id": "protected_wide", "kind": "protected", "display_label": "Wide protected area", "geometry": {"type": "circle", "center_enu_m": [0.0, 0.0, 0.0], "radius_m": 700.0}, "caveat": "Long-form replay policy overlay."},
        ],
    })
    _write_json(d / "overlays.json", {
        "overlays": [
            {"overlay_id": "long_ingress_corridor", "kind": "ingress_corridor", "geometry": {"type": "polygon", "vertices_enu_m": [[-2400, -150, 0], [-400, -180, 0], [-380, 220, 0], [-2380, 250, 0]]}, "caveat": "Extended ingress path.", "active_t_range": [0, 65]},
            {"overlay_id": "ridge_mask_lr", "kind": "ridge_mask", "geometry": {"type": "polygon", "vertices_enu_m": [[-1600, -100, 0], [-1200, -80, 0], [-1180, 200, 0], [-1580, 220, 0]], "ridge_outline_enu_m": [[-1500, 40, 38], [-1250, 120, 48]]}, "caveat": "Terrain-following ridge mask.", "active_t_range": [30, 50]},
        ],
    })
    _write_json(d / "annotations.json", {
        "annotations": [
            {"annotation_id": "anno_replay_pacing", "kind": "visibility_degraded", "title": "Replay pacing", "body": "Extended timeline stresses long-form replay readability.", "linked_event_ids": ["narrative_event_0001_detection_candidate_spawn"]},
            {"annotation_id": "anno_launch_visible", "kind": "tti_explanation", "title": "Launch progression visible", "body": "Interceptor launch segment visible in guidance samples — not live video.", "linked_event_ids": ["narrative_event_0002_selection_selection_block"]},
            {"annotation_id": "anno_terrain_follow", "kind": "los_blockage", "title": "Terrain-following ingress", "body": "Low-altitude heatmap ingress along fictional corridor.", "linked_event_ids": []},
            {"annotation_id": "anno_spatial_story", "kind": "los_blockage", "title": "Spatial storytelling", "body": "Long trajectory supports topology cognition at multiple clock positions.", "linked_event_ids": []},
        ],
    })
    _write_json(d / "terrain.json", {"include_fictional_terrain": True})
    lines = []
    alt = 70.0
    for i, x in enumerate(range(-2300, -350, 32)):
        alt = 70.0 + (i % 6) * 2.5 + ((i % 3) - 1) * 3.0
        y = 100.0 + ((i % 7) - 3) * 12.0
        lines.append(f"[interception_logic_node-1] [P_HEATMAP] pos=({float(x):.3f}, {y:.3f}, {alt:.3f})")
    lines.insert(12, "[tracking_node-1] Candidate detected: track_id=51")
    lines.insert(35, "[interception_logic_node-1] interceptor_pos=(-85.000, 6.000, 42.000) target_pos=(-1200.000, 95.000, 78.000)")
    lines.insert(50, "[interception_logic_node-1] interceptor_pos=(-55.000, 6.000, 42.000) target_pos=(-800.000, 110.000, 72.000)")
    lines.insert(58, "[interception_logic_node-1] interceptor_pos=(-25.000, 6.000, 42.000) target_pos=(-500.000, 120.000, 68.000)")
    lines.append("[interception_logic_node-1] [METRICS] id=interceptor_0  | dist=30.000 m | t_go=5.5 s | vel=16.0 m/s | mode=predict")
    _pack_bundle("long_range_ingress", "long_range_ingress_demo", "Long-range ingress cinematic replay", ["long_range_ingress"],
                 lines, "replay_long_range_ingress_demo", {"detection": 13, "selection": 36}, -2300.0, 100.0)


def write_catalog() -> None:
    from sync_sa_catalog import write_catalog as _write_catalog  # noqa: PLC0415

    _write_catalog()


def main() -> None:
    write_multi_ridge_pack()
    write_corridor_defense_pack()
    write_saturation_ingress_pack()
    write_urban_masking_pack()
    write_delayed_detection_pack()
    write_long_range_ingress_pack()
    write_catalog()
    print("B2 scenario packs and demos generated.")


if __name__ == "__main__":
    main()
