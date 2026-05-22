#!/usr/bin/env python3
"""Generate D1 sensor-placement experiment packs from valley_ingress baseline."""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path
from typing import Any

_REPO = Path(__file__).resolve().parents[2]
_BASE = _REPO / "fixtures/scenarios/valley_ingress"

EXPERIMENTS: dict[str, dict[str, Any]] = {
    "valley_ingress_radar_shifted_north": {
        "title_suffix": "radar shifted north",
        "narrative_focus_extra": ["sensor_placement"],
        "mutate": "radar_shifted_north",
    },
    "valley_ingress_extra_valley_sensor": {
        "title_suffix": "extra valley sensor",
        "narrative_focus_extra": ["sensor_placement"],
        "mutate": "extra_valley_sensor",
    },
    "valley_ingress_reduced_overlap_layout": {
        "title_suffix": "reduced overlap layout",
        "narrative_focus_extra": ["sensor_placement"],
        "mutate": "reduced_overlap",
    },
    "valley_ingress_delayed_interceptor_base": {
        "title_suffix": "delayed interceptor base",
        "narrative_focus_extra": ["sensor_placement"],
        "mutate": "delayed_interceptor_base",
    },
}


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _entity_by_id(entities: list[dict[str, Any]], entity_id: str) -> dict[str, Any] | None:
    for e in entities:
        if e.get("entity_id") == entity_id:
            return e
    return None


def _mutate_topology(topology: dict[str, Any], kind: str) -> dict[str, Any]:
    topo = copy.deepcopy(topology)
    entities = list(topo.get("entities_static") or [])
    zones = list(topo.get("zones") or [])

    if kind == "radar_shifted_north":
        radar = _entity_by_id(entities, "radar_01")
        if radar:
            pos = list(radar.get("position_enu_m") or [0, 0, 0])
            pos[1] = float(pos[1]) + 120.0
            radar["position_enu_m"] = pos
            radar["label"] = "Radar site (shifted north — experiment)"

    elif kind == "extra_valley_sensor":
        entities.append(
            {
                "entity_id": "radar_02",
                "kind": "radar",
                "position_enu_m": [-900.0, 200.0, 18.0],
                "label": "Secondary radar (valley experiment)",
                "authoritative": False,
            }
        )
        entities.append(
            {
                "entity_id": "eoir_02",
                "kind": "eoir",
                "position_enu_m": [-850.0, 180.0, 22.0],
                "label": "Secondary EO/IR (valley experiment)",
                "authoritative": False,
            }
        )

    elif kind == "reduced_overlap":
        for z in zones:
            g = z.get("geometry") or {}
            if g.get("type") == "circle" and z.get("kind") == "protected":
                g["radius_m"] = float(g.get("radius_m") or 500) * 0.75
            if g.get("type") == "circle" and z.get("kind") == "outer_detection":
                center = list(g.get("center_enu_m") or [0, 0, 0])
                center[0] = float(center[0]) + 80.0
                g["center_enu_m"] = center
                g["radius_m"] = float(g.get("radius_m") or 800) * 0.85
        if not zones:
            zones.append(
                {
                    "zone_id": "protected_core_exp",
                    "kind": "protected",
                    "geometry": {
                        "type": "circle",
                        "center_enu_m": [0.0, 0.0, 0.0],
                        "radius_m": 375.0,
                    },
                    "caveat": "Reduced protected zone — experiment fixture.",
                }
            )
        topo["zones"] = zones

    elif kind == "delayed_interceptor_base":
        base = _entity_by_id(entities, "int_base_01")
        if base:
            pos = list(base.get("position_enu_m") or [0, 0, 0])
            pos[0] = float(pos[0]) - 150.0
            pos[1] = float(pos[1]) - 80.0
            base["position_enu_m"] = pos
            base["label"] = "Interceptor base INT-01 (offset — experiment)"

    topo["entities_static"] = entities
    return topo


def build_pack(pack_id: str, spec: dict[str, Any]) -> None:
    out_dir = _REPO / "fixtures/scenarios" / pack_id
    base_meta = _read_json(_BASE / "metadata.json")
    base_topo = _read_json(_BASE / "topology.json")
    base_overlays = _read_json(_BASE / "overlays.json")
    base_annotations = _read_json(_BASE / "annotations.json")

    meta = copy.deepcopy(base_meta)
    meta["scenario_id"] = f"{pack_id}_demo"
    meta["title"] = f"Valley ingress — {spec['title_suffix']} (experiment)"
    tags = list(meta.get("topology_tags") or [])
    for t in ("sensor_experiment", "topology_experiment"):
        if t not in tags:
            tags.append(t)
    meta["topology_tags"] = tags
    replay_tags = list(meta.get("replay_tags") or [])
    if "sensor_placement" not in replay_tags:
        replay_tags.append("sensor_placement")
    meta["replay_tags"] = replay_tags
    focus = list(meta.get("narrative_focus") or [])
    for f in spec.get("narrative_focus_extra") or []:
        if f not in focus:
            focus.append(f)
    meta["narrative_focus"] = focus
    meta["provenance"] = {
        "fictional_disclaimer": "Experiment variant derived from valley_ingress — fictional replay only.",
        "fixture_source": f"fixtures/scenarios/{pack_id}",
        "baseline_pack_id": "valley_ingress",
    }

    topo = _mutate_topology(base_topo, str(spec["mutate"]))

    _write_json(out_dir / "metadata.json", meta)
    _write_json(out_dir / "topology.json", topo)
    _write_json(out_dir / "overlays.json", base_overlays)
    _write_json(out_dir / "annotations.json", base_annotations)
    if (_BASE / "terrain.json").is_file():
        _write_json(out_dir / "terrain.json", _read_json(_BASE / "terrain.json"))

    readme = (
        f"# {pack_id}\n\n"
        "D1 sensor-placement experiment pack. Shares `demo.log` with `valley_ingress`.\n\n"
        f"Regenerate: `python3 scripts/evaluation/gen_d1_experiment_packs.py`\n"
    )
    (out_dir / "README.md").write_text(readme, encoding="utf-8")


def main() -> int:
    for pack_id, spec in EXPERIMENTS.items():
        build_pack(pack_id, spec)
        print(f"Wrote {pack_id}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
