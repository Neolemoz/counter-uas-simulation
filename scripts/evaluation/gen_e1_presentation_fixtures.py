#!/usr/bin/env python3
"""Generate E1 presentation fixtures, storyboards, and sync public demo."""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path
from typing import Any

_REPO = Path(__file__).resolve().parents[2]
_EVAL = _REPO / "scripts" / "evaluation"
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from build_replay_presentation import enrich_bundle_presentation, enrich_sweep_presentation  # noqa: E402
from export_presentation_pack import export_all_presentation_packs  # noqa: E402
from gen_d3_sweep_enrichment import SWEEP_IDS, enrich_sweep  # noqa: E402

DEMO_ROOT = _REPO / "fixtures" / "sa_r0"
PRESENTATIONS_ROOT = DEMO_ROOT / "presentations"
PUBLIC_DEMO = _REPO / "platform" / "sa-r0-viewer" / "public" / "demo"
PUBLIC_PRESENTATIONS = PUBLIC_DEMO / "presentations"

GOVERNANCE = {
    "notice": "Derived replay presentation artifact — explanatory review only, not operational authority.",
    "anti_claims": [
        "Not deployment readiness or validated effectiveness.",
        "Not causal proof of runtime behavior.",
    ],
}

STORYBOARDS: list[dict[str, Any]] = [
    {
        "artifact_type": "replay_storyboard_v1",
        "schema_version": "replay_storyboard_v1",
        "storyboard_id": "showcase_best_narratives",
        "title": "Showcase: strongest replay narratives",
        "estimated_minutes": 18,
        "governance": GOVERNANCE,
        "scenes": [
            {
                "scene_id": "ridge_intro",
                "label": "Ridge defense sweep",
                "target_url": "?sweep=ridge_overlap_sweep&chapter=0",
                "copy": "Open ridge overlap sweep workstation narrative — explanatory cohort review.",
                "importance_tags": ["topology"],
            },
            {
                "scene_id": "valley_ingress",
                "label": "Valley ingress walkthrough",
                "target_url": "?demo=valley_ingress&walkthrough=1&chapter=0",
                "copy": "Follow deterministic ingress chapters on valley ingress demo bundle.",
                "importance_tags": ["topology", "pacing"],
            },
            {
                "scene_id": "saturation_narrative",
                "label": "Saturation narrative",
                "target_url": "?sweep=saturation_assignment_sweep&chapter=0",
                "copy": "Review saturation assignment sweep narrative headline and cohort patterns.",
                "importance_tags": ["assignment", "ambiguity"],
            },
        ],
    },
    {
        "artifact_type": "replay_storyboard_v1",
        "schema_version": "replay_storyboard_v1",
        "storyboard_id": "showcase_topology_divergence",
        "title": "Showcase: topology-sensitive divergence",
        "estimated_minutes": 15,
        "governance": GOVERNANCE,
        "scenes": [
            {
                "scene_id": "ridge_sweep",
                "label": "Ridge overlap sweep",
                "target_url": "?sweep=ridge_overlap_sweep&presentation=showcase_topology_divergence&chapter=0",
                "copy": "Compare ridge, multi-ridge, valley, and corridor packs in one sweep family.",
                "importance_tags": ["topology"],
            },
            {
                "scene_id": "filmstrip",
                "label": "Cohort filmstrip",
                "target_url": "?sweep=ridge_overlap_sweep&filmstrip=0,1,2",
                "copy": "Sync-clock filmstrip across topology variants — not operational comparison.",
                "importance_tags": ["topology", "pacing"],
            },
        ],
    },
    {
        "artifact_type": "replay_storyboard_v1",
        "schema_version": "replay_storyboard_v1",
        "storyboard_id": "showcase_ambiguity_heavy",
        "title": "Showcase: ambiguity-heavy sweeps",
        "estimated_minutes": 16,
        "governance": GOVERNANCE,
        "scenes": [
            {
                "scene_id": "saturation",
                "label": "Saturation assignment",
                "target_url": "?sweep=saturation_assignment_sweep&presentation=showcase_ambiguity_heavy&chapter=0",
                "copy": "Saturation-driven ambiguity patterns across matched-seed variants.",
                "importance_tags": ["ambiguity", "assignment"],
            },
            {
                "scene_id": "valley_sensor",
                "label": "Valley sensor sweep",
                "target_url": "?sweep=valley_sensor_sweep&chapter=0",
                "copy": "Sensor placement changes amplify ambiguity concentration in valley ingress.",
                "importance_tags": ["ambiguity", "topology"],
            },
        ],
    },
    {
        "artifact_type": "replay_storyboard_v1",
        "schema_version": "replay_storyboard_v1",
        "storyboard_id": "walkthrough_valley_ingress_long",
        "title": "Long-form valley ingress walkthrough",
        "estimated_minutes": 20,
        "governance": GOVERNANCE,
        "scenes": [
            {
                "scene_id": "ingress",
                "label": "Ingress",
                "target_url": "?demo=valley_ingress&presentation=walkthrough_valley_ingress_long&chapter=0",
                "copy": "Chapter 1: ingress geometry and initial detection markers.",
                "chapter": 0,
                "importance_tags": ["topology"],
            },
            {
                "scene_id": "los",
                "label": "LOS degradation",
                "target_url": "?demo=valley_ingress&presentation=walkthrough_valley_ingress_long&chapter=1",
                "copy": "Chapter 2: LOS instability near terrain masking.",
                "chapter": 1,
                "importance_tags": ["los"],
            },
            {
                "scene_id": "ambiguity",
                "label": "Ambiguity",
                "target_url": "?demo=valley_ingress&presentation=walkthrough_valley_ingress_long&chapter=2",
                "copy": "Chapter 3: ambiguity window concentration.",
                "chapter": 2,
                "importance_tags": ["ambiguity"],
            },
            {
                "scene_id": "outcome",
                "label": "Outcome",
                "target_url": "?demo=valley_ingress&presentation=walkthrough_valley_ingress_long&chapter=4",
                "copy": "Final chapter: replay outcome summary.",
                "chapter": 4,
                "importance_tags": ["pacing"],
            },
        ],
    },
    {
        "artifact_type": "replay_storyboard_v1",
        "schema_version": "replay_storyboard_v1",
        "storyboard_id": "walkthrough_assignment_instability",
        "title": "Assignment instability walkthrough",
        "estimated_minutes": 17,
        "governance": GOVERNANCE,
        "scenes": [
            {
                "scene_id": "saturation_overview",
                "label": "Saturation sweep overview",
                "target_url": "?sweep=saturation_assignment_sweep&presentation=walkthrough_assignment_instability&chapter=0",
                "copy": "Review assignment instability narrative across saturation vs corridor packs.",
                "importance_tags": ["assignment"],
            },
            {
                "scene_id": "delayed_stagger",
                "label": "Delayed detection stagger",
                "target_url": "?sweep=delayed_detection_sweep&chapter=0",
                "copy": "Matched-seed stagger variants show detection timing divergence.",
                "importance_tags": ["pacing", "assignment"],
            },
        ],
    },
]


def _enrich_demo_bundles() -> None:
    for path in sorted(DEMO_ROOT.glob("demo_*/index.json")):
        bundle = json.loads(path.read_text(encoding="utf-8"))
        enriched = enrich_bundle_presentation(bundle)
        path.write_text(json.dumps(enriched, indent=2, sort_keys=False) + "\n", encoding="utf-8")
        demo_name = path.parent.name.replace("demo_", "")
        public = PUBLIC_DEMO / demo_name / "index.json"
        if public.parent.exists():
            public.write_text(json.dumps(enriched, indent=2, sort_keys=False) + "\n", encoding="utf-8")
        print(f"enriched bundle {path.parent.name}")


def _write_storyboards() -> None:
    PRESENTATIONS_ROOT.mkdir(parents=True, exist_ok=True)
    PUBLIC_PRESENTATIONS.mkdir(parents=True, exist_ok=True)
    index_rows = []
    for sb in STORYBOARDS:
        sid = sb["storyboard_id"]
        out = PRESENTATIONS_ROOT / f"{sid}.json"
        out.write_text(json.dumps(sb, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        pub = PUBLIC_PRESENTATIONS / f"{sid}.json"
        pub.write_text(json.dumps(sb, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        index_rows.append(
            {
                "storyboard_id": sid,
                "title": sb["title"],
                "estimated_minutes": sb["estimated_minutes"],
                "storyboard_url": f"/demo/presentations/{sid}.json",
            }
        )
    index = {
        "artifact_type": "replay_storyboard_index_v1",
        "schema_version": "replay_storyboard_index_v1",
        "governance": {"notice": GOVERNANCE["notice"]},
        "storyboards": index_rows,
    }
    (PRESENTATIONS_ROOT / "index.json").write_text(
        json.dumps(index, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (PUBLIC_PRESENTATIONS / "index.json").write_text(
        json.dumps(index, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(f"wrote {len(STORYBOARDS)} storyboards")


def main() -> None:
    _enrich_demo_bundles()
    for sweep_id in SWEEP_IDS:
        enrich_sweep(sweep_id)
    _write_storyboards()
    export_all_presentation_packs()
    print("E1 presentation fixtures complete.")


if __name__ == "__main__":
    main()
