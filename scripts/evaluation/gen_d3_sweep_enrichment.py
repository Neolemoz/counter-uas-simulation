#!/usr/bin/env python3
"""Enrich existing D2 sweep fixtures with D3 narrative intelligence fields."""

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

from replay_mc_sweep import validate_sweep, write_sweep  # noqa: E402
from replay_narrative_intelligence import enrich_sweep_manifest  # noqa: E402

SWEEPS_ROOT = _REPO / "fixtures" / "sa_r0" / "sweeps"
PUBLIC_SWEEPS = _REPO / "platform" / "sa-r0-viewer" / "public" / "demo" / "sweeps"

SWEEP_IDS = (
    "valley_sensor_sweep",
    "ridge_overlap_sweep",
    "delayed_detection_sweep",
    "saturation_assignment_sweep",
)


def _member_bundle_path(sweep_id: str, member: dict[str, Any]) -> Path:
    member_id = member["member_id"]
    return SWEEPS_ROOT / sweep_id / "members" / member_id / "index.json"


def _sync_sweep_public(sweep_id: str) -> None:
    src = SWEEPS_ROOT / sweep_id
    dst = PUBLIC_SWEEPS / sweep_id
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst)


def enrich_sweep(sweep_id: str) -> None:
    path = SWEEPS_ROOT / sweep_id / "sweep.json"
    manifest = json.loads(path.read_text(encoding="utf-8"))
    bundle_paths = [_member_bundle_path(sweep_id, m) for m in manifest.get("members") or []]
    enriched = enrich_sweep_manifest(manifest, bundle_paths)
    enriched.setdefault("lineage", {})["generator"] = "gen_d3_sweep_enrichment.py"
    write_sweep(path, enriched)
    _sync_sweep_public(sweep_id)
    print(f"enriched {sweep_id}")


def main() -> None:
    for sweep_id in SWEEP_IDS:
        enrich_sweep(sweep_id)
    print("D3 sweep enrichment complete.")


if __name__ == "__main__":
    main()
