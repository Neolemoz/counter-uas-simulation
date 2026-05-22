#!/usr/bin/env python3
"""Sync scenario catalog, compare pairs, and repack demo bundles (C1b + D1)."""

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

import replay_observability as obs  # noqa: E402
import replay_sa_bundle as sa  # noqa: E402
import replay_sa_scenario as scenario_mod  # noqa: E402
import replay_static_visualization as viz  # noqa: E402

ALL_PACK_IDS = [
    "ridge_defense",
    "valley_ingress",
    "multi_ridge",
    "corridor_defense",
    "saturation_ingress",
    "urban_masking",
    "delayed_detection",
    "long_range_ingress",
]

EXPERIMENT_PACK_IDS = [
    "valley_ingress_radar_shifted_north",
    "valley_ingress_extra_valley_sensor",
    "valley_ingress_reduced_overlap_layout",
    "valley_ingress_delayed_interceptor_base",
]

CATALOG_PACK_IDS = ALL_PACK_IDS + EXPERIMENT_PACK_IDS

NARRATIVE_BY_PACK: dict[str, str] = {
    "ridge_defense": "replay_narrative_minimal.json",
    "valley_ingress": "replay_narrative_minimal.json",
    "multi_ridge": "replay_narrative_multi_ridge.json",
    "corridor_defense": "replay_narrative_corridor_defense.json",
    "saturation_ingress": "replay_narrative_saturation_ingress.json",
    "urban_masking": "replay_narrative_urban_masking.json",
    "delayed_detection": "replay_narrative_delayed_detection.json",
    "long_range_ingress": "replay_narrative_long_range_ingress.json",
}

BASE_LOG_PACK = "valley_ingress"
VALLEY_NARRATIVE = "replay_narrative_minimal.json"


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8")


def demo_bundle_url(pack_id: str) -> str:
    if pack_id == "ridge_defense":
        return "/demo/index.json"
    return f"/demo/{pack_id}/index.json"


def demo_public_dir(pack_id: str) -> Path:
    if pack_id == "ridge_defense":
        return _REPO / "platform/sa-r0-viewer/public/demo"
    return _REPO / "platform/sa-r0-viewer/public/demo" / pack_id


def build_catalog_entry(pack_id: str) -> dict[str, Any]:
    pack_dir = _REPO / "fixtures/scenarios" / pack_id
    metadata = json.loads((pack_dir / "metadata.json").read_text(encoding="utf-8"))
    entry: dict[str, Any] = {
        "pack_id": pack_id,
        "scenario_id": metadata.get("scenario_id"),
        "title": metadata.get("title"),
        "pack_path": f"fixtures/scenarios/{pack_id}",
        "topology_tags": list(metadata.get("topology_tags") or []),
        "replay_tags": list(metadata.get("replay_tags") or []),
        "ingress_archetype": metadata.get("ingress_archetype"),
        "overlay_descriptors": metadata.get("overlay_descriptors"),
        "ambiguity_profile": metadata.get("ambiguity_profile"),
        "replay_duration_class": metadata.get("replay_duration_class"),
        "terrain_profile": metadata.get("terrain_profile"),
        "narrative_focus": list(metadata.get("narrative_focus") or []),
        "category": scenario_mod.catalog_category_for_metadata(metadata),
        "demo_bundle": f"fixtures/sa_r0/demo_{pack_id}/index.json",
        "demo_bundle_url": demo_bundle_url(pack_id),
    }
    prov = metadata.get("provenance") or {}
    if prov.get("baseline_pack_id"):
        entry["baseline_pack_id"] = prov["baseline_pack_id"]
    return entry


def write_catalog() -> None:
    catalog = {
        "artifact_type": "scenario_topology_catalog",
        "schema_version": "scenario_topology_catalog_v1",
        "governance": {
            "notice": "Catalog for portable scenario packs — not authoritative deployment data.",
        },
        "packs": [build_catalog_entry(pid) for pid in CATALOG_PACK_IDS],
    }
    _write_json(_REPO / "fixtures/scenarios/index.json", catalog)
    _write_json(_REPO / "platform/sa-r0-viewer/public/demo/catalog.json", catalog)


def sync_compare_pairs() -> None:
    src = _REPO / "fixtures/scenarios/compare_pairs_v1.json"
    dst = _REPO / "platform/sa-r0-viewer/public/demo/compare_pairs.json"
    shutil.copy2(src, dst)


def validate_compare_pairs() -> None:
    data = json.loads(
        (_REPO / "fixtures/scenarios/compare_pairs_v1.json").read_text(encoding="utf-8")
    )
    valid_ids = set(CATALOG_PACK_IDS)
    for pair in data.get("pairs") or []:
        for slot_key in ("slot_a", "slot_b"):
            slot = pair.get(slot_key) or {}
            pid = slot.get("pack_id")
            if pid not in valid_ids:
                raise SystemExit(f"compare pair {pair.get('pair_id')}: unknown pack_id {pid!r}")


def _pack_narrative(pack_id: str, log_path: Path, meta_path: Path) -> dict[str, Any]:
    narr_name = NARRATIVE_BY_PACK.get(pack_id, VALLEY_NARRATIVE)
    narrative = json.loads(
        (_REPO / "src/counter_uas/test/fixtures" / narr_name).read_text(encoding="utf-8")
    )
    narrative["lineage"]["log_path"] = str(log_path.relative_to(_REPO))
    narrative["lineage"]["meta_path"] = str(meta_path.relative_to(_REPO))
    return narrative


def repack_demo(pack_id: str, *, log_pack: str | None = None) -> None:
    log_source = log_pack or pack_id
    demo_dir = _REPO / "fixtures/sa_r0" / f"demo_{pack_id}"
    demo_dir.mkdir(parents=True, exist_ok=True)
    log_path = _REPO / "fixtures/sa_r0" / f"demo_{log_source}" / "demo.log"
    meta_path = _REPO / "fixtures/sa_r0" / f"demo_{log_source}" / "demo.meta.json"
    if not log_path.is_file():
        raise SystemExit(f"missing log for repack {pack_id}: {log_path}")

    pack_dir = _REPO / "fixtures/scenarios" / pack_id
    narrative = _pack_narrative(pack_id if pack_id in NARRATIVE_BY_PACK else BASE_LOG_PACK, log_path, meta_path)

    single = obs.build_single_run_report(log_path, meta_path=meta_path)
    manifest = viz.build_visualization_manifest(narrative, observability=single)
    metadata = json.loads((pack_dir / "metadata.json").read_text(encoding="utf-8"))
    bundle = sa.build_replay_sa_bundle(
        narrative,
        observability=single,
        viz_manifest=manifest,
        scenario_pack_path=pack_dir,
        scenario_title=str(metadata.get("title") or ""),
    )
    _write_json(demo_dir / "index.json", bundle)
    pub = demo_public_dir(pack_id)
    pub.mkdir(parents=True, exist_ok=True)
    _write_json(pub / "index.json", bundle)


def sync_sweeps_index() -> None:
    src = _REPO / "fixtures/scenarios/sweeps_index_v1.json"
    dst = _REPO / "platform/sa-r0-viewer/public/demo/sweeps_index.json"
    if src.is_file():
        shutil.copy2(src, dst)


def validate_sweeps_index() -> None:
    path = _REPO / "fixtures/scenarios/sweeps_index_v1.json"
    if not path.is_file():
        return
    data = json.loads(path.read_text(encoding="utf-8"))
    valid_ids = set(CATALOG_PACK_IDS)
    for entry in data.get("sweeps") or []:
        sweep_id = entry.get("sweep_id")
        manifest_path = _REPO / "fixtures/sa_r0/sweeps" / str(sweep_id) / "sweep.json"
        if not manifest_path.is_file():
            raise SystemExit(f"sweep manifest missing: {manifest_path}")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        for member in manifest.get("members") or []:
            pid = member.get("pack_id")
            if pid not in valid_ids:
                raise SystemExit(f"sweep {sweep_id}: unknown pack_id {pid!r}")


def check_promotion_policy(*, strict: bool = False) -> list[str]:
    import replay_sa_authoring as authoring  # noqa: E402

    warnings: list[str] = []
    errors: list[str] = []
    for pack_id in CATALOG_PACK_IDS:
        manifest = authoring.load_authoring_manifest(_REPO / "fixtures/scenarios" / pack_id)
        if not manifest:
            continue
        if not authoring.catalog_promotion_ready(pack_id):
            msg = f"{pack_id}: catalog entry has manifest but promotion_status < promoted"
            if strict:
                errors.append(msg)
            else:
                warnings.append(msg)
    return errors if strict else warnings


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Sync scenario catalog and demo bundles.")
    parser.add_argument(
        "--strict-promotion",
        action="store_true",
        help="Fail when authoring manifest exists but pack is not promoted",
    )
    args = parser.parse_args()

    promo_msgs = check_promotion_policy(strict=args.strict_promotion)
    for msg in promo_msgs:
        print(f"WARN: {msg}" if not args.strict_promotion else f"ERROR: {msg}", file=sys.stderr)
    if args.strict_promotion and promo_msgs:
        raise SystemExit(1)

    for pack_id in CATALOG_PACK_IDS:
        result = scenario_mod.lint_scenario_pack(_REPO / "fixtures/scenarios" / pack_id)
        if not result["ok"]:
            raise SystemExit(f"lint failed for {pack_id}: {result}")
    validate_compare_pairs()
    for pack_id in ALL_PACK_IDS:
        repack_demo(pack_id)
    for pack_id in EXPERIMENT_PACK_IDS:
        repack_demo(pack_id, log_pack=BASE_LOG_PACK)
    write_catalog()
    sync_compare_pairs()
    validate_sweeps_index()
    sync_sweeps_index()
    print("Catalog synced, compare pairs copied, demo bundles repacked (D1+D2).")


if __name__ == "__main__":
    main()
