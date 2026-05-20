#!/usr/bin/env python3
"""Generate D2 Monte Carlo sweep fixtures (four families)."""

from __future__ import annotations

import copy
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
from replay_mc_sweep import build_sweep_manifest, build_sweeps_index, write_sweep  # noqa: E402

SWEEPS_ROOT = _REPO / "fixtures" / "sa_r0" / "sweeps"
PUBLIC_SWEEPS = _REPO / "platform" / "sa-r0-viewer" / "public" / "demo" / "sweeps"

NARRATIVE_BY_PACK: dict[str, str] = {
    "ridge_defense": "replay_narrative_minimal.json",
    "valley_ingress": "replay_narrative_minimal.json",
    "multi_ridge": "replay_narrative_multi_ridge.json",
    "corridor_defense": "replay_narrative_corridor_defense.json",
    "saturation_ingress": "replay_narrative_saturation_ingress.json",
    "delayed_detection": "replay_narrative_delayed_detection.json",
    "valley_ingress_radar_shifted_north": "replay_narrative_minimal.json",
    "valley_ingress_extra_valley_sensor": "replay_narrative_minimal.json",
    "valley_ingress_reduced_overlap_layout": "replay_narrative_minimal.json",
    "valley_ingress_delayed_interceptor_base": "replay_narrative_minimal.json",
}


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8")


def _public_member_url(sweep_id: str, member_id: str) -> str:
    return f"/demo/sweeps/{sweep_id}/members/{member_id}/index.json"


def _pack_member_bundle(
    pack_id: str,
    member_dir: Path,
    *,
    log_pack: str | None = None,
    sweep_id: str,
    member_index: int,
    member_id: str,
    seed: int,
) -> Path:
    member_dir.mkdir(parents=True, exist_ok=True)
    local_log = member_dir / "demo.log"
    local_meta = member_dir / "demo.meta.json"
    if not local_log.is_file():
        log_source = log_pack or pack_id
        log_path = _REPO / "fixtures/sa_r0" / f"demo_{log_source}" / "demo.log"
        meta_path = _REPO / "fixtures/sa_r0" / f"demo_{log_source}" / "demo.meta.json"
        if not log_path.is_file():
            raise SystemExit(f"missing log: {log_path}")
        shutil.copy2(log_path, local_log)
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        meta = copy.deepcopy(meta)
        meta.setdefault("evaluation", {})["cohort"] = sweep_id
        if "noise_seed" not in str(meta.get("cmd") or ""):
            meta["noise_seed"] = seed
        _write_json(local_meta, meta)
    else:
        meta = json.loads(local_meta.read_text(encoding="utf-8"))

    narr_name = NARRATIVE_BY_PACK.get(pack_id, "replay_narrative_minimal.json")
    narrative = json.loads(
        (_REPO / "src/counter_uas/test/fixtures" / narr_name).read_text(encoding="utf-8")
    )
    narrative = copy.deepcopy(narrative)
    narrative.setdefault("lineage", {})["seed"] = seed
    narrative["lineage"]["log_path"] = str(local_log.relative_to(_REPO))
    narrative["lineage"]["meta_path"] = str(local_meta.relative_to(_REPO))

    pack_dir = _REPO / "fixtures/scenarios" / pack_id
    single = obs.build_single_run_report(local_log, meta_path=local_meta)
    manifest = viz.build_visualization_manifest(narrative, observability=single)
    metadata = json.loads((pack_dir / "metadata.json").read_text(encoding="utf-8"))
    bundle = sa.build_replay_sa_bundle(
        narrative,
        observability=single,
        viz_manifest=manifest,
        scenario_pack_path=pack_dir,
        scenario_title=str(metadata.get("title") or ""),
        sweep_context={
            "sweep_id": sweep_id,
            "member_index": member_index,
            "sweep_variant_id": member_id,
        },
    )
    out = member_dir / "index.json"
    _write_json(out, bundle)
    return out


def _sync_sweep_public(sweep_id: str) -> None:
    src = SWEEPS_ROOT / sweep_id
    dst = PUBLIC_SWEEPS / sweep_id
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst)


def _build_valley_sensor_sweep() -> dict[str, Any]:
    sweep_id = "valley_sensor_sweep"
    spec = [
        ("seed_9200_baseline", "valley_ingress", 9200, None),
        ("seed_9201_radar_shifted", "valley_ingress_radar_shifted_north", 9201, "valley_ingress"),
        ("seed_9202_extra_sensor", "valley_ingress_extra_valley_sensor", 9202, "valley_ingress"),
        ("seed_9203_reduced_overlap", "valley_ingress_reduced_overlap_layout", 9203, "valley_ingress"),
        ("seed_9204_delayed_base", "valley_ingress_delayed_interceptor_base", 9204, "valley_ingress"),
    ]
    members = []
    paths = []
    for idx, (mid, pack_id, seed, log_pack) in enumerate(spec):
        mdir = SWEEPS_ROOT / sweep_id / "members" / mid
        paths.append(
            _pack_member_bundle(
                pack_id,
                mdir,
                log_pack=log_pack,
                sweep_id=sweep_id,
                member_index=idx,
                member_id=mid,
                seed=seed,
            )
        )
        members.append(
            {
                "member_id": mid,
                "seed": seed,
                "pack_id": pack_id,
                "demo_bundle_url": _public_member_url(sweep_id, mid),
                "comparison_hints": {"sweep_id": sweep_id, "member_index": idx},
            }
        )
    manifest = build_sweep_manifest(
        sweep_id=sweep_id,
        sweep_kind="sensor_placement_sweep",
        title="Valley sensor placement sweep (replay family)",
        baseline_topology_key="valley_ingress",
        members=members,
        bundle_paths=paths,
        experiment_tags=["valley", "sensor_placement"],
        topology_linkage={
            "shared_log_ref": "valley_ingress",
            "topology_keys": [m["pack_id"] for m in members],
        },
        seed_base=9200,
    )
    write_sweep(SWEEPS_ROOT / sweep_id / "sweep.json", manifest)
    _sync_sweep_public(sweep_id)
    return manifest


def _build_ridge_overlap_sweep() -> dict[str, Any]:
    sweep_id = "ridge_overlap_sweep"
    spec = [
        ("seed_9210_ridge_defense", "ridge_defense", 9210),
        ("seed_9211_multi_ridge", "multi_ridge", 9211),
        ("seed_9212_valley_ingress", "valley_ingress", 9212),
        ("seed_9213_corridor_defense", "corridor_defense", 9213),
    ]
    members = []
    paths = []
    for idx, (mid, pack_id, seed) in enumerate(spec):
        mdir = SWEEPS_ROOT / sweep_id / "members" / mid
        paths.append(
            _pack_member_bundle(
                pack_id,
                mdir,
                sweep_id=sweep_id,
                member_index=idx,
                member_id=mid,
                seed=seed,
            )
        )
        members.append(
            {
                "member_id": mid,
                "seed": seed,
                "pack_id": pack_id,
                "demo_bundle_url": _public_member_url(sweep_id, mid),
                "comparison_hints": {"sweep_id": sweep_id, "member_index": idx},
            }
        )
    manifest = build_sweep_manifest(
        sweep_id=sweep_id,
        sweep_kind="topology_sweep",
        title="Ridge overlap topology sweep (replay family)",
        baseline_topology_key="ridge_defense",
        members=members,
        bundle_paths=paths,
        experiment_tags=["ridge", "topology_sweep"],
        topology_linkage={"topology_keys": [m["pack_id"] for m in members]},
        seed_base=9210,
    )
    write_sweep(SWEEPS_ROOT / sweep_id / "sweep.json", manifest)
    _sync_sweep_public(sweep_id)
    return manifest


def _build_delayed_detection_sweep() -> dict[str, Any]:
    sweep_id = "delayed_detection_sweep"
    base_log = _REPO / "fixtures/sa_r0/demo_delayed_detection/demo.log"
    lines = base_log.read_text(encoding="utf-8").splitlines()
    spec = [
        ("seed_9220_baseline", "delayed_detection", 9220, 0),
        ("seed_9221_stagger_early", "delayed_detection", 9221, 5),
        ("seed_9222_stagger_mid", "delayed_detection", 9222, 12),
        ("seed_9223_stagger_late", "delayed_detection", 9223, 20),
    ]
    members = []
    paths = []
    for idx, (mid, pack_id, seed, heatmap_offset) in enumerate(spec):
        mdir = SWEEPS_ROOT / sweep_id / "members" / mid
        mdir.mkdir(parents=True, exist_ok=True)
        variant_lines = list(lines)
        for i, li in enumerate(variant_lines):
            if "[P_HEATMAP]" in li and heatmap_offset:
                variant_lines[i] = li.replace(
                    "pos=(-820.000",
                    f"pos=(-{820 + heatmap_offset * 15}.000",
                )
        (mdir / "demo.log").write_text("\n".join(variant_lines) + "\n", encoding="utf-8")
        meta_src = _REPO / "fixtures/sa_r0/demo_delayed_detection/demo.meta.json"
        meta = json.loads(meta_src.read_text(encoding="utf-8"))
        meta = copy.deepcopy(meta)
        meta.setdefault("evaluation", {})["cohort"] = sweep_id
        _write_json(mdir / "demo.meta.json", meta)

        paths.append(
            _pack_member_bundle(
                pack_id,
                mdir,
                log_pack=None,
                sweep_id=sweep_id,
                member_index=idx,
                member_id=mid,
                seed=seed,
            )
        )
        members.append(
            {
                "member_id": mid,
                "seed": seed,
                "pack_id": pack_id,
                "demo_bundle_url": _public_member_url(sweep_id, mid),
                "comparison_hints": {"sweep_id": sweep_id, "member_index": idx},
            }
        )
    manifest = build_sweep_manifest(
        sweep_id=sweep_id,
        sweep_kind="ingress_variation",
        title="Delayed detection ingress variation sweep",
        baseline_topology_key="delayed_detection",
        members=members,
        bundle_paths=paths,
        experiment_tags=["detection_timing", "ingress_variation"],
        topology_linkage={"topology_keys": ["delayed_detection"]},
        seed_base=9220,
    )
    write_sweep(SWEEPS_ROOT / sweep_id / "sweep.json", manifest)
    _sync_sweep_public(sweep_id)
    return manifest


def _build_saturation_assignment_sweep() -> dict[str, Any]:
    sweep_id = "saturation_assignment_sweep"
    spec = [
        ("seed_9230_saturation", "saturation_ingress", 9230),
        ("seed_9231_corridor", "corridor_defense", 9231),
        ("seed_9232_saturation_b", "saturation_ingress", 9232),
        ("seed_9233_corridor_b", "corridor_defense", 9233),
    ]
    members = []
    paths = []
    for idx, (mid, pack_id, seed) in enumerate(spec):
        mdir = SWEEPS_ROOT / sweep_id / "members" / mid
        paths.append(
            _pack_member_bundle(
                pack_id,
                mdir,
                sweep_id=sweep_id,
                member_index=idx,
                member_id=mid,
                seed=seed,
            )
        )
        members.append(
            {
                "member_id": mid,
                "seed": seed,
                "pack_id": pack_id,
                "demo_bundle_url": _public_member_url(sweep_id, mid),
                "comparison_hints": {"sweep_id": sweep_id, "member_index": idx},
            }
        )
    manifest = build_sweep_manifest(
        sweep_id=sweep_id,
        sweep_kind="matched_seed",
        title="Saturation assignment matched-seed sweep",
        baseline_topology_key="saturation_ingress",
        members=members,
        bundle_paths=paths,
        experiment_tags=["multi_threat", "matched_seed"],
        topology_linkage={"topology_keys": ["saturation_ingress", "corridor_defense"]},
        seed_base=9230,
    )
    write_sweep(SWEEPS_ROOT / sweep_id / "sweep.json", manifest)
    report = {
        "artifact_type": "matched_seed_comparison_report",
        "governance": {
            "notice": "Synthetic fixture for sweep review — not statistical superiority.",
            "constraints": ["Explanatory only"],
        },
        "summary": {"paired_seed_count": 2, "bucket_counts": {"descriptive": 2}},
        "paired_seeds": [
            {"seed": 9230, "baseline": {"success": False}, "candidate": {"success": False}},
            {"seed": 9231, "baseline": {"success": False}, "candidate": {"success": False}},
        ],
        "interpretation_caveats": [
            "Matched seeds support descriptive comparison, not statistical superiority claims.",
        ],
    }
    _write_json(SWEEPS_ROOT / sweep_id / "matched_seed_report.json", report)
    _sync_sweep_public(sweep_id)
    return manifest


def write_sweeps_index_file(manifests: list[dict[str, Any]]) -> None:
    entries = []
    for m in manifests:
        entries.append(
            {
                "sweep_id": m["sweep_id"],
                "title": m["title"],
                "sweep_kind": m["sweep_kind"],
                "member_count": len(m.get("members") or []),
                "experiment_tags": m.get("experiment_tags") or [],
                "baseline_topology_key": m["baseline_topology_key"],
                "sweep_manifest_url": f"/demo/sweeps/{m['sweep_id']}/sweep.json",
            }
        )
    index = build_sweeps_index(entries)
    _write_json(_REPO / "fixtures/scenarios/sweeps_index_v1.json", index)
    _write_json(_REPO / "platform/sa-r0-viewer/public/demo/sweeps_index.json", index)


def main() -> None:
    for pack_id in (
        "ridge_defense",
        "valley_ingress",
        "multi_ridge",
        "corridor_defense",
        "saturation_ingress",
        "delayed_detection",
        "valley_ingress_radar_shifted_north",
        "valley_ingress_extra_valley_sensor",
        "valley_ingress_reduced_overlap_layout",
        "valley_ingress_delayed_interceptor_base",
    ):
        result = scenario_mod.lint_scenario_pack(_REPO / "fixtures/scenarios" / pack_id)
        if not result["ok"]:
            raise SystemExit(f"lint failed: {pack_id}")
    manifests = [
        _build_valley_sensor_sweep(),
        _build_ridge_overlap_sweep(),
        _build_delayed_detection_sweep(),
        _build_saturation_assignment_sweep(),
    ]
    write_sweeps_index_file(manifests)
    print(f"D2 sweeps generated: {len(manifests)} families.")


if __name__ == "__main__":
    main()
