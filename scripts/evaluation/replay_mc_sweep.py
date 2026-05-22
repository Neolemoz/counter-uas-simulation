#!/usr/bin/env python3
"""Validate and build replay_mc_sweep_v1 manifests."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from aggregate_spatial_analytics import aggregate_from_bundles, load_bundle

SWEEP_ARTIFACT = "replay_mc_sweep_v1"
SWEEP_KINDS = frozenset(
    {"matched_seed", "topology_sweep", "sensor_placement_sweep", "ingress_variation"}
)

GOVERNANCE_NOTICE = (
    "Deterministic replay sweep for spatial analytics exploration only — "
    "explanatory replay family, not operational planning."
)

ANTI_CLAIMS = [
    "Sweep aggregates are not parser authority.",
    "Spatial layers are replay concentration, not probability or readiness.",
    "No deployment confidence or tactical superiority implied.",
]


def validate_sweep(manifest: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if manifest.get("artifact_type") != SWEEP_ARTIFACT:
        errors.append(f"artifact_type must be {SWEEP_ARTIFACT}")
    if manifest.get("schema_version") != SWEEP_ARTIFACT:
        errors.append(f"schema_version must be {SWEEP_ARTIFACT}")
    for key in ("sweep_id", "sweep_kind", "title", "baseline_topology_key", "members"):
        if not manifest.get(key):
            errors.append(f"missing required field: {key}")
    kind = manifest.get("sweep_kind")
    if kind and kind not in SWEEP_KINDS:
        errors.append(f"invalid sweep_kind: {kind}")
    members = manifest.get("members") or []
    if not members:
        errors.append("members must be non-empty")
    for i, m in enumerate(members):
        for key in ("member_id", "pack_id", "demo_bundle_url"):
            if not m.get(key):
                errors.append(f"member[{i}] missing {key}")
    gov = manifest.get("governance") or {}
    if not gov.get("notice"):
        errors.append("governance.notice required")
    return errors


def build_sweep_manifest(
    *,
    sweep_id: str,
    sweep_kind: str,
    title: str,
    baseline_topology_key: str,
    members: list[dict[str, Any]],
    bundle_paths: list[Path],
    experiment_tags: list[str] | None = None,
    topology_linkage: dict[str, Any] | None = None,
    seed_base: int = 9200,
    generator: str = "gen_d2_sweep_fixtures.py",
) -> dict[str, Any]:
    bundles = [load_bundle(p) for p in bundle_paths]
    agg = aggregate_from_bundles(
        bundles,
        baseline_index=0,
        baseline_topology_key=baseline_topology_key,
    )
    from replay_corpus_lineage import attach_corpus_ref  # noqa: E402

    manifest = {
        "artifact_type": SWEEP_ARTIFACT,
        "schema_version": SWEEP_ARTIFACT,
        "sweep_id": sweep_id,
        "sweep_kind": sweep_kind,
        "title": title,
        "experiment_tags": list(experiment_tags or []),
        "baseline_topology_key": baseline_topology_key,
        "governance": {
            "notice": GOVERNANCE_NOTICE,
            "anti_claims": list(ANTI_CLAIMS),
        },
        "lineage": {
            "generator": generator,
            "seed_base": seed_base,
            "member_count": len(members),
        },
        "topology_linkage": topology_linkage or {},
        "members": members,
        "spatial_aggregate": agg["spatial_aggregate"],
        "replay_aggregation": agg["replay_aggregation"],
    }
    return attach_corpus_ref(manifest, "sweep_family", sweep_id)


def write_sweep(path: Path, manifest: dict[str, Any]) -> None:
    errs = validate_sweep(manifest)
    if errs:
        raise ValueError(f"invalid sweep {path}: {errs}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def build_sweeps_index(entries: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "artifact_type": "scenario_sweeps_index_v1",
        "schema_version": "scenario_sweeps_index_v1",
        "governance": {"notice": GOVERNANCE_NOTICE},
        "sweeps": entries,
    }
