from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_eval_module(name: str, rel: str):
    eval_dir = str(_REPO_ROOT / "scripts" / "evaluation")
    if eval_dir not in sys.path:
        sys.path.insert(0, eval_dir)
    path = _REPO_ROOT / rel
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_validate_sweep_manifests():
    mc = _load_eval_module("replay_mc_sweep", "scripts/evaluation/replay_mc_sweep.py")
    sweeps_dir = _REPO_ROOT / "fixtures/sa_r0/sweeps"
    assert sweeps_dir.is_dir()
    ids = []
    for sweep_json in sorted(sweeps_dir.glob("*/sweep.json")):
        manifest = json.loads(sweep_json.read_text(encoding="utf-8"))
        errs = mc.validate_sweep(manifest)
        assert errs == [], f"{sweep_json}: {errs}"
        if manifest.get("replay_narrative_summary"):
            assert manifest["replay_narrative_summary"].get("bullets")
        ids.append(manifest["sweep_id"])
    assert len(ids) >= 4


def test_aggregate_deterministic_hash():
    agg = _load_eval_module(
        "aggregate_spatial_analytics", "scripts/evaluation/aggregate_spatial_analytics.py"
    )
    bundle_path = (
        _REPO_ROOT
        / "fixtures/sa_r0/sweeps/valley_sensor_sweep/members/seed_9200_baseline/index.json"
    )
    if not bundle_path.is_file():
        return
    bundle = agg.load_bundle(bundle_path)
    r1 = agg.aggregate_from_bundles([bundle], baseline_topology_key="valley_ingress")
    r2 = agg.aggregate_from_bundles([bundle], baseline_topology_key="valley_ingress")
    assert r1 == r2


def test_sweeps_index_synced():
    src = _REPO_ROOT / "fixtures/scenarios/sweeps_index_v1.json"
    pub = _REPO_ROOT / "platform/sa-r0-viewer/public/demo/sweeps_index.json"
    if not src.is_file():
        return
    assert pub.is_file()
    assert json.loads(src.read_text()) == json.loads(pub.read_text())
