from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load(name: str, rel: str):
    eval_dir = str(_REPO_ROOT / "scripts" / "evaluation")
    if eval_dir not in sys.path:
        sys.path.insert(0, eval_dir)
    path = _REPO_ROOT / rel
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def test_narrative_summary_has_bullets():
    ni = _load(
        "replay_narrative_intelligence",
        "scripts/evaluation/replay_narrative_intelligence.py",
    )
    agg = _load(
        "aggregate_spatial_analytics",
        "scripts/evaluation/aggregate_spatial_analytics.py",
    )
    sweep_path = _REPO_ROOT / "fixtures/sa_r0/sweeps/valley_sensor_sweep/sweep.json"
    if not sweep_path.is_file():
        return
    manifest = json.loads(sweep_path.read_text(encoding="utf-8"))
    bundles = [
        agg.load_bundle(
            _REPO_ROOT
            / "fixtures/sa_r0/sweeps/valley_sensor_sweep/members"
            / m["member_id"]
            / "index.json"
        )
        for m in manifest["members"]
    ]
    summary = ni.build_sweep_narrative_summary(manifest, bundles)
    assert summary["headline"]
    assert len(summary["bullets"]) >= 1
    assert "importance_weights" in summary


def test_enriched_sweep_validates():
    mc = _load("replay_mc_sweep", "scripts/evaluation/replay_mc_sweep.py")
    path = _REPO_ROOT / "fixtures/sa_r0/sweeps/delayed_detection_sweep/sweep.json"
    if not path.is_file():
        return
    manifest = json.loads(path.read_text(encoding="utf-8"))
    assert manifest.get("replay_cohorts")
    errs = mc.validate_sweep(manifest)
    assert errs == []
