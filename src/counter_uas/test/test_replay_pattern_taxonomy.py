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


def test_classify_corridor_pack():
    classify = _load("classify_replay_pattern", "scripts/evaluation/classify_replay_pattern.py")
    bundle = {
        "clock": {"duration": {"start": 0, "end": 40}},
        "narrative": {"events": [], "windows": []},
        "los_segments": [],
        "comparison_hints": {"topology_key": "corridor_defense"},
    }
    member = {"pack_id": "corridor_defense"}
    tags = classify.classify_member_patterns(
        bundle, member=member, baseline_topology_key="ridge_defense"
    )
    assert "corridor_pressure_replay" in tags


def test_sweep_members_have_pattern_tags():
    sweep_path = _REPO_ROOT / "fixtures/sa_r0/sweeps/ridge_overlap_sweep/sweep.json"
    if not sweep_path.is_file():
        return
    manifest = json.loads(sweep_path.read_text(encoding="utf-8"))
    assert manifest.get("replay_narrative_summary")
    assert manifest.get("replay_cohorts")
    for m in manifest["members"]:
        assert "replay_pattern_tags" in m
