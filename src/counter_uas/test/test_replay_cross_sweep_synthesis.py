"""Tests for build_cross_sweep_synthesis.py (PLAT-SA-E2)."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
_EVAL = _REPO / "scripts" / "evaluation"


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(mod)
    return mod


def test_build_cross_sweep_synthesis_structure():
    mod = _load_module("build_cross_sweep_synthesis", _EVAL / "build_cross_sweep_synthesis.py")
    synthesis = mod.build_cross_sweep_synthesis()
    assert synthesis["artifact_type"] == "cross_sweep_synthesis_v1"
    assert len(synthesis["sweep_ids"]) == 4
    assert "pattern_frequency_rollup" in synthesis
    assert "ambiguity_concentration_comparison" in synthesis
    assert "los_instability_rollup" in synthesis


def test_cross_sweep_synthesis_fixture_matches():
    mod = _load_module("build_cross_sweep_synthesis", _EVAL / "build_cross_sweep_synthesis.py")
    cognition = _load_module("build_replay_cognition_rollup", _EVAL / "build_replay_cognition_rollup.py")
    expected = cognition.enrich_synthesis_with_cognition(mod.build_cross_sweep_synthesis())
    path = _REPO / "fixtures/sa_r0/synthesis/cross_sweep_synthesis_v1.json"
    actual = json.loads(path.read_text(encoding="utf-8"))
    assert json.dumps(actual, sort_keys=True) == json.dumps(expected, sort_keys=True)


def test_cross_sweep_summary_markdown():
    mod = _load_module("build_cross_sweep_synthesis", _EVAL / "build_cross_sweep_synthesis.py")
    synthesis = mod.build_cross_sweep_synthesis()
    md = mod.render_cross_sweep_summary(synthesis)
    assert "Cross-sweep replay synthesis summary" in md
    assert "deployment readiness" not in md.lower() or "not" in md.lower()
