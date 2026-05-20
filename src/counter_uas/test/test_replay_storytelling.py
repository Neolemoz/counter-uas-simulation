"""Tests for build_replay_storytelling.py (PLAT-SA-E1)."""

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


def test_build_storytelling_sections_bundle():
    mod = _load_module("build_replay_storytelling", _EVAL / "build_replay_storytelling.py")
    bundle_path = _REPO / "fixtures/sa_r0/demo_valley_ingress/index.json"
    bundle = json.loads(bundle_path.read_text(encoding="utf-8"))
    sections = mod.build_storytelling_sections(bundle=bundle)
    assert "what_changed" in sections
    assert "topology_pacing" in sections
    assert not mod.lint_storytelling_sections(sections)


def test_build_storytelling_sections_sweep():
    mod = _load_module("build_replay_storytelling", _EVAL / "build_replay_storytelling.py")
    sweep_path = _REPO / "fixtures/sa_r0/sweeps/ridge_overlap_sweep/sweep.json"
    manifest = json.loads(sweep_path.read_text(encoding="utf-8"))
    sections = mod.build_storytelling_sections(manifest=manifest)
    assert "what_changed" in sections
    assert not mod.lint_storytelling_sections(sections)
