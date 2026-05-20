"""Tests for build_replay_presentation.py (PLAT-SA-E1)."""

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


def _load_build_presentation():
    return _load_module("build_replay_presentation", _EVAL / "build_replay_presentation.py")


def test_build_bundle_presentation_valley_ingress():
    mod = _load_build_presentation()
    bundle_path = _REPO / "fixtures/sa_r0/demo_valley_ingress/index.json"
    bundle = json.loads(bundle_path.read_text(encoding="utf-8"))
    pres = mod.build_bundle_presentation(bundle)
    assert "walkthrough_id" in pres
    assert len(pres["chapters"]) >= 3
    titles = [c["title"] for c in pres["chapters"]]
    assert "Ingress" in titles
    assert pres["chapters"][0]["t_start"] <= pres["chapters"][0]["t_end"]


def test_enrich_bundle_presentation():
    mod = _load_build_presentation()
    bundle_path = _REPO / "fixtures/sa_r0/demo_valley_ingress/index.json"
    bundle = json.loads(bundle_path.read_text(encoding="utf-8"))
    enriched = mod.enrich_bundle_presentation(bundle)
    assert "presentation" in enriched
    assert enriched["presentation"]["chapters"]


def test_build_sweep_presentation_walkthrough():
    mod = _load_build_presentation()
    sweep_path = _REPO / "fixtures/sa_r0/sweeps/ridge_overlap_sweep/sweep.json"
    manifest = json.loads(sweep_path.read_text(encoding="utf-8"))
    walk = mod.build_sweep_presentation_walkthrough(manifest)
    assert walk["walkthrough_id"]
    assert len(walk["steps"]) >= 2
    kinds = {s["kind"] for s in walk["steps"]}
    assert "analytics_panel" in kinds
