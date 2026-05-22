"""Tests for export_research_bundle.py (PLAT-SA-E2)."""

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


def test_research_bundle_manifest():
    path = _REPO / "fixtures/sa_r0/research_bundles/sa_r0_corpus_r1/manifest.json"
    manifest = json.loads(path.read_text(encoding="utf-8"))
    assert manifest["artifact_type"] == "replay_research_bundle_v1"
    assert len(manifest["sweep_ids"]) == 4
    assert len(manifest["included_files"]) >= 5


def test_research_bundle_sha256_integrity():
    mod = _load_module("export_research_bundle", _EVAL / "export_research_bundle.py")
    mod.check_research_bundle("sa_r0_corpus_r1")


def test_research_bundle_zip_exists():
    path = _REPO / "fixtures/sa_r0/research_bundles/sa_r0_corpus_r1.zip"
    assert path.is_file()
    assert path.stat().st_size > 1000
