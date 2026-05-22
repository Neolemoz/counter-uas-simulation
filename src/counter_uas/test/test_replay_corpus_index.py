"""Tests for PLAT-SA-F1a replay corpus index."""

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


def test_build_corpus_index_structure():
    mod = _load_module("build_replay_corpus_index", _EVAL / "build_replay_corpus_index.py")
    index = mod.build_corpus_index()
    assert index["artifact_type"] == "replay_corpus_index_v1"
    assert index["corpus_id"] == "sa_r0_corpus_r1"
    assert len(index["entries"]) >= 50
    kinds = {e["entry_kind"] for e in index["entries"]}
    assert "sweep_family" in kinds
    assert "demo_bundle" in kinds
    assert "synthesis_report" in kinds


def test_corpus_index_fixture_matches():
    mod = _load_module("build_replay_corpus_index", _EVAL / "build_replay_corpus_index.py")
    expected = mod.build_corpus_index()
    path = _REPO / "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json"
    actual = json.loads(path.read_text(encoding="utf-8"))
    assert json.dumps(actual, sort_keys=True) == json.dumps(expected, sort_keys=True)


def test_validate_replay_corpus_index():
    mod = _load_module("validate_replay_corpus", _EVAL / "validate_replay_corpus.py")
    path = _REPO / "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json"
    index = json.loads(path.read_text(encoding="utf-8"))
    errors, warnings = mod.validate_replay_corpus_index(index)
    assert not errors
    assert isinstance(warnings, list)


def test_lineage_dag_no_cycles():
    mod = _load_module("replay_corpus_lineage", _EVAL / "replay_corpus_lineage.py")
    path = _REPO / "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json"
    index = json.loads(path.read_text(encoding="utf-8"))
    errors = mod.validate_lineage_dag(index["entries"], index.get("lineage_edges"))
    assert not errors


def test_research_bundle_includes_corpus_index():
    manifest_path = _REPO / "fixtures/sa_r0/research_bundles/sa_r0_corpus_r1/manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    paths = {e["path"] for e in manifest.get("included_files") or []}
    assert "synthesis/replay_corpus_index_v1.json" in paths
