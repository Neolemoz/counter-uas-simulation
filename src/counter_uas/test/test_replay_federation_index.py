"""Tests for PLAT-SA-F2A replay federation index."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
_EVAL = _REPO / "scripts" / "evaluation"
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(mod)
    return mod


def test_build_federation_index_structure():
    mod = _load_module("replay_federation_lineage", _EVAL / "replay_federation_lineage.py")
    index = mod.build_federation_index(_REPO)
    assert index["artifact_type"] == "replay_federation_index_v1"
    assert index["federation_id"] == "sa_replay_federation_r0_v1"
    assert len(index["corpus_group_summaries"]) == 2


def test_federation_manifest_fixture_matches():
    mod = _load_module("replay_federation_lineage", _EVAL / "replay_federation_lineage.py")
    expected = mod.build_federation_manifest(_REPO)
    path = _REPO / "fixtures/sa_r0/federation/replay_federation_manifest_v1.json"
    actual = json.loads(path.read_text(encoding="utf-8"))
    assert json.dumps(actual, sort_keys=True) == json.dumps(expected, sort_keys=True)


def test_federation_lineage_dag_no_cycles():
    mod = _load_module("replay_federation_lineage", _EVAL / "replay_federation_lineage.py")
    manifest = mod.build_federation_manifest(_REPO)
    graph = mod.build_federation_lineage_graph(manifest["corpus_groups"])
    errors = mod.validate_federation_lineage_dag(manifest["corpus_groups"], graph["edges"])
    assert not errors


def test_federation_index_fixture_matches():
    mod = _load_module("build_replay_federation_index", _EVAL / "build_replay_federation_index.py")
    expected = mod.build_federation_index(_REPO)
    path = _REPO / "fixtures/sa_r0/federation/replay_federation_index_v1.json"
    actual = json.loads(path.read_text(encoding="utf-8"))
    assert json.dumps(actual, sort_keys=True) == json.dumps(expected, sort_keys=True)
