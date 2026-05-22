"""Tests for build_replay_linkage.py (PLAT-SA-E2)."""

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


def test_build_linkage_index_structure():
    mod = _load_module("build_replay_linkage", _EVAL / "build_replay_linkage.py")
    linkage = mod.build_linkage_index()
    assert linkage["artifact_type"] == "replay_linkage_index_v1"
    assert len(linkage["nodes"]) == 4
    assert isinstance(linkage["edges"], list)


def test_linkage_edges_have_copy_and_kind():
    mod = _load_module("build_replay_linkage", _EVAL / "build_replay_linkage.py")
    linkage = mod.build_linkage_index()
    for edge in linkage["edges"]:
        assert edge.get("link_kind") in {
            "shared_pattern",
            "shared_topology",
            "metric_similarity",
            "storyline_reference",
        }
        assert edge.get("copy")
        assert edge.get("edge_id")


def test_linkage_fixture_matches():
    mod = _load_module("build_replay_linkage", _EVAL / "build_replay_linkage.py")
    expected = mod.build_linkage_index()
    path = _REPO / "fixtures/sa_r0/synthesis/replay_linkage_index_v1.json"
    actual = json.loads(path.read_text(encoding="utf-8"))
    assert json.dumps(actual, sort_keys=True) == json.dumps(expected, sort_keys=True)
