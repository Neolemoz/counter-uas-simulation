"""Tests for PLAT-SA-STAB platform integrity auditor."""

from __future__ import annotations

import importlib.util
import json
import re
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
_EVAL = _REPO / "scripts" / "evaluation"
_VIEWER_TAGS = (
    _REPO / "platform/sa-r0-viewer/src/replay/workstation/EventPatternGroupList.tsx"
)


def _load_audit():
    if str(_EVAL) not in sys.path:
        sys.path.insert(0, str(_EVAL))
    path = _EVAL / "audit_sa_platform_integrity.py"
    spec = importlib.util.spec_from_file_location("audit_sa_platform_integrity", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(mod)
    return mod


def _load_classify():
    if str(_EVAL) not in sys.path:
        sys.path.insert(0, str(_EVAL))
    path = _EVAL / "classify_replay_pattern.py"
    spec = importlib.util.spec_from_file_location("classify_replay_pattern", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(mod)
    return mod


def test_catalog_sync_passes():
    mod = _load_audit()
    assert mod.check_catalog_sync() == []


def test_linkage_graph_passes():
    mod = _load_audit()
    linkage_path = _REPO / "fixtures/sa_r0/synthesis/replay_linkage_index_v1.json"
    if not linkage_path.is_file():
        return
    assert mod.check_linkage_graph() == []


def test_bundle_catalog_passes():
    mod = _load_audit()
    assert mod.check_bundle_catalog() == []


def test_governance_batch_passes():
    mod = _load_audit()
    assert mod.check_governance_batch() == []


def test_pattern_priority_has_viewer_labels():
    classify = _load_classify()
    if not _VIEWER_TAGS.is_file():
        return
    text = _VIEWER_TAGS.read_text(encoding="utf-8")
    match = re.search(r"const TAG_LABELS[^=]+=\s*\{([^}]+)\}", text, re.S)
    assert match, "TAG_LABELS block not found in EventPatternGroupList.tsx"
    block = match.group(1)
    viewer_keys = set(re.findall(r"(\w+):\s*\"", block))
    for pid in classify.PATTERN_PRIORITY:
        assert pid in viewer_keys, f"missing viewer TAG_LABELS for {pid}"


def test_linkage_fixture_matches_builder():
    mod_path = _EVAL / "build_replay_linkage.py"
    spec = importlib.util.spec_from_file_location("build_replay_linkage", mod_path)
    bl = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(bl)
    fixture = _REPO / "fixtures/sa_r0/synthesis/replay_linkage_index_v1.json"
    if not fixture.is_file():
        return
    expected = bl.build_linkage_index()
    actual = json.loads(fixture.read_text(encoding="utf-8"))
    assert json.dumps(actual, sort_keys=True) == json.dumps(expected, sort_keys=True)
