"""Tests for PLAT-SA-F1c corpus navigation metadata."""

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


def test_all_entries_have_navigation_metadata():
    mod = _load_module("build_replay_corpus_index", _EVAL / "build_replay_corpus_index.py")
    index = mod.build_corpus_index()
    for entry in index["entries"]:
        assert entry.get("navigation_hint"), entry["entry_id"]
        assert entry.get("reviewer_category"), entry["entry_id"]
        assert entry.get("replay_family"), entry["entry_id"]
        assert entry.get("chronology_group"), entry["entry_id"]
        assert isinstance(entry.get("navigation_tags"), list), entry["entry_id"]


def test_sweep_family_navigation_targets():
    mod = _load_module("build_replay_corpus_index", _EVAL / "build_replay_corpus_index.py")
    index = mod.build_corpus_index()
    sweep = next(e for e in index["entries"] if e["entry_kind"] == "sweep_family")
    assert sweep["sweep_scope"]["sweep_id"]
    assert sweep["reviewer_category"] == "sweep_experiment"
    assert "mc_sweep" in sweep["navigation_tags"]


def test_drift_entry_ids_exist_in_index():
    drift_path = _REPO / "fixtures/sa_r0/corpus_audits/replay_corpus_drift_report_v1.json"
    index_path = _REPO / "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json"
    drift = json.loads(drift_path.read_text(encoding="utf-8"))
    index = json.loads(index_path.read_text(encoding="utf-8"))
    entry_ids = {e["entry_id"] for e in index["entries"]}
    for f in drift.get("findings") or []:
        eid = f.get("entry_id")
        if eid and f.get("kind") not in ("unindexed_file",):
            assert eid in entry_ids, f"drift references unknown entry: {eid}"


def test_viewer_drift_mirror_matches():
    canonical = _REPO / "fixtures/sa_r0/corpus_audits/replay_corpus_drift_report_v1.json"
    mirror = (
        _REPO
        / "platform/sa-r0-viewer/public/demo/corpus_audits/replay_corpus_drift_report_v1.json"
    )
    assert mirror.is_file()
    assert canonical.read_bytes() == mirror.read_bytes()
