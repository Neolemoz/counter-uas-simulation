"""Tests for PLAT-SA-F1d long-horizon synthesis and publication operations."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
_EVAL = _REPO / "scripts" / "evaluation"
_PY = sys.executable


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(mod)
    return mod


def test_evolution_manifest_covers_chronology_groups():
    index = json.loads(
        (_REPO / "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json").read_text(encoding="utf-8")
    )
    manifest = json.loads(
        (_REPO / "fixtures/sa_r0/synthesis/replay_corpus_evolution_manifest_v1.json").read_text(
            encoding="utf-8"
        )
    )
    groups = {e.get("chronology_group") for e in index["entries"] if e.get("chronology_group")}
    tier_entry_ids: set[str] = set()
    for tier in manifest["chronology_tiers"]:
        tier_entry_ids.update(tier["entry_ids"])
    for entry in index["entries"]:
        assert entry["entry_id"] in tier_entry_ids, entry["entry_id"]
    assert manifest["artifact_type"] == "replay_corpus_evolution_manifest_v1"
    assert any(r.get("release_id") == "sa_r0_corpus_r1_r1" for r in manifest["releases"])
    assert any(r.get("release_id") == "canonical_index" for r in manifest["releases"])
    assert groups  # sanity: index has chronology metadata


def test_evolution_summary_matches_builder():
    evo = _load_module("evo", _EVAL / "build_replay_corpus_evolution.py")
    index_mod = _load_module("idx", _EVAL / "build_replay_corpus_index.py")
    index = index_mod.build_corpus_index()
    expected_summary = evo.build_evolution_summary(index)
    actual = json.loads(
        (_REPO / "fixtures/sa_r0/synthesis/replay_corpus_evolution_summary_v1.json").read_text(
            encoding="utf-8"
        )
    )
    assert json.dumps(actual, sort_keys=True) == json.dumps(expected_summary, sort_keys=True)


def test_evolution_manifest_matches_builder():
    evo = _load_module("evo", _EVAL / "build_replay_corpus_evolution.py")
    index_mod = _load_module("idx", _EVAL / "build_replay_corpus_index.py")
    index = index_mod.build_corpus_index()
    expected = evo.build_evolution_manifest(index)
    actual = json.loads(
        (_REPO / "fixtures/sa_r0/synthesis/replay_corpus_evolution_manifest_v1.json").read_text(
            encoding="utf-8"
        )
    )
    assert json.dumps(actual, sort_keys=True) == json.dumps(expected, sort_keys=True)


def test_index_entries_have_evolution_metadata():
    index = json.loads(
        (_REPO / "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json").read_text(encoding="utf-8")
    )
    for entry in index["entries"]:
        assert entry.get("release_generation_id"), entry["entry_id"]
        assert entry.get("replay_chronology_descriptor"), entry["entry_id"]
        assert isinstance(entry.get("evolution_tags"), list), entry["entry_id"]


def test_publication_packet_sha256_on_disk():
    packet = json.loads(
        (_REPO / "fixtures/sa_r0/synthesis/replay_corpus_publication_packet_v1.json").read_text(
            encoding="utf-8"
        )
    )
    assert packet["artifact_type"] == "replay_corpus_publication_packet_v1"
    for item in packet.get("included_artifacts") or []:
        rel = item["path"]
        full = _REPO / rel
        assert full.is_file(), rel
        digest = hashlib.sha256(full.read_bytes()).hexdigest()
        assert digest == item["sha256"], rel


def test_release_export_archive_check():
    subprocess.run(
        [_PY, str(_EVAL / "export_replay_corpus_release.py"), "--check"],
        cwd=_REPO,
        check=True,
    )


def test_viewer_evolution_mirror_matches():
    for name in (
        "replay_corpus_evolution_manifest_v1.json",
        "replay_corpus_evolution_summary_v1.json",
        "replay_corpus_publication_packet_v1.json",
    ):
        canonical = _REPO / "fixtures/sa_r0/synthesis" / name
        mirror = _REPO / "platform/sa-r0-viewer/public/demo/synthesis" / name
        assert mirror.is_file(), name
        assert canonical.read_bytes() == mirror.read_bytes(), name
