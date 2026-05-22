"""Tests for PLAT-SA-F1b corpus audit operations."""

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


def test_build_drift_report_structure():
    mod = _load_module("build_drift", _EVAL / "build_replay_corpus_drift_report.py")
    index_path = _REPO / "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json"
    index = json.loads(index_path.read_text(encoding="utf-8"))
    report = mod.build_drift_report(index)
    assert report["artifact_type"] == "replay_corpus_drift_report_v1"
    assert "summary" in report
    assert isinstance(report["findings"], list)


def test_drift_report_fixture_matches():
    mod = _load_module("build_drift", _EVAL / "build_replay_corpus_drift_report.py")
    index = json.loads(
        (_REPO / "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json").read_text(encoding="utf-8")
    )
    build_index = _load_module("build_index", _EVAL / "build_replay_corpus_index.py")
    expected = mod.build_drift_report(index, expected_index=build_index.build_corpus_index())
    actual = json.loads(
        (_REPO / "fixtures/sa_r0/corpus_audits/replay_corpus_drift_report_v1.json").read_text(
            encoding="utf-8"
        )
    )
    assert json.dumps(actual, sort_keys=True) == json.dumps(expected, sort_keys=True)


def test_release_diff_structure():
    mod = _load_module("diff_rel", _EVAL / "diff_replay_corpus_releases.py")
    baseline = json.loads(
        (
            _REPO / "fixtures/sa_r0/corpus_releases/sa_r0_corpus_r1_r1/replay_corpus_index_v1.json"
        ).read_text(encoding="utf-8")
    )
    target = json.loads(
        (_REPO / "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json").read_text(encoding="utf-8")
    )
    diff = mod.build_release_diff(baseline, target, baseline_id="r1", target_id="canonical")
    assert diff["artifact_type"] == "replay_corpus_release_diff_v1"
    assert "entries_changed" in diff


def test_provenance_audit_passes():
    mod = _load_module("prov", _EVAL / "audit_replay_corpus_provenance.py")
    issues = mod.audit_corpus_provenance()
    assert not issues


def test_regen_dry_run_steps():
    mod = _load_module("regen", _EVAL / "run_replay_corpus_regen.py")
    results = mod.run_regen(dry_run=True)
    step_ids = [r["step_id"] for r in results]
    assert "gen_f1_corpus" in step_ids
    assert "drift_report" in step_ids
