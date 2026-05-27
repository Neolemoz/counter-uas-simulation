"""Tests for PLAT-RT-F5b P2 rt_experiment_fidelity_metrics.py."""

from __future__ import annotations

import json
from pathlib import Path

from scripts.rt import rt_experiment_fidelity_metrics as fidelity_metrics

_REPO = Path(__file__).resolve().parents[3]
_F5B = _REPO / "fixtures" / "rt_experiments" / "f5b_fidelity_examples"


def _strip_derived(report: dict) -> dict:
    out = dict(report)
    out.pop("derived_at_utc", None)
    return out


def test_derive_matches_typescript_golden() -> None:
    manifest = json.loads((_F5B / "manifest_fidelity_golden.json").read_text(encoding="utf-8"))
    report = fidelity_metrics.derive_fidelity_metrics(manifest, repo_root=_REPO)
    golden = json.loads((_F5B / "fidelity_metrics_report_golden.json").read_text(encoding="utf-8"))
    assert _strip_derived(report) == golden


def test_schema_literal() -> None:
    manifest = json.loads((_F5B / "manifest_fidelity_golden.json").read_text(encoding="utf-8"))
    report = fidelity_metrics.derive_fidelity_metrics(manifest, repo_root=_REPO)
    assert report["schema"] == "rt_experiment_fidelity_metrics_report_v1"
    assert report["coupling_required"] is True
    assert len(report["per_run_fidelity"]) == 2


def test_cognition_divergence_on_divergent_run() -> None:
    manifest = json.loads((_F5B / "manifest_fidelity_golden.json").read_text(encoding="utf-8"))
    report = fidelity_metrics.derive_fidelity_metrics(manifest, repo_root=_REPO)
    divergent = next(r for r in report["per_run_fidelity"] if r["run_id"] == "run-divergent")
    assert divergent["cognition_truth_divergence"] is True
    assert divergent["los_truth_label"] == "terrain_blocked"


def test_forbidden_rollup_keys_absent() -> None:
    manifest = json.loads((_F5B / "manifest_fidelity_golden.json").read_text(encoding="utf-8"))
    report = fidelity_metrics.derive_fidelity_metrics(manifest, repo_root=_REPO)
    rollup_text = json.dumps(report["rollup_fidelity"])
    for key in fidelity_metrics.FORBIDDEN_ROLLUP_KEYS:
        assert f'"{key}"' not in rollup_text


def test_coupling_off_without_staging() -> None:
    manifest = json.loads((_F5B / "manifest_fidelity_golden.json").read_text(encoding="utf-8"))
    for run in manifest["runs"]:
        run.pop("fidelity_context", None)
    report = fidelity_metrics.derive_fidelity_metrics(manifest, repo_root=_REPO)
    assert report["coupling_required"] is False
    assert all(r["fidelity_attestation_status"] == "unavailable" for r in report["per_run_fidelity"])
