"""Tests for PLAT-SA-I1 orchestration operations integrity."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
_EVAL = _REPO / "scripts" / "evaluation"
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_sa_orchestration_integrity as orch_int  # noqa: E402
import replay_sa_orchestration_ops as orch_ops  # noqa: E402
from experiment_orchestration import load_manifest  # noqa: E402


def test_catalog_has_twelve_packs():
    packs = orch_int.catalog_pack_ids()
    assert len(packs) == 12


def test_all_catalog_packs_have_validation_manifest():
    catalog = set(orch_int.catalog_pack_ids())
    covered = set()
    for mf in orch_ops.list_all_manifest_files():
        manifest = load_manifest(mf)
        if manifest.get("manifest_id") == "capture_pipeline_template":
            continue
        for job in manifest.get("jobs") or []:
            pid = job.get("scenario_pack_id")
            if pid:
                covered.add(str(pid))
    assert catalog <= covered


def test_ops_sidecar_exists_for_validation_manifests():
    missing = []
    for mf in orch_ops.list_all_manifest_files():
        manifest = load_manifest(mf)
        mid = str(manifest["manifest_id"])
        if mid == "capture_pipeline_template":
            continue
        if not orch_ops.ops_path(mid).is_file():
            missing.append(mid)
    assert missing == [], f"missing ops sidecars: {missing}"


def test_integrity_audit_non_strict_ok():
    result = orch_int.run_integrity_audit(strict=False)
    assert result["artifact_type"] == orch_int.INTEGRITY_ARTIFACT_TYPE
    assert result["pack_count"] == 12


def test_integrity_audit_strict_passes():
    result = orch_int.run_integrity_audit(strict=True)
    assert result["ok"] is True
    assert not result.get("errors")


def test_repro_check_ridge_synthetic():
    mf = _REPO / "fixtures/orchestration/manifests/ridge_defense_synthetic.json"
    result = orch_ops.repro_check(mf)
    assert result.get("ok") is True


def test_lineage_report_valley():
    report = orch_int.lineage_continuity_report("valley_ingress_validation_only")
    assert report.get("manifest_id") == "valley_ingress_validation_only"
    assert "execution_lineage" in report
