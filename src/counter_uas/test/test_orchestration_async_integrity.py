"""Tests for PLAT-SA-I2 async orchestration foundations."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
_EVAL = _REPO / "scripts" / "evaluation"
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_sa_orchestration_async as orch_async  # noqa: E402
import replay_sa_orchestration_ops as orch_ops  # noqa: E402
from experiment_orchestration import load_manifest  # noqa: E402


def test_async_integrity_audit_strict_passes() -> None:
    result = orch_async.run_async_integrity_audit(strict=True)
    assert result["artifact_type"] == orch_async.ASYNC_INTEGRITY_ARTIFACT_TYPE
    assert result["ok"] is True
    assert not result.get("issues")


def test_execution_fingerprint_stable_ridge() -> None:
    mf = _REPO / "fixtures/orchestration/manifests/ridge_defense_synthetic.json"
    manifest_id = "ridge_defense_synthetic_pipeline"
    ops = orch_ops.load_ops_manifest(manifest_id) or {}
    queue_path, report_path = orch_async._resolve_queue_report(manifest_id, ops)
    assert queue_path and queue_path.is_file()
    fp1 = orch_async.compute_execution_fingerprint(mf, queue_path, report_path, ops=ops)
    fp2 = orch_async.compute_execution_fingerprint(mf, queue_path, report_path, ops=ops)
    assert fp1 == fp2
    async_m = orch_async.load_async_manifest(manifest_id)
    assert async_m and async_m.get("execution_fingerprint") == fp1


def test_promote_blocked_when_quarantined(tmp_path: Path) -> None:
    mf = _REPO / "fixtures/orchestration/manifests/capture_pipeline_template.json"
    manifest_id = "capture_pipeline_template"
    assert orch_async.is_quarantined(manifest_id)
    result = orch_ops.promote_experiment_manifest(
        mf,
        target_status="executed",
        dry_run=True,
    )
    assert result.get("ok") is False
    assert "quarantined" in str(result.get("error", ""))


def test_duplicate_claim_detected(tmp_path: Path) -> None:
    claims_dir = tmp_path / "claims"
    claims_dir.mkdir()
    dup_claim = {
        "artifact_type": "queue_claim_token_v1",
        "schema_version": "1",
        "claim_id": "claim-dup-a",
        "manifest_id": "test_manifest",
        "queue_id": "test_queue",
        "snapshot_hash": "sha256:abc",
        "manifest_fingerprint": "sha256:def",
        "worker_id": "w1",
        "claim_status": "open",
        "queue_snapshot_ref": "fixtures/orchestration/queues/missing.json",
        "governance": {"notice": "test", "anti_claims": []},
    }
    (claims_dir / "a_claim.json").write_text(json.dumps(dup_claim), encoding="utf-8")
    dup_claim["claim_id"] = "claim-dup-b"
    (claims_dir / "b_claim.json").write_text(json.dumps(dup_claim), encoding="utf-8")

    orig_claims = orch_async._CLAIMS
    orch_async._CLAIMS = claims_dir
    try:
        collected = orch_async._collect_open_claims()
        assert ("sha256:abc", "test_manifest") in collected
        assert len(collected[("sha256:abc", "test_manifest")]) == 2
    finally:
        orch_async._CLAIMS = orig_claims


def test_async_worker_metadata_blocked_without_flag() -> None:
    from experiment_orchestration import run_manifest  # noqa: E402

    mf = _REPO / "fixtures/orchestration/manifests/ridge_defense_synthetic.json"
    result = run_manifest(mf, dry_run=True, write_queue=False, allow_async_worker=False)
    meta = result["report"].get("async_worker_governance") or {}
    assert meta.get("worker_provenance") == "blocked"

    result2 = run_manifest(mf, dry_run=True, write_queue=False, allow_async_worker=True)
    meta2 = result2["report"].get("async_worker_governance") or {}
    assert meta2.get("worker_provenance") == "enabled"
