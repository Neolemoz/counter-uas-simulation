"""Tests for PLAT-SA-I3 async recovery and reconciliation."""

from __future__ import annotations

import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
_EVAL = _REPO / "scripts" / "evaluation"
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_sa_orchestration_recovery as recovery  # noqa: E402
import replay_sa_orchestration_async as orch_async  # noqa: E402


def test_recovery_integrity_audit_strict_passes() -> None:
    result = recovery.run_recovery_integrity_audit(strict=True)
    assert result["artifact_type"] == recovery.RECOVERY_INTEGRITY_TYPE
    assert result["ok"] is True
    assert not result.get("issues")


def test_retry_chain_ridge_has_two_workers() -> None:
    manifest_id = "ridge_defense_synthetic_pipeline"
    chain = recovery.build_retry_chain(manifest_id)
    worker_hops = [h for h in chain if h.get("hop_type") == "worker"]
    assert len(worker_hops) >= 2
    assert worker_hops[1].get("execution_attempt") == 2


def test_recovery_report_quarantine_shape() -> None:
    report = recovery.build_recovery_report("capture_pipeline_template")
    assert report["artifact_type"] == recovery.RECOVERY_REPORT_TYPE
    assert report["quarantine_hold"] is True
    assert report["manifest_id"] == "capture_pipeline_template"


def test_superseded_manifest_no_open_claim_issue() -> None:
    issues = recovery.audit_replay_supersession("valley_ingress_validation_only", strict=True)
    assert not issues


def test_batch_audit_has_status_counts() -> None:
    recovery_report = recovery.run_recovery_integrity_audit(strict=False)
    batch = recovery.build_async_batch_audit(recovery_report)
    assert batch["artifact_type"] == recovery.BATCH_AUDIT_TYPE
    assert batch.get("async_manifest_count", 0) >= 3
    assert "quarantined" in (batch.get("status_counts") or {})


def test_ridge_retry_lineage_no_gap() -> None:
    issues = recovery.audit_retry_lineage("ridge_defense_synthetic_pipeline", strict=True)
    assert not issues


def test_async_integrity_still_passes_with_i3_fixtures() -> None:
    result = orch_async.run_async_integrity_audit(strict=True)
    assert result["ok"] is True
