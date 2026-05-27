"""Pose sync G6 hardening tests."""

from __future__ import annotations

from rt_sandbox.governance import GovernanceConfig
from rt_sandbox.pose_sync import PoseSyncMirror, pose_yaw_drift_deg


def test_pose_yaw_drift_deg() -> None:
    assert pose_yaw_drift_deg({"yaw_deg": 0}, {"yaw_deg": 90}) == 90.0
    assert pose_yaw_drift_deg({"yaw_deg": 10}, {"yaw_deg": 350}) == 20.0


def test_missing_feedback_entity_mismatch() -> None:
    mirror = PoseSyncMirror(session_id="s1")
    mirror.record_command("e1", "drone", {"x": 0, "y": 0, "z": 10}, sync_revision=1)
    cfg = GovernanceConfig(pose_sync_drift_threshold_m=2.0)
    health, err, detail = mirror.update_from_feedback(
        {
            "schema": "rt_adapter_feedback_v1",
            "timestamp_utc": "2026-05-25T12:00:00+00:00",
            "sync_seq": 1,
            "entities": [],
        },
        {"e1"},
        cfg,
    )
    assert health == "mismatch"
    assert err == "SYNC_MISMATCH"
    assert detail is not None
    assert detail.get("reason") == "missing_feedback_entity"


def test_summary_includes_lag_fields() -> None:
    mirror = PoseSyncMirror(session_id="s1")
    mirror.last_command_utc = "2026-05-25T12:00:00+00:00"
    mirror.last_feedback_utc = "2026-05-25T12:00:01+00:00"
    mirror.apply_lag_ms = 1000.0
    summary = mirror.summary(3, adapter_mode="mock")
    assert summary["adapter_mode"] == "mock"
    assert summary["apply_lag_ms"] == 1000.0
