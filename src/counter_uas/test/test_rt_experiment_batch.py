"""Tests for PLAT-RT-X1 rt_experiment_batch.py (dry-run)."""

from __future__ import annotations

import json
from pathlib import Path
from unittest.mock import patch

import pytest

from scripts.rt import rt_experiment_batch as batch


@pytest.fixture
def batch_spec(tmp_path: Path) -> Path:
    spec = {
        "schema": "rt_experiment_batch_v1",
        "experiment_id": "test-exp",
        "default_dwell_s": 0,
        "runs": [{"run_id": "r1", "label": "one"}],
    }
    path = tmp_path / "batch.json"
    path.write_text(json.dumps(spec), encoding="utf-8")
    return path


def test_load_spec_json(batch_spec: Path) -> None:
    data = batch._load_spec(batch_spec)
    assert data["experiment_id"] == "test-exp"


def test_annex_summary(tmp_path: Path) -> None:
    staging = tmp_path / "cap1"
    staging.mkdir()
    (staging / "tactical_annex.json").write_text(
        json.dumps(
            {
                "final_tactical_mode": "manual",
                "mode_switches": [{"t_utc": "x"}],
                "assignment_timeline": [],
            }
        ),
        encoding="utf-8",
    )
    summary = batch._annex_summary(staging)
    assert summary is not None
    assert summary["timeline_counts"]["mode_switches"] == 1


def test_run_batch_dry_run(batch_spec: Path, tmp_path: Path) -> None:
    manifest = tmp_path / "manifest.json"
    audit = tmp_path / "audit.jsonl"
    code = batch.run_batch(
        spec=batch._load_spec(batch_spec),
        repo_root=tmp_path,
        manifest_path=manifest,
        command_url="http://127.0.0.1:18765/v1/command",
        dry_run=True,
        audit_path=audit,
    )
    assert code == 0
    assert audit.is_file()


@patch("scripts.rt.rt_experiment_batch.send_command")
def test_run_batch_writes_manifest(mock_send, batch_spec: Path, tmp_path: Path) -> None:
    mock_send.side_effect = [
        {"ok": True, "session_id": "sess-1"},
        {"ok": True},
        {
            "ok": True,
            "detail": {"capture_candidate_id": "cap-abc"},
            "tactical_state": {"schema": "rt_tactical_state_v1", "tactical_mode": "manual"},
            "world_summary": {"entity_count": 1},
        },
    ]
    manifest = tmp_path / "manifest.json"
    code = batch.run_batch(
        spec=batch._load_spec(batch_spec),
        repo_root=tmp_path,
        manifest_path=manifest,
        command_url="http://127.0.0.1:18765/v1/command",
        dry_run=False,
        audit_path=tmp_path / "audit.jsonl",
    )
    assert code == 0
    data = json.loads(manifest.read_text(encoding="utf-8"))
    assert data["schema"] == "rt_experiment_manifest_v1"
    assert len(data["runs"]) == 1
    assert data["runs"][0]["capture_candidate_id"] == "cap-abc"
