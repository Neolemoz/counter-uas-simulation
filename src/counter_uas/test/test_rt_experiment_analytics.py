"""Tests for PLAT-RT-F1 rt_experiment_analytics.py."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from scripts.rt import rt_experiment_analytics as analytics


@pytest.fixture
def manifest(tmp_path: Path) -> Path:
    data = {
        "schema": "rt_experiment_manifest_v1",
        "experiment_id": "exp-analytics-test",
        "created_at_utc": "2026-05-26T12:00:00+00:00",
        "governance_banner": "RT EXPERIMENT — explanatory compare only; not operational authority",
        "runs": [
            {
                "run_id": "b-run",
                "label": "B",
                "session_id": "sess-bbbbbbbb",
                "recorded_at_utc": "2026-05-26T12:01:00+00:00",
                "capture_candidate_id": "cap-1",
                "snapshot": {
                    "tactical_state": {
                        "tactical_mode": "assisted",
                        "tti_s": 2.0,
                        "assigned_target_id": "tgt-a",
                    },
                    "world_summary": {"entity_count": 3},
                },
            },
            {
                "run_id": "a-run",
                "label": "A",
                "session_id": "sess-aaaaaaaa",
                "recorded_at_utc": "2026-05-26T12:00:00+00:00",
                "snapshot": {
                    "tactical_state": {
                        "tactical_mode": "manual",
                        "tti_s": 2.1,
                        "assigned_target_id": "tgt-b",
                    },
                    "world_summary": {"entity_count": 1},
                },
            },
        ],
    }
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


@pytest.fixture
def batch_spec(tmp_path: Path) -> Path:
    spec = {
        "schema": "rt_experiment_batch_v1",
        "experiment_id": "exp-analytics-test",
        "default_dwell_s": 2,
        "runs": [
            {"run_id": "a-run", "label": "A", "template_id": "tpl-ridge", "dwell_s": 3},
            {"run_id": "b-run", "label": "B", "template_id": "tpl-valley"},
        ],
    }
    path = tmp_path / "batch.json"
    path.write_text(json.dumps(spec), encoding="utf-8")
    return path


def test_derive_report_fields(manifest: Path, batch_spec: Path) -> None:
    manifest_data = analytics._load_json(manifest)
    batch_data = analytics._load_batch(batch_spec)
    report = analytics.derive_analytics(manifest_data, batch_data, repo_root=manifest.parent)

    assert report["schema"] == "rt_experiment_analytics_report_v1"
    assert report["experiment_id"] == "exp-analytics-test"
    assert report["rollup"]["run_count"] == 2
    assert report["rollup"]["capture_count"] == 1
    assert "success_rate" not in report["rollup"]
    assert "readiness" not in report["rollup"]

    per_run = {row["run_id"]: row for row in report["per_run"]}
    assert list(per_run.keys()) == ["a-run", "b-run"]
    assert per_run["a-run"]["template_id"] == "tpl-ridge"
    assert per_run["a-run"]["dwell_s"] == 3
    assert per_run["b-run"]["has_capture"] is True
    assert per_run["b-run"]["normalization_status_ref"] == "unavailable"

    pair = next(
        p
        for p in report["compare_pairs"]
        if p["run_id_a"] == "a-run" and p["run_id_b"] == "b-run"
    )
    badge_ids = {b["id"] for b in pair["badges"]}
    assert "mode_changed" in badge_ids
    assert "assignment_changed" in badge_ids


def test_main_writes_out(manifest: Path, batch_spec: Path, tmp_path: Path) -> None:
    out = tmp_path / "report.json"
    code = analytics.main(
        [
            "--manifest",
            str(manifest),
            "--batch",
            str(batch_spec),
            "--repo-root",
            str(tmp_path),
            "--out",
            str(out),
        ]
    )
    assert code == 0
    data = json.loads(out.read_text(encoding="utf-8"))
    assert data["derived_at_utc"]
    assert data["rollup"]["mode_counts"]["manual"] == 1
