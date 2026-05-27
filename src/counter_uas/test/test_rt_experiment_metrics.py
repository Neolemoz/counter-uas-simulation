"""Tests for PLAT-RT-F5 P2 rt_experiment_metrics.py."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from scripts.rt import rt_experiment_analytics as analytics
from scripts.rt import rt_experiment_metrics as metrics
from scripts.rt import rt_experiment_spec_compile as spec_compile

_REPO = Path(__file__).resolve().parents[3]
_EXAMPLES = _REPO / "fixtures" / "rt_experiments" / "f5_spec_examples"
_GOLDEN = _REPO / "fixtures" / "rt_experiments" / "f5_metrics_golden"


def _matrix_fixture() -> tuple[dict, dict, dict]:
    spec = spec_compile._load_spec(_EXAMPLES / "parameter_matrix.json")
    fp = spec_compile.compute_spec_fingerprint(spec)
    manifest_raw = json.loads((_GOLDEN / "manifest.json").read_text(encoding="utf-8"))
    for run in manifest_raw["runs"]:
        run["spec_fingerprint"] = fp
    f1 = analytics.derive_analytics(manifest_raw)
    return manifest_raw, f1, spec


def _strip_derived(report: dict) -> dict:
    out = dict(report)
    out.pop("derived_at_utc", None)
    return out


def test_derive_matches_typescript_golden() -> None:
    manifest, f1, spec = _matrix_fixture()
    report = metrics.derive_metrics(manifest, f1, spec=spec)
    golden = json.loads((_GOLDEN / "metrics_report.json").read_text(encoding="utf-8"))
    assert _strip_derived(report) == _strip_derived(golden)


def test_repeatability_rollup_fingerprints() -> None:
    spec = spec_compile._load_spec(_EXAMPLES / "repeatability_sweep.json")
    fp = spec_compile.compute_spec_fingerprint(spec)
    batch = spec_compile.compile_experiment_spec(spec)
    manifest = {
        "schema": "rt_experiment_manifest_v1",
        "experiment_id": spec["experiment_id"],
        "created_at_utc": "2026-05-26T15:00:00+00:00",
        "governance_banner": "RT EXPERIMENT — explanatory compare only; not operational authority",
        "runs": [],
    }
    for i, row in enumerate(batch["runs"]):
        manifest["runs"].append(
            {
                "run_id": row["run_id"],
                "label": row["label"],
                "session_id": f"sess-{i:08d}",
                "recorded_at_utc": f"2026-05-26T15:0{i}:00+00:00",
                "experiment_class": "repeatability_sweep",
                "spec_fingerprint": fp,
                "repeat_index": row.get("repeat_index"),
                "repeat_group_id": row.get("repeat_group_id"),
                "snapshot": {
                    "tactical_state": {"tactical_mode": "manual"},
                    "world_summary": {"entity_count": 2 + i},
                },
            }
        )
    f1 = analytics.derive_analytics(manifest, batch)
    report = metrics.derive_metrics(manifest, f1, spec=spec)
    assert report["experiment_class"] == "repeatability_sweep"
    assert len(report["rollup_extended"]["repeatability_rollup"]["fingerprints"]) == 1
    assert report["rollup_extended"]["repeatability_rollup"]["fingerprints"][0]["run_count"] == 3


def test_handoff_ineligible_when_captures_missing() -> None:
    manifest, f1, spec = _matrix_fixture()
    for run in manifest["runs"]:
        run.pop("capture_candidate_id", None)
        run.pop("capture_staging_ref", None)
    f1 = analytics.derive_analytics(manifest)
    report = metrics.derive_metrics(manifest, f1, spec=spec)
    assert report["handoff_eligibility"]["experiment_level"] == "ineligible"
    gate = next(g for g in report["handoff_eligibility"]["gates"] if g["id"] == "all_captures_present")
    assert gate["pass"] is False


def test_handoff_eligible_when_gates_pass() -> None:
    manifest, f1, spec = _matrix_fixture()
    for run in manifest["runs"]:
        run["capture_candidate_id"] = f"cap-{run['run_id']}"
        run["capture_staging_ref"] = f"runs/rt_sandbox/captures/cap-{run['run_id']}"
    f1 = analytics.derive_analytics(manifest)
    for row in f1["per_run"]:
        row["normalization_status_ref"] = "normalized"
    report = metrics.derive_metrics(
        manifest,
        f1,
        spec=spec,
        maintainer_ack_pose_reviewed=True,
    )
    assert report["handoff_eligibility"]["experiment_level"] == "eligible"


def test_cli_writes_report(tmp_path: Path) -> None:
    manifest, f1, spec = _matrix_fixture()
    manifest_path = tmp_path / "manifest.json"
    spec_path = tmp_path / "spec.json"
    out_path = tmp_path / "metrics.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    spec_path.write_text(json.dumps(spec), encoding="utf-8")
    rc = metrics.main(
        [
            "--manifest",
            str(manifest_path),
            "--spec",
            str(spec_path),
            "--out",
            str(out_path),
        ]
    )
    assert rc == 0
    data = json.loads(out_path.read_text(encoding="utf-8"))
    assert data["schema"] == "rt_experiment_metrics_report_v1"
    assert "derived_at_utc" in data


def test_forbidden_rollup_keys_absent() -> None:
    manifest, f1, spec = _matrix_fixture()
    report = metrics.derive_metrics(manifest, f1, spec=spec)
    text = json.dumps(report)
    for key in metrics.FORBIDDEN_ROLLUP_KEYS:
        assert f'"{key}"' not in text
