"""Tests for PLAT-RT-F6 P2 batch advisory CLIs."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
_BRIDGE = _REPO / "platform" / "rt-sandbox-bridge"
if str(_BRIDGE) not in sys.path:
    sys.path.insert(0, str(_BRIDGE))

from rt_sandbox.advisory_queue import (  # noqa: E402
    ADVISORY_BATCH_SUMMARY_SCHEMA,
    ADVISORY_BATCH_SUMMARY_V2_SCHEMA,
)
from rt_sandbox.batch_advisory import (  # noqa: E402
    ADVISORY_BATCH_REVIEW_V2_SCHEMA,
    ADVISORY_DRY_RUN_REVIEW_SCHEMA,
    BATCH_REVIEW_SCHEMA,
    BatchFilters,
    aggregate_report,
    build_advisory_batch_review_v2_document,
    build_advisory_batch_summary_document,
    build_dry_run_review_document,
    build_batch_review_document,
    build_grouped_export_indexes,
    corpus_preview_for_capture,
    derive_capture_row,
    filter_rows,
    filter_rows_by_group,
    next_maintainer_cli_hint,
    scan_staged,
    validate_advisory_batch_review_v2,
)
from rt_sandbox.isolation import rt_sandbox_captures_dir  # noqa: E402


def _stage_capture(tmp_path: Path, cid: str, *, rejected: bool = False) -> None:
    staging = rt_sandbox_captures_dir(tmp_path) / cid
    staging.mkdir(parents=True)
    candidate = {
        "schema": "rt_capture_candidate_v1",
        "capture_candidate_id": cid,
        "session_id": "sess-1",
        "ephemeral_session_ref": "sess-1",
        "normalization_status": "normalized",
        "approval_status": "pending",
        "origin": "rt_sandbox_capture_v1",
    }
    (staging / "candidate.json").write_text(json.dumps(candidate) + "\n", encoding="utf-8")
    (staging / "normalization_validation.json").write_text(
        json.dumps({"valid": True}) + "\n",
        encoding="utf-8",
    )
    if rejected:
        review = {
            "schema": "rt_handoff_review_v1",
            "capture_candidate_id": cid,
            "decision": "rejected",
            "reviewer": "test",
        }
        (staging / "handoff_review.json").write_text(json.dumps(review) + "\n", encoding="utf-8")


def test_next_maintainer_cli_hint_capture_ready() -> None:
    hint = next_maintainer_cli_hint(
        {"advisory_state": "capture_ready", "capture_candidate_id": "cap-a"}
    )
    assert hint is not None
    assert "cap-a" in hint
    assert "rt_handoff_review" in hint


def test_scan_and_aggregate(tmp_path: Path) -> None:
    _stage_capture(tmp_path, "batch-cap-1")
    _stage_capture(tmp_path, "batch-cap-2")
    rows = scan_staged(tmp_path)
    assert len(rows) == 2
    summary = aggregate_report(rows)
    assert summary["total"] == 2
    assert summary["by_advisory_state"].get("capture_ready", 0) >= 1


def test_filter_blocked(tmp_path: Path) -> None:
    _stage_capture(tmp_path, "ok-cap")
    _stage_capture(tmp_path, "rej-cap", rejected=True)
    rows = scan_staged(tmp_path)
    blocked = filter_rows(rows, BatchFilters(blocked_only=True))
    assert len(blocked) == 1
    assert blocked[0].capture_candidate_id == "rej-cap"


def test_batch_review_schema(tmp_path: Path) -> None:
    _stage_capture(tmp_path, "export-cap")
    rows = scan_staged(tmp_path)
    doc = build_batch_review_document(tmp_path, rows, dry_run=True)
    assert doc["schema"] == BATCH_REVIEW_SCHEMA
    assert doc["dry_run"] is True
    assert len(doc["captures"]) == 1


def test_dry_run_review_includes_status_buckets(tmp_path: Path) -> None:
    _stage_capture(tmp_path, "dry-cap-a")
    rows = scan_staged(tmp_path)
    doc = build_dry_run_review_document(tmp_path, rows, max_captures=5)
    assert doc["schema"] == ADVISORY_DRY_RUN_REVIEW_SCHEMA
    assert doc["dry_run"] is True
    assert doc["captures"]
    cap = doc["captures"][0]
    assert cap.get("status") in ("ran", "skipped", "error")


def test_corpus_preview_read_only(tmp_path: Path) -> None:
    _stage_capture(tmp_path, "preview-cap")
    sa = tmp_path / "fixtures" / "sa_r0" / "synthesis"
    sa.mkdir(parents=True)
    (sa / "replay_corpus_index_v1.json").write_text(
        json.dumps({"entries": []}) + "\n",
        encoding="utf-8",
    )
    preview = corpus_preview_for_capture(
        tmp_path,
        "preview-cap",
        corpus_dest=tmp_path / "fixtures" / "sa_r0" / "demo_preview_cap",
    )
    assert preview["schema"] == "rt_handoff_corpus_preview_v1"
    assert preview["would_add"] or preview["missing_staging_ref"]


def test_corpus_preview_rejects_outside_fixtures_sa_r0(tmp_path: Path) -> None:
    _stage_capture(tmp_path, "preview-cap2")
    preview = corpus_preview_for_capture(
        tmp_path,
        "preview-cap2",
        corpus_dest=tmp_path / "somewhere_else" / "demo_preview_cap2",
    )
    assert preview["dest_policy"] == "fixtures_sa_r0_only"
    assert preview["dest_valid"] is False
    assert "outside_fixtures_sa_r0" in (preview.get("dest_errors") or [])
    assert preview.get("would_add") == []
    assert preview.get("would_conflict") == []


def test_cli_report_no_audit_mutation(tmp_path: Path) -> None:
    audit = tmp_path / "runs" / "rt_sandbox" / "export_audit" / "export_boundary.jsonl"
    audit.parent.mkdir(parents=True)
    audit.write_text('{"event_type":"handoff_ready"}\n', encoding="utf-8")
    before = audit.read_text(encoding="utf-8")
    _stage_capture(tmp_path, "cli-cap")

    script = _REPO / "scripts" / "rt" / "rt_handoff_batch_advisory.py"
    proc = subprocess.run(
        [
            sys.executable,
            str(script),
            "report",
            "--repo-root",
            str(tmp_path),
            "--json",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0
    data = json.loads(proc.stdout)
    assert data["schema"] == ADVISORY_BATCH_SUMMARY_SCHEMA
    assert "readiness_cohorts" in data["summary"]
    assert audit.read_text(encoding="utf-8") == before


def test_f7_summary_document(tmp_path: Path) -> None:
    _stage_capture(tmp_path, "f7-export-cap")
    rows = scan_staged(tmp_path)
    doc = build_advisory_batch_summary_document(tmp_path, rows, dry_run=True)
    assert doc["schema"] == ADVISORY_BATCH_SUMMARY_SCHEMA
    cap = doc["captures"][0]
    assert "queue_priority" in cap
    assert "readiness_cohort" in cap


def test_filter_rows_by_group(tmp_path: Path) -> None:
    _stage_capture(tmp_path, "ok-cap")
    _stage_capture(tmp_path, "rej-cap", rejected=True)
    rows = scan_staged(tmp_path)
    blocked = filter_rows_by_group(rows, "terminal_block", repo_root=tmp_path)
    assert len(blocked) == 1
    assert blocked[0].capture_candidate_id == "rej-cap"


def test_cli_schema_f6_flag(tmp_path: Path) -> None:
    _stage_capture(tmp_path, "schema-cap")
    script = _REPO / "scripts" / "rt" / "rt_handoff_batch_advisory.py"
    proc = subprocess.run(
        [
            sys.executable,
            str(script),
            "report",
            "--repo-root",
            str(tmp_path),
            "--schema",
            "f6",
            "--json",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0
    data = json.loads(proc.stdout)
    assert data["schema"] == BATCH_REVIEW_SCHEMA


def test_cli_forbidden_commit_all_flag_absent() -> None:
    script = _REPO / "scripts" / "rt" / "rt_handoff_batch_advisory.py"
    proc = subprocess.run(
        [sys.executable, str(script), "--help"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0
    assert "  --commit-all" not in proc.stdout
    assert "  --auto-import" not in proc.stdout


def test_dry_run_import_never_writes_corpus(tmp_path: Path) -> None:
    cid = "dry-cap"
    staging = rt_sandbox_captures_dir(tmp_path) / cid
    staging.mkdir(parents=True)
    candidate = {
        "capture_candidate_id": cid,
        "session_id": "s1",
        "normalization_status": "normalized",
        "approval_status": "approved",
        "origin": "rt_sandbox_capture_v1",
    }
    (staging / "candidate.json").write_text(json.dumps(candidate) + "\n", encoding="utf-8")
    (staging / "conversion.json").write_text(
        json.dumps({"staging_refs": {}, "conversion_steps": []}) + "\n",
        encoding="utf-8",
    )
    corpus = tmp_path / "fixtures" / "sa_r0" / "demo_dry_cap"
    before_entries = list((tmp_path / "fixtures" / "sa_r0").glob("**/*"))

    script = _REPO / "scripts" / "rt" / "rt_sa_import_dry_run.py"
    subprocess.run(
        [sys.executable, str(script), "--repo-root", str(tmp_path), cid, "--json"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert not corpus.exists()
    after_entries = list((tmp_path / "fixtures" / "sa_r0").glob("**/*"))
    assert len(after_entries) == len(before_entries)


def test_derive_capture_row_missing(tmp_path: Path) -> None:
    row = derive_capture_row(tmp_path, "missing-id")
    assert row.error == "staging_not_found"


def test_v2_document_and_validation(tmp_path: Path) -> None:
    _stage_capture(tmp_path, "v2-cap")
    rows = scan_staged(tmp_path)
    doc = build_advisory_batch_review_v2_document(tmp_path, rows, dry_run=True)
    assert doc["schema"] == ADVISORY_BATCH_REVIEW_V2_SCHEMA
    assert doc["dry_run"] is True
    assert "grouped" in doc
    assert "standup" in doc
    assert doc["grouped"]["by_readiness_cohort"]
    assert validate_advisory_batch_review_v2(doc) == []


def test_grouped_export_indexes(tmp_path: Path) -> None:
    _stage_capture(tmp_path, "g1")
    _stage_capture(tmp_path, "g2")
    rows = scan_staged(tmp_path)
    dicts = [r.to_dict(repo_root=tmp_path) for r in rows]
    grouped = build_grouped_export_indexes(dicts)
    assert "by_queue_band" in grouped
    assert "by_blocker_group" in grouped
    assert "by_readiness_cohort" in grouped


def test_cli_standup_export(tmp_path: Path) -> None:
    _stage_capture(tmp_path, "standup-cap")
    script = _REPO / "scripts" / "rt" / "rt_handoff_batch_advisory.py"
    proc = subprocess.run(
        [
            sys.executable,
            str(script),
            "standup-export",
            "--repo-root",
            str(tmp_path),
            "--json",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0
    data = json.loads(proc.stdout)
    assert data["schema"] == ADVISORY_BATCH_REVIEW_V2_SCHEMA
    assert validate_advisory_batch_review_v2(data) == []


def test_cli_export_schema_v2(tmp_path: Path) -> None:
    _stage_capture(tmp_path, "v2-cli-cap")
    script = _REPO / "scripts" / "rt" / "rt_handoff_batch_advisory.py"
    proc = subprocess.run(
        [
            sys.executable,
            str(script),
            "export",
            "--repo-root",
            str(tmp_path),
            "--schema",
            "v2",
            "--json",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0
    assert json.loads(proc.stdout)["schema"] == ADVISORY_BATCH_REVIEW_V2_SCHEMA


def test_cli_dry_run_review_json(tmp_path: Path) -> None:
    _stage_capture(tmp_path, "dry-review-cap")
    script = _REPO / "scripts" / "rt" / "rt_handoff_batch_advisory.py"
    proc = subprocess.run(
        [
            sys.executable,
            str(script),
            "dry-run-review",
            "--repo-root",
            str(tmp_path),
            "--json",
            "--max-captures",
            "1",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0
    data = json.loads(proc.stdout)
    assert data["schema"] == ADVISORY_DRY_RUN_REVIEW_SCHEMA
    assert data["dry_run"] is True


def test_sa_import_dry_run_rejects_no_dry_run(tmp_path: Path) -> None:
    cid = "no-dry-cap"
    staging = rt_sandbox_captures_dir(tmp_path) / cid
    staging.mkdir(parents=True)
    (staging / "candidate.json").write_text(
        json.dumps(
            {
                "capture_candidate_id": cid,
                "session_id": "s1",
                "normalization_status": "normalized",
                "approval_status": "approved",
                "origin": "rt_sandbox_capture_v1",
            }
        )
        + "\n",
        encoding="utf-8",
    )
    script = _REPO / "scripts" / "rt" / "rt_sa_import_dry_run.py"
    proc = subprocess.run(
        [sys.executable, str(script), "--no-dry-run", cid, "--repo-root", str(tmp_path)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 2


def test_cli_export_f8_schema(tmp_path: Path) -> None:
    _stage_capture(tmp_path, "f8-cli-cap")
    script = _REPO / "scripts" / "rt" / "rt_handoff_batch_advisory.py"
    proc = subprocess.run(
        [
            sys.executable,
            str(script),
            "export",
            "--repo-root",
            str(tmp_path),
            "--schema",
            "f8",
            "--preset",
            "all_staged",
            "--template-pack",
            "standup_md_minimal",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0, proc.stderr
    data = json.loads(proc.stdout)
    assert data["schema"] == ADVISORY_BATCH_SUMMARY_V2_SCHEMA
    assert data["dry_run"] is True
    assert "template_render" in data
    assert "readiness_score" not in proc.stdout
