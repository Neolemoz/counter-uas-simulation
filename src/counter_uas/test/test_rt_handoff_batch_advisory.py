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

from rt_sandbox.batch_advisory import (  # noqa: E402
    BATCH_REVIEW_SCHEMA,
    BatchFilters,
    aggregate_report,
    build_batch_review_document,
    corpus_preview_for_capture,
    derive_capture_row,
    filter_rows,
    next_maintainer_cli_hint,
    scan_staged,
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
    assert data["schema"] == BATCH_REVIEW_SCHEMA
    assert audit.read_text(encoding="utf-8") == before


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
