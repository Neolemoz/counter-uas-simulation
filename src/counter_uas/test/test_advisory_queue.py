"""Tests for PLAT-RT-F7 P0 advisory queue and aggregation."""

from __future__ import annotations

import json
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
    apply_filter_preset,
    apply_focus_set,
    classify_blocker_groups,
    compute_queue_priority,
    detect_lineage_warnings,
    readiness_cohort,
    rollup_blocker_groups,
    rollup_handoff_rollup,
    rollup_multi_capture_cohorts,
    rollup_readiness_cohorts,
    rollup_readiness_cohorts_v2,
    sort_rows_by_queue,
)
from rt_sandbox.batch_advisory import (  # noqa: E402
    build_advisory_batch_summary_document,
    build_advisory_batch_summary_v2_document,
    render_template_pack,
    scan_staged,
)
from rt_sandbox.isolation import rt_sandbox_captures_dir  # noqa: E402


def test_queue_priority_blocked_before_import() -> None:
    blocked = compute_queue_priority(
        {"blocked": True, "block_reasons": ["handoff_rejected"]},
        capture_id="a",
    )
    ready = compute_queue_priority(
        {"advisory_state": "import_ready", "blocked": False},
        capture_id="b",
    )
    assert blocked["rank"] < ready["rank"]
    assert blocked["band"] == "P0_block"


def test_readiness_cohort_mapping() -> None:
    assert readiness_cohort({"advisory_state": "approval_ready"}) == "needs_approve"
    assert readiness_cohort({"advisory_state": "import_ready"}) == "ready_for_commit_advisory"
    assert readiness_cohort({"terminal": "handoff_import_committed"}) == "terminal"


def test_lineage_warning_session_parent() -> None:
    warnings = detect_lineage_warnings(
        {
            "schema": "rt_normalized_capture_v1",
            "origin": "rt_sandbox_capture_v1",
            "capture_candidate_id": "cap-x",
            "parent_ref": "sess-1",
            "session_id": "sess-1",
        }
    )
    assert any("LIN-01" in w for w in warnings)


def test_blocker_group_rollup() -> None:
    rows = [
        {
            "capture_candidate_id": "c1",
            "blocker_groups": ["normalization", "review_attestation"],
        },
        {"capture_candidate_id": "c2", "blocker_groups": ["normalization"]},
    ]
    groups = rollup_blocker_groups(rows)
    assert groups["normalization"]["count"] == 2
    assert "c1" in groups["normalization"]["exemplar_capture_ids"]


def test_sort_rows_by_queue() -> None:
    rows = [
        {
            "capture_candidate_id": "late",
            "queue_priority": {"rank": 600, "band": "P6_import"},
        },
        {
            "capture_candidate_id": "early",
            "queue_priority": {"rank": 100, "band": "P1_error"},
            "error": "staging_not_found",
        },
    ]
    sorted_rows = sort_rows_by_queue(rows)
    assert sorted_rows[0]["capture_candidate_id"] == "early"


def _stage(tmp_path: Path, cid: str, **extra: object) -> None:
    staging = rt_sandbox_captures_dir(tmp_path) / cid
    staging.mkdir(parents=True)
    candidate = {
        "schema": "rt_capture_candidate_v1",
        "capture_candidate_id": cid,
        "session_id": "sess-1",
        "normalization_status": "normalized",
        "approval_status": "pending",
        "origin": "rt_sandbox_capture_v1",
        "generated_at": "2026-05-01T10:00:00+00:00",
    }
    candidate.update(extra)  # type: ignore[arg-type]
    (staging / "candidate.json").write_text(json.dumps(candidate) + "\n", encoding="utf-8")
    (staging / "normalization_validation.json").write_text(
        json.dumps({"valid": True}) + "\n",
        encoding="utf-8",
    )


def test_f7_batch_summary_schema(tmp_path: Path) -> None:
    _stage(tmp_path, "f7-cap-a")
    _stage(tmp_path, "f7-cap-b", generated_at="2026-05-02T10:00:00+00:00")
    rows = scan_staged(tmp_path)
    doc = build_advisory_batch_summary_document(tmp_path, rows, dry_run=True)
    assert doc["schema"] == ADVISORY_BATCH_SUMMARY_SCHEMA
    assert "blocker_groups" in doc["summary"]
    assert "readiness_cohorts" in doc["summary"]
    assert doc["captures"][0].get("queue_priority")
    assert doc["captures"][0].get("readiness_cohort")


def test_classify_experiment_warn_group() -> None:
    adv = {"advisory_state": "approval_ready", "blocked": False, "block_reasons": []}
    groups = classify_blocker_groups(adv, experiment_warn=True)
    assert "experiment_warn" in groups


def test_rollup_cohorts() -> None:
    rows = [
        {"readiness_cohort": "needs_review"},
        {"readiness_cohort": "needs_review"},
        {"readiness_cohort": "needs_approve"},
    ]
    counts = rollup_readiness_cohorts(rows)
    assert counts["needs_review"] == 2
    assert counts["needs_approve"] == 1


def test_f8_filter_preset_import_advisory() -> None:
    rows = [
        {
            "capture_candidate_id": "a",
            "advisory": {"advisory_state": "import_ready", "blocked": False},
            "blocker_groups": [],
        },
        {
            "capture_candidate_id": "b",
            "advisory": {"advisory_state": "capture_ready", "blocked": False},
            "blocker_groups": [],
        },
    ]
    filtered = apply_filter_preset(rows, "import_advisory_only")
    assert [r["capture_candidate_id"] for r in filtered] == ["a"]


def test_f8_focus_set() -> None:
    rows = [
        {"capture_candidate_id": "a"},
        {"capture_candidate_id": "b"},
    ]
    focused = apply_focus_set(rows, ["a"])
    assert len(focused) == 1
    assert focused[0]["in_focus_set"] is True


def test_f8_batch_summary_v2_schema(tmp_path: Path) -> None:
    _stage(tmp_path, "f8-cap-a")
    rows = scan_staged(tmp_path)
    doc = build_advisory_batch_summary_v2_document(
        tmp_path,
        rows,
        dry_run=True,
        preset_applied="all_staged",
    )
    assert doc["schema"] == ADVISORY_BATCH_SUMMARY_V2_SCHEMA
    assert doc["dry_run"] is True
    assert "readiness_cohorts_v2" in doc["summary"]
    assert "multi_capture_cohorts" in doc["summary"]
    assert "handoff_rollup" in doc["summary"]
    assert "readiness_score" not in json.dumps(doc)


def test_f8_handoff_rollup() -> None:
    rows = [
        {"readiness_cohort": "needs_review"},
        {"readiness_cohort": "ready_for_commit_advisory"},
    ]
    hr = rollup_handoff_rollup(rows)
    assert hr["by_stage"]["review"] == 1
    assert hr["by_stage"]["import_advisory"] == 1


def test_f8_template_pack_minimal(tmp_path: Path) -> None:
    _stage(tmp_path, "tpl-cap")
    rows = scan_staged(tmp_path)
    doc = build_advisory_batch_summary_v2_document(tmp_path, rows, dry_run=True)
    rendered = render_template_pack(doc, "standup_md_minimal")
    assert rendered["format"] == "markdown"
    assert "next_cli" not in rendered["content"]
    assert "readiness_score" not in rendered["content"]


def test_f8_multi_capture_cohorts() -> None:
    mc = rollup_multi_capture_cohorts(
        [{"readiness_cohort": "needs_review"}, {"readiness_cohort": "needs_approve"}]
    )
    assert mc["by_primary_lane"]["review"] == 1
    assert mc["note"]
