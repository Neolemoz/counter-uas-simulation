"""Tests for PLAT-RT-F6 P0 advisory derive."""

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

from rt_sandbox.advisory_derive import (  # noqa: E402
    build_advisory_input,
    derive_advisory_status,
    derive_advisory_status_for_capture,
)

FIXTURE_DIR = _REPO / "fixtures" / "rt_handoff" / "f6_advisory_examples"
EXPECTED_DIR = FIXTURE_DIR / "expected"


def _load_fixture_input(data: dict) -> dict:
    inp = dict(data.get("inputs") or {})
    if "capture_advisory_inputs" in inp:
        merged = dict(inp["capture_advisory_inputs"])
        merged["capture_candidate_id"] = "test-cap"
        return merged
    inp.setdefault("capture_candidate_id", "test-cap")
    return inp


@pytest.mark.parametrize(
    "fixture_name",
    sorted(p.name for p in FIXTURE_DIR.glob("*.json")),
)
def test_golden_advisory_derive(fixture_name: str) -> None:
    raw = json.loads((FIXTURE_DIR / fixture_name).read_text(encoding="utf-8"))
    inp = _load_fixture_input(raw)
    got = derive_advisory_status(inp)
    expected_path = EXPECTED_DIR / fixture_name.replace(".json", ".expected.json")
    expected = json.loads(expected_path.read_text(encoding="utf-8"))
    assert got["advisory_state"] == expected["advisory_state"]
    assert got["blocked"] == expected["blocked"]
    assert got["advisory_state_label"] == expected["advisory_state_label"]


def test_derive_is_deterministic() -> None:
    raw = json.loads((FIXTURE_DIR / "approval_ready.json").read_text(encoding="utf-8"))
    inp = _load_fixture_input(raw)
    assert derive_advisory_status(inp) == derive_advisory_status(inp)


def test_cli_status_read_only(tmp_path: Path) -> None:
    audit = tmp_path / "runs" / "rt_sandbox" / "export_audit" / "export_boundary.jsonl"
    audit.parent.mkdir(parents=True)
    audit.write_text('{"event_type":"handoff_ready"}\n', encoding="utf-8")
    before = audit.read_text(encoding="utf-8")

    script = _REPO / "scripts" / "rt" / "rt_handoff_advisory_status.py"
    proc = subprocess.run(
        [
            sys.executable,
            str(script),
            "--repo-root",
            str(tmp_path),
            "status",
            "missing-cap",
            "--json",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 1
    assert audit.read_text(encoding="utf-8") == before


def test_integration_normalized_capture_advisory(tmp_path: Path) -> None:
    from rt_sandbox.isolation import rt_sandbox_captures_dir

    cid = "adv-test-cap"
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
    (staging / "candidate.json").write_text(
        json.dumps(candidate, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (staging / "normalization_validation.json").write_text(
        json.dumps({"valid": True}) + "\n",
        encoding="utf-8",
    )

    status = derive_advisory_status_for_capture(tmp_path, cid)
    assert status["advisory_state"] == "capture_ready"
    assert status["blocked"] is False

    inp = build_advisory_input(tmp_path, cid)
    assert inp["workflow_phase"] in {"normalized", "review_pending", "staged"}
