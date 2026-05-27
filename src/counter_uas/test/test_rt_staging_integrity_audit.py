"""Tests for rt_staging_integrity_audit.py (PLAT-RT-F2 P1)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
_BRIDGE = _REPO / "platform" / "rt-sandbox-bridge"
if str(_BRIDGE) not in sys.path:
    sys.path.insert(0, str(_BRIDGE))

from rt_sandbox.isolation import rt_sandbox_captures_dir  # noqa: E402
from scripts.rt import rt_staging_integrity_audit as staging_audit  # noqa: E402

audit_staging = staging_audit.audit_staging


def test_audit_staging_flags_missing_candidate(tmp_path: Path) -> None:
    root = rt_sandbox_captures_dir(tmp_path)
    root.mkdir(parents=True, exist_ok=True)
    (root / "cap-no-cand").mkdir()
    (root / "cap-no-cand" / "staging_placeholder.txt").write_text("pending", encoding="utf-8")
    (root / "cap-ok").mkdir()
    (root / "cap-ok" / "candidate.json").write_text(
        json.dumps({"capture_id": "cap-ok"}),
        encoding="utf-8",
    )
    issues = audit_staging(tmp_path)
    assert "cap-no-cand" in issues["missing_candidate"]
    assert issues["unreadable_candidate"] == []


def test_audit_staging_flags_unreadable_candidate(tmp_path: Path) -> None:
    root = rt_sandbox_captures_dir(tmp_path)
    bad = root / "cap-bad"
    bad.mkdir(parents=True, exist_ok=True)
    (bad / "candidate.json").write_text("{", encoding="utf-8")
    issues = audit_staging(tmp_path)
    assert "cap-bad" in issues["unreadable_candidate"]
