"""Tests for PLAT-SA-F2A federation recovery continuity."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
_EVAL = _REPO / "scripts" / "evaluation"
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(mod)
    return mod


def test_federation_recovery_continuity_ok():
    mod = _load_module(
        "audit_federation_recovery_continuity",
        _EVAL / "audit_federation_recovery_continuity.py",
    )
    report = mod.audit_federation_recovery_continuity(_REPO)
    assert report["continuity_ok"] is True


def test_reconciliation_index_has_corpus_group_id():
    path = _REPO / "fixtures/orchestration/reconciliation/reconciliation_lineage_index_v1.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    for hold in data.get("quarantine_holds") or []:
        assert hold.get("corpus_group_id") == "canonical_r1"
    for group in data.get("retry_groups") or []:
        assert group.get("corpus_group_id") == "canonical_r1"
