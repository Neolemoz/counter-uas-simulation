"""Tests for PLAT-SA-F2A federation integrity audit."""

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


def test_federation_integrity_ok():
    mod = _load_module(
        "audit_replay_federation_integrity",
        _EVAL / "audit_replay_federation_integrity.py",
    )
    report = mod.audit_federation_integrity(_REPO, strict=False)
    assert report["integrity_ok"] is True
    errors = [f for f in report["findings"] if f["severity"] == "error"]
    assert not errors


def test_federation_integrity_fixture_matches():
    mod = _load_module(
        "audit_replay_federation_integrity",
        _EVAL / "audit_replay_federation_integrity.py",
    )
    expected = mod.audit_federation_integrity(_REPO, strict=False)
    path = _REPO / "fixtures/sa_r0/federation/audits/replay_federation_integrity_report_v1.json"
    actual = json.loads(path.read_text(encoding="utf-8"))
    assert json.dumps(actual, sort_keys=True) == json.dumps(expected, sort_keys=True)
