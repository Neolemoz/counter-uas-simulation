"""Offline selection-vs-oracle log parsing (evaluation harness)."""

from __future__ import annotations

from pathlib import Path
import sys

_REPO = Path(__file__).resolve().parents[3]
_EVAL = _REPO / 'scripts' / 'evaluation'
sys.path.insert(0, str(_EVAL))

from selection_audit import extract_selection_blocks, selection_audit_summary  # noqa: E402

_SAMPLE_LOG = r"""
[INFO] [interception_logic_node]: === Interceptor Selection ===

interceptor_0: feasible=True, tti=3.2
interceptor_1: feasible=False, tti=n/a
interceptor_2: feasible=True, tti=8.9
selected: interceptor_0
"""


def test_extract_oracle_matches_argmin_feasible() -> None:
    blocks = extract_selection_blocks(_SAMPLE_LOG)
    assert len(blocks) == 1
    blk = blocks[0]
    assert blk["selected"] == "interceptor_0"
    assert "interceptor_0" in blk["oracle_ids"]
    assert blk["oracle_match"] is True


def test_selection_audit_summary_aggregates() -> None:
    summary = selection_audit_summary(_SAMPLE_LOG)
    assert summary["n_selection_blocks"] >= 1
    assert summary["last_selected"] == "interceptor_0"


def test_ambiguous_feasibility_creates_low_signal() -> None:
    messy = "[INFO] interception_logic_node: === Interceptor Selection ===\n"
    messy += "\nselected: interceptor_0\n"
    summ = selection_audit_summary(messy)
    assert summ["selection_oracle_match_rate"] is None
